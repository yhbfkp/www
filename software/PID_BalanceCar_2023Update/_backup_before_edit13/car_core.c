#include "car_core.h"
#include "car_config.h"
#include "car_motor.h"
#include "car_encoder.h"
#include "car_gray.h"
#include "car_track.h"
#include "car_pid.h"
#include "car_task.h"
#include "car_mpu6050.h"
#include "car_oled.h"
#include "car_key.h"
#include "car_ultra.h"
#include "main.h"
#include "tim.h"
#include "delay.h"

volatile uint32_t g_ms = 0;

/* 100ms 速度环分频 */
static uint8_t s_pid_div = 0;

/* 10ms 周期任务(在 TIM3 中断里跑) */
static void Car_Tick10ms(void);

#if CAR_USE_ULTRA
static uint32_t s_last_ultra = 0;
#endif
static uint32_t s_last_oled  = 0;
static uint32_t s_last_print = 0;

/* ---------------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------------- */
void Car_Init(void)
{
    Motor_Init();
    Encoder_Init();

    pid_init(&motorA, DELTA_PID, SPD_KP, SPD_KI, SPD_KD);
    pid_init(&motorB, DELTA_PID, SPD_KP, SPD_KI, SPD_KD);

#if CAR_USE_OLED
    OLED_Init();
    OLED_Clear();

    /* 面包板飞线时不一定看得见串口, 用板载 LED(PA4, 低电平亮)直接报探测结果:
         有 ACK -> 慢闪 3 次, 说明屏已挂上 I2C, 不亮就是显示配置问题
         无 ACK -> 急闪 20 次(约4秒), 说明屏根本没挂到总线上, 先查接线
       报完之后 LED 交还给任务层正常使用 */
    {
        int i, n = g_oled_ack ? 3 : 20;

        for (i = 0; i < n; i++)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            delay_ms(g_oled_ack ? 200 : 100);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            delay_ms(g_oled_ack ? 200 : 100);
        }
    }
#endif

#if CAR_USE_GRAY
    if (!Gray_Init())
    {
        /* 默认地址扫不到就全扫一遍, 拿到地址自动记住 */
        uint8_t a = Gray_Scan(0x40, 0x47);
        if (!a) a = Gray_Scan(0x08, 0x77);
    }
#endif

#if CAR_USE_MPU
    if (MPU6050_Init() == 0)
    {
#if MPU_CAL_ENABLE
        MPU6050_CalGyroBias();      /* 静止 2.2 秒采零偏, 期间车不能动 */
#endif
    }
#endif

#if CAR_USE_ULTRA
    Ultra_Init();
#endif

#if CAR_USE_KEY
    Key_Init();
#endif

    Task_Init();

#if CAR_USE_PRINT
    printf("\r\n=== Car Ready ===\r\n");
    printf("gray_addr=0x%02X  mpu_ok=%d\r\n", (unsigned)g_gray_addr, (int)g_mpu_ok);
#if CAR_USE_OLED
    printf("oled_addr=0x%02X  oled_ack=%d%s\r\n", (unsigned)g_oled_addr, (int)g_oled_ack,
           g_oled_ack ? "" : "  <-- NO ACK! check wiring/VCC/pin");
#endif
#endif

#if CAR_USE_OLED
    /* [DIAG] 屏没应答: 打印总线扫描结果, 并按开关停在引脚自检方波上 */
    if (!g_oled_ack) OLED_FailDiag();
#endif
}

/* ---------------------------------------------------------------------------
 * TIM3 每 10ms 中断 (ISR)
 *
 * 所有周期性任务都在这里跑, 主循环只剩 OLED/超声波/串口这些慢活。
 * 这样 OLED 一次全屏≈37ms 的阻塞再也不会拖慢灰度采样/停止线判定/速度环。
 *
 * 注意: 这里会调 HAL_I2C(读灰度), 它的超时靠 HAL_GetTick(即 SysTick)计时,
 *       所以 SysTick 抢占优先级必须高于 TIM3, 否则 I2C 出错时会死等在中断里
 *       (见 Core/Inc/stm32f1xx_hal_conf.h 的 TICK_INT_PRIORITY)。
 * ------------------------------------------------------------------------- */
void TIM3_10ms_Task(void)
{
    g_ms += CTRL_TICK_MS;

#if CAR_USE_MPU
    MPU_Task10ms();                 /* burst 读 14 字节 + yaw 积分 (软I2C) */
#endif

    Car_Tick10ms();                 /* 灰度 -> 按键 -> 任务层 -> 蜂鸣LED */

    if (++s_pid_div >= CTRL_PID_DIV)
    {
        s_pid_div = 0;
        pid_control();              /* 100ms 速度环: 放最后, 用刚算出的目标值 */
    }
}

/* ---------------------------------------------------------------------------
 * 每 10ms 的周期任务 (由 TIM3 中断调用, 不在主循环)
 * ------------------------------------------------------------------------- */
static void Car_Tick10ms(void)
{
    uint8_t bits[8];
    uint8_t i;
    uint8_t k;

    /* --- 读灰度 --- */
#if CAR_USE_GRAY
    if (Gray_ReadBits(bits))
    {
        for (i = 0; i < GRAY_CHANNELS; i++) g_sensor_data[i] = bits[i];
    }
    else
    {
        /* 通信失败: 保持上一次的值, 别让车瞎转 */
    }
#else
    for (i = 0; i < GRAY_CHANNELS; i++) g_sensor_data[i] = 0;
#endif

    /* --- 按键: 两个键功能完全一样, 都是"启动 / 停止"切换 ---
     * 不再切模式。模式固定 MODE_TRACK (Task_Init 里定), 启动后一直循迹不自动停。 */
#if CAR_USE_KEY
    k = Key_Scan();
    if (k == KEY_MODE || k == KEY_RUN)
    {
        if (g_running) { Task_StopFinish(); }
        else           { Task_Start(); }
    }
#endif

    /* --- 任务层: 转向 / 停止线 / 里程 --- */
    Task_Update();

    /* --- 蜂鸣器 + LED --- */
    Task_BeepLed();
}

/* ---------------------------------------------------------------------------
 * 主循环
 * ------------------------------------------------------------------------- */
void Car_Poll(void)
{
    /* 10ms 的活(灰度/按键/任务层/蜂鸣)已搬到 TIM3 中断, 主循环只留慢活 */

    /* --- 超声波: 100ms 触发一次 --- */
#if CAR_USE_ULTRA
    if (g_ms - s_last_ultra >= 100)
    {
        s_last_ultra = g_ms;
        Ultra_GetMM();
    }
#endif

    /* --- OLED: 模式 + 用时 --- */
#if CAR_USE_OLED
    if (g_ms - s_last_oled >= OLED_REFRESH_MS)
    {
        s_last_oled = g_ms;
        {
            uint8_t st;
            if (g_obs_flag)      st = ST_OBS;
            else if (g_running)  st = ST_RUN;
            else if (g_finished) st = ST_DONE;
            else                 st = ST_IDLE;
            OLED_ShowStatus((uint8_t)g_mode, Task_ModeName((uint8_t)g_mode), st, g_run_ms);
        }
    }
#endif

    /* --- 串口全量打印 --- */
#if CAR_USE_PRINT
    if (g_ms - s_last_print >= 500)
    {
        s_last_print = g_ms;
        printf("m%d %s | ms=%lu | bits=%02X hits=%d err=%.1f st=%s | ",
               (int)g_mode, Task_ModeName((uint8_t)g_mode),
               (unsigned long)g_run_ms, (unsigned)g_gray_bits, (int)g_gray_hits,
               g_track_err, g_track_state);
        printf("spd L %.0f/%.0f R %.0f/%.0f | out %.0f/%.0f | ",
               g_spd_now_l, g_spd_tgt_l, g_spd_now_r, g_spd_tgt_r, g_out_l, g_out_r);
        printf("dist=%ld/%ld | yaw=%.1f | ultra=%u | grayok=%d mpuok=%d\r\n",
               (long)g_dist_cnt, (long)g_ab_cnt, g_yaw,
               (unsigned)g_ultra_mm, (int)g_gray_ok, (int)g_mpu_ok);
    }
#endif
}
