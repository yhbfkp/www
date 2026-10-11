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

/* 主循环里的软定时 */
static uint32_t s_last_10ms  = 0;
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
#endif
}

/* ---------------------------------------------------------------------------
 * TIM3 每 10ms 中断 (ISR)
 * ------------------------------------------------------------------------- */
void TIM3_10ms_Task(void)
{
    g_ms += CTRL_TICK_MS;

#if CAR_USE_MPU
    MPU_Task10ms();                 /* burst 读 14 字节 + yaw 积分 */
#endif

    if (++s_pid_div >= CTRL_PID_DIV)
    {
        s_pid_div = 0;
        pid_control();              /* 100ms 速度环 */
    }
}

/* ---------------------------------------------------------------------------
 * 每 10ms 的主循环任务
 * ------------------------------------------------------------------------- */
static void Car_Poll10ms(void)
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

    /* --- 按键 --- */
#if CAR_USE_KEY
    k = Key_Scan();
    if (k == KEY_MODE)
    {
        uint8_t m = g_mode + 1;
        if (m > MODE_MAX) m = 0;
        Task_SetMode(m);
    }
    else if (k == KEY_RUN)
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
    if (g_ms != s_last_10ms)
    {
        s_last_10ms = g_ms;
        Car_Poll10ms();
    }

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
