/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_core.c/h
  * @brief      核心调度实现，包括初始化，TIM3 10ms 中断任务，主循环轮询
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

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

/* [调试便民] 给 Keil Watch 窗口用的两个量(见 car_core.h 的说明):
   g_dbg_state 把"整定/单轮/开环"三个 RAM 状态汇总成一个数(0=正常);
   g_dbg_clear 写 1 就一次性把它们全部复位, 不用断电重启。 */
volatile uint8_t g_dbg_state = 0;
volatile uint8_t g_dbg_clear = 0;

/* 100ms 速度环分频 */
static uint8_t s_pid_div = 0;

/* 10ms 周期任务(在 TIM3 中断里跑) */
static void Car_Tick10ms(void);

#if CAR_USE_ULTRA
static uint32_t s_last_ultra = 0;
#endif
#if CAR_USE_OLED
static uint32_t s_last_oled  = 0;       //只在 OLED 使能时用得到, 否则会有未引用告警
#endif
static uint32_t s_last_print = 0;
static uint32_t s_last_gray  = 0;       //主循环里灰度+转向的 10ms 节流
static uint8_t  s_fault_warned = 0;     // [诊断] 极性告警边沿触发, 只打印一次

/* 整车就绪标志: TIM3 中断在 MX_TIM3_Init() 里就开始跑了, 比 Car_Init() 还早,
   所以必须等初始化真正做完再认按键 —— 否则上电时按住键(或按键被卡住)会在初始化
   期间触发一次 Task_Start(), 初始化一结束车就自己窜出去。 */
static volatile uint8_t s_car_ready = 0;

/* ---------------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------------- */
/**
  * @brief          car init: all drivers, PID params and task layer
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          整车初始化: 各驱动、PID 参数和任务层
  * @param[in]      none
  * @retval         none
  */
void Car_Init(void)
{
    Motor_Init();
    Encoder_Init();

    pid_init(&motorA, PID_DELTA, SPD_KP, SPD_KI, SPD_KD);
    pid_init(&motorB, PID_DELTA, SPD_KP, SPD_KI, SPD_KD);

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
        MPU6050_CalGyroBias();      // 静止 2.2 秒采零偏, 期间车不能动
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
    /* [DIAG] 屏没应答: 打印扫描结果, 并按开关停在引脚自检方波上。
       OLED_FORCE_PIN_TEST=1 则不论应答都进(SPI 模式没有 ACK, 查线时用这个)。 */
#if OLED_FORCE_PIN_TEST
    OLED_FailDiag();
#else
    if (!g_oled_ack) OLED_FailDiag();
#endif
#endif

    s_car_ready = 1;        //到这一步才算真的就绪, 之后按键才会被响应
}

/* ---------------------------------------------------------------------------
 * TIM3 每 10ms 中断 (ISR)
 *
 * [2026-10-08 按框架图调整] 中断只留速度闭环(编码器->速度环PID->电机) + 按键/蜂鸣,
 * 灰度读取和转向计算(Task_Update)移到主循环 Car_Poll 以 10ms 节流跑,
 * 算出的左右目标转速由中断里的速度环消费。OLED 刷屏期间灰度会短暂停采, 属已知取舍。
 *
 * 注意: 这里会调 HAL_I2C(读灰度), 它的超时靠 HAL_GetTick(即 SysTick)计时,
 *       所以 SysTick 抢占优先级必须高于 TIM3, 否则 I2C 出错时会死等在中断里
 *       (见 Core/Inc/stm32f1xx_hal_conf.h 的 TICK_INT_PRIORITY)。
 * ------------------------------------------------------------------------- */
/**
  * @brief          TIM3 10ms interrupt task
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          TIM3 每 10ms 中断任务
  * @param[in]      none
  * @retval         none
  */
void TIM3_10ms_Task(void)
{
    g_ms += CTRL_TICK_MS;

    /* 初始化没做完: 只累计时间, 不碰外设 —— 否则 Task_BeepLed 每 10ms 把 LED
       写灭, 和 Car_Init 里的诊断闪烁/报信打架(50Hz 亮灭调制, 看着像乱闪);
       MPU/灰度/PID 也不会在外设未初始化时被中断裸跑。 */
    if (!s_car_ready) return;

#if CAR_USE_MPU
    MPU_Task10ms();                 // burst 读 14 字节 + yaw 积分 (软I2C)
#endif

    Car_Tick10ms();                 // 按键 -> 蜂鸣LED (灰度/转向已移主循环)

    if (++s_pid_div >= CTRL_PID_DIV)
    {
        s_pid_div = 0;
        pid_control();              // 100ms 速度环: 放最后, 用刚算出的目标值
    }
}

/* ---------------------------------------------------------------------------
 * 每 10ms 的周期任务 (由 TIM3 中断调用, 不在主循环)
 * ------------------------------------------------------------------------- */
/**
  * @brief          10ms periodic task
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          每 10ms 的周期任务
  * @param[in]      none
  * @retval         none
  */
static void Car_Tick10ms(void)
{
    uint8_t k;

    /* --- [调试便民] Watch 里把 g_dbg_clear 写 1: 一次性清掉所有测试/整定状态 --- */
    if (g_dbg_clear)
    {
        g_dbg_clear = 0;
        g_pid_tune  = 0;            //关: 整定旁路
        g_pid_wheel = 0;            //关: 单轮选择(它会强制另一侧 out=0)
        g_open_loop = 0;            //关: 开环固定 duty
        g_test_duty   = 25000;      //回到"两轮都转"的默认
        g_test_duty_r = 25000;
        g_test_dir_l  = 0;
        g_test_dir_r  = 0;
    }

    /* 汇总成一位掩码, 方便 Watch 一眼看: 0=正常 */
    g_dbg_state = (uint8_t)((g_pid_tune  ? 0x01u : 0u) |
                            (g_pid_wheel ? 0x02u : 0u) |
                            (g_open_loop ? 0x04u : 0u));

    /* --- 按键 = 题目两个任务 ---
     *     KEY1 = SW2 (PA15): A->B, 循迹走够里程到 B 点停车 (MODE_AB)
     *     KEY2 = SW5 (PA12): A->A, 绕一圈回起点压停止线停车 (MODE_LAP)
     *     运行中按任意键 = 立即急停。 */
#if CAR_USE_KEY
    k = Key_Scan();
    if (s_car_ready && k != KEY_NONE)
    {
        if (g_running)
        {
            Task_StopFinish();                      //运行中: 任意键急停
        }
        else
        {
            Task_SetMode((k == KEY_MODE) ? MODE_AB : MODE_LAP);
            Task_Start();
        }
    }
#endif

    /* --- 蜂鸣器 + LED --- */
    Task_BeepLed();
}

/* ---------------------------------------------------------------------------
 * 主循环
 * ------------------------------------------------------------------------- */
/**
  * @brief          main loop poll
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          主循环轮询
  * @param[in]      none
  * @retval         none
  */
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

    /* --- 灰度 + 转向 (框架图: 主循环负责传感与转向, 目标转速交给中断里的速度环) --- */
    if (g_ms - s_last_gray >= 10)
    {
        s_last_gray = g_ms;
#if CAR_USE_GRAY
        {
            uint8_t bits[8];
            uint8_t i;

            if (Gray_ReadBits(bits))
            {
                for (i = 0; i < GRAY_CHANNELS; i++) g_sensor_data[i] = bits[i];
            }
            /* 通信失败: 保持上一次的值, 别让车瞎转 */
        }
#else
        {
            uint8_t i;

            for (i = 0; i < GRAY_CHANNELS; i++) g_sensor_data[i] = 0;
        }
#endif
        Task_Update();      //转向: 误差 -> 左右目标转速, 速度环在 TIM3 中断里消费
    }

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
            OLED_ShowStatus((uint8_t)g_mode, Task_ModeName((uint8_t)g_mode), st, g_run_ms, g_dbg_state);
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
        /* tick = TIM3 心跳推算的毫秒 / SysTick 的真实毫秒。两者越跑越差, 说明 10ms
           中断里有活(灰度硬件 I2C、MPU 软 I2C)超时把节拍吞了 -> 速度环标称的 100ms
           其实不准。差得多就先怀疑这两处, 别去调 PID 参数。 */
        printf("dist=%ld/%ld | tick=%lu/%lu | yaw=%.1f | ultra=%u | grayok=%d mpuok=%d\r\n",
               (long)g_dist_cnt, (long)g_ab_cnt,
               (unsigned long)(g_tim3_tick * 10U), (unsigned long)HAL_GetTick(),
               g_yaw, (unsigned)g_ultra_mm, (int)g_gray_ok, (int)g_mpu_ok);

        /* [诊断] 调试/整定状态还在生效时高亮提示。
           这几个都是 RAM 变量(调试器里写过就一直在, 只有复位/断电才回默认值),
           忘了复位会表现成"只有一个轮子转 / 不循迹 / 开环固定占空比", 很像硬件坏了。 */
        if (g_pid_tune || g_pid_wheel || g_open_loop)
        {
            printf("!! DEBUG STATE: tune=%u wheel=%u open_loop=%u"
                   "  (test/tune not reset; power cycle or reset to clear)\r\n",
                   (unsigned)g_pid_tune, (unsigned)g_pid_wheel, (unsigned)g_open_loop);
        }

        /* [诊断] 编码器极性/失控告警: 目标是正的、实测速度却持续为负、out 又顶满,
           就是那一路编码器极性接反了(正反馈 -> 满占空比狂奔)。故障出现时只报一次。
           串口一律用 ASCII, 别往 printf 里塞中文(终端编码会对不上)。 */
        if ((g_pid_fault_l || g_pid_fault_r) && !s_fault_warned)
        {
            s_fault_warned = 1;
            printf("!! PID FAULT: L=%u R=%u | tgt>0 but now<0 and out saturated"
                   " -> ENCODER POLARITY? flip g_enc_dir_l / g_enc_dir_r\r\n",
                   (unsigned)g_pid_fault_l, (unsigned)g_pid_fault_r);
        }
        else if (!g_pid_fault_l && !g_pid_fault_r)
        {
            s_fault_warned = 0;         // 恢复正常后重新武装, 下次再报
        }
    }
#endif
}
