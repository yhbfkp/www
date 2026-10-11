#include "car_task.h"
#include "car_pid.h"
#include "car_track.h"
#include "car_motor.h"
#include "car_encoder.h"
#include "car_ultra.h"
#include "main.h"

/* 板上资源: BEEP = PB5, LED = PA4 (来自 main.h) */
/* LED 是低电平点亮(gpio.c 初始把它置高=灭); 如果你的板相反, 把下面两行对调 */
#define LED_ON_LEVEL    GPIO_PIN_SET
#define LED_OFF_LEVEL   GPIO_PIN_RESET
/* 蜂鸣器: 高电平响(三极管驱动); 如果你的板相反, 把下面两行对调 */
#define BEEP_ON_LEVEL   GPIO_PIN_SET
#define BEEP_OFF_LEVEL  GPIO_PIN_RESET

volatile uint8_t  g_mode      = MODE_AB;
volatile uint8_t  g_running   = 0;
volatile uint8_t  g_finished  = 0;
volatile uint8_t  g_obs_flag  = 0;
volatile uint8_t  g_gray_hits = 0;
volatile uint32_t g_run_ms    = 0;
volatile int32_t  g_dist_cnt  = 0;
volatile int32_t  g_ab_cnt    = 0;
volatile int32_t  g_lap_min_cnt = 0;
volatile int32_t  g_lap_max_cnt = 0;
volatile float    g_ab_mm     = TASK_AB_MM;
volatile float    g_cnt_per_mm = ENC_CNT_PER_MM;
volatile uint8_t  g_open_loop = 0;
volatile int16_t  g_test_duty   = 8000;    /* MODE_TEST duty(0..50000): A路 = 左轮 */
volatile int16_t  g_test_duty_r = 8000;    /* MODE_TEST duty: B路 = 右轮; 设0 = 只转左轮 */
volatile uint8_t  g_test_dir_l  = 0;       /* 左轮方向 0=正转 1=反转 */
volatile uint8_t  g_test_dir_r  = 0;       /* 右轮方向 0=正转 1=反转 */

extern volatile uint32_t g_ms;             /* 系统毫秒, 由 TIM3 每 10ms 中断累加 (car_core.c) */

/* 停止线状态机 */
static uint8_t  s_armed     = 0;
static uint8_t  s_leave_cnt = 0;
static uint16_t s_black_ms  = 0;
static uint8_t  s_cand_cnt  = 0;    /* 次级判据(>=3路)连续帧计数 */

/* 计时节拍: 记录上次 Task_Update 时的 g_ms, 用差值累加, 防止漏 tick 导致计时偏慢 */
static uint32_t s_last_tick = 0;

/* 蜂鸣/LED 计时 */
static uint16_t s_beep_ms = 0;
static uint16_t s_led_ms  = 0;
static uint8_t  s_led_on  = 0;

const char *Task_ModeName(uint8_t m)
{
    switch (m)
    {
        case MODE_IDLE:  return "IDLE";
        case MODE_AB:    return "A->B";
        case MODE_LAP:   return "A->A";
        case MODE_TRACK: return "TRACK";
        case MODE_TEST:  return "TEST";
        default:         return "----";
    }
}

void Task_Init(void)
{
    g_mode = MODE_AB;
    g_running = 0;
    g_finished = 0;
    g_obs_flag = 0;
    g_run_ms = 0;
    g_dist_cnt = 0;
    s_armed = 0;
    s_leave_cnt = 0;
    s_black_ms = 0;
    s_cand_cnt  = 0;
    s_last_tick = g_ms;

    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, BEEP_OFF_LEVEL);
    HAL_GPIO_WritePin(LED_GPIO_Port,  LED_Pin,  LED_OFF_LEVEL);
}

void Task_SetMode(uint8_t m)
{
    if (m > MODE_MAX) m = 0;
    g_mode = m;
    HARD_BRAKE();
    g_running = 0;
    g_finished = 0;
    g_obs_flag = 0;
    g_run_ms = 0;
    g_dist_cnt = 0;
    s_armed = 0;
    s_leave_cnt = 0;
    s_black_ms = 0;
    s_cand_cnt  = 0;
    s_last_tick = g_ms;
    g_open_loop = (m == MODE_TEST) ? 1 : 0;
}

void Task_Start(void)
{
    /* 里程目标: 用 g_cnt_per_mm 换算, 实测后可在 watch 里直接改这两个值 */
    g_ab_cnt      = (int32_t)(g_ab_mm * g_cnt_per_mm);
    g_lap_min_cnt = (int32_t)(LAP_MM * LAP_MIN_RATIO * g_cnt_per_mm);
    g_lap_max_cnt = (int32_t)(LAP_MM * LAP_MAX_RATIO * g_cnt_per_mm);

    g_enc_l_total = 0;
    g_enc_r_total = 0;
    Encoder_GetL();          /* 清掉上次残留 */
    Encoder_GetR();

    g_dist_cnt = 0;
    g_run_ms   = 0;
    g_finished = 0;
    g_obs_flag = 0;
    s_armed    = 0;
    s_leave_cnt = 0;
    s_black_ms = 0;
    s_cand_cnt  = 0;
    s_last_tick = g_ms;

    track_reset();
    pid_angle_reset();
    HARD_BRAKE();

    g_open_loop = (g_mode == MODE_TEST) ? 1 : 0;
    g_running   = 1;
}

void Task_StopFinish(void)
{
    HARD_BRAKE();
    track_reset();
    g_running  = 0;
    g_finished = 1;
    s_beep_ms  = STOP_BEEP_MS;      /* 蜂鸣 1s */
    s_led_ms   = STOP_LED_MS;       /* LED 同步闪 1s */
}

/* 蜂鸣器 / LED 计时, 10ms 一次 */
void Task_BeepLed(void)
{
    if (s_beep_ms)
    {
        s_beep_ms -= 10;
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, BEEP_ON_LEVEL);
    }
    else
    {
        /* 被障碍挡住时持续鸣响 */
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin,
                          g_obs_flag ? BEEP_ON_LEVEL : BEEP_OFF_LEVEL);
    }

    if (s_led_ms)
    {
        s_led_ms -= 10;
        s_led_on = !s_led_on;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, s_led_on ? LED_ON_LEVEL : LED_OFF_LEVEL);
    }
    else if (g_obs_flag)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LED_ON_LEVEL);   /* 障碍时常亮 */
    }
    else
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LED_OFF_LEVEL);
    }
}

/* 已走里程: 两轮绝对值取平均(不依赖极性, 也不怕单轮打滑) */
static int32_t Task_DistCnt(void)
{
    int32_t dl = g_enc_l_total; if (dl < 0) dl = -dl;
    int32_t dr = g_enc_r_total; if (dr < 0) dr = -dr;
    return (dl + dr) / 2;
}

void Task_Update(void)
{
    uint8_t  i;
    uint32_t now = g_ms;
    uint32_t dtick;

    /* 先吃掉本次经过的 tick。不能用固定 "+10":
       主循环会被 OLED 刷屏(一次全屏≈37ms)拖慢, tick 会被漏掉,
       硬加 10 会让计时越走越慢(实测每 500ms 少算约 30ms, 25s 会显示成 23.5s)。 */
    dtick = now - s_last_tick;
    s_last_tick = now;
    if (dtick > 100) dtick = 100;       /* 异常保护: 单次最多补 1 秒 */

    if (!g_running)
    {
        motor_target_set(0, 0);
        return;
    }

    /* ---------- 超声波附加项: 遇障碍停车 ---------- */
#if (CAR_USE_ULTRA && OBSTACLE_ENABLE)
    if (!g_obs_flag && g_ultra_mm > 0 && g_ultra_mm <= OBS_NEAR_MM)
    {
        g_obs_flag = 1;
        HARD_BRAKE();          /* 立即停 */
        return;
    }
    if (g_obs_flag)
    {
        if (g_ultra_mm == 0 || g_ultra_mm > OBS_FAR_MM)
        {
            g_obs_flag = 0;    /* 障碍移开 -> 恢复 */
            track_reset();
        }
        else
        {
            motor_target_set(0, 0);
            return;            /* 障碍还在: 保持停, 且不计时 */
        }
    }
#endif

    /* 计时 (被障碍挡住时不计时: 两个 return 都在累加之前, 而 s_last_tick 已更新) */
    g_run_ms += dtick * CTRL_TICK_MS;

    /* 里程 */
    g_dist_cnt = Task_DistCnt();

    /* 压黑路数 */
    g_gray_hits = 0;
    for (i = 0; i < GRAY_CHANNELS; i++)
        if (g_sensor_data[i]) g_gray_hits++;

    /* ---------- 模式分支 ---------- */
    switch (g_mode)
    {
        case MODE_TEST:                     /* 开环: PID 已在 pid_control 里被旁路 */
            break;

        case MODE_TRACK:                    /* 一直循迹, 不停 */
            track();
            break;

        case MODE_AB:                       /* A->B: 走够里程停车 */
            if (g_dist_cnt >= g_ab_cnt) { Task_StopFinish(); break; }
            track();
            break;

        case MODE_LAP:                      /* A->A: 靠停止线停车 */
            if (!s_armed)
            {
                /* 起步时车就压在A点的启停线上, 必须先离开线 */
                if (g_gray_hits < STOP_HITS_MIN)
                {
                    if (++s_leave_cnt >= 3) s_armed = 1;
                }
                else s_leave_cnt = 0;
            }
            else
            {
                /* 判据照搬天猛星实测:
                   >=4 路压黑 -> 一帧就认定, 持续 STOP_HOLD_MS 后停
                   >=3 路压黑 -> 连续 STOP_HITS_CAND_FRAMES 帧也认定 */
                if (g_gray_hits >= STOP_HITS_MIN)
                {
                    s_cand_cnt = 0;
                    s_black_ms += 10;
                    if (s_black_ms >= STOP_HOLD_MS && g_dist_cnt >= g_lap_min_cnt)
                    {
                        Task_StopFinish();
                        break;
                    }
                }
                else if (g_gray_hits >= STOP_HITS_CAND)
                {
                    s_black_ms += 10;
                    if (++s_cand_cnt >= STOP_HITS_CAND_FRAMES &&
                        s_black_ms >= STOP_HOLD_MS && g_dist_cnt >= g_lap_min_cnt)
                    {
                        Task_StopFinish();
                        break;
                    }
                }
                else { s_black_ms = 0; s_cand_cnt = 0; }
            }
            /* 里程兜底: 跑够了还没看到线也停 */
            if (g_dist_cnt >= g_lap_max_cnt) { Task_StopFinish(); break; }
            track();
            break;

        default:
            motor_target_set(0, 0);
            break;
    }
}
