/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_track.c
  * @brief      8 路灰度巡线实现: 加权差比和 -> 丢线惯性保持 -> 转向PD -> 差速下发
  * @note       巡线算法 — 移植自 car-main v1.3 (2024电赛H题), 算法一字未改:
  *               · 8路灰度 -> 加权差比和/压线数(归一) -> error (约 ±6)
  *               · 丢线 FIND: 保持上一帧 error 走惯性 (不是阶跃打死舵)
  *               · 转向PD -> 差速; 弯道减速 curve_factor = 1 - |err|/12*0.7
  *             黑线 = 1, 白 = 0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    三层结构:
      第1层 加权差比和 -> error
      第2层 丢线处理: 全白(cnt==0) 且从未见过线 -> 原地找线 "FIND";
                        见过线则保持上一帧 error 走惯性
      第3层 转向PD + 弯道减速 -> L/R -> motor_target_set()
    dump: g_track_err / g_track_state 供串口与 OLED 观察
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#include "car_track.h"
#include "car_pid.h"
#include <string.h>                 //strcpy: 原来是靠 sys.h 间接引入, 不依赖它

#ifndef M_PI
#define M_PI 3.14159265f
#endif

uint16_t g_sensor_data[GRAY_CHANNELS];
char     g_track_state[8] = "INIT";

int16_t g_speed_fast = SPEED_FAST;
int16_t g_speed_slow = SPEED_SLOW;
int16_t g_speed_min  = SPEED_MIN;

fp32    g_track_kp   = TRACK_KP;
fp32    g_track_kd   = TRACK_KD;

fp32    g_track_err  = 0.0f;
int16_t g_track_L    = 0;
int16_t g_track_R    = 0;

#define D1 digtal(1)
#define D2 digtal(2)
#define D3 digtal(3)
#define D4 digtal(4)
#define D5 digtal(5)
#define D6 digtal(6)
#define D7 digtal(7)
#define D8 digtal(8)

static uint8_t ever_seen_line   = 0;
static fp32    last_valid_error = 0.0f;
static fp32    prev_error       = 0.0f;

/**
  * @brief          set fast / slow / min speed used by tracking
  * @param[in]      fast: straight line speed
  * @param[in]      slow: line-lost search speed
  * @param[in]      min: inner wheel minimum speed
  * @retval         none
  */
/**
  * @brief          设置循迹用的 直道/丢线搜索/内轮最低 速度
  * @param[in]      fast: 直道速度
  * @param[in]      slow: 丢线搜索速度
  * @param[in]      min: 内轮最低速度
  * @retval         none
  */
void track_set_speed(int16_t fast, int16_t slow, int16_t min)
{
    g_speed_fast = fast;
    g_speed_slow = slow;
    g_speed_min  = min;
}

/**
  * @brief          reset tracking internal state
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          复位内部状态(起步/切模式时调, 防D项残留跳变)
  * @param[in]      none
  * @retval         none
  */
static int16_t s_last_L = 0, s_last_R = 0;   /* [防跳变] L/R 输出斜率限幅记忆 */
static uint16_t s_start_hold = 0;              /* [起步直行] 启动后直行帧计数 */

void track_reset(void)
{
    ever_seen_line   = 0;
    last_valid_error = 0.0f;
    prev_error       = 0.0f;
    s_last_L         = 0;      /* [防跳变] 清斜率限幅记忆, 重新从 0 起坡 */
    s_last_R         = 0;
    s_start_hold     = 0;      /* [起步直行] 重新计帧 */
}

/**
  * @brief          run one tracking step: gray sensors -> error -> steering PD -> wheel targets
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          算一次转向并下发给速度环( motor_target_set )
  * @param[in]      none
  * @retval         none
  */
void track(void)
{
    uint8_t d1 = D1, d2 = D2, d3 = D3, d4 = D4;
    uint8_t d5 = D5, d6 = D6, d7 = D7, d8 = D8;

    int16_t L, R;

    /* ---- 第1层: 加权差比和 -> error (不除cnt) ---- */
    int16_t sum_L = (int16_t)d1 * W1 + d2 * W2 + d3 * W3 + d4 * W4;
    int16_t sum_R = (int16_t)d5 * W5 + d6 * W6 + d7 * W7 + d8 * W8;
    uint8_t cnt   = d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8;

    fp32    raw_error;
    uint8_t branch;

    if (cnt == 0)                       //全白 -> 丢线
    {
        branch = 7;
        raw_error = 0.0f;
    }
    else if (cnt == 8)                  //全黑 -> 横穿线/十字
    {
        branch = 8;
        raw_error = 0.0f;
    }
    else
    {
        raw_error = (fp32)(sum_L - sum_R) / (fp32)cnt;   /* [归一] 除以压线数, 误差量级稳定约 ±6 */

        fp32 abs_e = (raw_error > 0.0f) ? raw_error : -raw_error;
        if      (abs_e < 0.8f) branch = 0;
        else if (abs_e < 2.0f) branch = (raw_error > 0.0f) ? 1 : 4;
        else if (abs_e < 3.8f) branch = (raw_error > 0.0f) ? 2 : 5;
        else                   branch = (raw_error > 0.0f) ? 3 : 6;
    }

    /* ---- 第2层: 丢线保持上帧 error 走惯性 ---- */
    fp32 error = raw_error;
    if (branch != 7) ever_seen_line = 1;

    /* [起步直行] 启动后前 TRACK_START_HOLD 帧(10ms/帧)恒速直行不纠偏:
       刚起步压着 A 点启停线, 灰度扫线过渡期读数不对称, 一上来就纠偏会起步甩尾。 */
    if (s_start_hold < TRACK_START_HOLD)
    {
        s_start_hold++;
        L = g_speed_fast;
        R = g_speed_fast;
        if (L - s_last_L >  TRACK_OUT_SLEW) L = (int16_t)(s_last_L + TRACK_OUT_SLEW);
        if (s_last_L - L >  TRACK_OUT_SLEW) L = (int16_t)(s_last_L - TRACK_OUT_SLEW);
        if (R - s_last_R >  TRACK_OUT_SLEW) R = (int16_t)(s_last_R + TRACK_OUT_SLEW);
        if (s_last_R - R >  TRACK_OUT_SLEW) R = (int16_t)(s_last_R - TRACK_OUT_SLEW);
        s_last_L = L;
        s_last_R = R;
        g_track_L = L; g_track_R = R;
        g_track_err = 0.0f;
        strcpy(g_track_state, "HOLD");
        motor_target_set(L, R);
        return;
    }

    if (branch == 7)
    {
        if (!ever_seen_line)
        {
            L =  g_speed_min;        /* [前进式] 不反转: 小差速往左兜着找线 */
            R =  g_speed_slow;
            /* [防跳变] 目标轮速斜率限幅: 转向算出的 L/R 是阶跃, 速度环(100ms 一拍)追
       不上阶跃就表现为一顿一顿。这里每帧最多变 TRACK_OUT_SLEW, 变成斜坡。 */
    if (L - s_last_L >  TRACK_OUT_SLEW) L = (int16_t)(s_last_L + TRACK_OUT_SLEW);
    if (s_last_L - L >  TRACK_OUT_SLEW) L = (int16_t)(s_last_L - TRACK_OUT_SLEW);
    if (R - s_last_R >  TRACK_OUT_SLEW) R = (int16_t)(s_last_R + TRACK_OUT_SLEW);
    if (s_last_R - R >  TRACK_OUT_SLEW) R = (int16_t)(s_last_R - TRACK_OUT_SLEW);
    s_last_L = L;
    s_last_R = R;

    g_track_L = L; g_track_R = R;
            g_track_err = 0.0f;
            strcpy(g_track_state, "FIND");
            motor_target_set(L, R);
            return;
        }
        error = last_valid_error;
    }
    else
    {
        last_valid_error = error;
    }

    /* [防跳变] error 斜率限幅: 灰度探头进出黑线时 error 按权重整档跳(如 0->4),
       进 PD 后 P 项突变、D 项又正比跳变量(双重放大), 转向输出跟着跳。
       每帧相对上帧最多变 TRACK_ERR_SLEW —— diff(D 项输入)同时被夹住。 */
    {
        fp32 hi = prev_error + TRACK_ERR_SLEW;
        fp32 lo = prev_error - TRACK_ERR_SLEW;
        if (error > hi) error = hi;
        if (error < lo) error = lo;
    }

    g_track_err = error;

    /* ---- 第3层: 转向PD ---- */
    fp32 diff  = error - prev_error;
    prev_error  = error;

    fp32 steer_out = g_track_kp * error + g_track_kd * diff;
    if (steer_out >  STEER_MAX) steer_out =  STEER_MAX;
    if (steer_out < -STEER_MAX) steer_out = -STEER_MAX;

    /* 弯道减速: |error| 越大 base 越低 (最多减 70%) */
    fp32 abs_err = (error > 0.0f) ? error : -error;
    fp32 curve_factor = 1.0f - abs_err / 6.0f * 0.7f;
    int16_t base = (int16_t)((fp32)g_speed_fast * curve_factor);
    if (base < g_speed_min) base = g_speed_min;

    L = (int16_t)((fp32)base - steer_out);
    R = (int16_t)((fp32)base + steer_out);

#if ANGLE_ENABLE
    {   //可选: 叠加航向修正 (需要 MPU6050 先验通)
        extern fp32 g_yaw;
        fp32 steer = pid_angle(0.0f, g_yaw);
        L -= (int16_t)steer;
        R += (int16_t)steer;
    }
#endif

    /* 内轮 >= g_speed_min 始终前进(不停转不反转), 外轮不超过直道速度 */
    if (L < g_speed_min) L = g_speed_min;
    if (R < g_speed_min) R = g_speed_min;
    if (L > g_speed_fast) L = g_speed_fast;
    if (R > g_speed_fast) R = g_speed_fast;

    g_track_L = L;
    g_track_R = R;

    switch (branch)
    {
        case 0: strcpy(g_track_state, "GO  "); break;
        case 1: strcpy(g_track_state, "R1  "); break;
        case 2: strcpy(g_track_state, "R2  "); break;
        case 3: strcpy(g_track_state, "R3!!"); break;
        case 4: strcpy(g_track_state, "L1  "); break;
        case 5: strcpy(g_track_state, "L2  "); break;
        case 6: strcpy(g_track_state, "L3!!"); break;
        case 7: strcpy(g_track_state, "FIND"); break;
        case 8: strcpy(g_track_state, "CROSS");break;
        default:strcpy(g_track_state, "----"); break;
    }

    motor_target_set(L, R);
}

/**
  * @brief          read one channel of the 8-way gray sensor
  * @param[in]      channel: channel index, 1~8
  * @retval         0: white / invalid channel, 1: black line
  */
/**
  * @brief          读第 channel 路(1~8)灰度值
  * @param[in]      channel: 通道号, 1~8
  * @retval         0: 白 / 通道号非法, 1: 压黑线
  */
unsigned char digtal(unsigned char channel)
{
    if (channel == 0 || channel > GRAY_CHANNELS) return 0;
    return (unsigned char)g_sensor_data[channel - 1];
}
