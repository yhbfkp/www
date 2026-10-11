/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_encoder.c
  * @brief      编码器测速实现 (TIM2 / TIM4 编码器模式 TI12 = 4 倍频)
  * @note       增量式读取: 每次读走并清零, 用 int16 差值自动吃掉 16 位回绕
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

#include "car_encoder.h"
#include "car_config.h"
#include "tim.h"

/* 编码器计数极性: 初值来自 car_config.h 的 ENC_L_DIR / ENC_R_DIR。
   这两个是变量而不是宏 —— 轮子正转读出来是负数时, 直接在 debug 的 Watch 里
   把 g_enc_dir_l 改成 -1 就能立刻翻转, 不用改配置重新编译。 */
volatile int16_t g_enc_dir_l = ENC_L_DIR;
volatile int16_t g_enc_dir_r = ENC_R_DIR;

volatile int16_t g_enc_l_cnt   = 0;
volatile int16_t g_enc_r_cnt   = 0;
volatile fp32    g_enc_l_rpm   = 0;
volatile fp32    g_enc_r_rpm   = 0;
volatile int32_t g_enc_l_total = 0;
volatile int32_t g_enc_r_total = 0;

static uint16_t s_last_l = 0;
static uint16_t s_last_r = 0;

/**
  * @brief          read count delta of one timer, 16bit wrap handled
  * @param[in]      h: encoder timer handle
  * @param[in,out]  last: last counter value of this timer
  * @retval         count delta
  */
/**
  * @brief          读一个编码器定时器的增量: 用 int16 差值自动吃掉 16 位回绕
  * @param[in]      h: 编码器定时器句柄
  * @param[in,out]  last: 上一次的计数器值
  * @retval         增量计数
  */
static int16_t Encoder_Delta(TIM_HandleTypeDef *h, uint16_t *last)
{
    uint16_t now  = (uint16_t)__HAL_TIM_GET_COUNTER(h);
    int16_t  diff = (int16_t)(now - *last);

    *last = now;
    return diff;
}

/**
  * @brief          init encoder counter baseline
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          编码器初始化: 记录当前计数器基准, 清累计量
  * @param[in]      none
  * @retval         none
  */
void Encoder_Init(void)
{
    s_last_l = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    s_last_r = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    g_enc_l_cnt = 0;
    g_enc_r_cnt = 0;
    g_enc_l_rpm = 0;
    g_enc_r_rpm = 0;
    g_enc_l_total = 0;
    g_enc_r_total = 0;
}

/**
  * @brief          get left wheel count since last call
  * @param[in]      none
  * @retval         left wheel count delta(带极性)
  */
/**
  * @brief          取左轮"自上次调用以来的增量计数"(读走即清零)
  * @param[in]      none
  * @retval         左轮增量计数(已按 g_enc_dir_l 定极性)
  */
int16_t Encoder_GetL(void)
{
#if CAR_USE_ENCODER
    g_enc_l_cnt = Encoder_Delta(&htim2, &s_last_l) * g_enc_dir_l;
#else
    g_enc_l_cnt = 0;
#endif
    g_enc_l_total += g_enc_l_cnt;
    return g_enc_l_cnt;
}

/**
  * @brief          get right wheel count since last call
  * @param[in]      none
  * @retval         right wheel count delta(带极性)
  */
/**
  * @brief          取右轮"自上次调用以来的增量计数"(读走即清零)
  * @param[in]      none
  * @retval         右轮增量计数(已按 g_enc_dir_r 定极性)
  */
int16_t Encoder_GetR(void)
{
#if CAR_USE_ENCODER
    g_enc_r_cnt = Encoder_Delta(&htim4, &s_last_r) * g_enc_dir_r;
#else
    g_enc_r_cnt = 0;
#endif
    g_enc_r_total += g_enc_r_cnt;
    return g_enc_r_cnt;
}

/**
  * @brief          count to wheel revolution
  * @param[in]      cnt: raw 4x count
  * @retval         revolution number
  */
/**
  * @brief          计数 -> 圈数(估算里程用, ENC_CPR 在 car_config.h)
  * @param[in]      cnt: 原始 4 倍频计数
  * @retval         圈数
  */
fp32 Encoder_CountToRev(int32_t cnt)
{
    return (fp32)cnt / ENC_CPR;
}
