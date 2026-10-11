/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_key.c
  * @brief      板载按键(SW2 / SW5)消抖扫描实现
  * @note       两个键都是上拉输入、按下接地, 主循环 10ms 调一次 Key_Scan()
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    消抖: 连续 KEY_DEBOUNCE 次(10ms 一次 -> 30ms)扫到按下才认, 认过一次就置
    s_fired* 标志, 松手才清零 —— 所以长按只报一次, 不会连发。

    判定顺序: 先判 KEY1(模式), 后判 KEY2(启停)。同一拍两键都满足时 ret 取后者,
    即 KEY2 覆盖 KEY1, 保持原逻辑不动。

    引脚: KEY1 = PA15 (SW2, 启动 A->B 任务), KEY2 = PA12 (SW5, 启动 A->A 任务)。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#include "car_key.h"
#include "car_config.h"
#include "delay.h"

/* PA15 = SW2, PA12 = SW5 (都是上拉输入, 按下接地) */
#define KEY1_READ()   (PAin(15) == 0 ? 1 : 0)
#define KEY2_READ()   (PAin(12) == 0 ? 1 : 0)

volatile uint8_t g_key1_now = 0;
volatile uint8_t g_key2_now = 0;

static uint8_t s_cnt1 = 0, s_cnt2 = 0;
static uint8_t s_fired1 = 0, s_fired2 = 0;

/**
  * @brief          reset key scan state
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          按键初始化: 清计数器、触发标志和实时电平
  * @param[in]      none
  * @retval         none
  */
void Key_Init(void)
{
    s_cnt1 = 0; s_cnt2 = 0;
    s_fired1 = 0; s_fired2 = 0;
    g_key1_now = 0; g_key2_now = 0;
}

/**
  * @brief          debounce scan, 10ms period, return the key pressed this time
  * @param[in]      none
  * @retval         KEY_NONE / KEY_MODE / KEY_RUN
  */
/**
  * @brief          消抖: 连续 KEY_DEBOUNCE 次扫到按下才算, 松开后清标志
  * @param[in]      none
  * @retval         KEY_NONE / KEY_MODE / KEY_RUN
  */
uint8_t Key_Scan(void)
{
    uint8_t ret = KEY_NONE;

#if CAR_USE_KEY
    uint8_t k1 = KEY1_READ();
    uint8_t k2 = KEY2_READ();

    g_key1_now = k1;
    g_key2_now = k2;

    if (k1)
    {
        if (s_cnt1 < 255) s_cnt1++;
        if (s_cnt1 >= KEY_DEBOUNCE && !s_fired1)
        {
            s_fired1 = 1;
            ret = KEY_MODE;
        }
    }
    else
    {
        s_cnt1 = 0;
        s_fired1 = 0;
    }

    if (k2)
    {
        if (s_cnt2 < 255) s_cnt2++;
        if (s_cnt2 >= KEY_DEBOUNCE && !s_fired2)
        {
            s_fired2 = 1;
            ret = KEY_RUN;
        }
    }
    else
    {
        s_cnt2 = 0;
        s_fired2 = 0;
    }
#endif

    return ret;
}
