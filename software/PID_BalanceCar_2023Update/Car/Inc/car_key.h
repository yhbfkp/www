/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_key.h
  * @brief      板载按键(SW2 / SW5)扫描接口
  * @note       板上只焊了 SW2 / SW5 两个:
  *               KEY1 = PA15 (SW2) : 切换模式
  *               KEY2 = PA12 (SW5) : 启动 / 停止
  *             内部上拉, 按下 = 低电平
  *             用法: 主循环里调 Key_Scan(), 返回本次按下的键
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    返回值是"本次新按下"的键(消抖 + 单次触发), 无按键返回 KEY_NONE(0);
    两个键同拍都满足时返回 KEY_RUN(判定顺序: 先 KEY1 后 KEY2)。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#ifndef CAR_KEY_H
#define CAR_KEY_H

#include "sys.h"
#include "struct_typedef.h"

#define KEY_NONE    0
#define KEY_MODE    1   //KEY1 短按: 切模式
#define KEY_RUN     2   //KEY2 短按: 启停

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
extern void    Key_Init(void);

/**
  * @brief          debounce scan, 10ms period, return the key pressed this time
  * @param[in]      none
  * @retval         KEY_xxx, 0 when no key
  */
/**
  * @brief          消抖扫描, 10ms 调一次
  * @param[in]      none
  * @retval         本次按下的 KEY_xxx, 无按键返回 0
  */
extern uint8_t Key_Scan(void);

extern volatile uint8_t g_key1_now;    //实时电平, 1=按下
extern volatile uint8_t g_key2_now;

#endif
