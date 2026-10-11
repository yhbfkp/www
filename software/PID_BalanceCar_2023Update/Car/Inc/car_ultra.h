/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_ultra.h
  * @brief      HC-SR04 超声波测距(可选模块)接口
  * @note       TRIG = PB1 (输出)
  *             ECHO = PB0 = TIM3_CH3 输入捕获
  *             注意: 网表里超声波座 U23 没有走线, 要自己飞线到 PB0/PB1,
  *             默认 CAR_USE_ULTRA=0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    返回值单位 mm, 0 = 没测到(超时)或模块未使能(CAR_USE_ULTRA=0)。
    上一次成功测到的值一直留在 g_ultra_mm 里, 可以当"最后已知距离"看。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#ifndef CAR_ULTRA_H
#define CAR_ULTRA_H

#include "sys.h"
#include "struct_typedef.h"

/**
  * @brief          init ultrasonic trigger pin and last distance
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          超声波初始化: TRIG(PB1) 拉低, 上次距离清零
  * @param[in]      none
  * @retval         none
  */
extern void     Ultra_Init(void);

/**
  * @brief          send one trigger pulse and return the distance in mm
  * @param[in]      none
  * @retval         distance in mm, 0 = no valid capture
  */
/**
  * @brief          发一次触发脉冲并返回距离(mm)
  * @param[in]      none
  * @retval         距离(mm), 0=本次没测到
  */
extern uint16_t Ultra_GetMM(void);

extern volatile uint16_t g_ultra_mm;

#endif
