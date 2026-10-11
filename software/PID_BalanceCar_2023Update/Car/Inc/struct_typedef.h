/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       struct_typedef.h
  * @brief      统一类型别名, 沿用 RM/DJI 标准代码习惯
  * @note       只补 fp32/fp64/bool_t: 标准整型(CMSIS 的 stdint.h)已经有了,
  *             再 typedef 一遍在 ARMCC 下会报重定义, 所以不重复抄。
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

#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include <stdint.h>

typedef float   fp32;
typedef double  fp64;
typedef uint8_t bool_t;

#endif
