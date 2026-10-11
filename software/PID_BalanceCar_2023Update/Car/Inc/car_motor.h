/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_motor.h
  * @brief      TB6612 双路电机驱动对外接口, 含 duty 换算与短接制动
  * @note       引脚(来自网表, TB6612 模块 U3 标准 16 脚排布):
  *               A路: PWMA=PA11 = TIM1_CH4 -> TIM1->CCR4 ; AIN1=PB13  AIN2=PB12 -> CN1
  *               B路: PWMB=PA8  = TIM1_CH1 -> TIM1->CCR1 ; BIN1=PB14  BIN2=PB15 -> CN2
  *               STBY 硬件接 +5V, 软件不用管
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    [!] main.h / .ioc 里的 GPIO 标签(PWMA=PA8, AIN1=PB14 ...)和这里的 A/B 命名是反的,
        CubeMX 标签不能当作电机信号的依据。以本文件的宏为唯一权威 —— 它是按网表 +
        TB6612 标准 16 脚排布推出来的, 而且 (PWM, IN1, IN2) 三根必然同属一个驱动通道。

    每个 6P 座(CN1/CN2)是"电机两根动力线 + 编码器四根线"捆在同一根排线上, 所以只要
    整根线插对一个座, 驱动和反馈就自动配对, 通道叫 A 还是 B 不影响功能。
    左右装反了改 car_config.h 的 MOTOR_L_IS_A 即可。

    接口沿用 car-main v1.3:
      motorA_duty(duty) / motorB_duty(duty)   duty = 0 .. MAX_DUTY(50000)
      方向由 motorA_dir / motorB_dir 决定 (0=前进 1=后退), motor_target_set() 里设置
      内部自动换算 CCR = duty * PWM_MAX / MAX_DUTY

    停止语义(TB6612 真值表, 两者不一样, 别混):
      duty==0        -> IN1=IN2=0 -> Stop, 输出关断 = 轮子自由滑行
      Motor_Brake()  -> IN1=IN2=1 -> Short brake, 绕组被短路 = 真刹车
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#ifndef CAR_MOTOR_H
#define CAR_MOTOR_H

#include "sys.h"
#include "car_config.h"

/* 调试观察: 实际写进去的 duty(0..50000)。
   必须是 32 位 —— 50000 > INT16_MAX(32767), 用 int16_t 会在 Watch 里显示成负数
   (50000 -> -15536), 看着就像电机链路在乱跳。 */
extern volatile int32_t g_motor_l_duty;
extern volatile int32_t g_motor_r_duty;

/* 方向: 0=正转 1=反转 (由 motor_target_set() 按目标速度正负设置) */
extern int motorA_dir;
extern int motorB_dir;

/**
  * @brief          init motor output, brake on power-up
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          电机初始化: 方向脚拉低, PWM 清零, 上电即短接制动
  * @param[in]      none
  * @retval         none
  */
extern void Motor_Init(void);

/**
  * @brief          stop motor by short brake
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          停车: 短接制动(真刹车)
  * @param[in]      none
  * @retval         none
  */
extern void Motor_Stop(void);

/**
  * @brief          short brake, held until any duty > 0
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          短接制动(IN1=IN2=1)。会一直保持, 直到某一路出现 duty>0
  * @param[in]      none
  * @retval         none
  */
extern void Motor_Brake(void);

/**
  * @brief          set A channel duty, A channel = left wheel when MOTOR_L_IS_A
  * @param[in]      duty: 0 .. MAX_DUTY(50000)
  * @retval         none
  */
/**
  * @brief          设置 A 路占空比(A路 = 左轮 还是 右轮 由 MOTOR_L_IS_A 决定)
  * @param[in]      duty: 0 .. MAX_DUTY(50000), 0 = 输出关断(滑行)
  * @retval         none
  */
extern void motorA_duty(int duty);

/**
  * @brief          set B channel duty, B channel = right wheel when MOTOR_L_IS_A
  * @param[in]      duty: 0 .. MAX_DUTY(50000)
  * @retval         none
  */
/**
  * @brief          设置 B 路占空比(B路 = 右轮 还是 左轮 由 MOTOR_L_IS_A 决定)
  * @param[in]      duty: 0 .. MAX_DUTY(50000), 0 = 输出关断(滑行)
  * @retval         none
  */
extern void motorB_duty(int duty);

#endif
