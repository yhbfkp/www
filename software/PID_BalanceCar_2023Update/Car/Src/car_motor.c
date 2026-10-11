/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_motor.c
  * @brief      TB6612 双路电机驱动实现: duty->CCR 换算, 方向控制, 短接制动
  * @note       硬件映射集中在本文件顶部的宏里, 换路/换极性只改宏, 不用动逻辑
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    网表核对(U3 = TB6612 模块, 标准 16 脚排布):
      A路 = U3.1 PWMA = PA11 = TIM1_CH4 -> CCR4 ; U3.3 AIN1 = PB13 ; U3.2 AIN2 = PB12
            输出 AO1/AO2 -> CN1
      B路 = U3.7 PWMB = PA8  = TIM1_CH1 -> CCR1 ; U3.5 BIN1 = PB14 ; U3.6 BIN2 = PB15
            输出 BO1/BO2 -> CN2
      (PWM / IN1 / IN2 三根必须同属一个驱动通道, 已核对过, 不要只改其中一根)
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#include "car_motor.h"

#define MA_PWM      TIM1->CCR4      //PWMA = PA11
#define MA_IN1      PBout(13)
#define MA_IN2      PBout(12)

#define MB_PWM      TIM1->CCR1      //PWMB = PA8
#define MB_IN1      PBout(14)
#define MB_IN2      PBout(15)

int motorA_dir = 0;
int motorB_dir = 0;

volatile int32_t g_motor_l_duty = 0;
volatile int32_t g_motor_r_duty = 0;

/* 短接制动标志: Motor_Brake() 置位, 任一路出现 duty>0 时自动解除。
   必须用标志"保持"住, 否则 pid_control() 每 100ms 写一次 duty=0 就把刹车解掉、
   又变回滑行 —— 这正是原来 HARD_BRAKE() 名义上急停、实际不刹车的原因。 */
static volatile uint8_t s_brake = 0;

/**
  * @brief          output A channel: clamp duty, convert to CCR, set IN1/IN2
  * @param[in]      duty: 0 .. MAX_DUTY
  * @param[in]      dir: 0 = forward(IN1=1,IN2=0), other = backward
  * @retval         none
  */
/**
  * @brief          A 路输出: duty 限幅 -> 换算 CCR -> 写方向脚
  * @param[in]      duty: 0 .. MAX_DUTY
  * @param[in]      dir: 0=前进(IN1=1,IN2=0), 非0=后退
  * @retval         none
  */
static void OutA(int duty, int dir)
{
    uint32_t ccr;

    if (s_brake)
    {
        MA_IN1 = 1;
        MA_IN2 = 1;
        MA_PWM = 0;
        return;
    }

    if (duty < 0)
    {
        duty = 0;
    }
    if (duty > MAX_DUTY)
    {
        duty = MAX_DUTY;
    }

    ccr = (uint32_t)duty * (uint32_t)PWM_MAX / (uint32_t)MAX_DUTY;

    if (duty == 0)                                                  //停: 输出关断(滑行)
    {
        MA_IN1 = 0;
        MA_IN2 = 0;
        MA_PWM = 0;
    }
    else if (dir)
    {
        MA_IN1 = 0;
        MA_IN2 = 1;
        MA_PWM = (uint16_t)ccr;
    }
    else
    {
        MA_IN1 = 1;
        MA_IN2 = 0;
        MA_PWM = (uint16_t)ccr;
    }
}

/**
  * @brief          output B channel: clamp duty, convert to CCR, set IN1/IN2
  * @param[in]      duty: 0 .. MAX_DUTY
  * @param[in]      dir: 0 = forward(IN1=1,IN2=0), other = backward
  * @retval         none
  */
/**
  * @brief          B 路输出: duty 限幅 -> 换算 CCR -> 写方向脚
  * @param[in]      duty: 0 .. MAX_DUTY
  * @param[in]      dir: 0=前进(IN1=1,IN2=0), 非0=后退
  * @retval         none
  */
static void OutB(int duty, int dir)
{
    uint32_t ccr;

    if (s_brake)
    {
        MB_IN1 = 1;
        MB_IN2 = 1;
        MB_PWM = 0;
        return;
    }

    if (duty < 0)
    {
        duty = 0;
    }
    if (duty > MAX_DUTY)
    {
        duty = MAX_DUTY;
    }

    ccr = (uint32_t)duty * (uint32_t)PWM_MAX / (uint32_t)MAX_DUTY;

    if (duty == 0)                                                  //停: 输出关断(滑行)
    {
        MB_IN1 = 0;
        MB_IN2 = 0;
        MB_PWM = 0;
    }
    else if (dir)
    {
        MB_IN1 = 0;
        MB_IN2 = 1;
        MB_PWM = (uint16_t)ccr;
    }
    else
    {
        MB_IN1 = 1;
        MB_IN2 = 0;
        MB_PWM = (uint16_t)ccr;
    }
}

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
void Motor_Init(void)
{
    motorA_dir = 0;
    motorB_dir = 0;
    g_motor_l_duty = 0;
    g_motor_r_duty = 0;

    /* 上电就短接制动: 一是不让车自己溜, 二是后面 MPU6050 采陀螺零偏要求车静止 */
    Motor_Brake();
}

/**
  * @brief          short brake, held until any duty > 0
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          真刹车: IN1=IN2=1 把绕组短路。会一直保持, 直到某一路出现 duty>0
  * @param[in]      none
  * @retval         none
  */
void Motor_Brake(void)
{
    s_brake = 1;
    MA_IN1 = 1;
    MA_IN2 = 1;
    MA_PWM = 0;
    MB_IN1 = 1;
    MB_IN2 = 1;
    MB_PWM = 0;
}

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
void Motor_Stop(void)
{
    Motor_Brake();
}

/**
  * @brief          set A channel duty
  * @param[in]      duty: 0 .. MAX_DUTY(50000)
  * @retval         none
  */
/**
  * @brief          设置 A 路占空比, 同时按 MOTOR_L_IS_A / MOTOR_x_DIR 决定记录到哪个轮
  * @param[in]      duty: 0 .. MAX_DUTY(50000)
  * @retval         none
  */
void motorA_duty(int duty)
{
    if (duty > 0)
    {
        s_brake = 0;                //有输出就解除急停刹车
    }

#if MOTOR_L_IS_A
    g_motor_l_duty = duty;
    OutA(duty, motorA_dir ^ ((MOTOR_L_DIR < 0) ? 1 : 0));
#else
    g_motor_r_duty = duty;
    OutA(duty, motorA_dir ^ ((MOTOR_R_DIR < 0) ? 1 : 0));
#endif
}

/**
  * @brief          set B channel duty
  * @param[in]      duty: 0 .. MAX_DUTY(50000)
  * @retval         none
  */
/**
  * @brief          设置 B 路占空比, 同时按 MOTOR_L_IS_A / MOTOR_x_DIR 决定记录到哪个轮
  * @param[in]      duty: 0 .. MAX_DUTY(50000)
  * @retval         none
  */
void motorB_duty(int duty)
{
    if (duty > 0)
    {
        s_brake = 0;
    }

#if MOTOR_L_IS_A
    g_motor_r_duty = duty;
    OutB(duty, motorB_dir ^ ((MOTOR_R_DIR < 0) ? 1 : 0));
#else
    g_motor_l_duty = duty;
    OutB(duty, motorB_dir ^ ((MOTOR_L_DIR < 0) ? 1 : 0));
#endif
}
