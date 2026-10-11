/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_encoder.h
  * @brief      编码器测速接口 (TIM2 / TIM4 编码器模式 TI12 = 4 倍频)
  * @note       左轮: TIM2  PA0(A相) / PA1(B相)  接 CN1
  *             右轮: TIM4  PB6(A相) / PB7(B相)  接 CN2
  *             Encoder_GetL()/GetR() 返回"自上次调用以来的增量计数"(读走即清零),
  *             已经处理了 16 位计数器回绕。
  *             速度环(car_pid.c)在 100ms 周期里调它, 并把读数 /ENC_DIV(4) 换算成
  *             "等效单相计数", 与 car-main v1.3 的速度量级对齐
  *             (目标速度 45 = 每 100ms 45 个计数)
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

#ifndef CAR_ENCODER_H
#define CAR_ENCODER_H

#include "sys.h"
#include "struct_typedef.h"

/* 计数极性(运行时可调, 读出来是负数就改成 -1); 初值 = ENC_L_DIR / ENC_R_DIR */
extern volatile int16_t g_enc_dir_l;
extern volatile int16_t g_enc_dir_r;

/* 调试用: 观察原始量 */
extern volatile int16_t g_enc_l_cnt;        //本次 10ms 左轮增量
extern volatile int16_t g_enc_r_cnt;
extern volatile fp32    g_enc_l_rpm;
extern volatile fp32    g_enc_r_rpm;
extern volatile int32_t g_enc_l_total;      //累计计数, 可当里程用
extern volatile int32_t g_enc_r_total;

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
extern void Encoder_Init(void);

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
extern int16_t Encoder_GetL(void);

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
extern int16_t Encoder_GetR(void);

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
extern fp32 Encoder_CountToRev(int32_t cnt);

#endif
