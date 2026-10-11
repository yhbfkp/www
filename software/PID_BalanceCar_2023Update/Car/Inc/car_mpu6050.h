/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_mpu6050.h
  * @brief      MPU6050 (GY-521) 姿态/偏航角接口: 软件 I2C + 陀螺零偏 + yaw 积分
  * @note       移植自 car-main v1.3 ml_mpu6050.c
  *             底层换成工程自带的软件 I2C (CSL/Src/IOI2C.c, PA6=SCL PA7=SDA)
  *             配置与 v1.3 完全一致: 100Hz采样(DLPF 92Hz), 陀螺 ±2000dps(16.4 LSB/(°/s)), 加计 ±2g
  *             Yaw 用纯陀螺 Z 积分 (无磁力计), 上电静止 2 秒采 200 帧求零偏
  *             注意: PA6/PA7 不耐 5V, GY-521 的 VCC 要飞线到 H9.20(3.3V), 别插座子的 5V
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    [初始化诊断] 出问题时按这个顺序看 (都是运行时可 watch 的全局量):
      g_mpu_init_step  0=没跑  1=没ACK(查接线/供电)  2=WHO_AM_I 不对(AD0 必须接地)  255=成功
      g_mpu_init_id    实际读到的 WHO_AM_I, 正常就是 0x68
      g_mpu_init_try   已经试到第几次(MPU_INIT_RETRY 见 car_config.h, 每轮约 220ms)
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#ifndef CAR_MPU6050_H
#define CAR_MPU6050_H

#include "sys.h"
#include "car_config.h"
#include "struct_typedef.h"

/* 寄存器 */
#define MPU_SMPLRT_DIV      0x19
#define MPU_CONFIG          0x1A
#define MPU_GYRO_CONFIG     0x1B
#define MPU_ACCEL_CONFIG    0x1C
#define MPU_ACCEL_XOUT_H    0x3B
#define MPU_GYRO_XOUT_H     0x43
#define MPU_PWR_MGMT_1      0x6B
#define MPU_PWR_MGMT_2      0x6C
#define MPU_INT_ENABLE      0x38
#define MPU_WHO_AM_I        0x75

/* 原始 ADC */
extern int16_t ax, ay, az, gx, gy, gz;
extern volatile uint16_t g_mpu_cal_frames;  //零偏标定实际采到的有效帧数(0=总线有问题)
extern fp32    g_yaw;                           //累计偏航角(度), 上电归零
extern fp32    gyro_bias_z;                     //陀螺Z零偏
extern volatile uint8_t  g_mpu_i2c_err;         //0=OK 1=ACK失败 2=ID不匹配
extern volatile uint8_t  g_mpu_ok;
extern volatile uint32_t g_mpu_ok_cnt, g_mpu_err_cnt;
/* 初始化诊断: step 0=没跑 1=没ACK 2=ID不对 255=成功;  id=实际 WHO_AM_I;  try=尝试次数 */
extern volatile uint8_t  g_mpu_init_step, g_mpu_init_id, g_mpu_init_try;

/**
  * @brief          mpu6050 init: bus clear, soft reset, id check, config
  * @param[in]      none
  * @retval         0: success, 1: who_am_i wrong or no ack (wiring / power)
  */
/**
  * @brief          MPU6050 初始化: 总线恢复 -> 软复位 -> 读 WHO_AM_I -> 写配置
  * @param[in]      none
  * @retval         0 = 成功, 1 = WHO_AM_I 不对(接线/供电)
  */
extern uint8_t MPU6050_Init(void);

/**
  * @brief          burst read 14 bytes: accel + temp + gyro
  * @param[in]      none
  * @retval         0: success, not 0: i2c error (drop this frame)
  */
/**
  * @brief          一次 burst 读 14 字节(加计6 + 温度2 + 陀螺6)
  * @param[in]      none
  * @retval         0 = 成功, 非0 = I2C通信失败(本次数据丢弃)
  */
extern uint8_t MPU6050_GetData(void);

/**
  * @brief          sample gyro z bias while the car keeps still, about 2.2s
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          上电静止采陀螺Z零偏(车必须绝对静止, 约2.2秒)
  * @param[in]      none
  * @retval         none
  */
extern void MPU6050_CalGyroBias(void);

/**
  * @brief          10ms task: burst read + yaw integral
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          10ms 调一次: burst读 + yaw积分
  * @param[in]      none
  * @retval         none
  */
extern void MPU_Task10ms(void);

#endif
