/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_mpu.h
  * @brief      MPU6050 (GY-521) 简化版接口: 软件 I2C PA6(SCL) / PA7(SDA)
  * @note       走 CSL/Src/IOI2C.c, 与天猛星那版完全相同的初始化:
  *               0x6B = 0x00 唤醒
  *               0x1B = 0x18 陀螺仪 ±2000 °/s  (灵敏度 16.4 LSB/(°/s))
  *               0x1C = 0x10 加速度 ±8 g       (灵敏度 4096 LSB/g)
  *             注意: PA6/PA7 不耐 5V, GY-521 的 VCC 要飞线到 H9.20(3.3V), 别插座子的 5V
  *             本文件未被加入 Keil 工程, 保留作参考实现
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    MPU_Init() 不做 IIC_Init(): PA6/PA7 已在 gpio.c 里初始化成推挽输出, 而且
    ioi2c.h 里声明的 IIC_Init() 在 IOI2C.c 中没有实现, 调了会链接失败。
    调试观察量都在 g_mpu_* 里, 停车时看 g_mpu_yaw 可以估零漂。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#ifndef CAR_MPU_H
#define CAR_MPU_H

#include "sys.h"
#include "struct_typedef.h"

typedef struct
{
    fp32 ax, ay, az;   //单位 g
    fp32 gx, gy, gz;   //单位 °/s
} MPU_Data_t;

/* 调试用 */
extern volatile uint8_t g_mpu_id;                 //WHO_AM_I, 正常 0x68
extern volatile uint8_t g_mpu_ok_simple;
extern volatile fp32    g_mpu_ax, g_mpu_ay, g_mpu_az;
extern volatile fp32    g_mpu_gx, g_mpu_gy, g_mpu_gz;
extern volatile fp32    g_mpu_yaw;                //gz 积分出来的角度, 仅参考
extern volatile int16_t g_mpu_raw[6];             //原始 ADC

/**
  * @brief          mpu6050 init
  * @param[in]      none
  * @retval         1: who_am_i is 0x68, 0: id check failed
  */
/**
  * @brief          MPU6050 初始化: 读 WHO_AM_I + 写唤醒/量程/分频/DLPF 配置
  * @param[in]      none
  * @retval         1 = WHO_AM_I 读到 0x68, 0 = ID 不对
  */
extern uint8_t MPU_Init(void);

/**
  * @brief          read accel and gyro, integrate yaw
  * @param[out]     d: converted data point (g / °/s)
  * @retval         1: read success, 0: i2c error
  */
/**
  * @brief          读加计/陀螺并换算成物理量, 顺带积分 yaw
  * @param[out]     d: 换算结果指针 (加计 g, 陀螺 °/s)
  * @retval         1 = 读成功, 0 = I2C 失败
  */
extern uint8_t MPU_Read(MPU_Data_t *d);

#endif
