#ifndef __CAR_MPU6050_H
#define __CAR_MPU6050_H

#include "sys.h"
#include "car_config.h"

/* ---------------------------------------------------------------------------
 * MPU6050 (GY-521) — 移植自 car-main v1.3 ml_mpu6050.c
 * 底层换成工程自带的软件 I2C (CSL/Src/IOI2C.c, PA6=SCL PA7=SDA)
 *
 * 配置与 v1.3 完全一致: 100Hz采样(DLPF 92Hz), 陀螺 ±2000dps(16.4 LSB/(°/s)), 加计 ±2g
 * Yaw 用纯陀螺 Z 积分 (无磁力计), 上电静止 2 秒采 200 帧求零偏
 * 注意: PA6/PA7 不耐 5V, GY-521 的 VCC 要飞线到 H9.20(3.3V), 别插座子的 5V
 * ------------------------------------------------------------------------- */

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

uint8_t MPU6050_Init(void);          /* 0=成功 1=WHO_AM_I 不对(接线/供电) */
uint8_t MPU6050_GetData(void);       /* 0=成功, 非0=I2C通信失败(本次数据丢弃) */
void    MPU6050_CalGyroBias(void);   /* 上电静止采零偏, 约2.2秒 */
void    MPU_Task10ms(void);          /* 10ms 调一次: burst读 + yaw积分 */

extern int16_t ax, ay, az, gx, gy, gz;          /* 原始 ADC */
extern float   g_yaw;                           /* 累计偏航角(度), 上电归零 */
extern float   gyro_bias_z;                     /* 陀螺Z零偏 */
extern volatile uint8_t  g_mpu_i2c_err;         /* 0=OK 1=ACK失败 2=ID不匹配 */
extern volatile uint8_t  g_mpu_ok;
extern volatile uint32_t g_mpu_ok_cnt, g_mpu_err_cnt;

#endif
