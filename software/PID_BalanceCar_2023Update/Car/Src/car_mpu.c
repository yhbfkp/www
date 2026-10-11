/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_mpu.c
  * @brief      MPU6050 (GY-521) 简化版驱动: 初始化 + 读 14 字节 + 换算 + yaw 积分
  * @note       走 CSL/Src/IOI2C.c 的 IICreadBytes / IICwriteByte (设备地址已 <<1)
  *             与天猛星那版完全相同的初始化(唤醒 0x6B=0x00, 陀螺 ±2000°/s 0x1B=0x18,
  *             加计 ±8g 0x1C=0x10, 采样分频 0x19=0x00, DLPF 42Hz 0x1A=0x03)
  *             注意: PA6/PA7 不耐 5V, GY-521 的 VCC 要飞线到 H9.20(3.3V), 别插座子的 5V
  *             本文件未被加入 Keil 工程, 保留作参考实现
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    与 car_mpu6050.c 的区别: 这里没有重试、没有总线恢复、没有零偏校准,
    量化系数也不一样 —— 加计按 ±8g(4096 LSB/g), 陀螺按 ±2000°/s(16.4 LSB/(°/s))。
    yaw 直接拿换算后的 gz 乘 0.01 积分, 没有死区, 停车时可以在 watch 里看零漂。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#include "car_mpu.h"
#include "car_config.h"
#include "ioi2c.h"
#include "delay.h"

#define MPU_DEV     (uint8_t)(MPU_ADDR << 1)   //0xD0

volatile uint8_t  g_mpu_id  = 0;
volatile uint8_t  g_mpu_ok_simple  = 0;
volatile fp32     g_mpu_ax = 0, g_mpu_ay = 0, g_mpu_az = 0;
volatile fp32     g_mpu_gx = 0, g_mpu_gy = 0, g_mpu_gz = 0;
volatile fp32     g_mpu_yaw = 0;
volatile int16_t  g_mpu_raw[6] = {0};

/**
  * @brief          mpu6050 init: read id, wake up, set full scale and dlpf
  * @param[in]      none
  * @retval         1: who_am_i is 0x68, 0: id check failed
  */
/**
  * @brief          MPU6050 初始化: 读 WHO_AM_I -> 退出睡眠 -> 设量程/分频/DLPF
  * @param[in]      none
  * @retval         1 = WHO_AM_I 读到 0x68, 0 = ID 不对
  */
uint8_t MPU_Init(void)
{
    uint8_t id = 0;

    /* PA6/PA7 已在 gpio.c 里初始化成推挽输出, 这里不用再 IIC_Init()
       (ioi2c.h 里声明的 IIC_Init() 在 IOI2C.c 中没有实现, 调了会链接失败) */
    delay_ms(50);

    IICreadBytes(MPU_DEV, 0x75, 1, &id);   //WHO_AM_I
    g_mpu_id = id;

    IICwriteByte(MPU_DEV, 0x6B, 0x00);   //退出睡眠
    delay_ms(10);
    IICwriteByte(MPU_DEV, 0x1B, 0x18);   //陀螺 ±2000°/s
    IICwriteByte(MPU_DEV, 0x1C, 0x10);   //加计 ±8g
    IICwriteByte(MPU_DEV, 0x19, 0x00);   //采样分频
    IICwriteByte(MPU_DEV, 0x1A, 0x03);   //DLPF 42Hz
    delay_ms(10);

    g_mpu_ok_simple = (id == 0x68) ? 1 : 0;
    return g_mpu_ok_simple;
}

/**
  * @brief          read accel and gyro, convert and integrate yaw
  * @param[out]     d: converted data point (g / °/s)
  * @retval         1: read success, 0: i2c error
  */
/**
  * @brief          读 14 字节(加计6 + 温度2 + 陀螺6), 换算成 g / °/s 并积分 yaw
  * @param[out]     d: 换算结果指针 (加计 g, 陀螺 °/s)
  * @retval         1 = 读成功, 0 = I2C 失败
  */
uint8_t MPU_Read(MPU_Data_t *d)
{
    uint8_t buf[14];

    if (!IICreadBytes(MPU_DEV, 0x3B, 14, buf)) return 0;

    g_mpu_raw[0] = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
    g_mpu_raw[1] = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
    g_mpu_raw[2] = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
    g_mpu_raw[3] = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    g_mpu_raw[4] = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    g_mpu_raw[5] = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    d->ax = (fp32)g_mpu_raw[0] / 4096.0f;   //±8g  -> 4096 LSB/g
    d->ay = (fp32)g_mpu_raw[1] / 4096.0f;
    d->az = (fp32)g_mpu_raw[2] / 4096.0f;
    d->gx = (fp32)g_mpu_raw[3] / 16.4f;     //±2000°/s -> 16.4 LSB/(°/s)
    d->gy = (fp32)g_mpu_raw[4] / 16.4f;
    d->gz = (fp32)g_mpu_raw[5] / 16.4f;

    g_mpu_ax = d->ax; g_mpu_ay = d->ay; g_mpu_az = d->az;
    g_mpu_gx = d->gx; g_mpu_gy = d->gy; g_mpu_gz = d->gz;

    /* YAW 角速度积分, 停车时可以在 watch 里看零漂 */
    g_mpu_yaw += d->gz * 0.01f;

    return 1;
}
