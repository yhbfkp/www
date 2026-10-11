#include "car_mpu6050.h"
#include "delay.h"

int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

float g_yaw       = 0.0f;
float gyro_bias_z = 0.0f;

volatile uint8_t  g_mpu_i2c_err = 0;
volatile uint8_t  g_mpu_ok      = 0;
volatile uint32_t g_mpu_ok_cnt  = 0;
volatile uint32_t g_mpu_err_cnt = 0;

/* 底层: 复用工程自带的软件I2C (i2cRead/i2cWrite 内部会 <<1) */
static uint8_t MPU_WriteReg(uint8_t reg, uint8_t dat)
{
    if (i2cWrite(MPU_ADDR, reg, 1, &dat)) { g_mpu_i2c_err = 1; g_mpu_err_cnt++; return 1; }
    return 0;
}

static uint8_t MPU_ReadReg(uint8_t reg, uint8_t *dat)
{
    if (i2cRead(MPU_ADDR, reg, 1, dat))  { g_mpu_i2c_err = 1; g_mpu_err_cnt++; return 1; }
    return 0;
}

/* 连续读: 从 startAddr 开始地址自动递增 */
static uint8_t MPU_BurstRead(uint8_t startAddr, uint8_t *buf, uint8_t len)
{
    if (i2cRead(MPU_ADDR, startAddr, len, buf))
    {
        g_mpu_i2c_err = 1;
        g_mpu_err_cnt++;
        return 1;
    }
    return 0;
}

uint8_t MPU6050_Init(void)
{
    uint8_t id = 0;

    MPU_WriteReg(MPU_PWR_MGMT_1, 0x80);     /* 复位 */
    delay_ms(100);

    if (MPU_ReadReg(MPU_WHO_AM_I, &id)) { g_mpu_i2c_err = 2; return 1; }
    if (id != 0x68) { g_mpu_i2c_err = 2; return 1; }

    MPU_WriteReg(MPU_PWR_MGMT_1, 0x01);     /* 唤醒, PLL X轴陀螺 */
    delay_ms(10);
    MPU_WriteReg(MPU_SMPLRT_DIV, 9);        /* 1000/(1+9) = 100Hz, dt=10ms */
    MPU_WriteReg(MPU_CONFIG, 0x02);         /* DLPF 92Hz */
    MPU_WriteReg(MPU_GYRO_CONFIG, 0x18);    /* ±2000 dps */
    MPU_WriteReg(MPU_ACCEL_CONFIG, 0x00);   /* ±2 g */
    MPU_WriteReg(MPU_INT_ENABLE, 0x01);     /* 数据就绪中断(本工程没接INT脚, 无副作用) */

    g_mpu_i2c_err = 0;
    g_mpu_ok = 1;
    return 0;
}

/* 上电静止采集陀螺Z零偏 (车必须绝对静止! 约2.2秒) */
void MPU6050_CalGyroBias(void)
{
    int      i;
    int32_t  sum_gz = 0;
    uint8_t  buf[6];

    for (i = 0; i < 20; i++)                /* 丢前20帧 */
    {
        MPU_BurstRead(MPU_GYRO_XOUT_H, buf, 6);
        delay_ms(10);
    }
    for (i = 0; i < 200; i++)
    {
        MPU_BurstRead(MPU_GYRO_XOUT_H, buf, 6);
        sum_gz += (int16_t)(buf[5] | ((uint16_t)buf[4] << 8));
        delay_ms(10);
    }

    gyro_bias_z = (float)sum_gz / 200.0f;
    g_yaw = 0.0f;
}

/* 一次 burst 读 14 字节: 加计6 + 温度2 + 陀螺6 */
uint8_t MPU6050_GetData(void)
{
    uint8_t buf[14];

    if (MPU_BurstRead(MPU_ACCEL_XOUT_H, buf, 14)) return 1;

    ax = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
    ay = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
    az = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
    gx = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    g_mpu_i2c_err = 0;
    g_mpu_ok_cnt++;
    return 0;
}

/* 10ms 一次: 读数据 + 积分 yaw (dt=10ms 与 MPU 的 100Hz 输出对齐) */
void MPU_Task10ms(void)
{
    float rate;

    if (MPU6050_GetData() != 0) return;     /* 通信失败: 丢弃本帧, 不积分 */

    rate = ((float)gz - gyro_bias_z) / 16.4f;           /* °/s */
    if (rate > -0.15f && rate < 0.15f) rate = 0.0f;     /* 死区 ±0.15°/s */
    g_yaw += rate * 0.01f;
}
