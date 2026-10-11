/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_mpu6050.c
  * @brief      MPU6050 (GY-521) 驱动 + 陀螺 Z 零偏校准 + yaw 积分
  * @note       移植自 car-main v1.3 ml_mpu6050.c, 底层换成工程自带的软件 I2C
  *             (CSL/Src/IOI2C.c 的 i2cRead / i2cWrite, 内部会自己做 <<1)
  *             配置: 100Hz 采样(0x19=9), DLPF 92Hz, 陀螺 ±2000dps, 加计 ±2g
  *                         (写 0x80 软复位)
  *             WHO_AM_I 必须读到 0x68, 否则 AD0 接地 / 接线 / 供电有问题
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    [初始化流程] MPU6050_Init() 最多跑 MPU_INIT_RETRY 轮, 每轮:
      1) MPU_BusClear()   打 9 个 SCL 脉冲 + 补一个 Stop, 放开被从机卡住的 SDA
      2) delay_ms(20)     等总线稳定
      3) 写 0x80 软复位   (复位期间从机不响应 ACK, 所以这一步不看返回值)
      4) delay_ms(200)    规格书要求复位后 100ms 内勿访问, 留足裕量
      5) 读 WHO_AM_I     失败 -> g_mpu_init_step=1(没ACK)  值不对 -> =2(ID不对)
      6) 写 0x6B/0x19/0x1A/0x1B/0x1C/0x38 -> g_mpu_init_step=255, g_mpu_ok=1

    [yaw] MPU_Task10ms() 每 10ms 调一次: 读 14 字节 -> gz 去零偏 -> 死区 -> 积分
      dt = 10ms 与 MPU 的 100Hz 输出对齐, 无磁力计 -> 纯陀螺积分会缓慢漂移
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#include "car_mpu6050.h"
#include "ioi2c.h"      //SDA_OUT() / IIC_SCL / IIC_SDA: 初始化前用来恢复卡死的总线
#include "delay.h"

int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

fp32 g_yaw       = 0.0f;
fp32 gyro_bias_z = 0.0f;

volatile uint8_t  g_mpu_i2c_err = 0;
volatile uint8_t  g_mpu_ok      = 0;
volatile uint32_t g_mpu_ok_cnt  = 0;
volatile uint32_t g_mpu_err_cnt = 0;
volatile uint16_t g_mpu_cal_frames = 0;     //零偏标定实际采到的有效帧数(0=总线有问题)

/* 初始化诊断 (原来 Init 一次不中就没有第二次机会, 失败原因也看不出来) */
volatile uint8_t  g_mpu_init_step = 0;    //0=没跑   1=没ACK   2=ID不对   255=成功
volatile uint8_t  g_mpu_init_id   = 0;    //实际读到的 WHO_AM_I
volatile uint8_t  g_mpu_init_try  = 0;    //第几次尝试

/* 底层: 复用工程自带的软件I2C (i2cRead/i2cWrite 内部会 <<1) */
/**
  * @brief          write one mpu6050 register
  * @param[in]      reg: register address
  * @param[in]      dat: data to write
  * @retval         1: i2c error, 0: success
  */
/**
  * @brief          写一个 MPU6050 寄存器(失败时置 I2C 错误标志并累加错误计数)
  * @param[in]      reg: 寄存器地址
  * @param[in]      dat: 要写入的数据
  * @retval         1 = I2C 失败, 0 = 成功
  */
static uint8_t MPU_WriteReg(uint8_t reg, uint8_t dat)
{
    if (i2cWrite(MPU_ADDR, reg, 1, &dat))
    {
        g_mpu_i2c_err = 1;
        g_mpu_err_cnt++;
        return 1;
    }
    return 0;
}

/**
  * @brief          read one mpu6050 register
  * @param[out]     dat: data point
  * @param[in]      reg: register address
  * @retval         1: i2c error, 0: success
  */
/**
  * @brief          读一个 MPU6050 寄存器(失败时置 I2C 错误标志并累加错误计数)
  * @param[in]      reg: 寄存器地址
  * @param[out]     dat: 读回的数据指针
  * @retval         1 = I2C 失败, 0 = 成功
  */
static uint8_t MPU_ReadReg(uint8_t reg, uint8_t *dat)
{
    if (i2cRead(MPU_ADDR, reg, 1, dat))
    {
        g_mpu_i2c_err = 1;
        g_mpu_err_cnt++;
        return 1;
    }
    return 0;
}

/* 连续读: 从 startAddr 开始地址自动递增 */
/**
  * @brief          burst read registers
  * @param[in]      startAddr: first register address
  * @param[out]     buf: data buffer point
  * @param[in]      len: byte number
  * @retval         1: i2c error, 0: success
  */
/**
  * @brief          连续读: 从 startAddr 开始地址自动递增
  * @param[in]      startAddr: 起始寄存器地址
  * @param[out]     buf: 数据缓冲区指针
  * @param[in]      len: 读取字节数
  * @retval         1 = I2C 失败, 0 = 成功
  */
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

/* 总线恢复: 打 9 个 SCL 脉冲再补一个 Stop, 把上一次下电/复位时卡住 SDA 的从机放开。
   不恢复的话 IIC_Start() 里 READ_SDA 一直是 0, 直接 return 0, 整个 Init 就废了。 */
/**
  * @brief          recover a stuck i2c bus: 9 scl pulses + one stop
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          总线恢复: 打 9 个 SCL 脉冲再补一个 Stop, 放开被从机卡住的 SDA
  * @param[in]      none
  * @retval         none
  */
static void MPU_BusClear(void)
{
    uint8_t i;

    SDA_OUT();
    IIC_SCL = 0;
    IIC_SDA = 1;
    delay_us(10);

    for (i = 0; i < 9; i++)
    {
        IIC_SCL = 1;
        delay_us(10);
        IIC_SCL = 0;
        delay_us(10);
    }

    IIC_SDA = 0;  delay_us(10);       //Stop: SCL 高电平时 SDA 由低变高
    IIC_SCL = 1;  delay_us(10);
    IIC_SDA = 1;  delay_us(10);
}

/**
  * @brief          mpu6050 init with retry
  * @param[in]      none
  * @retval         0: success, 1: init failed after MPU_INIT_RETRY times
  */
/**
  * @brief          MPU6050 初始化(带重试): 总线恢复 + 软复位 + 查ID + 写配置
  * @param[in]      none
  * @retval         0 = 成功, 1 = 重试 MPU_INIT_RETRY 次仍未成功
  */
uint8_t MPU6050_Init(void)
{
    uint8_t id  = 0;
    uint8_t t;

    for (t = 0; t < MPU_INIT_RETRY; t++)
    {
        g_mpu_init_try = t + 1;

        MPU_BusClear();
        delay_ms(20);

        //软件复位。写这一步拿不到 ACK 是正常的(复位期间从机不响应), 所以不看返回值
        MPU_WriteReg(MPU_PWR_MGMT_1, 0x80);
        delay_ms(200);                          //规格书要求复位后 100ms 内勿访问, 200ms 留足裕量

        if (MPU_ReadReg(MPU_WHO_AM_I, &id))
        {
            g_mpu_init_step = 1;                //没 ACK: 查 VCC/GND/SCL/SDA
            continue;
        }

        g_mpu_init_id = id;
        if (id != 0x68)
        {
            g_mpu_init_step = 2;                //ID 不对: AD0 必须接地
            continue;
        }

        MPU_WriteReg(MPU_PWR_MGMT_1, 0x01);     //唤醒, PLL X轴陀螺
        delay_ms(10);
        MPU_WriteReg(MPU_SMPLRT_DIV, 9);        //1000/(1+9) = 100Hz, dt=10ms
        MPU_WriteReg(MPU_CONFIG, 0x02);         //DLPF 92Hz
        MPU_WriteReg(MPU_GYRO_CONFIG, 0x18);    //±2000 dps
        MPU_WriteReg(MPU_ACCEL_CONFIG, 0x00);   //±2 g
        MPU_WriteReg(MPU_INT_ENABLE, 0x01);     //数据就绪中断(本工程没接INT脚, 无副作用)

        g_mpu_init_step = 255;
        g_mpu_i2c_err   = 0;
        g_mpu_err_cnt   = 0;                    //初始化重试期间的失败不计入运行错误
        g_mpu_ok        = 1;
        return 0;
    }

    g_mpu_i2c_err = 2;
    g_mpu_ok      = 0;
    return 1;
}

/* 上电静止采集陀螺Z零偏 (车必须绝对静止! 约2.2秒) */
/**
  * @brief          sample gyro z bias: drop 20 frames, average 200 frames
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          上电静止采集陀螺Z零偏: 丢前20帧, 再采200帧求平均 (车必须绝对静止!)
  * @param[in]      none
  * @retval         none
  */
void MPU6050_CalGyroBias(void)
{
    int      i;
    int32_t  sum_gz = 0;
    uint16_t n = 0;                         //真正读成功的有效帧数
    uint8_t  buf[6];

    for (i = 0; i < 20; i++)                //丢前20帧
    {
        MPU_BurstRead(MPU_GYRO_XOUT_H, buf, 6);
        delay_ms(10);
    }

    /* 只把"读成功"的帧计入平均: 原来不判返回值, 通信失败时 buf 还是上一帧的内容,
       脏数据会被当成有效样本平均进去 -> 零偏被污染 -> yaw 会一直漂。
       最多等 600 次(约 6s); 采够 200 帧就提前结束。 */
    for (i = 0; i < 600 && n < 200; i++)
    {
        if (MPU_BurstRead(MPU_GYRO_XOUT_H, buf, 6) == 0)
        {
            sum_gz += (int16_t)(buf[5] | ((uint16_t)buf[4] << 8));
            n++;
        }
        delay_ms(10);
    }

    g_mpu_cal_frames = n;                   //观察用: 0 = 一帧都没读到(总线有问题)

    if (n > 0)
    {
        gyro_bias_z = (fp32)sum_gz / (fp32)n;
        g_yaw = 0.0f;
    }
}

/* 一次 burst 读 14 字节: 加计6 + 温度2 + 陀螺6 */
/**
  * @brief          burst read 14 bytes and split into ax..gz
  * @param[in]      none
  * @retval         0: success, 1: i2c error
  */
/**
  * @brief          一次 burst 读 14 字节(加计6 + 温度2 + 陀螺6)并拆到 ax..gz
  * @param[in]      none
  * @retval         0 = 成功, 1 = I2C 通信失败
  */
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
/**
  * @brief          10ms task: read data and integrate yaw
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          10ms 调一次: 读数据 + 积分 yaw (dt=10ms 与 MPU 的 100Hz 输出对齐)
  * @param[in]      none
  * @retval         none
  */
void MPU_Task10ms(void)
{
    fp32 rate;

    if (MPU6050_GetData() != 0) return;     //通信失败: 丢弃本帧, 不积分

    rate = ((fp32)gz - gyro_bias_z) / 16.4f;            //°/s
    if (rate > -0.15f && rate < 0.15f) rate = 0.0f;     //死区 ±0.15°/s
    g_yaw += rate * 0.01f;
}
