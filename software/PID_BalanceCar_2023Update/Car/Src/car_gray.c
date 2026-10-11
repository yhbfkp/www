/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_gray.c
  * @brief      芯流科技 极光哨卫T8 八路灰度传感器 (IIC) 实现
  * @note       通信格式(手册 1.3): 主机发 1 字节指令, 从机返回 N+3 字节:
  *             [指令回显][长度预告N][数据0..N-1][校验和]; 本文件只取数据区
  *             I2C2 = PB10(SCL) / PB11(SDA), 100kHz, 地址 GRAY_ADDR 可在运行时改
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    位序与取反:
      GRAY_REVERSE=1     -> 8 路数据整体镜像(bit0 <-> bit7), 装反了用
      GRAY_ACTIVE_LOW=1  -> 位取反, 使 bit=1 表示"压线"
    两者都是编译期宏, 处理顺序: 先镜像(Gray_ReadBitMask 内), 再取反。

    超时: GRAY_TIMEOUT_MS(默认 5ms) 不能调大 —— 读灰度现在跑在 TIM3 中断里,
    超时太长会拖垮心跳。scan 用 2 次重试 / 10ms, init 探测用 3 次重试 / GRAY_TIMEOUT_MS。

    统计量: g_gray_err 通信失败次数, g_gray_ok_cnt 成功次数, 都只增不减, 只供观察。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#include "car_gray.h"
#include "car_config.h"
#include "i2c.h"

volatile uint8_t  g_gray_ok     = 0;
volatile uint8_t  g_gray_bits   = 0;
volatile uint8_t  g_gray_raw[8] = {0};
volatile uint16_t g_gray_adc[8] = {0};
volatile uint8_t  g_gray_addr   = GRAY_ADDR;
volatile uint32_t g_gray_err    = 0;
volatile uint32_t g_gray_ok_cnt = 0;

/**
  * @brief          send one command byte and read back N+3 bytes, copy N data bytes
  * @param[in]      cmd: command byte(0x09 / 0x0C / 0x19 / 0xAD ...)
  * @param[in]      n: number of data bytes expected
  * @param[out]     buf: data buffer, NULL = only check communication
  * @retval         1: ok, 0: communication fail
  */
/**
  * @brief          发 1 字节指令, 收回显+长度+N个数据+校验和 = N+3 字节
  * @param[in]      cmd: 操作指令(0x09 / 0x0C / 0x19 / 0xAD ...)
  * @param[in]      n: 需要的数据字节数
  * @param[out]     buf: 数据缓冲区, 传 NULL 表示只要通信结果
  * @retval         1=成功, 0=通信失败
  */
static uint8_t Gray_Trans(uint8_t cmd, uint8_t n, uint8_t *buf)
{
    HAL_StatusTypeDef st;
    uint8_t rx[32];
    uint8_t need = n + 3;

    /* 防御: 协议的长度预告只有 1 字节(最大 255), 而 rx 只有 32 字节。
       当前调用最大 n=16 -> need=19, 是安全的; 但将来加一条 n>29 的指令就会
       静默越界, 所以这里先钳一下。 */
    if (need > sizeof(rx))
    {
        need = sizeof(rx);
    }

    st = HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)(g_gray_addr << 1), &cmd, 1, GRAY_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        g_gray_err++;
        g_gray_ok = 0;
        return 0;
    }

    st = HAL_I2C_Master_Receive(&hi2c2, (uint16_t)(g_gray_addr << 1), rx, need, GRAY_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        g_gray_err++;
        g_gray_ok = 0;
        return 0;
    }

    if (buf)
    {
        uint8_t i;
        for (i = 0; i < n; i++) buf[i] = rx[2 + i];
    }

    g_gray_ok = 1;
    g_gray_ok_cnt++;
    return 1;
}

/**
  * @brief          init gray sensor: probe GRAY_ADDR then read one frame
  * @param[in]      none
  * @retval         1: ok, 0: no answer
  */
/**
  * @brief          灰度初始化: 先探测地址, 通了再读一帧
  * @param[in]      none
  * @retval         1=OK, 0=没应答
  */
uint8_t Gray_Init(void)
{
    uint8_t bits[8];
    g_gray_addr = GRAY_ADDR;

    /* 先探测地址, 不通就别往下走 */
    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(g_gray_addr << 1), 3, GRAY_TIMEOUT_MS) != HAL_OK)
    {
        g_gray_ok = 0;
        return 0;
    }
    g_gray_ok = 1;
    Gray_ReadBits(bits);
    return 1;
}

/**
  * @brief          scan 7-bit I2C address, return the first one that answers
  * @param[in]      start: first address of the scan range(7 bit)
  * @param[in]      end: last address of the scan range(7 bit)
  * @retval         first address that answered, 0 = none found
  */
/**
  * @brief          扫地址, 返回第一个应答的地址(7位)
  * @param[in]      start: 扫描起始地址(7位)
  * @param[in]      end: 扫描结束地址(7位)
  * @retval         第一个应答的地址(7位), 0=没扫到
  */
uint8_t Gray_Scan(uint8_t start, uint8_t end)
{
    uint8_t a;

    if (start < 0x08) start = 0x08;
    if (end   > 0x77) end   = 0x77;

    for (a = start; a <= end; a++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(a << 1), 2, 10) == HAL_OK)
        {
            g_gray_addr = a;
            g_gray_ok   = 1;
            return a;
        }
    }
    return 0;
}

/**
  * @brief          read 8 channels as one bit mask byte
  * @param[in]      none
  * @retval         bit mask, keep last frame on communication fail
  */
/**
  * @brief          读 1 字节位掩码: 先按 GRAY_REVERSE 镜像, 再按 GRAY_ACTIVE_LOW 取反
  * @param[in]      none
  * @retval         位掩码(通信失败时保留上一次的值)
  */
uint8_t Gray_ReadBitMask(void)
{
    uint8_t d = 0;

    if (!Gray_Trans(GRAY_CMD_BITS, 1, &d)) return g_gray_bits;

#if GRAY_REVERSE
    {
        uint8_t i, r = 0;
        for (i = 0; i < 8; i++) if (d & (1 << i)) r |= (uint8_t)(1 << (7 - i));
        d = r;
    }
#endif

#if GRAY_ACTIVE_LOW
    d = (uint8_t)(~d);
#endif

    g_gray_bits = d;
    return d;
}

/**
  * @brief          read 8 channels as 0/1 into bits[]
  * @param[out]     bits: 8 channels, bits[0] = channel 1
  * @retval         bit mask of this frame
  */
/**
  * @brief          读 8 路 0/1 电平到 bits[]: bits[i] = (mask >> i) & 1
  * @param[out]     bits: 8路 0/1, bits[0]=第1路
  * @retval         本次读到的位掩码
  */
uint8_t Gray_ReadBits(uint8_t bits[8])
{
    uint8_t m = Gray_ReadBitMask();
    uint8_t i;

    for (i = 0; i < 8; i++) bits[i] = (m >> i) & 0x01;

    return m;
}

/**
  * @brief          read 8 channels 8-bit ADC
  * @param[out]     adc: 8 channels 8-bit ADC, NULL = only refresh g_gray_raw
  * @retval         1: ok, 0: communication fail
  */
/**
  * @brief          读 8 路 8 位 ADC, 按 GRAY_REVERSE 决定存放入序
  * @param[out]     adc: 8路 8位ADC, 传 NULL 表示只刷新 g_gray_raw
  * @retval         1=成功, 0=通信失败
  */
uint8_t Gray_ReadADC8(uint8_t adc[8])
{
    uint8_t buf[8];
    uint8_t i;

    if (!Gray_Trans(GRAY_CMD_ADC8, 8, buf)) return 0;

    for (i = 0; i < 8; i++)
    {
#if GRAY_REVERSE
        g_gray_raw[i] = buf[7 - i];
#else
        g_gray_raw[i] = buf[i];
#endif
        if (adc) adc[i] = g_gray_raw[i];
    }
    return 1;
}

/**
  * @brief          read 8 channels 16-bit ADC (little endian)
  * @param[out]     adc: 8 channels 16-bit ADC, NULL = only refresh g_gray_adc
  * @retval         1: ok, 0: communication fail
  */
/**
  * @brief          读 8 路 16 位 ADC (小端), 按 GRAY_REVERSE 决定存放入序
  * @param[out]     adc: 8路 16位ADC, 传 NULL 表示只刷新 g_gray_adc
  * @retval         1=成功, 0=通信失败
  */
uint8_t Gray_ReadADC16(uint16_t adc[8])
{
    uint8_t buf[16];
    uint8_t i;

    if (!Gray_Trans(GRAY_CMD_ADC16, 16, buf)) return 0;

    for (i = 0; i < 8; i++)
    {
        uint16_t v = (uint16_t)(buf[i * 2] | ((uint16_t)buf[i * 2 + 1] << 8));  //小端
        uint8_t  k = i;
#if GRAY_REVERSE
        k = (uint8_t)(7 - i);
#endif
        g_gray_adc[k] = v;
        if (adc) adc[k] = v;
    }
    return 1;
}

/**
  * @brief          read the module's own I2C address
  * @param[in]      none
  * @retval         address, 0 = communication fail
  */
/**
  * @brief          读模块自身地址(指令 0xAD)
  * @param[in]      none
  * @retval         模块地址, 0=通信失败
  */
uint8_t Gray_ReadAddr(void)
{
    uint8_t d = 0;
    if (!Gray_Trans(GRAY_CMD_ADDR, 1, &d)) return 0;
    return d;
}
