/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_gray.h
  * @brief      芯流科技 极光哨卫T8 八路灰度传感器 (IIC) 接口
  * @note       硬件: I2C2  PB10(SCL) / PB11(SDA), 100kHz
  *             (板上 4P 座脚位是 5V / PB10 / PB11 / GND, 模块端要按 VCC/GND/SCL/SDA 手接杜邦线)
  *             通信格式(手册 1.3): 主机发 1 字节操作指令, 从机返回
  *             [指令回显][长度预告N][数据0..N-1][校验和], 所以主机要收 N+3 个字节
  *             常用指令:
  *               0x09 -> 8 字节, 1-8路 ADC/8 (0-255)
  *               0x0C -> 1 字节, 8路数字量按位或 (bit0=第1路 ... bit7=第8路)
  *               0x19 -> 16 字节, 1-8路 16位ADC, 小端
  *               0x1A -> 16 字节, 黑色阈值  0x1B -> 白色阈值
  *               0xAD -> 读 IIC 地址
  *             IIC 地址: 拨码 000=串口配置(出厂未配置为 0x40)
  *                       001=0x41 010=0x42 011=0x43 100=0x44 101=0x45 110=0x46 111=0x47
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    读失败时的口径: Gray_ReadBitMask() 保留上一次的位掩码, 其它读接口返回 0;
    g_gray_err / g_gray_ok_cnt 只统计通信结果, 不改控制逻辑。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#ifndef CAR_GRAY_H
#define CAR_GRAY_H

#include "sys.h"
#include "struct_typedef.h"

#define GRAY_CMD_ADC8       0x09
#define GRAY_CMD_BITS       0x0C
#define GRAY_CMD_ADC16      0x19
#define GRAY_CMD_BLK16      0x1A
#define GRAY_CMD_WHT16      0x1B
#define GRAY_CMD_ADDR       0xAD

/**
  * @brief          probe GRAY_ADDR, 1 = sensor answered
  * @param[in]      none
  * @retval         1: sensor ready, 0: no answer
  */
/**
  * @brief          探测 GRAY_ADDR 是否应答
  * @param[in]      none
  * @retval         1=OK, 0=没应答
  */
extern uint8_t Gray_Init(void);                       //探测 GRAY_ADDR 是否应答, 1=OK

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
extern uint8_t Gray_Scan(uint8_t start, uint8_t end);

/**
  * @brief          read 8 channels as 0/1 into bits[]
  * @param[out]     bits: 8 channels, bits[0] = channel 1
  * @retval         bit mask of this frame
  */
/**
  * @brief          读 8 路 0/1 电平到 bits[]
  * @param[out]     bits: 8路 0/1, bits[0]=第1路
  * @retval         本次读到的位掩码
  */
extern uint8_t Gray_ReadBits(uint8_t bits[8]);

/**
  * @brief          read 8 channels as one bit mask byte
  * @param[in]      none
  * @retval         bit mask, keep last frame on communication fail
  */
/**
  * @brief          直接读 1 字节位掩码
  * @param[in]      none
  * @retval         位掩码(通信失败时保留上一次的值)
  */
extern uint8_t Gray_ReadBitMask(void);

/**
  * @brief          read 8 channels 8-bit ADC
  * @param[out]     adc: 8 channels 8-bit ADC
  * @retval         1: ok, 0: communication fail
  */
/**
  * @brief          读 8 路 8 位 ADC
  * @param[out]     adc: 8路 8位ADC
  * @retval         1=成功, 0=通信失败
  */
extern uint8_t Gray_ReadADC8(uint8_t adc[8]);

/**
  * @brief          read 8 channels 16-bit ADC (little endian)
  * @param[out]     adc: 8 channels 16-bit ADC
  * @retval         1: ok, 0: communication fail
  */
/**
  * @brief          读 8 路 16 位 ADC (小端)
  * @param[out]     adc: 8路 16位ADC
  * @retval         1=成功, 0=通信失败
  */
extern uint8_t Gray_ReadADC16(uint16_t adc[8]);

/**
  * @brief          read the module's own I2C address
  * @param[in]      none
  * @retval         address, 0 = communication fail
  */
/**
  * @brief          读模块自身地址
  * @param[in]      none
  * @retval         模块地址, 0=通信失败
  */
extern uint8_t Gray_ReadAddr(void);

/* 调试用 */
extern volatile uint8_t  g_gray_ok;       //通信是否成功
extern volatile uint8_t  g_gray_bits;     //最近一次位掩码
extern volatile uint8_t  g_gray_raw[8];   //最近一次 8 位ADC
extern volatile uint16_t g_gray_adc[8];   //最近一次 16位ADC
extern volatile uint8_t  g_gray_addr;     //实际使用的地址
extern volatile uint32_t g_gray_err;      //通信失败次数
extern volatile uint32_t g_gray_ok_cnt;   //通信成功次数

#endif
