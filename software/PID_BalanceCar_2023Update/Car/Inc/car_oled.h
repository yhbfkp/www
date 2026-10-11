/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_oled.h
  * @brief      OLED SSD1306 128x64 接口: 软件 SPI / 软件 I2C 二选一 + 显存 + 一屏状态显示
  * @note       这块屏只干一件事: 显示 "当前模式 + 循迹用时", 调参不靠它(调参看串口/watch)
  *             传输层由 car_config.h 的 OLED_USE_SPI 切; 引脚表和接线说明在 car_oled.c 顶部。
  *               SPI(默认): SCK=PB8 MOSI=PB9 DC=PA5 RES=PB1, CS 接 GND
  *               I2C      : SCL=PB8 SDA=PB9, 7 位地址 0x3C/0x3D 上电自动探测
  *             显存是 128x64 的页格式: s_gram[x][page], page = y / 8
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    [DIAG] 两个诊断钩子由 car_config.h 的宏控制, 都是现场排故用的:
      OLED_PIN_TEST_ON_FAIL   -> 屏没应答时打 1Hz 方波, 万用表量 MCU->屏 那段
      OLED_ALLON_TEST_ON_BOOT -> 上电全屏点亮 1.5s, 白闪就说明屏和升压命令都通
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#ifndef CAR_OLED_H
#define CAR_OLED_H

#include "sys.h"
#include "struct_typedef.h"

/* 上电自动探测的结果 (4脚屏 0x3C / 0x3D 二选一):
   g_oled_addr = 实际使用的 7 位地址
   g_oled_ack  = 1 有应答(屏在线) / 0 两个地址都没应答 -> 查硬件, 不是地址问题 */
extern uint8_t g_oled_addr;
extern uint8_t g_oled_ack;

/* [DIAG] 仅 I2C 模式有效: 全总线扫描结果, 第 n 个 bit 为 1 表示 7 位地址 n 有应答。
   挂个 MPU6050(0x68) 或灰度(0x40)上去扫得到 -> 证明板子这一侧完全正常。
   SPI 模式没有地址/ACK 可探, 这个数组恒为 0(符号保留, 免得改上层)。 */
extern uint8_t g_i2c_scan[16];

/**
  * @brief          oled fail diagnostic, never return
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          屏没应答时调用(不会返回): 打印扫描结果 + PB8/PB9 输出 1Hz 方波,
  *                 用万用表量屏的 SCL/SDA 判断 "MCU->屏" 这段通不通
  * @param[in]      none
  * @retval         none
  */
extern void OLED_FailDiag(void);

/**
  * @brief          init ssd1306: pin init, bus scan, address auto detect, init sequence
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          初始化 SSD1306: 引脚初始化 -> 全总线扫描 -> 地址自动识别 -> 初始化序列
  * @param[in]      none
  * @retval         none
  */
extern void OLED_Init(void);

/**
  * @brief          clear gram
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          清显存(清完顺带刷一次屏)
  * @param[in]      none
  * @retval         none
  */
extern void OLED_Clear(void);

/**
  * @brief          refresh gram to screen
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          把显存整屏刷到屏上
  * @param[in]      none
  * @retval         none
  */
extern void OLED_Refresh(void);

/**
  * @brief          show a ascii string
  * @param[in]      x: start x (0..127)
  * @param[in]      y: start y (0..63)
  * @param[in]      s: ascii string point
  * @param[in]      size: font height, 12 / 16 / 24
  * @retval         none
  */
/**
  * @brief          显示一个 ASCII 字符串
  * @param[in]      x: 起始 x (0..127)
  * @param[in]      y: 起始 y (0..63)
  * @param[in]      s: ASCII 字符串指针
  * @param[in]      size: 字号高度, 12 / 16 / 24
  * @retval         none
  */
extern void OLED_ShowString(uint8_t x, uint8_t y, const char *s, uint8_t size);  //size: 12/16/24

/**
  * @brief          show a unsigned number
  * @param[in]      x: start x (0..127)
  * @param[in]      y: start y (0..63)
  * @param[in]      num: number to show
  * @param[in]      len: digit number
  * @param[in]      size: font height, 12 / 16 / 24
  * @retval         none
  */
/**
  * @brief          显示一个无符号数(固定位数)
  * @param[in]      x: 起始 x (0..127)
  * @param[in]      y: 起始 y (0..63)
  * @param[in]      num: 要显示的数
  * @param[in]      len: 显示位数
  * @param[in]      size: 字号高度, 12 / 16 / 24
  * @retval         none
  */
extern void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);

/**
  * @brief          show one screen status: mode + running time
  * @param[in]      mode: mode number
  * @param[in]      name: mode name string point
  * @param[in]      running: 0=idle 1=running 2=done 3=stop
  * @param[in]      time_ms: current run time (ms)
  * @retval         none
  */
/**
  * @brief          给上层用的一屏显示: mode 是模式号, name 是模式名
  * @param[in]      mode: 模式号
  * @param[in]      name: 模式名指针
  * @param[in]      running: 0=待机 1=运行中 2=完成 3=停
  * @param[in]      time_ms: 本轮计时 (ms)
  * @param[in]      dbg: 调试状态位(0=正常; bit0=整定 bit1=单轮 bit2=开环), 非0时屏上多一行 DBG
  * @retval         none
  */
extern void OLED_ShowStatus(uint8_t mode, const char *name, uint8_t running, uint32_t time_ms, uint8_t dbg);

#endif
