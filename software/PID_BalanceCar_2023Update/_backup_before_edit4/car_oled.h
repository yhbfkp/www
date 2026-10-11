#ifndef __CAR_OLED_H
#define __CAR_OLED_H

#include "sys.h"

/* ---------------------------------------------------------------------------
 * OLED SSD1306 128x64 — 软件 I2C PB8(SCL) / PB9(SDA), 地址 0x3C
 * 这块屏只干一件事: 显示 "当前模式 + 循迹用时", 调参不靠它(调参看串口/watch)
 * ------------------------------------------------------------------------- */

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_ShowString(uint8_t x, uint8_t y, const char *s, uint8_t size);  /* size: 12/16/24 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);

/* 给上层用的一屏显示: mode 是模式号, name 是模式名,
   running: 0=待机 1=运行中, time_ms 是本轮计时 */
void OLED_ShowStatus(uint8_t mode, const char *name, uint8_t running, uint32_t time_ms);

#endif
