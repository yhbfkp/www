#include "car_oled.h"
#include "car_config.h"
#include "oledfont.h"
#include "delay.h"
#include <stdio.h>

/* ---------------- 软件 I2C (PB8=SCL, PB9=SDA), 只写不读 ---------------- */
#define OL_SCL_H()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define OL_SCL_L()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define OL_SDA_H()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define OL_SDA_L()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)

#define OL_ADDR     (uint8_t)(OLED_ADDR << 1)

static void OL_Start(void)
{
    OL_SDA_H(); OL_SCL_H(); delay_us(2);
    OL_SDA_L(); delay_us(2);
    OL_SCL_L(); delay_us(2);
}

static void OL_Stop(void)
{
    OL_SCL_L(); OL_SDA_L(); delay_us(2);
    OL_SCL_H(); delay_us(2);
    OL_SDA_H(); delay_us(2);
}

static void OL_SendByte(uint8_t d)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        OL_SCL_L();
        if (d & 0x80) OL_SDA_H(); else OL_SDA_L();
        d <<= 1;
        delay_us(2);
        OL_SCL_H();
        delay_us(2);
    }
    /* 第9个时钟(ACK), 不读应答 */
    OL_SCL_L(); OL_SDA_H(); delay_us(2);
    OL_SCL_H(); delay_us(2);
    OL_SCL_L();
}

static void OL_Write(uint8_t dat, uint8_t is_data)
{
    OL_Start();
    OL_SendByte(OL_ADDR);
    OL_SendByte(is_data ? 0x40 : 0x00);
    OL_SendByte(dat);
    OL_Stop();
}

#define OL_WCMD(d)  OL_Write((d), 0)
#define OL_WDAT(d)  OL_Write((d), 1)

/* ---------------- 显存 ---------------- */
static uint8_t s_gram[128][8];

static void OLED_DrawPoint(uint8_t x, uint8_t y)
{
    if (x < 128 && y < 64) s_gram[x][y / 8] |= (uint8_t)(1 << (y % 8));
}

static void OLED_ClearPoint(uint8_t x, uint8_t y)
{
    if (x < 128 && y < 64) s_gram[x][y / 8] &= (uint8_t)~(1 << (y % 8));
}

void OLED_Refresh(void)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        OL_WCMD((uint8_t)(0xB0 | i));
        OL_WCMD(0x00);
        OL_WCMD(0x10);

        OL_Start();
        OL_SendByte(OL_ADDR);
        OL_SendByte(0x40);
        for (n = 0; n < 128; n++) OL_SendByte(s_gram[n][i]);
        OL_Stop();
    }
}

void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++) for (n = 0; n < 128; n++) s_gram[n][i] = 0;
    OLED_Refresh();
}

static void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size)
{
    uint8_t i, m, temp, size2, chr1, y0 = y;

    size2 = (uint8_t)((size / 8 + ((size % 8) ? 1 : 0)) * (size / 2));
    chr1  = chr - ' ';
    if (chr1 > 94) return;

    for (i = 0; i < size2; i++)
    {
        if      (size == 12) temp = oled_asc2_1206[chr1][i];
        else if (size == 16) temp = oled_asc2_1608[chr1][i];
        else return;

        for (m = 0; m < 8; m++)
        {
            if (temp & 0x80) OLED_DrawPoint(x, y); else OLED_ClearPoint(x, y);
            temp <<= 1;
            y++;
            if ((y - y0) == size) { y = y0; x++; break; }
        }
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *s, uint8_t size)
{
    while (*s)
    {
        if (*s < ' ' || *s > '~') { s++; continue; }
        OLED_ShowChar(x, y, (uint8_t)*s, size);
        x += size / 2;
        if (x > 128 - size) { x = 0; y += size; }
        s++;
    }
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t t;
    uint32_t div = 1;

    for (t = 1; t < len; t++) div *= 10;
    for (t = 0; t < len; t++)
    {
        OLED_ShowChar((uint8_t)(x + (size / 2) * t), y, (uint8_t)((num / div) % 10 + '0'), size);
        div /= 10;
    }
}

/* 初始化序列: 与天猛星(MSPM0 实测点亮)逐条一致, 只是把时钟/电源三项提为宏
   0xAE 关显示      0x00/0x10 列地址低/高   0x40 起始行
   0x81 0xCF 对比度 0xA1 段重映射          0xC8 COM扫描方向  0xA6 正常显示
   0xA8 0x3F 多路复用 64
   0xD3 0x00 显示偏移
   0xD5 OLED_CLK_DIV    显示时钟分频 + 振荡器频率 (外部/内部时钟档位)
   0xD9 OLED_PRECHARGE  预充电周期
   0xDA 0x12 COM引脚配置   0xDB 0x40 VCOMH
   0x20 0x02 页寻址模式
   0x8D OLED_CHARGE_PUMP  电荷泵(内部升压) 开关
   0xA4 全屏点亮关闭  0xA6 正常显示  0xAF 开显示 */
void OLED_Init(void)
{
    OL_SCL_H(); OL_SDA_H();
    delay_ms(100);

    OL_WCMD(0xAE); OL_WCMD(0x00); OL_WCMD(0x10); OL_WCMD(0x40);
    OL_WCMD(0x81); OL_WCMD(0xCF);
    OL_WCMD(0xA1); OL_WCMD(0xC8); OL_WCMD(0xA6);
    OL_WCMD(0xA8); OL_WCMD(0x3F);
    OL_WCMD(0xD3); OL_WCMD(0x00);
    OL_WCMD(0xD5); OL_WCMD(OLED_CLK_DIV);
    OL_WCMD(0xD9); OL_WCMD(OLED_PRECHARGE);
    OL_WCMD(0xDA); OL_WCMD(0x12);
    OL_WCMD(0xDB); OL_WCMD(0x40);
    OL_WCMD(0x20); OL_WCMD(0x02);
    OL_WCMD(0x8D); OL_WCMD(OLED_CHARGE_PUMP);
    OL_WCMD(0xA4); OL_WCMD(0xA6);
    OL_WCMD(0xAF);

    OLED_Clear();
}

/* ---------------- 一屏状态: 模式 + 用时 ---------------- */
void OLED_ShowStatus(uint8_t mode, const char *name, uint8_t running, uint32_t time_ms)
{
    char buf[24];
    uint32_t sec  = time_ms / 1000;
    uint32_t dsec = (time_ms % 1000) / 100;

    /* 清显存(不刷屏, 后面统一刷一次) */
    {
        uint8_t i, n;
        for (i = 0; i < 8; i++) for (n = 0; n < 128; n++) s_gram[n][i] = 0;
    }

    /* 第一行: 模式号 + 模式名 + 运行状态
       running: 0=IDLE 1=RUN 2=DONE 3=OBS(超声波附加项, 显示 stop!) */
    {
        const char *st;
        switch (running)
        {
            case 1:  st = "RUN"; break;
            case 2:  st = "DONE";break;
            case 3:  st = "stop!";break;
            default: st = "RDY"; break;
        }
        snprintf(buf, sizeof(buf), "M%d %s", (int)mode, name ? name : "");
        OLED_ShowString(0, 0, buf, 16);
        OLED_ShowString(96, 0, st, 16);
    }

    /* 第二行: 用时 秒.十分秒 */
    snprintf(buf, sizeof(buf), "TIME %lu.%lu s", (unsigned long)sec, (unsigned long)dsec);
    OLED_ShowString(0, 24, buf, 16);

    OLED_Refresh();
}
