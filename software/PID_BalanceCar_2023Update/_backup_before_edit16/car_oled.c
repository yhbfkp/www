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

/* 实际从机地址: 上电自动探测, 4脚屏背面电阻决定是 0x3C 还是 0x3D,
   不用再手工来回改。串口 "=== Car Ready ===" 那行会打印实际值。
   g_oled_ack=0 表示两个地址都没应答 -> 是硬件问题(接线/VCC/引脚), 不是地址。*/
uint8_t g_oled_addr = OLED_ADDR;
uint8_t g_oled_ack  = 0;

#define OL_ADDR     ((uint8_t)(g_oled_addr << 1))

/* SDA 方向切换: 读 ACK 时必须把 SDA 放开(输入上拉), 否则推挽输出的 MCU
   会把 SDA 硬拉高, 从机根本拉不低, 永远读不到应答 */
static void OL_SDA_OutPP(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_9;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
}

static void OL_SDA_InPU(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_9;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
}

static void OL_Start(void)
{
    OL_SDA_H(); OL_SCL_H(); delay_us(OLED_I2C_DLY_US);
    OL_SDA_L(); delay_us(OLED_I2C_DLY_US);
    OL_SCL_L(); delay_us(OLED_I2C_DLY_US);
}

static void OL_Stop(void)
{
    OL_SCL_L(); OL_SDA_L(); delay_us(OLED_I2C_DLY_US);
    OL_SCL_H(); delay_us(OLED_I2C_DLY_US);
    OL_SDA_H(); delay_us(OLED_I2C_DLY_US);
}

static void OL_SendByte(uint8_t d)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        OL_SCL_L();
        if (d & 0x80) OL_SDA_H(); else OL_SDA_L();
        d <<= 1;
        delay_us(OLED_I2C_DLY_US);
        OL_SCL_H();
        delay_us(OLED_I2C_DLY_US);
    }
    /* 第9个时钟(ACK), 不读应答 */
    OL_SCL_L(); OL_SDA_H(); delay_us(OLED_I2C_DLY_US);
    OL_SCL_H(); delay_us(OLED_I2C_DLY_US);
    OL_SCL_L();
}

/* 发一个字节并读第9位 ACK: 返回 1 = 有从机应答 */
static uint8_t OL_SendByteAck(uint8_t d)
{
    uint8_t i, ack;

    for (i = 0; i < 8; i++)
    {
        OL_SCL_L();
        if (d & 0x80) OL_SDA_H(); else OL_SDA_L();
        d <<= 1;
        delay_us(OLED_I2C_DLY_US);
        OL_SCL_H();
        delay_us(OLED_I2C_DLY_US);
    }
    /* 第9个时钟: 放开 SDA, 读从机是否拉低 */
    OL_SCL_L();
    OL_SDA_InPU();
    delay_us(OLED_I2C_DLY_US);
    OL_SCL_H();
    delay_us(OLED_I2C_DLY_US);
    ack = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET) ? 1U : 0U;
    OL_SCL_L();
    delay_us(OLED_I2C_DLY_US);
    OL_SDA_OutPP();
    OL_SDA_H();
    delay_us(OLED_I2C_DLY_US);
    return ack;
}

/* 探测某个 7 位地址有没有设备应答 */
static uint8_t OL_Probe(uint8_t a7)
{
    uint8_t ack;

    OL_SDA_OutPP();
    OL_SCL_H(); OL_SDA_H();
    OL_Start();
    ack = OL_SendByteAck((uint8_t)(a7 << 1));
    OL_Stop();
    return ack;
}

/* 自动识别地址: 先试 car_config.h 里的 OLED_ADDR, 不通再试另一个常见地址
   (4脚 SSD1306 只有 0x3C / 0x3D 两种可能, 由模块背面电阻决定) */
static uint8_t OLED_AutoAddr(void)
{
    uint8_t alt = (OLED_ADDR == 0x3C) ? 0x3D : 0x3C;

    g_oled_ack = OL_Probe(OLED_ADDR);
    if (g_oled_ack) { g_oled_addr = OLED_ADDR; return 1; }

    g_oled_ack = OL_Probe(alt);
    if (g_oled_ack) { g_oled_addr = alt;       return 1; }

    g_oled_addr = OLED_ADDR;    /* 都没应答: 保持默认值, 后续命令照发(无害) */
    return 0;
}

/* [DIAG] 扫 0x00~0x7F 全部地址, 结果按位存进 g_i2c_scan:
     只有 1~2 个 bit 置位  -> 真实设备, 通信没问题
     一个 bit 都没有       -> 总线是空的(或那段线有问题)
   拿 MPU6050(0x68)/灰度(0x40) 挂上去扫得到 -> 证明 MCU 这侧完全正常。 */
uint8_t g_i2c_scan[16];

static void OLED_DiagScan(void)
{
    uint8_t a;

    for (a = 0; a < 16; a++) g_i2c_scan[a] = 0;
    for (a = 0; a < 128; a++)
    {
        if (OL_Probe((uint8_t)a))
            g_i2c_scan[(uint8_t)(a >> 3)] |= (uint8_t)(1u << (a & 7u));
    }
}

/* [DIAG] 屏没应答时: 打印扫描结果 + 引脚自检方波(死循环, 不返回) */
void OLED_FailDiag(void)
{
    uint8_t a, found = 0;

    printf("i2c_scan:");
    for (a = 0; a < 128; a++)
    {
        if (g_i2c_scan[(uint8_t)(a >> 3)] & (uint8_t)(1u << (a & 7u)))
        {
            printf(" 0x%02X", (unsigned)a);
            found = 1;
        }
    }
    if (!found) printf(" (empty)");
    printf("\r\n");

#if OLED_PIN_TEST_ON_FAIL
    printf("PIN TEST: PB8(SCL)/PB9(SDA) 1Hz 方波, 万用表量屏的 SCL/SDA 脚:\r\n");
    printf("  0<->3.3V 跳 = MCU->屏这段通(问题在屏); 死值 = 这段坏(板子/线)\r\n");
    OL_SDA_OutPP();
    for (;;)
    {
        OL_SCL_H(); OL_SDA_L(); delay_ms(500);
        OL_SCL_L(); OL_SDA_H(); delay_ms(500);
    }
#endif
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

/* 初始化序列: 逐条照抄官方规格书 QG-2864KLBEG01 Rev C 第29~30页
   ("Internal setting (Charge pump)" 分支, 即 4 脚 I2C 模块的用法)
     write_i(0xAE) 关显示        write_i(0x00) 列地址低
     write_i(0x10) 列地址高      write_i(0x40) 显示起始行
     write_i(0xB0) 页地址        write_i(0x81) write_i(0xCF) 对比度
     write_i(0xA1) 段重映射      write_i(0xA6) 正常/反色
     write_i(0xA8) write_i(0x3F) 多路复用 1/64
     write_i(0xC8) COM 扫描方向  write_i(0xD3) write_i(0x00) 显示偏移
     write_i(0xD5) write_i(0x80) 时钟分频
     write_i(0xD9) write_i(0xF1) 预充电
     write_i(0xDA) write_i(0x12) COM 引脚
     write_i(0xDB) write_i(0x30) VCOMH   <- 官方是 0x30, 之前写死 0x40
     write_i(0x8D) write_i(0x14) 电荷泵使能
     write_i(0xAF) 开显示
   官方时序还要求: 上电后 VDD/VCC 稳定 -> 延时 -> 发序列 -> 清屏 -> AF -> 延时100ms */
void OLED_Init(void)
{
    OL_SCL_H(); OL_SDA_H();
    delay_ms(100);              /* 等 VDD/VCC 稳定 */

    OLED_DiagScan();            /* [DIAG] 全总线扫描, 结果进 g_i2c_scan */
    OLED_AutoAddr();            /* 地址不对就换, 不用手工来回改 */

    OL_WCMD(0xAE);
    OL_WCMD(0x00);              /* set lower column address */
    OL_WCMD(0x10);              /* set higher column address */
    OL_WCMD(0x40);              /* set display start line */
    OL_WCMD(0xB0);              /* set page address */
    OL_WCMD(0x81); OL_WCMD(0xCF);   /* contrast, Rev C 指定 0xCF */
    OL_WCMD(0xA1);              /* set segment remap */
    OL_WCMD(0xA6);              /* normal display */
    OL_WCMD(0xA8); OL_WCMD(0x3F);   /* multiplex ratio, duty = 1/64 */
    OL_WCMD(0xC8);              /* COM scan direction */
    OL_WCMD(0xD3); OL_WCMD(0x00);   /* display offset */
    OL_WCMD(0xD5); OL_WCMD(OLED_CLK_DIV);
    OL_WCMD(0xD9); OL_WCMD(OLED_PRECHARGE);
    OL_WCMD(0xDA); OL_WCMD(0x12);
    OL_WCMD(0xDB); OL_WCMD(OLED_VCOMH);
    OL_WCMD(0x8D); OL_WCMD(OLED_CHARGE_PUMP);
    OL_WCMD(0xA4);              /* resume to RAM content display */
    OL_WCMD(0xAF);              /* display ON */
    delay_ms(100);              /* 官方要求 AF 后再稳定 100ms */

    OLED_Clear();

#if OLED_ALLON_TEST_ON_BOOT
    /* [DIAG] 全屏点亮 1.5s —— 屏硬件死活的第一判断依据:
       白闪 = 屏/升压/命令全通; 不闪(且 oled_ack=1) = 屏硬件坏 */
    {
        uint8_t i;
        OL_WCMD(0xA5);
        for (i = 0; i < 3; i++) delay_ms(500);
        OL_WCMD(0xA4);
    }
#endif
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
