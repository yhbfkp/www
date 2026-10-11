/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_oled.c
  * @brief      OLED SSD1306 128x64 驱动(软件 SPI / 软件 I2C 二选一) + 一屏模式/用时显示
  * @note       传输层由 car_config.h 的 OLED_USE_SPI 切换, 引脚在本文件顶部(见接线表)。
  *             这几个脚不走 CubeMX, 复位后是浮空输入, 必须 OL_PinInit() 配成推挽输出,
  *             否则写 ODR 根本不驱动引脚, 时钟/数据都出不来, 屏永远不亮。
  *             [I2C] 从机地址上电自动探测: 4 脚屏只有 0x3C / 0x3D 两种, 由模块背面电阻决定。
  *             [SPI] 没有地址也没有 ACK, 探不到"屏在不在"; 见 OLED_Init() 里的说明。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    初始化序列: 逐条照抄官方规格书 QG-2864KLBEG01 Rev C 第29~30页
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
      官方时序还要求: 上电后 VDD/VCC 稳定 -> 延时 -> 发序列 -> 清屏 -> AF -> 延时100ms
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#include "car_oled.h"
#include "car_config.h"
#include "oledfont.h"
#include "delay.h"
#include <stdio.h>

/* =====================================================================================
 * 传输层: 软件 SPI(默认) / 软件 I2C, 由 car_config.h 的 OLED_USE_SPI 切换
 *
 * 接线表(SPI):
 *   模块脚        接到板子上                  位置
 *   SCK  / D0  -> PB8                       H9.16 (原 OLED 座的 SCL 脚)
 *   MOSI / D1  -> PB9                       H9.17 (原 OLED 座的 SDA 脚)
 *   DC   / A0  -> PA5                       H8.11 (固件里配成上拉输入但从没读过的备用键)
 *   RES        -> PB1 或直接接 3.3V          H8.7  (接 3.3V 就把 OLED_RES_ENABLE 改 0)
 *   CS         -> GND  (单从机常低, SSD1306 允许, 不占 MCU 脚)
 *   VCC        -> 3.3V (H9.20), 别接 5V: SPI 是推挽输出, 5V 供电时逻辑高门限可能到 3.5V,
 *                 3.3V 驱动会不稳; 而且 PA5 不耐 5V。
 *   GND        -> GND
 *   旧 I2C 模块若还插在原座上, 建议拔掉(它的 4.7k 上拉会挂在 PB8/PB9 上)。
 *
 * ⚠️ 不要用这几只脚做 SPI: PB5 = 蜂鸣器(Q2 基极会把电平钳住, 还会一直叫);
 *    PB3/PB4/PA12/PA15 各有一颗 10µF 去抖电容(板载按键 SW2~SW5), 时钟/数据上去直接被
 *    电容吃掉, 连 DC/RES 翻转都要约 1.5ms —— 这正是不够用的原因, 也是原来那份
 *    OLED_SDA=PB4 / OLED_SCL=PB5 的老配置点不亮屏的原因。
 * ===================================================================================== */

/* car_config.h 的 OLED_TYPE 是总开关(0=I2C 单色 / 1=SPI 单色 / 2=ST7789 彩屏)。
   老代码里的 OLED_USE_SPI 分支全部保留, 这里把它按 OLED_TYPE 推出来:
   只要不是 I2C, 底层就走同一套软件 SPI 时序, 彩屏只是多一层"窗口 + 16 位色"。 */
#define OLED_USE_SPI        (OLED_TYPE != 0)

/* ---- 引脚(要换线就改这 12 行) ---- */
#define OL_PIN_SCK_PORT     GPIOB
#define OL_PIN_SCK          GPIO_PIN_8
#define OL_PIN_MOSI_PORT    GPIOB
#define OL_PIN_MOSI         GPIO_PIN_9
#define OL_PIN_DC_PORT      GPIOA
#define OL_PIN_DC           GPIO_PIN_5
#define OL_PIN_RES_PORT     GPIOB
#define OL_PIN_RES          GPIO_PIN_1
#define OL_PIN_SCL_PORT     GPIOB
#define OL_PIN_SCL          GPIO_PIN_8
#define OL_PIN_SDA_PORT     GPIOB
#define OL_PIN_SDA          GPIO_PIN_9

#if OLED_USE_SPI
#define OL_SCK_H()   HAL_GPIO_WritePin(OL_PIN_SCK_PORT,  OL_PIN_SCK,  GPIO_PIN_SET)
#define OL_SCK_L()   HAL_GPIO_WritePin(OL_PIN_SCK_PORT,  OL_PIN_SCK,  GPIO_PIN_RESET)
#define OL_MOSI_H()  HAL_GPIO_WritePin(OL_PIN_MOSI_PORT, OL_PIN_MOSI, GPIO_PIN_SET)
#define OL_MOSI_L()  HAL_GPIO_WritePin(OL_PIN_MOSI_PORT, OL_PIN_MOSI, GPIO_PIN_RESET)
#define OL_DC_H()    HAL_GPIO_WritePin(OL_PIN_DC_PORT,   OL_PIN_DC,   GPIO_PIN_SET)
#define OL_DC_L()    HAL_GPIO_WritePin(OL_PIN_DC_PORT,   OL_PIN_DC,   GPIO_PIN_RESET)
#if OLED_RES_ENABLE
#define OL_RES_H()   HAL_GPIO_WritePin(OL_PIN_RES_PORT,  OL_PIN_RES,  GPIO_PIN_SET)
#define OL_RES_L()   HAL_GPIO_WritePin(OL_PIN_RES_PORT,  OL_PIN_RES,  GPIO_PIN_RESET)
#endif
#else
#define OL_SCL_H()   HAL_GPIO_WritePin(OL_PIN_SCL_PORT,  OL_PIN_SCL,  GPIO_PIN_SET)
#define OL_SCL_L()   HAL_GPIO_WritePin(OL_PIN_SCL_PORT,  OL_PIN_SCL,  GPIO_PIN_RESET)
#define OL_SDA_H()   HAL_GPIO_WritePin(OL_PIN_SDA_PORT,  OL_PIN_SDA,  GPIO_PIN_SET)
#define OL_SDA_L()   HAL_GPIO_WritePin(OL_PIN_SDA_PORT,  OL_PIN_SDA,  GPIO_PIN_RESET)
#endif

/* 实际从机地址(I2C 模式用): 上电自动探测, 4脚屏背面电阻决定是 0x3C 还是 0x3D,
   不用再手工来回改。串口 "=== Car Ready ===" 那行会打印实际值。
   g_oled_ack: I2C 模式 = 两个地址有没有应答(0 = 硬件问题);
               SPI 模式 = 根本没有 ACK 可读, 初始化完直接置 1, 语义退化成"通信已初始化" ——
               不这么做的话 car_core.c 会以为屏坏了去跑引脚自检(死循环), 车就动不了了。 */
uint8_t g_oled_addr = OLED_ADDR;
uint8_t g_oled_ack  = 0;

#define OL_ADDR     ((uint8_t)(g_oled_addr << 1))

#if OLED_USE_SPI
/* ---------------- 软件 SPI(位翻转) ----------------
   用 SSD1306 4 线 SPI 的模式0: 数据在 SCK 上升沿被屏采样, MSB 在前。
   一次"传输"= 先建立 DC(0=命令 / 1=数据) -> 8 个 bit; CS 常低不用管。 */
/**
  * @brief          send one bit on software spi, sample on rising edge
  * @param[in]      b: 0 or 0x80
  * @retval         none
  */
/**
  * @brief          软件 SPI 发一个 bit: SCK 上升沿被屏采样, MSB 在前
  * @param[in]      b: 0 或 0x80
  * @retval         none
  */
static void OL_SpiBit(uint8_t b)
{
    OL_SCK_L();
    if (b) OL_MOSI_H(); else OL_MOSI_L();
#if OLED_SPI_DLY_US
    delay_us(OLED_SPI_DLY_US);
#endif
    OL_SCK_H();
#if OLED_SPI_DLY_US
    delay_us(OLED_SPI_DLY_US);
#endif
}

/**
  * @brief          send one byte on software spi
  * @param[in]      d: byte to send
  * @retval         none
  */
/**
  * @brief          软件 SPI 发一个字节(高位在前)
  * @param[in]      d: 要发送的字节
  * @retval         none
  */
static void OL_SpiByte(uint8_t d)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        OL_SpiBit((uint8_t)(d & 0x80));
        d <<= 1;
    }
}
#endif

#if OLED_TYPE == 2
/* =====================================================================================
 * ST7789 IPS 彩屏 (1.14 / 1.3 / 1.47 / 2.0 寸) —— 软件 SPI, 复用上面的 SCK/MOSI/DC/RES
 * 初始化序列照 逐飞 zf_device_ips114.c 抄(只把硬件 SPI 换成位翻转);
 * 面板尺寸和偏移在 car_config.h 的 LCD_PANEL_*, 驱动不用改。
 * BLK(背光) 直接接 3.3V —— 没有空闲脚给它, 也不需要调光。
 * 颜色是 RGB565: 0xF800 红 / 0x07E0 绿 / 0x001F 蓝 / 0xFFFF 白 / 0x0000 黑 / 0xFFE0 黄
 * ===================================================================================== */
#define LCD_BLACK   0x0000
#define LCD_WHITE   0xFFFF
#define LCD_RED     0xF800
#define LCD_GREEN   0x07E0
#define LCD_YELLOW  0xFFE0
#define LCD_CYAN    0x07FF

/**
  * @brief          write one byte with dc level
  * @param[in]      d: byte
  * @param[in]      is_data: 1 = data, 0 = command
  * @retval         none
  */
/**
  * @brief          写一个字节(先建立 DC): is_data=0 命令 / 1 数据
  * @param[in]      d: 要写的字节
  * @param[in]      is_data: 1=数据, 0=命令
  * @retval         none
  */
static void LCD_WrByte(uint8_t d, uint8_t is_data)
{
    if (is_data) OL_DC_H(); else OL_DC_L();
    OL_SpiByte(d);
}

static void LCD_WrCmd(uint8_t c) { LCD_WrByte(c, 0); }
static void LCD_WrDat(uint8_t d) { LCD_WrByte(d, 1); }

/**
  * @brief          write 16 bit value (high byte first), used by window setting
  * @param[in]      v: value
  * @retval         none
  */
/**
  * @brief          写 16 位(高字节在前): 列/行地址用它
  * @param[in]      v: 值
  * @retval         none
  */
static void LCD_WrDat16(uint16_t v)
{
    LCD_WrDat((uint8_t)(v >> 8));
    LCD_WrDat((uint8_t)(v & 0xFF));
}

static void LCD_WrPixel(uint16_t c)          //RGB565 像素, 高字节在前
{
    LCD_WrDat((uint8_t)(c >> 8));
    LCD_WrDat((uint8_t)(c & 0xFF));
}

/**
  * @brief          set display window, following writes fill this rect
  * @param[in]      x1,y1: top-left
  * @param[in]      x2,y2: bottom-right
  * @retval         none
  */
/**
  * @brief          设置显示窗口, 之后连续写就是往这个矩形里灌像素(左->右, 上->下)
  * @param[in]      x1,y1: 左上角
  * @param[in]      x2,y2: 右下角
  * @retval         none
  */
static void LCD_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_WrCmd(0x2A);                                            //列地址
    LCD_WrDat16((uint16_t)(x1 + LCD_PANEL_XOFF));
    LCD_WrDat16((uint16_t)(x2 + LCD_PANEL_XOFF));
    LCD_WrCmd(0x2B);                                            //行地址
    LCD_WrDat16((uint16_t)(y1 + LCD_PANEL_YOFF));
    LCD_WrDat16((uint16_t)(y2 + LCD_PANEL_YOFF));
    LCD_WrCmd(0x2C);                                            //开始写显存
}

/**
  * @brief          fill a rect with one color
  * @param[in]      x1,y1,x2,y2: rect
  * @param[in]      color: RGB565
  * @retval         none
  */
/**
  * @brief          用纯色填一个矩形(整屏清屏也走它)
  * @param[in]      x1,y1,x2,y2: 矩形范围
  * @param[in]      color: RGB565 颜色
  * @retval         none
  */
static void LCD_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint32_t n;

    if (x1 > x2 || y1 > y2) return;
    LCD_SetWindow(x1, y1, x2, y2);
    n = (uint32_t)(x2 - x1 + 1) * (uint32_t)(y2 - y1 + 1);
    while (n--) LCD_WrPixel(color);
}

/**
  * @brief          show one 8x16 ascii char in color
  * @param[in]      x,y: top-left
  * @param[in]      ch: ascii char
  * @param[in]      fg: pen color (RGB565)
  * @param[in]      bg: background color (RGB565)
  * @retval         none
  */
/**
  * @brief          显示一个 8x16 彩色 ASCII(用 oledfont.h 的 1608 点阵, 按行序整块写)
  * @param[in]      x,y: 左上角
  * @param[in]      ch: ASCII 字符
  * @param[in]      fg: 字色(RGB565)
  * @param[in]      bg: 底色(RGB565)
  * @retval         none
  */
static void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t ch, uint16_t fg, uint16_t bg)
{
    uint8_t  chr1, temp;
    uint16_t row, col;

    chr1 = (uint8_t)(ch - ' ');
    if (chr1 > 94) chr1 = 0;                    //字库外(空格以外的控制字符)按空格画

    LCD_SetWindow(x, y, (uint16_t)(x + 7), (uint16_t)(y + 15));
    for (row = 0; row < 16; row++)
    {
        for (col = 0; col < 8; col++)
        {
            temp = oled_asc2_1608[chr1][col * 2 + (row >> 3)];
            LCD_WrPixel((temp & (0x80u >> (row & 7))) ? fg : bg);
        }
    }
}

/**
  * @brief          show a string in color (8x16 font)
  * @param[in]      x,y: top-left
  * @param[in]      s: string point
  * @param[in]      fg: pen color
  * @param[in]      bg: background color
  * @retval         none
  */
/**
  * @brief          彩色显示字符串(8x16 点阵, 不可见字符跳过)
  * @param[in]      x,y: 左上角
  * @param[in]      s: 字符串
  * @param[in]      fg: 字色
  * @param[in]      bg: 底色
  * @retval         none
  */
static void LCD_ShowString16(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg)
{
    while (*s)
    {
        if (*s >= ' ' && *s <= '~')
        {
            LCD_ShowChar(x, y, (uint8_t)*s, fg, bg);
            x += 8;
        }
        s++;
    }
}

/**
  * @brief          show fixed digits unsigned number in color (8x16 font)
  * @param[in]      x,y: top-left
  * @param[in]      num: number
  * @param[in]      len: digit number
  * @param[in]      fg: pen color
  * @param[in]      bg: background color
  * @retval         none
  */
/**
  * @brief          彩色显示固定位数无符号数(8x16 点阵)
  * @param[in]      x,y: 左上角
  * @param[in]      num: 数值
  * @param[in]      len: 位数
  * @param[in]      fg: 字色
  * @param[in]      bg: 底色
  * @retval         none
  */
static void LCD_ShowNum16(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fg, uint16_t bg)
{
    uint8_t  t;
    uint32_t div = 1;

    for (t = 1; t < len; t++) div *= 10;
    for (t = 0; t < len; t++)
    {
        LCD_ShowChar((uint16_t)(x + 8 * t), y, (uint8_t)((num / div) % 10 + '0'), fg, bg);
        div /= 10;
    }
}

#if LCD_BRINGUP_TEST
/**
  * @brief          [DIAG] bring-up test: fill the controller max range with 4 colors
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          [DIAG] 上电四色自检: 用控制器最大范围(0~239)铺 红/绿/蓝/白
  *                 故意不加面板偏移, 让面板自己裁剪 —— 只要通信通, 总能看到一块颜色
  * @param[in]      none
  * @retval         none
  */
static void LCD_Bringup(void)
{
    static const uint16_t cols[4] = { 0xF800, 0x07E0, 0x001F, 0xFFFF };   //红 绿 蓝 白
    uint8_t  i;
    uint32_t n;

    for (i = 0; i < 4; i++)
    {
        LCD_WrCmd(0x2A); LCD_WrDat16(0); LCD_WrDat16(239);     //列 0~239, 不加偏移
        LCD_WrCmd(0x2B); LCD_WrDat16(0); LCD_WrDat16(239);     //行 0~239
        LCD_WrCmd(0x2C);
        n = 240UL * 240UL;
        while (n--) LCD_WrPixel(cols[i]);
        delay_ms(150);                   //[DIAG] 缩短: 定位期间减少等待
    }
}
#endif      /* LCD_BRINGUP_TEST */

#endif      /* OLED_TYPE == 2 */

#if !OLED_USE_SPI
/* SDA 方向切换: 读 ACK 时必须把 SDA 放开(输入上拉), 否则推挽输出的 MCU
   会把 SDA 硬拉高, 从机根本拉不低, 永远读不到应答 */
/**
  * @brief          set sda pin to push-pull output
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          把 SDA(PB9) 切成推挽输出
  * @param[in]      none
  * @retval         none
  */
static void OL_SDA_OutPP(void)
{
    GPIO_InitTypeDef g = {0};

    g.Pin   = GPIO_PIN_9;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
}

/**
  * @brief          set sda pin to input with pull-up
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          把 SDA(PB9) 切成上拉输入(读 ACK 用)
  * @param[in]      none
  * @retval         none
  */
static void OL_SDA_InPU(void)
{
    GPIO_InitTypeDef g = {0};

    g.Pin   = GPIO_PIN_9;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
}
#endif      /* !OLED_USE_SPI: 以上两个是 I2C 专用的 SDA 方向切换 */

/* 引脚初始化: PB8=SCL 推挽输出, PB9=SDA 推挽输出(读ACK时再切成上拉输入)。
   ⚠️ 这一步必须做 —— 这两个脚不走 CubeMX: gpio.c 里没配 PB8/PB9,
   复位后默认是浮空输入, 不初始化就写 ODR 是不会驱动引脚的,
   SCL 出不来时钟, 屏自然永远不亮(之前"ACK能读到但屏不亮"的元凶之一)。 */
/**
  * @brief          init scl / sda pins as push-pull output
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          初始化 PB8(SCL) / PB9(SDA) 为推挽输出并开 GPIOB 时钟
  * @param[in]      none
  * @retval         none
  */
static void OL_PinInit(void)
{
    GPIO_InitTypeDef g = {0};

    /* 这几个脚不走 CubeMX, 复位后是浮空输入: 不配成推挽输出, 写 ODR 根本不驱动引脚,
       时钟/数据都出不来, 屏永远不亮(这是之前"ACK 能读到但屏不亮"的元凶之一)。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;

#if OLED_USE_SPI
    g.Pin = OL_PIN_SCK;   HAL_GPIO_Init(OL_PIN_SCK_PORT,  &g);
    g.Pin = OL_PIN_MOSI;  HAL_GPIO_Init(OL_PIN_MOSI_PORT, &g);
    g.Pin = OL_PIN_DC;    HAL_GPIO_Init(OL_PIN_DC_PORT,   &g);
    OL_SCK_L();                 //SPI 模式0: 空闲时钟为低
    OL_DC_H();                  //DC 先放"数据", 免得初始化的杂波被当成命令
#if OLED_RES_ENABLE
    g.Pin = OL_PIN_RES;   HAL_GPIO_Init(OL_PIN_RES_PORT,  &g);
    OL_RES_H();                 //复位脚平时保持高
#endif
#else
    g.Pin = OL_PIN_SCL;   HAL_GPIO_Init(OL_PIN_SCL_PORT,  &g);
    g.Pin = OL_PIN_SDA;   HAL_GPIO_Init(OL_PIN_SDA_PORT,  &g);
    OL_SCL_H();
    OL_SDA_H();
#endif
}

#if !OLED_USE_SPI
/**
  * @brief          i2c start condition
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          软件 I2C 起始条件
  * @param[in]      none
  * @retval         none
  */
static void OL_Start(void)
{
    OL_SDA_H();
    OL_SCL_H();
    delay_us(OLED_I2C_DLY_US);
    OL_SDA_L();
    delay_us(OLED_I2C_DLY_US);
    OL_SCL_L();
    delay_us(OLED_I2C_DLY_US);
}

/**
  * @brief          i2c stop condition
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          软件 I2C 停止条件
  * @param[in]      none
  * @retval         none
  */
static void OL_Stop(void)
{
    OL_SCL_L();
    OL_SDA_L();
    delay_us(OLED_I2C_DLY_US);
    OL_SCL_H();
    delay_us(OLED_I2C_DLY_US);
    OL_SDA_H();
    delay_us(OLED_I2C_DLY_US);
}

/**
  * @brief          send one byte, msi-first, ignore ack
  * @param[in]      d: byte to send
  * @retval         none
  */
/**
  * @brief          软件 I2C 发一个字节(高位在前), 第9个时钟不读应答
  * @param[in]      d: 要发送的字节
  * @retval         none
  */
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
    //第9个时钟(ACK), 不读应答
    OL_SCL_L();
    OL_SDA_H();
    delay_us(OLED_I2C_DLY_US);
    OL_SCL_H();
    delay_us(OLED_I2C_DLY_US);
    OL_SCL_L();
}

/* 发一个字节并读第9位 ACK: 返回 1 = 有从机应答 */
/**
  * @brief          send one byte and read ack
  * @param[in]      d: byte to send
  * @retval         1: slave ack, 0: no ack
  */
/**
  * @brief          软件 I2C 发一个字节并读第9位 ACK
  * @param[in]      d: 要发送的字节
  * @retval         1 = 有从机应答, 0 = 没应答
  */
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
    //第9个时钟: 放开 SDA, 读从机是否拉低
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
/**
  * @brief          probe one 7-bit i2c address
  * @param[in]      a7: 7-bit address
  * @retval         1: device ack, 0: no device
  */
/**
  * @brief          探测某个 7 位地址有没有设备应答
  * @param[in]      a7: 7 位地址
  * @retval         1 = 有应答, 0 = 没应答
  */
static uint8_t OL_Probe(uint8_t a7)
{
    uint8_t ack;

    OL_SDA_OutPP();
    OL_SCL_H();
    OL_SDA_H();
    OL_Start();
    ack = OL_SendByteAck((uint8_t)(a7 << 1));
    OL_Stop();
    return ack;
}

/* 自动识别地址: 先试 car_config.h 里的 OLED_ADDR, 不通再试另一个常见地址
   (4脚 SSD1306 只有 0x3C / 0x3D 两种可能, 由模块背面电阻决定) */
/**
  * @brief          auto detect oled 7-bit address
  * @param[in]      none
  * @retval         1: found a device, 0: no device on 0x3C / 0x3D
  */
/**
  * @brief          自动识别屏地址: 先试 OLED_ADDR, 再试另一个(0x3C / 0x3D 二选一)
  * @param[in]      none
  * @retval         1 = 探测到设备, 0 = 两个地址都没应答
  */
static uint8_t OLED_AutoAddr(void)
{
    uint8_t alt = (OLED_ADDR == 0x3C) ? 0x3D : 0x3C;

    g_oled_ack = OL_Probe(OLED_ADDR);
    if (g_oled_ack) { g_oled_addr = OLED_ADDR; return 1; }

    g_oled_ack = OL_Probe(alt);
    if (g_oled_ack) { g_oled_addr = alt;       return 1; }

    g_oled_addr = OLED_ADDR;    //都没应答: 保持默认值, 后续命令照发(无害)
    return 0;
}

/* [DIAG] 扫 0x00~0x7F 全部地址, 结果按位存进 g_i2c_scan:
     只有 1~2 个 bit 置位  -> 真实设备, 通信没问题
     一个 bit 都没有       -> 总线是空的(或那段线有问题)
   拿 MPU6050(0x68)/灰度(0x40) 挂上去扫得到 -> 证明 MCU 这侧完全正常。 */
uint8_t g_i2c_scan[16];

/**
  * @brief          scan all 7-bit i2c address and store result into g_i2c_scan
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          [DIAG] 扫 0x00~0x7F 全部地址, 结果按位存进 g_i2c_scan
  * @param[in]      none
  * @retval         none
  */
static void OLED_DiagScan(void)
{
    uint8_t a;

    for (a = 0; a < 16; a++)
        g_i2c_scan[a] = 0;
    for (a = 0; a < 128; a++)
    {
        if (OL_Probe((uint8_t)a))
            g_i2c_scan[(uint8_t)(a >> 3)] |= (uint8_t)(1u << (a & 7u));
    }
}
#endif      /* !OLED_USE_SPI: 以上是软件 I2C 专用(起停/收发/ACK/探测/地址识别/扫描) */

/* [DIAG] 屏没应答时: 打印扫描结果 + 引脚自检方波(死循环, 不返回) */
/**
  * @brief          print scan result and output 1Hz square wave, never return
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          [DIAG] 屏没应答时调用: 打印扫描结果 + PB8/PB9 输出方波, 死循环不返回
  * @param[in]      none
  * @retval         none
  */
#if OLED_PIN_TEST_ON_FAIL && OLED_USE_SPI
/**
  * @brief          [DIAG] drive one of the 4 spi pins, used by the pin by pin cable test
  * @param[in]      p: 0=SCK 1=MOSI 2=DC 3=RES
  * @param[in]      lv: 1 = high, 0 = low
  * @retval         none
  */
/**
  * @brief          [DIAG] 逐脚自检用: 拉高/拉低第 p 根线(p: 0=SCK 1=MOSI 2=DC 3=RES)
  * @param[in]      p: 线序号
  * @param[in]      lv: 1=高, 0=低
  * @retval         none
  */
static void OL_TestPin(uint8_t p, uint8_t lv)
{
    switch (p)
    {
        case 0: if (lv) OL_SCK_H();  else OL_SCK_L();  break;
        case 1: if (lv) OL_MOSI_H(); else OL_MOSI_L(); break;
        case 2: if (lv) OL_DC_H();   else OL_DC_L();   break;
        default:
#if OLED_RES_ENABLE
            if (lv) OL_RES_H(); else OL_RES_L();
#else
            (void)lv;
#endif
            break;
    }
}
#endif

void OLED_FailDiag(void)
{
#if !OLED_USE_SPI
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
#else
    /* SPI 没有地址可扫: 只能靠下面这个引脚方波 + 万用表量线 */
    printf("spi mode: no address/ack to probe (check SCK/MOSI/DC/RES + 3.3V/GND)\r\n");
#endif

#if OLED_PIN_TEST_ON_FAIL
    /* 串口只能出 ASCII: 源码是 UTF-8, 而终端多半按 GBK 解 -> 中文必然乱码, 所以这里用英文 */
    printf("PIN TEST (pin by pin): buzzer beeps N times, then pin N toggles 1Hz for 4s, loops\r\n");
    printf("  1 beep=SCK(PB8)  2=MOSI(PB9)  3=DC(PA5)  4=RES(PB1)\r\n");
    printf("  meter: black lead to GND, red on the matching OLED pin -> 0<->3.3V = wire OK\r\n");
    OL_PinInit();               //先把引脚配成推挽输出, 否则方波根本出不来
#if OLED_USE_SPI
    {
        uint8_t p, b;

        OL_TestPin(0, 0); OL_TestPin(1, 0); OL_TestPin(2, 0); OL_TestPin(3, 0);
        for (;;)
        {
            for (p = 0; p < 4; p++)
            {
                /* 叫 p+1 声: 标出现在轮到第几根线 */
                for (b = 0; b <= p; b++)
                {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   //蜂鸣器 PB5(高电平响)
                    delay_ms(80);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
                    delay_ms(160);
                }
                delay_ms(400);

                /* 只让第 p 根线翻, 其余三根压低, 免得万用表看错 */
                OL_TestPin((uint8_t)((p + 1) & 3u), 0);
                OL_TestPin((uint8_t)((p + 2) & 3u), 0);
                OL_TestPin((uint8_t)((p + 3) & 3u), 0);

                for (b = 0; b < 4; b++)         //4 秒 = 4 个 1Hz 周期
                {
                    OL_TestPin(p, 1);
                    delay_ms(500);
                    OL_TestPin(p, 0);
                    delay_ms(500);
                }
            }
        }
    }
#else
    OL_SDA_OutPP();
    for (;;)
    {
        OL_SCL_H();
        OL_SDA_L();
        delay_ms(500);
        OL_SCL_L();
        OL_SDA_H();
        delay_ms(500);
    }
#endif
#endif
}

/**
  * @brief          write one byte to oled (command or data)
  * @param[in]      dat: byte to write
  * @param[in]      is_data: 1 = data, 0 = command
  * @retval         none
  */
/**
  * @brief          给屏写一个字节: is_data=1 写数据, is_data=0 写命令
  * @param[in]      dat: 要写的字节
  * @param[in]      is_data: 1 = 数据, 0 = 命令
  * @retval         none
  */
#if OLED_TYPE != 2
static void OL_Write(uint8_t dat, uint8_t is_data)
{
#if OLED_USE_SPI
    /* SPI: 建立 DC 之后直接出 8 个 bit, 没有地址/控制字节/ACK 那一套 */
    if (is_data) OL_DC_H(); else OL_DC_L();
    OL_SpiByte(dat);
#else
    OL_Start();
    OL_SendByte(OL_ADDR);
    OL_SendByte(is_data ? 0x40 : 0x00);
    OL_SendByte(dat);
    OL_Stop();
#endif
}

#define OL_WCMD(d)  OL_Write((d), 0)
#define OL_WDAT(d)  OL_Write((d), 1)

/* ---------------- 显存(仅单色屏: 1 位/像素的页格式) ---------------- */
static uint8_t s_gram[128][8];

/**
  * @brief          set one point in gram
  * @param[in]      x: point x (0..127)
  * @param[in]      y: point y (0..63)
  * @retval         none
  */
/**
  * @brief          在显存里点亮一个点(越界忽略)
  * @param[in]      x: 点 x 坐标 (0..127)
  * @param[in]      y: 点 y 坐标 (0..63)
  * @retval         none
  */
static void OLED_DrawPoint(uint8_t x, uint8_t y)
{
    if (x < 128 && y < 64) s_gram[x][y / 8] |= (uint8_t)(1 << (y % 8));
}

/**
  * @brief          clear one point in gram
  * @param[in]      x: point x (0..127)
  * @param[in]      y: point y (0..63)
  * @retval         none
  */
/**
  * @brief          在显存里擦掉一个点(越界忽略)
  * @param[in]      x: 点 x 坐标 (0..127)
  * @param[in]      y: 点 y 坐标 (0..63)
  * @retval         none
  */
static void OLED_ClearPoint(uint8_t x, uint8_t y)
{
    if (x < 128 && y < 64) s_gram[x][y / 8] &= (uint8_t)~(1 << (y % 8));
}

/**
  * @brief          refresh gram to screen, 8 pages
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          把显存整屏刷到屏上(8 页, 每页 128 字节)
  * @param[in]      none
  * @retval         none
  */
void OLED_Refresh(void)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        OL_WCMD((uint8_t)(0xB0 | i));
        OL_WCMD(0x00);
        OL_WCMD(0x10);

#if OLED_USE_SPI
        /* SPI: DC 置 1 之后整页 128 字节连续打出去(中途不用再动 DC), 比 I2C 快得多 */
        OL_DC_H();
        for (n = 0; n < 128; n++)
            OL_SpiByte(s_gram[n][i]);
#else
        OL_Start();
        OL_SendByte(OL_ADDR);
        OL_SendByte(0x40);
        for (n = 0; n < 128; n++)
            OL_SendByte(s_gram[n][i]);
        OL_Stop();
#endif
    }
}
#endif      /* OLED_TYPE != 2: 以上(显存 + 画点 + 刷屏)都是单色屏专用, 彩屏直接写屏 */

/**
  * @brief          clear gram and refresh
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          清显存并刷屏
  * @param[in]      none
  * @retval         none
  */
void OLED_Clear(void)
{
#if OLED_TYPE == 2
    LCD_Fill(0, 0, (uint16_t)(LCD_PANEL_W - 1), (uint16_t)(LCD_PANEL_H - 1), LCD_BLACK);
#else
    uint8_t i, n;

    for (i = 0; i < 8; i++)
        for (n = 0; n < 128; n++)
            s_gram[n][i] = 0;
    OLED_Refresh();
#endif
}

#if OLED_TYPE != 2
/**
  * @brief          show one ascii char, size 12 / 16
  * @param[in]      x: start x
  * @param[in]      y: start y
  * @param[in]      chr: ascii char
  * @param[in]      size: font height
  * @retval         none
  */
/**
  * @brief          显示一个 ASCII 字符(字库只支持 12 / 16)
  * @param[in]      x: 起始 x
  * @param[in]      y: 起始 y
  * @param[in]      chr: ASCII 字符
  * @param[in]      size: 字号高度
  * @retval         none
  */
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
#endif      /* OLED_TYPE != 2: 单色屏的 12/16 点阵字符渲染 */

/**
  * @brief          show ascii string
  * @param[in]      x: start x
  * @param[in]      y: start y
  * @param[in]      s: ascii string point
  * @param[in]      size: font height
  * @retval         none
  */
/**
  * @brief          显示 ASCII 字符串(不可见字符跳过, 到右边自动换行)
  * @param[in]      x: 起始 x
  * @param[in]      y: 起始 y
  * @param[in]      s: ASCII 字符串指针
  * @param[in]      size: 字号高度
  * @retval         none
  */
void OLED_ShowString(uint8_t x, uint8_t y, const char *s, uint8_t size)
{
#if OLED_TYPE == 2
    (void)size;                                 //彩屏固定 8x16 点阵
    LCD_ShowString16(x, y, s, LCD_WHITE, LCD_BLACK);
#else
    while (*s)
    {
        if (*s < ' ' || *s > '~') { s++; continue; }
        OLED_ShowChar(x, y, (uint8_t)*s, size);
        x += size / 2;
        if (x > 128 - size) { x = 0; y += size; }
        s++;
    }
#endif
}

/**
  * @brief          show unsigned number with fixed digits
  * @param[in]      x: start x
  * @param[in]      y: start y
  * @param[in]      num: number to show
  * @param[in]      len: digit number
  * @param[in]      size: font height
  * @retval         none
  */
/**
  * @brief          显示固定位数的无符号数(高位不足补 0)
  * @param[in]      x: 起始 x
  * @param[in]      y: 起始 y
  * @param[in]      num: 要显示的数
  * @param[in]      len: 显示位数
  * @param[in]      size: 字号高度
  * @retval         none
  */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
#if OLED_TYPE == 2
    (void)size;                                 //彩屏固定 8x16 点阵
    LCD_ShowNum16(x, y, num, len, LCD_WHITE, LCD_BLACK);
#else
    uint8_t  t;
    uint32_t div = 1;

    for (t = 1; t < len; t++)
        div *= 10;
    for (t = 0; t < len; t++)
    {
        OLED_ShowChar((uint8_t)(x + (size / 2) * t), y, (uint8_t)((num / div) % 10 + '0'), size);
        div /= 10;
    }
#endif
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
/**
  * @brief          oled init: pins, bus scan, auto address, init sequence, clear
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          屏初始化: 引脚 -> 总线扫描 -> 地址识别 -> 官方初始化序列 -> 清屏
  * @param[in]      none
  * @retval         none
  */
#if OLED_TYPE == 2
#endif

void OLED_Init(void)
{
#if OLED_TYPE == 2
    /* ---------------- ST7789 IPS 彩屏 ---------------- */
    OL_PinInit();                       //SCK/MOSI/DC/RES 推挽输出
#if OLED_RES_ENABLE
    OL_RES_L();                         //复位脉冲: 低 5ms -> 拉高, 再等 120ms
    delay_ms(5);
    OL_RES_H();
#endif
    delay_ms(120);

    LCD_WrCmd(0x11); delay_ms(120);     //退出睡眠
    LCD_WrCmd(0x36); LCD_WrDat(0x00);   //MADCTL 竖屏(画面反了就在 0xA0/0x00 之间换; 红蓝反了再 +0x08)
    LCD_WrCmd(0x3A); LCD_WrDat(0x05);   //16bit/pixel = RGB565
    LCD_WrCmd(0xB2); LCD_WrDat(0x0C); LCD_WrDat(0x0C); LCD_WrDat(0x00); LCD_WrDat(0x33); LCD_WrDat(0x33);
    LCD_WrCmd(0xB7); LCD_WrDat(0x35);
    LCD_WrCmd(0xBB); LCD_WrDat(0x37);
    LCD_WrCmd(0xC0); LCD_WrDat(0x2C);
    LCD_WrCmd(0xC2); LCD_WrDat(0x01);
    LCD_WrCmd(0xC3); LCD_WrDat(0x12);
    LCD_WrCmd(0xC4); LCD_WrDat(0x20);
    LCD_WrCmd(0xC6); LCD_WrDat(0x0F);
    LCD_WrCmd(0xD0); LCD_WrDat(0xA4); LCD_WrDat(0xA1);
    LCD_WrCmd(0xE0);                    //正极性伽马
    LCD_WrDat(0xD0); LCD_WrDat(0x04); LCD_WrDat(0x0D); LCD_WrDat(0x11); LCD_WrDat(0x13);
    LCD_WrDat(0x2B); LCD_WrDat(0x3F); LCD_WrDat(0x54); LCD_WrDat(0x4C); LCD_WrDat(0x18);
    LCD_WrDat(0x0D); LCD_WrDat(0x0B); LCD_WrDat(0x1F); LCD_WrDat(0x23);
    LCD_WrCmd(0xE1);                    //负极性伽马
    LCD_WrDat(0xD0); LCD_WrDat(0x04); LCD_WrDat(0x0C); LCD_WrDat(0x11); LCD_WrDat(0x13);
    LCD_WrDat(0x2C); LCD_WrDat(0x3F); LCD_WrDat(0x44); LCD_WrDat(0x51); LCD_WrDat(0x2F);
    LCD_WrDat(0x1F); LCD_WrDat(0x1F); LCD_WrDat(0x20); LCD_WrDat(0x23);
    LCD_WrCmd(0x21);                    //反显(IPS 不反显会偏色)
    LCD_WrCmd(0x29);                    //开显示

#if LCD_BRINGUP_TEST
    LCD_Bringup();                      //[DIAG] 四色轮播: 分辨"通信不通"还是"尺寸/偏移不对"
#endif

    g_oled_addr = OLED_ADDR;
    g_oled_ack  = 1;                    //彩屏同样没有 ACK 可探, 只能按"在线"处理

#if OLED_ALLON_TEST_ON_BOOT
    /* [DIAG] 全屏白闪 1.5s —— 屏/背光/供电活没活的第一判据 */
    LCD_Fill(0, 0, (uint16_t)(LCD_PANEL_W - 1), (uint16_t)(LCD_PANEL_H - 1), LCD_WHITE);
    delay_ms(300);
#endif
    LCD_Fill(0, 0, (uint16_t)(LCD_PANEL_W - 1), (uint16_t)(LCD_PANEL_H - 1), LCD_BLACK);
    return;
#else
    /* ---------------- SSD1306 单色屏 (软 I2C 或 软 SPI) ---------------- */
    OL_PinInit();               //SCK/MOSI/DC(+RES) 推挽输出, 必须在发命令之前

#if OLED_USE_SPI
    /* SPI 既没有地址也没有 ACK, 探不到"屏在不在", 只能按"屏在线"处理 ——
       初始化完把 g_oled_ack 置 1; 否则 car_core.c 会以为屏坏了去跑引脚自检
       (那个函数是死循环, 车就动不了了)。乱接线的现象会变成"屏不亮但车照跑"。 */
#if OLED_RES_ENABLE
    OL_RES_L();                 //复位脉冲: 低 20ms 再拉高
    delay_ms(20);
    OL_RES_H();
#endif
    delay_ms(100);              //等 VDD/VCC 稳定 + 模块内部升压起来
    g_oled_addr = OLED_ADDR;    //SPI 模式地址无意义, 保持默认值只为串口打印
    g_oled_ack  = 1;
#else
    delay_ms(100);              //等 VDD/VCC 稳定

    OLED_DiagScan();            //[DIAG] 全总线扫描, 结果进 g_i2c_scan
    OLED_AutoAddr();            //地址不对就换, 不用手工来回改
#endif

    OL_WCMD(0xAE);
    OL_WCMD(0x00);              //set lower column address
    OL_WCMD(0x10);              //set higher column address
    OL_WCMD(0x40);              //set display start line
    OL_WCMD(0xB0);              //set page address
    OL_WCMD(0x81); OL_WCMD(0xCF);   //contrast, Rev C 指定 0xCF
    OL_WCMD(0xA1);              //set segment remap
    OL_WCMD(0xA6);              //normal display
    OL_WCMD(0xA8); OL_WCMD(0x3F);   //multiplex ratio, duty = 1/64
    OL_WCMD(0xC8);              //COM scan direction
    OL_WCMD(0xD3); OL_WCMD(0x00);   //display offset
    OL_WCMD(0xD5); OL_WCMD(OLED_CLK_DIV);
    OL_WCMD(0xD9); OL_WCMD(OLED_PRECHARGE);
    OL_WCMD(0xDA); OL_WCMD(0x12);
    OL_WCMD(0xDB); OL_WCMD(OLED_VCOMH);
    OL_WCMD(0x8D); OL_WCMD(OLED_CHARGE_PUMP);
    OL_WCMD(0xA4);              //resume to RAM content display
    OL_WCMD(0xAF);              //display ON
    delay_ms(100);              //官方要求 AF 后再稳定 100ms

    OLED_Clear();

#if OLED_ALLON_TEST_ON_BOOT
    /* [DIAG] 全屏点亮 1.5s —— 屏硬件死活的第一判断依据:
       白闪 = 屏/升压/命令全通; 不闪(且 oled_ack=1) = 屏硬件坏 */
    {
        uint8_t i;
        OL_WCMD(0xA5);
        for (i = 0; i < 3; i++)
            delay_ms(500);
        OL_WCMD(0xA4);
    }
#endif
#endif      /* OLED_TYPE == 2 的彩屏分支到此结束 */
}

/* ---------------- 一屏状态: 模式 + 用时 ---------------- */
/**
  * @brief          show one screen: mode number, mode name and run time
  * @param[in]      mode: mode number
  * @param[in]      name: mode name string point
  * @param[in]      running: 0=idle 1=run 2=done 3=obs
  * @param[in]      time_ms: run time (ms)
  * @retval         none
  */
/**
  * @brief          一屏状态显示: 第一行 = 模式号 + 模式名 + 状态, 第二行 = 用时 秒.十分秒
  * @param[in]      mode: 模式号
  * @param[in]      name: 模式名指针
  * @param[in]      running: 0=待机 1=运行中 2=完成 3=停
  * @param[in]      time_ms: 本轮计时 (ms)
  * @retval         none
  */
void OLED_ShowStatus(uint8_t mode, const char *name, uint8_t running, uint32_t time_ms, uint8_t dbg)
{
    char     buf[24];
    uint32_t sec  = time_ms / 1000;
    uint32_t dsec = (time_ms % 1000) / 100;

#if OLED_TYPE == 2
    /* ---------------- 彩屏版: 黑底 + 彩色文字 ---------------- */
    {
        const char *st;
        uint16_t    stc;

        switch (running)
        {
            case 1:  st = "RUN";   stc = LCD_GREEN;  break;
            case 2:  st = "DONE";  stc = LCD_YELLOW; break;
            case 3:  st = "stop!"; stc = LCD_RED;    break;
            default: st = "RDY";   stc = LCD_CYAN;   break;
        }

        LCD_Fill(0, 0, (uint16_t)(LCD_PANEL_W - 1), 83, LCD_BLACK);     //只清上部, 少刷点

        snprintf(buf, sizeof(buf), "M%d %s", (int)mode, name ? name : "");
        LCD_ShowString16(6, 6, buf, LCD_YELLOW, LCD_BLACK);
        LCD_ShowString16(6, 26, st, stc, LCD_BLACK);
        snprintf(buf, sizeof(buf), "TIME %lu.%lu s", (unsigned long)sec, (unsigned long)dsec);
        LCD_ShowString16(6, 46, buf, LCD_WHITE, LCD_BLACK);

        if (dbg)
        {
            snprintf(buf, sizeof(buf), "DBG T%d W%d L%d",
                     (dbg & 0x01u) ? 1 : 0, (dbg & 0x02u) ? 1 : 0, (dbg & 0x04u) ? 1 : 0);
            LCD_ShowString16(6, 66, buf, LCD_RED, LCD_BLACK);
        }
    }
#else
    //清显存(不刷屏, 后面统一刷一次)
    {
        uint8_t i, n;
        for (i = 0; i < 8; i++)
            for (n = 0; n < 128; n++)
                s_gram[n][i] = 0;
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

    //第二行: 用时 秒.十分秒
    snprintf(buf, sizeof(buf), "TIME %lu.%lu s", (unsigned long)sec, (unsigned long)dsec);
    OLED_ShowString(0, 24, buf, 16);

    /* 第三行(只在"调试/整定状态还生效"时出现):
       这几个都是 RAM 变量, 在调试器里写过就一直在, 只有复位/断电才回默认值;
       忘了复位会表现成"只有一个轮子转 / 不循迹 / 开环固定占空比", 很像硬件坏了。
       T=整定 g_pid_tune   W=单轮 g_pid_wheel   L=开环 g_open_loop */
    if (dbg)
    {
        snprintf(buf, sizeof(buf), "DBG T%d W%d L%d",
                 (dbg & 0x01u) ? 1 : 0, (dbg & 0x02u) ? 1 : 0, (dbg & 0x04u) ? 1 : 0);
        OLED_ShowString(0, 48, buf, 12);
    }

    OLED_Refresh();
#endif      /* OLED_TYPE == 2 */
}
