#include "delay.h"
#include "stm32f1xx_hal.h"

/* ---------------------------------------------------------------------------
 * 2026-09-29 重写: 原实现直接操作 SysTick->LOAD/VAL/CTRL, 是颗定时炸弹。
 *
 * 原实现干了三件坏事:
 *   1) delay_init() 把 SysTick 时钟源切成 HCLK/8 (9MHz), 却不改 HAL 已算好的 LOAD
 *      -> HAL 的 1ms 心跳变成 8ms, HAL_GetTick() 慢 8 倍;
 *   2) 每次 delay_us() / delay_ms() 结束都执行 CTRL &= ~ENABLE 关掉计数器
 *      -> 期间 HAL_GetTick() 完全不涨;
 *   3) 上面这条 + HAL_I2C 的超时依赖 HAL_GetTick()
 *      -> 在 TIM3 中断里同时跑 MPU(软I2C, 内部全是 delay_us) 和 灰度(HAL I2C) 时,
 *         HAL 的 tick 被 delay_us 反复掐断, I2C 一旦出错就永远等不到超时。
 *      另外这也是 OLED 时好时坏的嫌疑对象(它同样用软 I2C + delay_us)。
 *
 * 现在改成用 Cortex-M3 的 DWT 周期计数器 (CYCCNT), 完全不碰 SysTick:
 *   - HAL_GetTick 不再被抢, HAL_I2C 超时正常;
 *   - 软 I2C (MPU PA6/PA7 / OLED PB8/PB9) 的时序依然准。
 * 主频 72MHz 时 1us = 72 个周期。
 * ------------------------------------------------------------------------- */

#define DWT_CYCCNTENA   (1UL << 0)
#define DEMCR_TRCENA    (1UL << 24)

/* 确保 CYCCNT 在跑: 万一谁忘了调 delay_init(), 也不会变成死等 */
static void DWT_Enable(void)
{
    if ((DWT->CTRL & DWT_CYCCNTENA) == 0UL)
    {
        CoreDebug->DEMCR |= DEMCR_TRCENA;
        DWT->CYCCNT       = 0UL;
        DWT->CTRL        |= DWT_CYCCNTENA;
    }
}

void delay_init(void)
{
    CoreDebug->DEMCR |= DEMCR_TRCENA;   /* 使能 DWT */
    DWT->CYCCNT       = 0UL;
    DWT->CTRL        |= DWT_CYCCNTENA;  /* 使能周期计数器 */
}

void delay_us(u32 nus)
{
    uint32_t start;
    uint32_t cycles;

    DWT_Enable();

    start  = DWT->CYCCNT;
    cycles = (uint32_t)nus * (SystemCoreClock / 1000000UL);

    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
        /* 忙等, 不碰任何硬件寄存器 */
    }
}

void delay_ms(u16 nms)
{
    while (nms--)
    {
        delay_us(1000);
    }
}
