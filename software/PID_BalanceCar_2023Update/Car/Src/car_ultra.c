/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_ultra.c
  * @brief      HC-SR04 超声波测距实现: 触发脉冲 + TIM3_CH3 输入捕获
  * @note       TRIG = PB1(推挽输出), ECHO = PB0 = TIM3_CH3 输入捕获
  *             捕获量由 tim.c 的 TIM3CH3_CAPTURE_STA / _VAL 提供, 本文件只读
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    时序(沿用 car-main 原实现, 一个数都没改):
      1) TRIG 拉高 15us -> 拉低, 模块发出 8 个 40kHz 脉冲
      2) 回波高电平时间由 TIM3_CH3 输入捕获测量:
         TIM3 Period=9999, 每次溢出 = 10000 个计数(bit6 记溢出次数, bit7 记完成)
      3) 距离 mm = 计数 * 170 / 1000  (1 个计数 = 1us, 声速换算 us * 0.17)
      4) 读完把 STA 清零, 打开下一次捕获

    CAR_USE_ULTRA=0 时整个函数体不编译, 直接返回 0(超声波座没走线, 默认关)。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#include "car_ultra.h"
#include "car_config.h"
#include "delay.h"

/* tim.c 里超声波输入捕获用的全局量 */
extern u16 TIM3CH3_CAPTURE_STA;
extern u16 TIM3CH3_CAPTURE_VAL;

volatile uint16_t g_ultra_mm = 0;

/**
  * @brief          init ultrasonic trigger pin and last distance
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          超声波初始化: TRIG(PB1) 拉低, 上次距离清零
  * @param[in]      none
  * @retval         none
  */
void Ultra_Init(void)
{
    PBout(1) = 0;
    g_ultra_mm = 0;
}

/**
  * @brief          send one trigger pulse and return the distance in mm
  * @param[in]      none
  * @retval         distance in mm, 0 = no valid capture
  */
/**
  * @brief          发一次触发脉冲并返回距离(mm)
  * @param[in]      none
  * @retval         距离(mm), 0=本次没测到
  */
uint16_t Ultra_GetMM(void)
{
#if CAR_USE_ULTRA
    uint32_t t;
    static uint8_t s_miss = 0;                   //连续没测到回波的次数

    /* 发 15us 触发脉冲。注意: 回波要等到下一个 100ms 周期才读得到,
       所以这里返回的其实是上一次触发的结果(天然滞后一拍)。 */
    PBout(1) = 1;
    delay_us(15);
    PBout(1) = 0;

    if (TIM3CH3_CAPTURE_STA & 0x80)              //捕获完成
    {
        t  = (uint32_t)(TIM3CH3_CAPTURE_STA & 0x3F) * 10000UL;  //TIM3 Period=9999 -> 每次溢出 10000 计数
        t += TIM3CH3_CAPTURE_VAL;
        t  = t * 170UL / 1000UL;                 //1个计数=1us, 距离mm = us * 0.17
        TIM3CH3_CAPTURE_STA = 0;                 //开启下一次捕获
        g_ultra_mm = (uint16_t)t;
        s_miss = 0;
    }
    else
    {
        if (s_miss < ULTRA_MISS_MAX)
        {
            /* 单次丢回波不能立刻当"没测到": car_task.c 把 0 当"障碍已移开",
               万一障碍就在眼前, 一次丢包会让车直接恢复行驶 —— 所以先沿用上一帧距离。 */
            s_miss++;
        }
        if (s_miss >= ULTRA_MISS_MAX)
        {
            g_ultra_mm = 0;                      //连续丢够 ULTRA_MISS_MAX 次: 报"没测到"
        }
    }
    return g_ultra_mm;
#else
    return 0;
#endif
}
