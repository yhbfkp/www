/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_core.h
  * @brief      循迹小车调度总入口: 模块初始化 + 主循环轮询 + TIM3 10ms 中断任务
  * @note       时间线:
  *               TIM3 每 10ms 中断 -> TIM3_10ms_Task()
  *                    · MPU6050 burst 读 + yaw 积分 (100Hz)
  *                    · 每 10 次(100ms)跑一次速度环 pid_control()   [与 car-main 周期一致]
  *               main 的 while(1) -> Car_Poll()
  *                    · 每 10ms: 读灰度 -> 任务层(转向/停止线/里程) -> 蜂鸣LED
  *                    · 每 100ms: 超声波触发
  *                    · 每 200ms: OLED 刷新(模式 + 用时)
  *                    · 每 500ms: 串口打印全量调试量
  *             按键: KEY1(PA15) 切模式   KEY2(PA12) 启动/停止
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#ifndef CAR_CORE_H
#define CAR_CORE_H

#include "sys.h"
#include "struct_typedef.h"

/**
  * @brief          init all modules
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          初始化所有模块
  * @param[in]      none
  * @retval         none
  */
extern void Car_Init(void);

/**
  * @brief          poll task, run in main while(1)
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          主循环轮询任务, 在 main 的 while(1) 里跑
  * @param[in]      none
  * @retval         none
  */
extern void Car_Poll(void);

/**
  * @brief          10ms task driven by TIM3 interrupt
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          TIM3 10ms 中断任务(覆盖 tim.c 的 __weak)
  * @param[in]      none
  * @retval         none
  */
extern void TIM3_10ms_Task(void);

extern volatile uint32_t g_ms;                  //系统毫秒

/* [调试便民] 这两个量专给 Keil 的 Watch 窗口用, 不用串口也能看清状态:
   g_dbg_state : 0 = 一切正常
                 bit0(1) = g_pid_tune 还开着(循迹目标被旁路, 只按 g_pid_tgt_* 走)
                 bit1(2) = g_pid_wheel 还开着(强制只让一个轮子转!)
                 bit2(4) = g_open_loop 还开着(开环, 直接给 g_test_duty)
                 例如读到 3 就是"整定 + 单轮"都还开着 —— 这时只有一个轮子转是正常的,
                 不是硬件坏。
   g_dbg_clear : 在 Watch 里写 1 -> 一次性把上面三个 + g_test_duty/方向 全部复位,
                 不用断电重启。 */
extern volatile uint8_t g_dbg_state;
extern volatile uint8_t g_dbg_clear;

#endif
