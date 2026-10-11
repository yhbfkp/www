/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_task.c/h
  * @brief      任务层实现，包括模式管理，里程与计时统计，停止线判定，蜂鸣器和LED控制
  * @note       
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
#ifndef CAR_TASK_H
#define CAR_TASK_H

#include "struct_typedef.h"
#include "sys.h"
#include "car_config.h"

/* ---------------------------------------------------------------------------
 * 任务层 — 二面题专用 (环形赛道: 直道1.5m + 半圆r0.5m, A/B/C/D, 顺时针)
 *
 *   MODE_AB   : A -> B, 沿上直道, 走够里程停车
 *   MODE_LAP  : A -> A 跑一圈, 靠 A 点的横穿停止线(8路里>=7路压黑)停车
 *               起步就压在线上 -> 必须先"离开线"再"第二次遇线"才算一圈
 *               另外有里程兜底, 防止丢线时永远不停
 *   MODE_TRACK: 一直循迹不停车(调参用)
 *   MODE_TEST : 开环固定占空比(验电机方向/编码器极性)
 *
 * 到位动作: HARD_BRAKE 急停 + 蜂鸣器 1s + LED 同步闪烁 1s (题目要求)
 * 附加项  : 超声波遇障 2~3cm 停车, OLED 显示 stop!, 蜂鸣持续; 障碍移开恢复计时继续跑
 * ------------------------------------------------------------------------- */

#define MODE_IDLE   0
#define MODE_AB     1
#define MODE_LAP    2
#define MODE_TRACK  3
#define MODE_TEST   4
#define MODE_MAX    4

/* OLED 显示用的状态 */
#define ST_IDLE     0
#define ST_RUN      1
#define ST_DONE     2
#define ST_OBS      3

/**
  * @brief          task layer init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          任务层初始化
  * @param[in]      none
  * @retval         none
  */
extern void Task_Init(void);

/**
  * @brief          task layer periodic update, called every 10ms
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          任务层周期更新, 10ms 调一次
  * @param[in]      none
  * @retval         none
  */
extern void Task_Update(void);

/**
  * @brief          beep and LED timing, called every 10ms
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          蜂鸣器和LED的计时, 10ms 调一次
  * @param[in]      none
  * @retval         none
  */
extern void Task_BeepLed(void);

/**
  * @brief          start: clear distance, time and integral
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          起步: 清零里程/计时/积分
  * @param[in]      none
  * @retval         none
  */
extern void Task_Start(void);

/**
  * @brief          stop at target: brake + beep + LED flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          到位停车 + 蜂鸣 + 闪灯
  * @param[in]      none
  * @retval         none
  */
extern void Task_StopFinish(void);

/**
  * @brief          set run mode
  * @param[in]      m: mode, MODE_IDLE/MODE_AB/MODE_LAP/MODE_TRACK/MODE_TEST
  * @retval         none
  */
/**
  * @brief          设置运行模式
  * @param[in]      m: 模式, MODE_IDLE/MODE_AB/MODE_LAP/MODE_TRACK/MODE_TEST
  * @retval         none
  */
extern void Task_SetMode(uint8_t m);

/**
  * @brief          get mode name string
  * @param[in]      m: mode, MODE_IDLE/MODE_AB/MODE_LAP/MODE_TRACK/MODE_TEST
  * @retval         mode name string
  */
/**
  * @brief          取模式名称字符串
  * @param[in]      m: 模式, MODE_IDLE/MODE_AB/MODE_LAP/MODE_TRACK/MODE_TEST
  * @retval         模式名称字符串
  */
extern const char *Task_ModeName(uint8_t m);

extern volatile uint8_t  g_mode;
extern volatile uint8_t  g_running;      // 1=运行中
extern volatile uint8_t  g_finished;     // 1=已到位(触发过蜂鸣)
extern volatile uint8_t  g_obs_flag;     // 1=当前被障碍挡住
extern volatile uint8_t  g_gray_hits;    // 当前压黑的路数 0..8
extern volatile uint32_t g_run_ms;       // 本轮计时(障碍时暂停)
extern volatile int32_t  g_dist_cnt;     // 已走里程(原始编码器计数)
extern volatile int32_t  g_ab_cnt;       // A->B 目标里程
extern volatile int32_t  g_lap_min_cnt;  // 一圈最短里程(防起步误停)
extern volatile int32_t  g_lap_max_cnt;  // 一圈兜底里程
extern volatile fp32     g_ab_mm;        // A->B 距离, 可在 watch 里改
extern volatile fp32     g_cnt_per_mm;   // 每mm对应计数, 实测后可改
extern volatile uint8_t  g_open_loop;
extern volatile int32_t  g_test_duty;    // A路 = 左轮 duty(0..50000), watch 里可直接改
extern volatile int32_t  g_test_duty_r;  // B路 = 右轮 duty, 设 0 就是只转左轮
/* 上面两个必须是 32 位: 50000 > INT16_MAX, 用 int16_t 时在 Watch 里写 40000 会变成
   负数, 到 OutA/OutB 里被当成 <0 夹成 0 -> 轮子不转, 看起来像"驱动坏了" */
extern volatile uint8_t  g_test_dir_l;   // 左轮方向 0=正转 1=反转
extern volatile uint8_t  g_test_dir_r;   // 右轮方向 0=正转 1=反转

#endif
