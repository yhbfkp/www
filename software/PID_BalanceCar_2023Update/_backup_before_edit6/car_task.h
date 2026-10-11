#ifndef __CAR_TASK_H
#define __CAR_TASK_H

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

void Task_Init(void);
void Task_Update(void);          /* 10ms 调一次 */
void Task_BeepLed(void);         /* 10ms 调一次, 管蜂鸣器和LED的计时 */
void Task_Start(void);           /* 起步: 清零里程/计时/积分 */
void Task_StopFinish(void);      /* 到位停车 + 蜂鸣 + 闪灯 */
void Task_SetMode(uint8_t m);
const char *Task_ModeName(uint8_t m);

extern volatile uint8_t  g_mode;
extern volatile uint8_t  g_running;      /* 1=运行中 */
extern volatile uint8_t  g_finished;     /* 1=已到位(触发过蜂鸣) */
extern volatile uint8_t  g_obs_flag;     /* 1=当前被障碍挡住 */
extern volatile uint8_t  g_gray_hits;    /* 当前压黑的路数 0..8 */
extern volatile uint32_t g_run_ms;       /* 本轮计时(障碍时暂停) */
extern volatile int32_t  g_dist_cnt;     /* 已走里程(原始编码器计数) */
extern volatile int32_t  g_ab_cnt;       /* A->B 目标里程 */
extern volatile int32_t  g_lap_min_cnt;  /* 一圈最短里程(防起步误停) */
extern volatile int32_t  g_lap_max_cnt;  /* 一圈兜底里程 */
extern volatile float    g_ab_mm;        /* A->B 距离, 可在 watch 里改 */
extern volatile float    g_cnt_per_mm;   /* 每mm对应计数, 实测后可改 */
extern volatile uint8_t  g_open_loop;
extern volatile int16_t  g_test_duty;

#endif
