#ifndef __CAR_PID_H
#define __CAR_PID_H

#include "sys.h"
#include "car_config.h"

/* ---------------------------------------------------------------------------
 * PID — 移植自 car-main v1.3 (2024电赛H题), 算法一字未改
 *   · 速度环: 增量式 PID, 参数 110 / 85 / 5, 输出 0..MAX_DUTY(50000)
 *   · 角度环: pid_angle() 锁 yaw, 最短路径误差 + 积分抗饱和 + 过零清零
 *   · HARD_BRAKE: 停车用 —— duty 归零 + 清空积分(error[0..2]), 不清会自己又动
 * 速度单位: 每 100ms 的等效单相编码器计数 (F103 的4倍频读数已在 pid_control 里 /4)
 * ------------------------------------------------------------------------- */

enum { POSITION_PID = 0, DELTA_PID };

typedef struct
{
    float target;
    float now;
    float error[3];
    float p, i, d;
    float pout, dout, iout;
    float out;
    uint32_t pid_mode;
} pid_t;

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d);
void pid_cal(pid_t *pid);
void pidout_limit(pid_t *pid);
void pid_control(void);                 /* 100ms 调一次: 读编码器 -> PID -> 输出duty */
void motor_target_set(int spe1, int spe2);

float pid_angle(float target, float yaw);
void  pid_angle_reset(void);

/* 急停: duty 归零 + 清积分 (car-main 原宏, 停车必须清积分否则会自己溜) */
#define HARD_BRAKE()  do { \
    motor_target_set(0, 0); \
    motorA_duty(0); motorB_duty(0); \
    motorA.out = 0; motorA.error[0] = motorA.error[1] = motorA.error[2] = 0; \
    motorB.out = 0; motorB.error[0] = motorB.error[1] = motorB.error[2] = 0; \
} while (0)

extern pid_t motorA;
extern pid_t motorB;

/* 角度环参数(运行时可调) */
extern float g_angle_kp, g_angle_ki, g_angle_kd, g_angle_out_max;

/* 调试观察量 */
extern volatile float g_spd_now_l;      /* 实测速度(等效计数/100ms) */
extern volatile float g_spd_now_r;
extern volatile float g_spd_tgt_l;
extern volatile float g_spd_tgt_r;
extern volatile float g_out_l;          /* PID 输出 duty 0..50000 */
extern volatile float g_out_r;
extern volatile uint8_t g_serial_plot_enable;

#endif
