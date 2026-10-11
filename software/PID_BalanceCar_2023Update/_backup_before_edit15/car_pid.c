#include "car_pid.h"
#include "car_motor.h"
#include "car_encoder.h"
#include "car_config.h"

/* 开环测试模式(MODE_TEST)时由任务层置位, 跳过PID直接给固定duty */
extern volatile uint8_t g_open_loop;
extern volatile int16_t g_test_duty;
extern volatile int16_t g_test_duty_r;
extern volatile uint8_t g_test_dir_l;
extern volatile uint8_t g_test_dir_r;

pid_t motorA;
pid_t motorB;

float g_angle_kp      = ANGLE_KP;
float g_angle_ki      = ANGLE_KI;
float g_angle_kd      = ANGLE_KD;
float g_angle_out_max = ANGLE_OUT_MAX;

static float angle_prev_err   = 0.0f;
static float angle_integral   = 0.0f;
static uint8_t angle_need_init = 1;

volatile float   g_spd_now_l = 0;
volatile float   g_spd_now_r = 0;
volatile float   g_spd_tgt_l = 0;
volatile float   g_spd_tgt_r = 0;
volatile float   g_out_l     = 0;
volatile float   g_out_r     = 0;
volatile uint8_t g_serial_plot_enable = CAR_USE_PLOT;

/* --- [PID整定] 全部可在 debug 的 Watch 里直接改, 改完立刻生效, 不用重新编译 ---
 * 用法:
 *   1) g_pid_tune 写 1       -> 循迹层算出的目标被旁路, 只用下面的 g_pid_tgt_*
 *   2) g_pid_tgt_l / _r      -> 左右轮目标速度(单位同 SPEED_FAST=45), 负数=反转
 *   3) g_spd_kp/ki/kd_l/_r   -> 左右轮各自一套速度环参数
 *   4) 调完把 g_pid_tune 写回 0 即恢复循迹控制
 * 速度单位: 每 100ms 的等效编码器计数。RPM = now * ENC_DIV * 600 / ENC_CPR
 *            (13线×20减速比: now*2.31, 即 45 -> 约 104 RPM, 轮径65mm -> 约 0.35 m/s)
 *            也可以直接看算好的 g_rpm_l / g_rpm_r
 */
volatile uint8_t g_pid_tune  = 0;
volatile float   g_pid_tgt_l = (float)SPEED_FAST;   /* 默认 45 (≈104 RPM) */
volatile float   g_pid_tgt_r = (float)SPEED_FAST;
/* 单轮选择: 0=两轮都转  1=只左轮  2=只右轮 (另一边目标强制 0) */
volatile uint8_t g_pid_wheel = 0;

/* 换算后的实际转速, 只看不改: RPM = now(每100ms等效计数) * ENC_DIV * 600 / ENC_CPR */
volatile float   g_rpm_l = 0.0f;
volatile float   g_rpm_r = 0.0f;
volatile float   g_spd_kp_l  = SPD_KP;
volatile float   g_spd_ki_l  = SPD_KI;
volatile float   g_spd_kd_l  = SPD_KD;
volatile float   g_spd_kp_r  = SPD_KP;
volatile float   g_spd_ki_r  = SPD_KI;
volatile float   g_spd_kd_r  = SPD_KD;

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
    pid->pid_mode = mode;
    pid->p = p;
    pid->i = i;
    pid->d = d;
}

void motor_target_set(int spe1, int spe2)
{
    if (spe1 >= 0) { motorA_dir = 0; motorA.target = (float)spe1; }
    else           { motorA_dir = 1; motorA.target = (float)(-spe1); }

    if (spe2 >= 0) { motorB_dir = 0; motorB.target = (float)spe2; }
    else           { motorB_dir = 1; motorB.target = (float)(-spe2); }

    g_spd_tgt_l = (float)spe1;
    g_spd_tgt_r = (float)spe2;
}

void pid_cal(pid_t *pid)
{
    pid->error[0] = pid->target - pid->now;

    if (pid->pid_mode == DELTA_PID)          /* 增量式 */
    {
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
        pid->out += pid->pout + pid->iout + pid->dout;
    }
    else                                      /* 位置式 */
    {
        pid->pout = pid->p * pid->error[0];
        pid->iout += pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);
        pid->out = pid->pout + pid->iout + pid->dout;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
}

void pidout_limit(pid_t *pid)
{
    if (pid->out >= MAX_DUTY) pid->out = MAX_DUTY;
    if (pid->out <= 0)        pid->out = 0;
}

/* ---------------------------------------------------------------------------
 * 速度环 — 100ms 一次 (与 car-main 的 tim_interrupt_ms_init(TIM_3,100) 一致)
 * ------------------------------------------------------------------------- */
void pid_control(void)
{
    int32_t cl, cr;
    float   l, r;

    /* 开环测试模式: 直接给固定 duty, 不走 PID (用来验电机方向/编码器极性) */
    if (g_open_loop)
    {
        motorA_dir = g_test_dir_l ? 1 : 0;
        motorB_dir = g_test_dir_r ? 1 : 0;
        motorA_duty(g_test_duty);       /* 左轮, 设 0 = 不转 */
        motorB_duty(g_test_duty_r);     /* 右轮, 设 0 = 不转 */
        g_out_l = (float)g_test_duty;
        g_out_r = (float)g_test_duty_r;
        Encoder_GetL();     /* 清掉累积, 保持里程可用 */
        Encoder_GetR();
        return;
    }

    /* [PID整定] 旁路循迹目标 + 左右轮独立参数 (见上面变量注释) */
    if (g_pid_tune)
    {
        motorA.p = g_spd_kp_l;
        motorA.i = g_spd_ki_l;
        motorA.d = g_spd_kd_l;
        motorB.p = g_spd_kp_r;
        motorB.i = g_spd_ki_r;
        motorB.d = g_spd_kd_r;

        if      (g_pid_wheel == 1) motor_target_set((int)g_pid_tgt_l, 0);
        else if (g_pid_wheel == 2) motor_target_set(0, (int)g_pid_tgt_r);
        else                       motor_target_set((int)g_pid_tgt_l, (int)g_pid_tgt_r);
    }

    cl = Encoder_GetL();    /* 100ms 内的原始4倍频计数 */
    cr = Encoder_GetR();

    /* 4倍频 -> 等效原版单相计数 */
    l = (float)cl / (float)ENC_DIV;
    r = (float)cr / (float)ENC_DIV;

    /* 方向: dir=0 前进时编码器正向计数 -> now 为正 */
    motorA.now = motorA_dir ? -l : l;
    motorB.now = motorB_dir ? -r : r;

    g_spd_now_l = motorA.now;
    g_spd_now_r = motorB.now;

    /* 换算成 RPM: now 是"每100ms的等效单相计数", 乘 ENC_DIV 还原4倍频,
       再 *10 得每秒计数, *60/ENC_CPR 得每分钟转数 */
    g_rpm_l = g_spd_now_l * (float)ENC_DIV * 600.0f / ENC_CPR;
    g_rpm_r = g_spd_now_r * (float)ENC_DIV * 600.0f / ENC_CPR;

    pid_cal(&motorA);
    pid_cal(&motorB);

    pidout_limit(&motorA);
    pidout_limit(&motorB);

    motorA_duty((int)motorA.out);
    motorB_duty((int)motorB.out);

    g_out_l = motorA.out;
    g_out_r = motorB.out;

#if CAR_USE_PLOT
    if (g_serial_plot_enable)
        printf("%d,%d,%d,%d\r\n",
               (int)motorA.target, (int)motorA.now,
               (int)motorB.target, (int)motorB.now);
#endif
}

/* ---------------------------------------------------------------------------
 * 角度环 (yaw) — car-main 原实现
 * ------------------------------------------------------------------------- */
float pid_angle(float target, float yaw)
{
    float err, out, diff;

    while (yaw < 0.0f)    yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;

    err = target - yaw;
    if      (err >  180.0f) err -= 360.0f;
    else if (err < -180.0f) err += 360.0f;

    angle_integral += err;
    if (angle_integral >  20.0f) angle_integral =  20.0f;
    if (angle_integral < -20.0f) angle_integral = -20.0f;
    if (err * angle_prev_err <= 0.0f) angle_integral = 0.0f;

    diff = err - angle_prev_err;
    angle_prev_err = err;

    if (angle_need_init)        /* 复位后首帧跳过D项 */
    {
        angle_need_init = 0;
        diff = 0.0f;
    }

    out = g_angle_kp * err + g_angle_ki * angle_integral + g_angle_kd * diff;

    if (out >  g_angle_out_max) out =  g_angle_out_max;
    if (out < -g_angle_out_max) out = -g_angle_out_max;

    return out;
}

void pid_angle_reset(void)
{
    angle_prev_err   = 0.0f;
    angle_integral   = 0.0f;
    angle_need_init  = 1;
}
