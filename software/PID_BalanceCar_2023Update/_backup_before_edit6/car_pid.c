#include "car_pid.h"
#include "car_motor.h"
#include "car_encoder.h"
#include "car_config.h"

/* 开环测试模式(MODE_TEST)时由任务层置位, 跳过PID直接给固定duty */
extern volatile uint8_t g_open_loop;
extern volatile int16_t g_test_duty;

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
        motorA_dir = 0;
        motorB_dir = 0;
        motorA_duty(g_test_duty);
        motorB_duty(g_test_duty);
        g_out_l = (float)g_test_duty;
        g_out_r = (float)g_test_duty;
        Encoder_GetL();     /* 清掉累积, 保持里程可用 */
        Encoder_GetR();
        return;
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
