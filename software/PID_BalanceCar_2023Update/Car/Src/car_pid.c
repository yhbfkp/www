/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_pid.c
  * @brief      速度环(增量式 PID) + 角度环(yaw) 实现
  * @note       移植自 car-main v1.3 (2024电赛H题), 算法一字未改
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    速度单位: 每 100ms 的等效单相编码器计数。RPM = now * ENC_DIV * 600 / ENC_CPR
      (按 13 线 x 20 减速比: now*2.31, 即 45 -> 约 104 RPM, 轮径 65mm -> 约 0.35 m/s)

    [PID整定] 全部可在 debug 的 Watch 里直接改, 改完立刻生效, 不用重新编译:
      1) g_pid_tune 写 1       -> 循迹层算出的目标被旁路, 只用 g_pid_tgt_*
      2) g_pid_tgt_l / _r      -> 左右轮目标速度(单位同 SPEED_FAST=45), 负数=反转
      3) g_spd_kp/ki/kd_l/_r   -> 左右轮各自一套速度环参数
      4) 调完把 g_pid_tune 写回 0 即恢复循迹控制
    判编码器极性: g_pid_tune=1, g_pid_wheel=0, g_pid_tgt_* 写 0, 用手把车往前推,
      看 g_spd_now_l / g_spd_now_r —— 前进必须都是正数, 负的那个就翻 g_enc_dir_l/_r
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#include "car_pid.h"
#include "car_motor.h"
#include "car_encoder.h"
#include "car_config.h"

/* 开环测试模式(MODE_TEST)时由任务层置位, 跳过 PID 直接给固定 duty
   (这几个变量定义在 car_task.c, 按 RM 代码习惯在使用的文件里就地 extern) */
extern volatile uint8_t g_open_loop;
extern volatile int32_t g_test_duty;
extern volatile int32_t g_test_duty_r;
extern volatile uint8_t g_test_dir_l;
extern volatile uint8_t g_test_dir_r;

pid_type_def motorA;
pid_type_def motorB;

fp32 g_angle_kp      = ANGLE_KP;
fp32 g_angle_ki      = ANGLE_KI;
fp32 g_angle_kd      = ANGLE_KD;
fp32 g_angle_out_max = ANGLE_OUT_MAX;

static fp32    angle_prev_err  = 0.0f;
static fp32    angle_integral  = 0.0f;
static uint8_t angle_need_init = 1;

volatile fp32    g_spd_now_l = 0;
volatile fp32    g_spd_now_r = 0;
volatile fp32    g_spd_tgt_l = 0;
volatile fp32    g_spd_tgt_r = 0;
volatile fp32    g_out_l     = 0;
volatile fp32    g_out_r     = 0;
volatile uint8_t g_serial_plot_enable = CAR_USE_PLOT;

/* [诊断] 编码器极性/失控标志, 定义见 car_pid.h。
   要连续 20 拍(2 秒)满足才置位, 免得正常反转 / 被手推着走时误报。 */
volatile uint8_t g_pid_fault_l = 0;
volatile uint8_t g_pid_fault_r = 0;
static   uint8_t s_bad_l = 0;
static   uint8_t s_bad_r = 0;

volatile uint8_t g_pid_tune  = 0;
volatile fp32    g_pid_tgt_l = (fp32)SPEED_FAST;    //默认 45 (约 104 RPM)
volatile fp32    g_pid_tgt_r = (fp32)SPEED_FAST;

/* 单轮选择: 0=两轮都转  1=只左轮  2=只右轮 (另一边目标强制 0) */
volatile uint8_t g_pid_wheel = 0;

/* 换算后的实际转速, 只看不改: RPM = now(每100ms等效计数) * ENC_DIV * 600 / ENC_CPR */
volatile fp32 g_rpm_l = 0.0f;
volatile fp32 g_rpm_r = 0.0f;

volatile fp32 g_spd_kp_l = SPD_KP;
volatile fp32 g_spd_ki_l = SPD_KI;
volatile fp32 g_spd_kd_l = SPD_KD;
volatile fp32 g_spd_kp_r = SPD_KP;
volatile fp32 g_spd_ki_r = SPD_KI;
volatile fp32 g_spd_kd_r = SPD_KD;

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid, PID_DELTA: delta pid
  * @param[in]      p: kp
  * @param[in]      i: ki
  * @param[in]      d: kd
  * @retval         none
  */
/**
  * @brief          PID 结构体初始化
  * @param[out]     pid: PID 结构体指针
  * @param[in]      mode: PID_POSITION=位置式, PID_DELTA=增量式
  * @param[in]      p: Kp
  * @param[in]      i: Ki
  * @param[in]      d: Kd
  * @retval         none
  */
void pid_init(pid_type_def *pid, uint32_t mode, fp32 p, fp32 i, fp32 d)
{
    pid->pid_mode = mode;
    pid->p = p;
    pid->i = i;
    pid->d = d;
}

/**
  * @brief          set both wheel target speed, negative = backward
  * @param[in]      spe1: A channel target speed
  * @param[in]      spe2: B channel target speed
  * @retval         none
  */
/**
  * @brief          设置左右轮目标速度(负值=反转), 处理方向脚 + 换向时清 PID 状态
  * @param[in]      spe1: A 路目标速度
  * @param[in]      spe2: B 路目标速度
  * @retval         none
  */
void motor_target_set(int spe1, int spe2)
{
    int d1 = (spe1 < 0) ? 1 : 0;
    int d2 = (spe2 < 0) ? 1 : 0;

    /* 方向翻转 / 目标归零 -> 清掉 PID 状态。
       方向一变, now 会立刻跟着变号, 而增量式 PID 的 out/error[] 是"有记忆"的:
       留着它就会朝最坏方向猛推一把(通常直接顶到 MAX_DUTY), 表现就是每次换向
       都被踹一下 —— 起步和"原地找线"时最明显。清掉之后从 0 重新爬, 换向就平顺了。
       (必须在更新 motorA_dir / motorB_dir 之前比较) */
    if (d1 != motorA_dir || spe1 == 0)
    {
        motorA.out = 0;
        motorA.error[0] = motorA.error[1] = motorA.error[2] = 0;
    }
    if (d2 != motorB_dir || spe2 == 0)
    {
        motorB.out = 0;
        motorB.error[0] = motorB.error[1] = motorB.error[2] = 0;
    }

    if (spe1 >= 0)
    {
        motorA_dir = 0;
        motorA.target = (fp32)spe1;
    }
    else
    {
        motorA_dir = 1;
        motorA.target = (fp32)(-spe1);
    }

    if (spe2 >= 0)
    {
        motorB_dir = 0;
        motorB.target = (fp32)spe2;
    }
    else
    {
        motorB_dir = 1;
        motorB.target = (fp32)(-spe2);
    }

    g_spd_tgt_l = (fp32)spe1;
    g_spd_tgt_r = (fp32)spe2;
}

/**
  * @brief          one step pid calculate
  * @param[in,out]  pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          单步 PID 计算(位置式/增量式由 pid->pid_mode 决定)
  * @param[in,out]  pid: PID 结构体指针
  * @retval         none
  */
void pid_cal(pid_type_def *pid)
{
    pid->error[0] = pid->target - pid->now;

    if (pid->pid_mode == PID_DELTA)                     //增量式
    {
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
        pid->out += pid->pout + pid->iout + pid->dout;
    }
    else                                                //位置式
    {
        pid->pout = pid->p * pid->error[0];
        pid->iout += pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);
        pid->out = pid->pout + pid->iout + pid->dout;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
}

/**
  * @brief          limit pid out into 0 .. MAX_DUTY
  * @param[in,out]  pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          PID 输出限幅到 0 .. MAX_DUTY
  * @param[in,out]  pid: PID 结构体指针
  * @retval         none
  */
void pidout_limit(pid_type_def *pid)
{
    if (pid->out >= MAX_DUTY)
    {
        pid->out = MAX_DUTY;
    }
    if (pid->out <= 0)
    {
        pid->out = 0;
    }
}

/**
  * @brief          speed loop, call every 100ms: read encoder -> PID -> output duty
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          速度环, 100ms 一次 (与 car-main 的 tim_interrupt_ms_init(TIM_3,100) 一致)
  * @param[in]      none
  * @retval         none
  */
void pid_control(void)
{
    fp32 l, r;

    if (g_open_loop)                                    //开环测试: 固定 duty, 不走 PID
    {
        int16_t ol, orr;

        motorA_dir = g_test_dir_l ? 1 : 0;
        motorB_dir = g_test_dir_r ? 1 : 0;
        motorA_duty(g_test_duty);                       //左轮, 设 0 = 不转
        motorB_duty(g_test_duty_r);                     //右轮, 设 0 = 不转
        g_out_l = (fp32)g_test_duty;
        g_out_r = (fp32)g_test_duty_r;

        /* 开环也必须把实测速度算出来! 原来这里只把编码器增量取走就 return,
           g_spd_now_* / g_rpm_* 永远是 0 —— 拿开环去判编码器极性会被这个 0 骗到
           (看起来像"编码器没信号"), 然后就会把 g_enc_dir_* 改错。
           符号口径与闭环一致: now>0 表示"按当前指令方向在动"。 */
        ol  = Encoder_GetL();
        orr = Encoder_GetR();
        g_spd_now_l = motorA_dir ? -(fp32)ol  / (fp32)ENC_DIV : (fp32)ol  / (fp32)ENC_DIV;
        g_spd_now_r = motorB_dir ? -(fp32)orr / (fp32)ENC_DIV : (fp32)orr / (fp32)ENC_DIV;
        g_rpm_l = g_spd_now_l * (fp32)ENC_DIV * 600.0f / ENC_CPR;
        g_rpm_r = g_spd_now_r * (fp32)ENC_DIV * 600.0f / ENC_CPR;
        return;
    }

    if (g_pid_tune)                                     //[PID整定] 旁路循迹目标
    {
        motorA.p = g_spd_kp_l;
        motorA.i = g_spd_ki_l;
        motorA.d = g_spd_kd_l;
        motorB.p = g_spd_kp_r;
        motorB.i = g_spd_ki_r;
        motorB.d = g_spd_kd_r;

        if (g_pid_wheel == 1)
        {
            motor_target_set((int)g_pid_tgt_l, 0);
        }
        else if (g_pid_wheel == 2)
        {
            motor_target_set(0, (int)g_pid_tgt_r);
        }
        else
        {
            motor_target_set((int)g_pid_tgt_l, (int)g_pid_tgt_r);
        }
    }

    l = (fp32)Encoder_GetL() / (fp32)ENC_DIV;           //100ms 内原始4倍频计数 -> 等效单相
    r = (fp32)Encoder_GetR() / (fp32)ENC_DIV;

    /* 方向: dir=0 前进时编码器正向计数 -> now 为正 */
    motorA.now = motorA_dir ? -l : l;
    motorB.now = motorB_dir ? -r : r;

    g_spd_now_l = motorA.now;
    g_spd_now_r = motorB.now;

    /* 换算成 RPM: now 是"每100ms的等效单相计数", 乘 ENC_DIV 还原4倍频,
       再 *10 得每秒计数, *60/ENC_CPR 得每分钟转数 */
    g_rpm_l = g_spd_now_l * (fp32)ENC_DIV * 600.0f / ENC_CPR;
    g_rpm_r = g_spd_now_r * (fp32)ENC_DIV * 600.0f / ENC_CPR;

    pid_cal(&motorA);
    pid_cal(&motorB);

    pidout_limit(&motorA);
    pidout_limit(&motorB);

    /* 目标为 0 的那一侧强制 out=0, 这一拍不再走 PID。
       否则"目标0 + 极性反导致的负反馈"会凑成正反馈(误差 = 0-负数 = 正数),
       那一侧会一路加速到满占空比 —— 就是之前看到的"另一个疯转"。
       原来这段只在 g_pid_tune(整定模式) 下生效, 现在改成常驻: 循迹时内轮被
       track() 夹到 0 的场景同样适用, 而且目标 0 本来就应该输出 0。 */
    {
        int tgt_l = (int)motorA.target;                 //target 已取过绝对值, 恒 >= 0
        int tgt_r = (int)motorB.target;

        if (g_pid_tune)                                 //整定模式的单轮选择: 屏蔽另一侧
        {
            if (g_pid_wheel == 2)
            {
                tgt_l = 0;
            }
            if (g_pid_wheel == 1)
            {
                tgt_r = 0;
            }
        }

        if (tgt_l == 0)
        {
            motorA.out = 0;
            motorA.error[0] = motorA.error[1] = motorA.error[2] = 0;
        }
        if (tgt_r == 0)
        {
            motorB.out = 0;
            motorB.error[0] = motorB.error[1] = motorB.error[2] = 0;
        }
    }

    motorA_duty((int)motorA.out);
    motorB_duty((int)motorB.out);

    g_out_l = motorA.out;
    g_out_r = motorB.out;

    /* [诊断] 编码器极性 / 失控
       极性接反的现象: 目标是正的, 反馈却是负的 -> 误差 = target - (负值) 越滚越大
       -> 增量式 PID 每一拍都往上加 -> out 顶死在 MAX_DUTY, 轮子满速狂奔。
       这里只置标志、不改控制律, 由 car_core.c 在主循环里打印告警。 */
    if (motorA.target > 1.0f && motorA.now < -1.0f &&
        motorA.out >= (fp32)MAX_DUTY * 0.95f)
    {
        if (s_bad_l < 250)
        {
            s_bad_l++;
        }
    }
    else
    {
        s_bad_l = 0;
    }

    if (motorB.target > 1.0f && motorB.now < -1.0f &&
        motorB.out >= (fp32)MAX_DUTY * 0.95f)
    {
        if (s_bad_r < 250)
        {
            s_bad_r++;
        }
    }
    else
    {
        s_bad_r = 0;
    }

    g_pid_fault_l = (s_bad_l >= 20) ? 1 : 0;
    g_pid_fault_r = (s_bad_r >= 20) ? 1 : 0;

#if CAR_USE_PLOT
    if (g_serial_plot_enable)
    {
        printf("%d,%d,%d,%d\r\n",
               (int)motorA.target, (int)motorA.now,
               (int)motorB.target, (int)motorB.now);
    }
#endif
}

/**
  * @brief          angle loop for yaw hold (car-main original)
  * @param[in]      target: target yaw (deg)
  * @param[in]      yaw: current yaw (deg)
  * @retval         angle pid out
  */
/**
  * @brief          角度环(锁 yaw) — car-main 原实现
  * @param[in]      target: 目标 yaw (度)
  * @param[in]      yaw: 当前 yaw (度)
  * @retval         角度环输出
  */
fp32 pid_angle(fp32 target, fp32 yaw)
{
    fp32 err, out, diff;

    while (yaw < 0.0f)
    {
        yaw += 360.0f;
    }
    while (yaw >= 360.0f)
    {
        yaw -= 360.0f;
    }

    err = target - yaw;
    if (err > 180.0f)
    {
        err -= 360.0f;
    }
    else if (err < -180.0f)
    {
        err += 360.0f;
    }

    angle_integral += err;
    if (angle_integral > 20.0f)
    {
        angle_integral = 20.0f;
    }
    if (angle_integral < -20.0f)
    {
        angle_integral = -20.0f;
    }
    if (err * angle_prev_err <= 0.0f)
    {
        angle_integral = 0.0f;
    }

    diff = err - angle_prev_err;
    angle_prev_err = err;

    if (angle_need_init)                                //复位后首帧跳过 D 项
    {
        angle_need_init = 0;
        diff = 0.0f;
    }

    out = g_angle_kp * err + g_angle_ki * angle_integral + g_angle_kd * diff;

    if (out > g_angle_out_max)
    {
        out = g_angle_out_max;
    }
    if (out < -g_angle_out_max)
    {
        out = -g_angle_out_max;
    }

    return out;
}

/**
  * @brief          clear angle loop state
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          清角度环状态
  * @param[in]      none
  * @retval         none
  */
void pid_angle_reset(void)
{
    angle_prev_err  = 0.0f;
    angle_integral  = 0.0f;
    angle_need_init = 1;
}
