/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_pid.h
  * @brief      速度环(增量式 PID) + 角度环(yaw) 接口
  * @note       移植自 car-main v1.3 (2024电赛H题), 算法一字未改:
  *               · 速度环: 增量式 PID, 参数 110 / 85 / 5, 输出 0..MAX_DUTY(50000)
  *               · 角度环: pid_angle() 锁 yaw, 最短路径误差 + 积分抗饱和 + 过零清零
  *             速度单位: 每 100ms 的等效单相编码器计数
  *             (F103 的 4 倍频读数已在 pid_control() 里 /ENC_DIV)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    HARD_BRAKE: 停车用 —— 真短接制动(Motor_Brake, IN1=IN2=1) + 清空积分(error[0..2]),
    不清积分会自己又动。注意只写 duty=0 在 TB6612 上是"输出关断=滑行", 不是刹车。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#ifndef CAR_PID_H
#define CAR_PID_H

#include "sys.h"
#include "car_config.h"
#include "car_motor.h"              //HARD_BRAKE() 要用 Motor_Brake()
#include "struct_typedef.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    fp32 target;                    //目标速度
    fp32 now;                       //实测速度
    fp32 error[3];                  //误差项 0最新 1上一次 2上上次
    fp32 p, i, d;                   //PID 三参数
    fp32 pout, dout, iout;          //分项输出
    fp32 out;                       //总输出 duty 0..MAX_DUTY
    uint32_t pid_mode;              //PID_POSITION / PID_DELTA
} pid_type_def;

/* 两个速度环实例: motorA = A路(左轮, MOTOR_L_IS_A=1 时), motorB = B路(右轮) */
extern pid_type_def motorA;
extern pid_type_def motorB;

/* 角度环参数(运行时可调) */
extern fp32 g_angle_kp;
extern fp32 g_angle_ki;
extern fp32 g_angle_kd;
extern fp32 g_angle_out_max;

/* 调试观察量 */
extern volatile fp32    g_spd_now_l;        //实测速度(等效计数/100ms)
extern volatile fp32    g_spd_now_r;
extern volatile fp32    g_spd_tgt_l;
extern volatile fp32    g_spd_tgt_r;
extern volatile fp32    g_out_l;            //PID 输出 duty 0..50000
extern volatile fp32    g_out_r;
extern volatile uint8_t g_serial_plot_enable;

/* [PID整定] 运行时可调 (g_pid_tune=1 生效, 旁路循迹目标) */
extern volatile uint8_t g_pid_tune;
extern volatile fp32    g_pid_tgt_l;
extern volatile fp32    g_pid_tgt_r;
extern volatile uint8_t g_pid_wheel;        //0=两轮 1=只左 2=只右
extern volatile fp32    g_rpm_l;            //实测转速 RPM (只读)
extern volatile fp32    g_rpm_r;
extern volatile fp32    g_spd_kp_l;
extern volatile fp32    g_spd_ki_l;
extern volatile fp32    g_spd_kd_l;
extern volatile fp32    g_spd_kp_r;
extern volatile fp32    g_spd_ki_r;
extern volatile fp32    g_spd_kd_r;

/* [诊断] 编码器极性/失控标志: 1 = 目标为正、实测速度却持续为负、且 out 已顶满,
   基本就是那一路的编码器极性接反 -> 翻 g_enc_dir_l / g_enc_dir_r。只报不动。 */
extern volatile uint8_t g_pid_fault_l;
extern volatile uint8_t g_pid_fault_r;

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
extern void pid_init(pid_type_def *pid, uint32_t mode, fp32 p, fp32 i, fp32 d);

/**
  * @brief          set both wheel target speed, negative = backward
  * @param[in]      spe1: A channel target speed(等效计数/100ms)
  * @param[in]      spe2: B channel target speed
  * @retval         none
  */
/**
  * @brief          设置左右轮目标速度(负值=反转), 顺带处理方向脚与换向时的 PID 复位
  * @param[in]      spe1: A 路目标速度(等效计数/100ms)
  * @param[in]      spe2: B 路目标速度
  * @retval         none
  */
extern void motor_target_set(int spe1, int spe2);

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
extern void pid_cal(pid_type_def *pid);

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
extern void pidout_limit(pid_type_def *pid);

/**
  * @brief          speed loop, call every 100ms: read encoder -> PID -> output duty
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          速度环, 100ms 调一次: 读编码器 -> PID -> 输出 duty
  * @param[in]      none
  * @retval         none
  */
extern void pid_control(void);

/**
  * @brief          angle loop for yaw hold
  * @param[in]      target: target yaw (deg)
  * @param[in]      yaw: current yaw (deg)
  * @retval         angle pid out
  */
/**
  * @brief          角度环(锁 yaw)
  * @param[in]      target: 目标 yaw (度)
  * @param[in]      yaw: 当前 yaw (度)
  * @retval         角度环输出
  */
extern fp32 pid_angle(fp32 target, fp32 yaw);

/**
  * @brief          clear angle loop state
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          清角度环状态(起步/重新进入循迹时调)
  * @param[in]      none
  * @retval         none
  */
extern void pid_angle_reset(void);

/* 急停: 短接制动 + 清积分 (car-main 原宏, 停车必须清积分否则会自己溜)。
   原来这里写的是 motorA_duty(0)/motorB_duty(0) —— 那只是"输出关断=滑行", 不是刹车;
   而且 pid_control() 下一拍又会写一次 duty=0, 连"一次性刹车"都保持不住。
   现在走 Motor_Brake() 真短接制动, 由 car_motor.c 的 s_brake 标志保持到下次 duty>0。 */
#define HARD_BRAKE()                                                      \
    do                                                                    \
    {                                                                     \
        motor_target_set(0, 0);                                           \
        Motor_Brake();                                                    \
        motorA.out = 0;                                                   \
        motorA.error[0] = motorA.error[1] = motorA.error[2] = 0;          \
        motorB.out = 0;                                                   \
        motorB.error[0] = motorB.error[1] = motorB.error[2] = 0;          \
    } while (0)

#endif
