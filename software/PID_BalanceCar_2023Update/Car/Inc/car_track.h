/**
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  * @file       car_track.h
  * @brief      8 路灰度巡线(加权差比和 + 丢线惯性 + 转向PD)对外接口
  * @note       巡线算法 — 移植自 car-main v1.3 (2024电赛H题), 算法一字未改:
  *               · 8路灰度 -> 加权差比和(不除cnt) -> error (约 ±12)
  *                 灯越多 = 弯越急 -> error 越大 -> 转向越猛
  *               · 丢线 FIND: 保持上一帧 error 走惯性 (不是阶跃打死舵)
  *               · 转向PD -> 差速; 弯道减速 curve_factor = 1 - |err|/12*0.7
  *                 (弯越急 base 越低, 且外轮 clamp 到 <= 直道速度, 防止甩出黑线)
  *             黑线 = 1, 白 = 0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-06-2026     BalanceCar      1. 完成
  *
  @verbatim
  ==============================================================================
    track()      : 算一次转向并下发给速度环( motor_target_set ), car_core 每 10ms 调一次
    track_reset(): 复位内部状态, 起步/切模式时调, 防 D 项残留跳变
    digtal(n)    : 读第 n 路(1~8)灰度, 由 car_core 每 10ms 刷新 g_sensor_data
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 BalanceCar****************************
  */

#ifndef CAR_TRACK_H
#define CAR_TRACK_H

#include "sys.h"
#include "car_config.h"
#include "struct_typedef.h"

#define GRAY_CHANNELS   8

/**
  * @brief          run one tracking step: gray sensors -> error -> steering PD -> wheel targets
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          算一次转向并下发给速度环( motor_target_set )
  * @param[in]      none
  * @retval         none
  */
extern void track(void);

/**
  * @brief          reset tracking internal state
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          复位内部状态(起步/切模式时调, 防D项残留跳变)
  * @param[in]      none
  * @retval         none
  */
extern void track_reset(void);

/**
  * @brief          read one channel of the 8-way gray sensor
  * @param[in]      channel: channel index, 1~8
  * @retval         0: white / invalid channel, 1: black line
  */
/**
  * @brief          读第 channel 路(1~8)灰度值
  * @param[in]      channel: 通道号, 1~8
  * @retval         0: 白 / 通道号非法, 1: 压黑线
  */
extern unsigned char digtal(unsigned char channel);

/**
  * @brief          set fast / slow / min speed used by tracking
  * @param[in]      fast: straight line speed
  * @param[in]      slow: line-lost search speed
  * @param[in]      min: inner wheel minimum speed
  * @retval         none
  */
/**
  * @brief          设置循迹用的 直道/丢线搜索/内轮最低 速度
  * @param[in]      fast: 直道速度
  * @param[in]      slow: 丢线搜索速度
  * @param[in]      min: 内轮最低速度
  * @retval         none
  */
extern void track_set_speed(int16_t fast, int16_t slow, int16_t min);

extern uint16_t g_sensor_data[GRAY_CHANNELS];   //8路 0/1, 由 car_core 每10ms刷新
extern char     g_track_state[8];               //"GO"/"L1"/"R2"/"FIND"/"CROSS"

/* [可调] */
extern int16_t g_speed_fast;
extern int16_t g_speed_slow;
extern int16_t g_speed_min;
extern fp32    g_track_kp;
extern fp32    g_track_kd;
extern fp32    g_track_err;                     //最近一次误差, 调参看它
extern int16_t g_track_L;                       //下发给左右轮的目标速度
extern int16_t g_track_R;

#endif
