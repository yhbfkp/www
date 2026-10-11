#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define PI 3.14159265							//PI圆周率
#define Control_Frequency  200.0	//编码器读取频率
#define Diameter_67  67.0 				//轮子直径67mm 
#define EncoderMultiples   4.0 		//编码器倍频数
#define Encoder_precision  13.0 	//编码器精度 13线
#define Reduction_Ratio  30.0			//减速比30
#define Perimeter  210.4867 			//周长，单位mm


#define DIFFERENCE 100
#define INT PAin(12)   //注意：PA12 现在已改作 KEY2 按键，不再是 MPU6050 的中断脚

int Balance(float angle,float gyro);
void Control_Task(void);          // 原 HAL_GPIO_EXTI_Callback 的控制体，改由 TIM3 每10ms调用
void TIM3_10ms_Task(void);        // 覆盖 tim.c 里的 __weak 定义
int Get_battery_volt(void);       // ADC1 已删除，当前返回常量 1200(12.00V)
int Velocity(int encoder_left,int encoder_right);
int Turn(float gyro);
void Set_Pwm(int motor_left,int motor_right);
void Key(void);
void Limit_Pwm(void);
int PWM_Limit(int IN,int max,int min);
u8 Turn_Off(float angle, int voltage);
void Get_Angle(void);
int myabs(int a);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
void Choose(int encoder_left,int encoder_right);

#endif
