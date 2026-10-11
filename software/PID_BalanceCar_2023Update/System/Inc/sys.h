#ifndef __SYS_H
#define __SYS_H	
#include "stm32f1xx.h"

//定义一些常用的数据类型短关键字 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  
typedef const int16_t sc16;  
typedef const int8_t sc8;  

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  
typedef __I int16_t vsc16; 
typedef __I int8_t vsc8;   

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  
typedef const uint16_t uc16;  
typedef const uint8_t uc8; 

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  
typedef __I uint16_t vuc16; 
typedef __I uint8_t vuc8;  		

//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

void JTAG_Set(u8 mode);

////JTAG模式设置定义
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00	

typedef struct{
    u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity; //蓝牙遥控相关的变量
    u8 Flag_Stop,Flag_Show;                                     //电机停止标志位和显示标志位  默认停止 显示打开
    int Motor_Left,Motor_Right;                                 //电机PWM变量	
    int Temperature;                                            //温度变量
    int Voltage;                                                //电池电压采样相关的变量
    float Angle_Balance,Gyro_Balance,Gyro_Turn;                 //平衡倾角 平衡陀螺仪 转向陀螺仪
    u32 Distance;                                               //超声波测距

    u8 Flag_follow,Flag_avoid;						      	    //超声波跟随、超声波壁障标志位
    float Acceleration_Z;                                       //Z轴加速度计  
}BalanceCarTypeDef;

extern BalanceCarTypeDef BalanceCar;

//////////////////////////////////////////////////////////////////////////////
//以下为汇编函数
void WFI_SET(void);		  //执行WFI指令
void INTX_DISABLE(void);//关闭所有中断
void INTX_ENABLE(void);	//开启所有中断
void MSR_MSP(u32 addr);	//设置堆栈地址

#include "main.h"
/* ADC1 已删除(PA6 让给 MPU_SCL), CubeMX 也把 HAL_ADC_MODULE_ENABLED 注释掉了。
   若继续 #include "adc.h"，ADC_HandleTypeDef 未定义会导致全工程报错，故去掉。
   电池电压检测改用 control.c 里的 Get_battery_volt() 常量版本。 */
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "delay.h"

/* 2026-09-27: 平衡车那套(control/filter/pid/inv_mpu/DMP/mpu6050/show/encoder)
   已从 Keil 工程移出编译。这几个头留在这里只会带来符号冲突:
     App/Inc/pid.h  的 PID_Init()        vs Car/Inc/car_pid.h 的 PID_Init()
     CSL/Inc/oled.h 的 OLED_ShowString() vs Car/Inc/car_oled.h 的同名函数
   所以这里不再引入, 循迹小车各模块自己在 .c 里 include 需要的头。
   如果哪天要恢复平衡车代码, 把下面这些注释打开, 同时把 Car 组从工程里移除。 */
/* #include "control.h" */
/* #include "filter.h"  */
/* #include "pid.h"     */
/* #include "inv_mpu.h" */
/* #include "inv_mpu_dmp_motion_driver.h" */
/* #include "mpu6050.h" */
/* #include "show.h"    */
/* #include "encoder.h" */

#include "ioi2c.h"
#include "key.h"
#include "led.h"
/* oled.h 是原来 SPI OLED 的声明(OLED_ShowString/OLED_DrawPoint/OLED_ShowChar),
   与 Car/Inc/car_oled.h 重名冲突, oled.c 也已移出编译, 这里不再引入 */
/* #include "oled.h" */
#include "usart3.h"

/* DMP 相关两个大头, 平衡车专用, 同样移除 */
/* #include "dmpKey.h" */
/* #include "dmpmap.h" */

#include <math.h>
#include <string.h> 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#endif
