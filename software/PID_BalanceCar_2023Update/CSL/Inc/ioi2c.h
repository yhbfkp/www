#ifndef __IOI2C_H
#define __IOI2C_H

#include "sys.h"

/* 2026-09-27: 软I2C 由 PB8/PB9 改到 PA6/PA7, 给 MPU6050(GY-521 座) 用。
   板子 PB8/PB9 是 OLED 座, 后续 OLED 换 car-main 的 I2C 驱动(自带时序), 不再走这里。
   PA6 = SCL(输出), PA7 = SDA(输入/输出切换)
   PA6/PA7 属于 GPIOA->CRL(低8位): PA7 控制位[31:28], PA6 控制位[27:24]
   8<<28 = CNF7=10(上拉/下拉输入) MODE7=00
   3<<28 = CNF7=00(推挽输出)      MODE7=11(50MHz)
   注意: PA6/PA7 不耐 5V, GY-521 建议飞线供 3.3V(H9.20), 不要插座子的 5V */
//IO方向设置
#define SDA_IN()  {GPIOA->CRL&=0X0FFFFFFFu;GPIOA->CRL|=(u32)8<<28;}
#define SDA_OUT() {GPIOA->CRL&=0X0FFFFFFFu;GPIOA->CRL|=(u32)3<<28;}

//IO操作函数
#define IIC_SCL    PAout(6) //SCL
#define IIC_SDA    PAout(7) //SDA
#define READ_SDA   PAin(7)  //输入SDA

//IIC所有操作函数
void IIC_Init(void);           //初始化IIC的IO口				 
int IIC_Start(void);					 //发送IIC开始信号
void IIC_Stop(void);	  			 //发送IIC停止信号
void IIC_Send_Byte(u8 txd);		 //IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
int IIC_Wait_Ack(void); 			 //IIC等待ACK信号
void IIC_Ack(void);						 //IIC发送ACK信号
void IIC_NAck(void);					 //IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif

//------------------End of File----------------------------
