#ifndef _I2C_H_
#define _I2C_H_

#include "stdint.h"

//IIC初始化  PB6-SDA PB7-SCL
#define GPIO_PIN_SDA GPIO_PIN_6
#define GPIO_PIN_SCL GPIO_PIN_7


/*******************??????*************************/
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
/*******************?????*************************/

//IO操作函数	 
//#define IIC_SCL    PBout(7) //SCL
//#define IIC_SDA    PBout(6) //SDA	 
//#define READ_SDA   PBin(6)  //输入SDA 

#define READ_SDA HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_SDA)

void IIC_SCL(u8 flag);
void SDA_IN(void);
void SDA_OUT(void);
//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 

#endif

