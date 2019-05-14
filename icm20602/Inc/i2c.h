#ifndef _I2C_H_
#define _I2C_H_

#include "stdint.h"

//IIC��ʼ��  PB6-SDA PB7-SCL
#define GPIO_PIN_SDA GPIO_PIN_6
#define GPIO_PIN_SCL GPIO_PIN_7


/*******************??????*************************/
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
/*******************?????*************************/

//IO��������	 
//#define IIC_SCL    PBout(7) //SCL
//#define IIC_SDA    PBout(6) //SDA	 
//#define READ_SDA   PBin(6)  //����SDA 

#define READ_SDA HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_SDA)

void IIC_SCL(u8 flag);
void SDA_IN(void);
void SDA_OUT(void);
//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 

#endif

