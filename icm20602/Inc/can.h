#ifndef _CAN_H_
#define _CAN_H_

#include "stm32f0xx_hal.h"
#include "stdint.h"

unsigned char CAN1_Send_Msg(unsigned char* msg,unsigned char len);
//unsigned char CANSendMag(short *data1, short *data2,short *data3,short *data4, uint32_t id);
unsigned char CANSendMag(float *data1, float *data2, uint32_t id);
unsigned char CAN1_Receive_Msg(unsigned char *buf);

#endif
