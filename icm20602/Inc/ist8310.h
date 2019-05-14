#ifndef _IST8310_H_
#define _IST8310_H_

#include "stdint.h"
typedef uint8_t  u8;
// IST8310 internal reg addr

//#define IST8310_ADDRESS 0x0E  //7Bit Address
#define IST8310_ADDRESS 0x1C    //8Bit Address
#define IST8310_DEVICE_ID_A 0x10

// IST8310 register map. For details see IST8310 datasheet
#define IST8310_WHO_AM_I 0x00
#define IST8310_R_MODE 0x02

#define IST8310_R_XL 0x03
#define IST8310_R_XM 0x04
#define IST8310_R_YL 0x05
#define IST8310_R_YM 0x06
#define IST8310_R_ZL 0x07
#define IST8310_R_ZM 0x08

#define IST8310_R_CONFA 0x0A
#define IST8310_R_CONFB 0x0B

#define IST8310_AVGCNTL 0x41
#define IST8310_PDCNTL 0x42

#define IST8310_ODR_MODE 0x01 //sigle measure mode

typedef struct
{
	float mx;
	float my;
	float mz;
}magnetometer_t;

u8 ist8310_ReadOneByte(u8 address, u8 reg);
void ist8310_WriteOneByte(u8 address,u8 reg,u8 command);
u8 ist8310Init(void);
void ist8310_ReadBuffer(u8 address, u8 reg, u8 *buffer, u8 len);
short ist8310_ReadProcess(void);

#endif
