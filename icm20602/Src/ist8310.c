/**********************************************************************************************************
 * @�ļ�     ist8310.c
 * @˵��     ģ��i2c��ȡ����������
 * @�汾  	 V1.0
 * @����     ����/��һ��
 * @����     2019.01
**********************************************************************************************************/
#include "ist8310.h"
#include "i2c.h"
#include "timer.h"
#include "math.h"

u8 ist8310_name = 0;
u8 ist8310_reg1 = 0;  //should be 0x24
u8 ist8310_reg2 = 0;  //should be 0xC0

magnetometer_t mag_data;//����������
float mag_x_bias = 0;//����������У׼���ƫ��
float mag_y_bias = 0;
float mag_z_bias = 0;

float mag_x_scale = 1;//����������У׼�����������
float mag_y_scale = 1;
float mag_z_scale = 1;
/**********************************************************************************************************
*�� �� ��: ist8310Init
*����˵��: ist8310��ʼ��������ȡ�Ĵ���1��2����name�����ж��Ƿ��ȡ�ɹ�
*��    ��: ��
*�� �� ֵ: 
			1----reg1����
			2----reg2����
			3----name����
**********************************************************************************************************/
u8 ist8310Init(void)
{
	ist8310_WriteOneByte(IST8310_ADDRESS, IST8310_AVGCNTL, 0x24);
	ist8310_reg1 = ist8310_ReadOneByte(IST8310_ADDRESS, IST8310_AVGCNTL);
	if(ist8310_reg1 != 0x24)
		return 1;
	HAL_Delay(10);
	ist8310_WriteOneByte(IST8310_ADDRESS, IST8310_PDCNTL, 0xC0);
	ist8310_reg2 = ist8310_ReadOneByte(IST8310_ADDRESS, IST8310_PDCNTL);
	if(ist8310_reg2 != 0xC0)
		return 2;
	
	HAL_Delay(10);
	ist8310_name = ist8310_ReadOneByte(IST8310_ADDRESS, IST8310_WHO_AM_I);
	if(ist8310_name != IST8310_DEVICE_ID_A)
		return 3;
	return 0;
}
/**********************************************************************************************************
*�� �� ��: ist8310_ReadOneByte
*����˵��: i2c��ȡһλ�Ĵ�������
*��    ��: 
           address-������ַ
           reg-----�Ĵ�����ַ
*�� �� ֵ: �Ĵ�����ֵ
**********************************************************************************************************/
u8 ist8310_ReadOneByte(u8 address, u8 reg)
{ 
    u8 temp = 0;
    IIC_Start();
    IIC_Send_Byte(address); 
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);     
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(address + 1); //�������ģʽ 
    IIC_Wait_Ack();
    temp = IIC_Read_Byte(0);  //���Ĵ��� 3
    IIC_Stop();//����һ��ֹͣ���� 
    return temp;
}
/**********************************************************************************************************
*�� �� ��: ist8310_ReadBuffer
*����˵��: i2c��ȡ�Ĵ�������
*��    ��: 
           address-������ַ
           reg-----�Ĵ�����ַ
           buffer--����ָ��
           len-----����
*�� �� ֵ: ��
**********************************************************************************************************/
void ist8310_ReadBuffer(u8 address, u8 reg, u8 *buffer, u8 len)
{
	int i;
	IIC_Start();
	IIC_Send_Byte(address); 
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(address + 1);  //begin to read process
	IIC_Wait_Ack();
	for(i=0;i<len-1;i++)
	{
		*buffer = IIC_Read_Byte(1);   //send ack
		buffer ++;
	}
	*buffer = IIC_Read_Byte(0);       //don't send ack
	IIC_Stop();
}
/**********************************************************************************************************
*�� �� ��: ist8310_WriteOneByte
*����˵��: i2cд�Ĵ���
*��    ��: 
           address-������ַ
           reg-----�Ĵ�����ַ
           command-д�������
*�� �� ֵ: ��
**********************************************************************************************************/
void ist8310_WriteOneByte(u8 address,u8 reg,u8 command)
{ 
    IIC_Start();
    IIC_Send_Byte(address);  
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Send_Byte(command); 
    IIC_Wait_Ack();
    IIC_Stop();//����һ��ֹͣ����
}
/**********************************************************************************************************
*�� �� ��: ist8310_ReadProcess
*����˵��: �����ƶ�ȡ����
*��    ��: ��
*�� �� ֵ: 
			1 ��ȡ�ɹ�
**********************************************************************************************************/
short ist8310_ReadProcess(void)
{
	u8 buffer1[6];
	int16_t mag_temp[3];
	float mag_temp1;     
	float mag_temp2;
	float mag_temp3;
	ist8310_WriteOneByte(IST8310_ADDRESS, IST8310_R_CONFA, IST8310_ODR_MODE);  //single measurement mode

	ist8310_ReadBuffer(IST8310_ADDRESS, IST8310_R_XL, buffer1, 6);

	mag_temp[0] = buffer1[0] | (buffer1[1] << 8);
	mag_temp[1] = buffer1[2] | (buffer1[3] << 8);
	mag_temp[2] = buffer1[4] | (buffer1[5] << 8);

		//ist8310 magnetometer calibration.
	mag_temp1 = (mag_temp[0] - (mag_x_bias))/mag_x_scale;
	mag_temp2 = (mag_temp[1] - (mag_y_bias))/mag_y_scale;
	mag_temp3 = (mag_temp[2] - (mag_z_bias))/mag_z_scale;
		//Complementary filter.
	mag_data.mx = mag_temp1 ;
	mag_data.my = mag_temp2 ;
	mag_data.mz = mag_temp3 ;
	
	return 1;
}
/**********************************************************************************************************
*�� �� ��: ist8310_calibration
*����˵��: LM�㷨У׼����������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ist8310_calibration(void)
{
	
}
