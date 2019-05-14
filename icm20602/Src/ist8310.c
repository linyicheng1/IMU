/**********************************************************************************************************
 * @文件     ist8310.c
 * @说明     模拟i2c读取磁力计数据
 * @版本  	 V1.0
 * @作者     刘垚/林一成
 * @日期     2019.01
**********************************************************************************************************/
#include "ist8310.h"
#include "i2c.h"
#include "timer.h"
#include "math.h"

u8 ist8310_name = 0;
u8 ist8310_reg1 = 0;  //should be 0x24
u8 ist8310_reg2 = 0;  //should be 0xC0

magnetometer_t mag_data;//磁力计数据
float mag_x_bias = 0;//磁力计数据校准后的偏差
float mag_y_bias = 0;
float mag_z_bias = 0;

float mag_x_scale = 1;//磁力计数据校准后的缩放因子
float mag_y_scale = 1;
float mag_z_scale = 1;
/**********************************************************************************************************
*函 数 名: ist8310Init
*功能说明: ist8310初始化，并读取寄存器1和2，与name，并判断是否读取成功
*形    参: 无
*返 回 值: 
			1----reg1错误
			2----reg2错误
			3----name错误
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
*函 数 名: ist8310_ReadOneByte
*功能说明: i2c读取一位寄存器数据
*形    参: 
           address-器件地址
           reg-----寄存器地址
*返 回 值: 寄存器的值
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
    IIC_Send_Byte(address + 1); //进入接收模式 
    IIC_Wait_Ack();
    temp = IIC_Read_Byte(0);  //读寄存器 3
    IIC_Stop();//产生一个停止条件 
    return temp;
}
/**********************************************************************************************************
*函 数 名: ist8310_ReadBuffer
*功能说明: i2c读取寄存器数据
*形    参: 
           address-器件地址
           reg-----寄存器地址
           buffer--数组指针
           len-----长度
*返 回 值: 无
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
*函 数 名: ist8310_WriteOneByte
*功能说明: i2c写寄存器
*形    参: 
           address-器件地址
           reg-----寄存器地址
           command-写入的数据
*返 回 值: 无
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
    IIC_Stop();//产生一个停止条件
}
/**********************************************************************************************************
*函 数 名: ist8310_ReadProcess
*功能说明: 磁力计读取数据
*形    参: 无
*返 回 值: 
			1 读取成功
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
*函 数 名: ist8310_calibration
*功能说明: LM算法校准磁力计数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ist8310_calibration(void)
{
	
}
