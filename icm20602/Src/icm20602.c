/**********************************************************************************************************
 * @文件     icm20602.c
 * @说明     读取陀螺仪与加速度计数据
 * @版本  	 V1.0
 * @作者     刘垚/林一成
 * @日期     2019.01
**********************************************************************************************************/
#include "icm20602.h"
extern SPI_HandleTypeDef hspi1;

//内部实现函数，操作寄存器
static uint8_t icm20602_write_reg(uint8_t reg,uint8_t val);
static uint8_t icm20602_read_reg(uint8_t reg);
static uint8_t icm20602_read_buffer(uint8_t reg,uint8_t *buffer,uint8_t len);
/**********************************************************************************************************
*函 数 名:  icm20602_init
*功能说明:  icm20602初始化，读取id判断是否正常工作，设置量程为
            ICM20_ACCEL_FS_4G
			ICM20_GYRO_FS_500
*形    参:  无
*返 回 值:  1 --- 失败
			0 --- 成功
**********************************************************************************************************/
uint8_t icm20602_init(void)
{ 
	uint8_t id;
	icm20602_write_reg(ICM20_PWR_MGMT_1,0x80);	//复位，复位后位0x41,睡眠模式，
	HAL_Delay(100);
	icm20602_write_reg(ICM20_PWR_MGMT_1,0x01);		//关闭睡眠，自动选择时钟
	HAL_Delay(100);
		
	id = icm20602_read_reg(ICM20_WHO_AM_I);//读取ID	

	if(id != 0x12)
	{
		return 1;
	}
	
	icm20602_write_reg(ICM20_PWR_MGMT_2, 0x00);      
	HAL_Delay(10);
	
	icm20602_write_reg(ICM20_SMPLRT_DIV,0);			//分频数=为0+1，数据输出速率为内部采样速率
	HAL_Delay(10);
	icm20602_write_reg(ICM20_CONFIG,DLPF_BW_20);	//GYRO低通滤波设置
	HAL_Delay(10);
	icm20602_write_reg(ICM20_ACCEL_CONFIG2,ACCEL_AVER_4|ACCEL_DLPF_BW_21);	//ACCEL低通滤波设置
	HAL_Delay(10);
	
	//设置量程
	icm20602_set_accel_fullscale(ICM20_ACCEL_FS_4G);
	HAL_Delay(10);
	icm20602_set_gyro_fullscale(ICM20_GYRO_FS_1000);
	HAL_Delay(10);
	
	icm20602_write_reg(ICM20_LP_MODE_CFG, 0x00);	//关闭低功耗
	HAL_Delay(10);
	icm20602_write_reg(ICM20_FIFO_EN, 0x00);		//关闭FIFO
	HAL_Delay(10);
	
	HAL_Delay(100);
	
	return 0;
}
/**********************************************************************************************************
*函 数 名: icm20602_set_accel_fullscale
*功能说明: 设置加速度计的量程
*形    参:  fs 量程
			ICM20_ACCEL_FS_2G
			ICM20_ACCEL_FS_4G
			ICM20_ACCEL_FS_8G
			ICM20_ACCEL_FS_16G
*返 回 值:  1 成功
			0 失败
**********************************************************************************************************/
uint8_t icm20602_set_accel_fullscale(uint8_t fs)
{
	switch(fs)
	{
		case ICM20_ACCEL_FS_2G:
			_accel_scale = 1.0f/16348.0f;
		break;
		case ICM20_ACCEL_FS_4G:
			_accel_scale = 1.0f/8192.0f;
		break;
		case ICM20_ACCEL_FS_8G:
			_accel_scale = 1.0f/4096.0f;
		break;
		case ICM20_ACCEL_FS_16G:
			_accel_scale = 1.0f/2048.0f;
		break;
		default:
			fs = ICM20_ACCEL_FS_8G;
			_accel_scale = 1.0f/4096.0f;
		break;

	}
	_accel_scale *= GRAVITY_MSS;
	return icm20602_write_reg(ICM20_ACCEL_CONFIG,fs);
}
/**********************************************************************************************************
*函 数 名: icm20602_set_gyro_fullscale
*功能说明: 设置陀螺仪的量程
*形    参:  fs 量程
			ICM20_GYRO_FS_250
			ICM20_GYRO_FS_500
			ICM20_GYRO_FS_1000
			ICM20_GYRO_FS_2000
*返 回 值:  1 成功
			0 失败
**********************************************************************************************************/
uint8_t icm20602_set_gyro_fullscale(uint8_t fs)
{
	switch(fs)
	{
		case ICM20_GYRO_FS_250:
			_gyro_scale = 1.0f/131.068f;	//32767/250
		break;
		case ICM20_GYRO_FS_500:
			_gyro_scale = 1.0f/65.534f;
		break;
		case ICM20_GYRO_FS_1000:
			_gyro_scale = 1.0f/32.767f;
		break;
		case ICM20_GYRO_FS_2000:
			_gyro_scale = 1.0f/16.3835f;
		break;
		default:
			fs = ICM20_GYRO_FS_2000;
			_gyro_scale = 1.0f/16.3835f;
		break;

	}
	_gyro_scale *= _DEG_TO_RAD;
	return icm20602_write_reg(ICM20_GYRO_CONFIG,fs);
	
}
/**********************************************************************************************************
*函 数 名:  icm20602_get_accel_adc
*功能说明:  读取加速度计的数据的内部实现函数
*形    参:  accel 加速度计数据的数组指针
*返 回 值:  0 成功
**********************************************************************************************************/
uint8_t icm20602_get_accel_adc(int16_t *accel)
{
	uint8_t buf[6];
	if(icm20602_read_buffer(ICM20_ACCEL_XOUT_H,buf,6))return 1;
	
	accel[0] = ((int16_t)buf[0]<<8) + buf[1];
	accel[1] = ((int16_t)buf[2]<<8) + buf[3];
	accel[2] = ((int16_t)buf[4]<<8) + buf[5];
	return 0;
}
/**********************************************************************************************************
*函 数 名:  icm20602_get_gyro_adc
*功能说明:  读取陀螺仪的数据的内部实现函数
*形    参:  gyro 陀螺仪数据的数组指针
*返 回 值:  0 成功
**********************************************************************************************************/
uint8_t icm20602_get_gyro_adc(int16_t *gyro)
{
	uint8_t buf[6];
	if(icm20602_read_buffer(ICM20_GYRO_XOUT_H,buf,6))return 1;
	gyro[0] = (buf[0]<<8) + buf[1];
	gyro[1] = (buf[2]<<8) + buf[3];
	gyro[2] = (buf[4]<<8) + buf[5];
	return 0;
}
/**********************************************************************************************************
*函 数 名:  icm20602_get_gyro
*功能说明:  读取陀螺仪的数据
*形    参:  gyro 陀螺仪数据的数组指针
*返 回 值:  0 成功
**********************************************************************************************************/
uint8_t icm20602_get_gyro(float *gyro)
{
	int16_t gyro_adc[3];
	if(icm20602_get_gyro_adc(gyro_adc))return 1;
	
	gyro[0] = _gyro_scale * gyro_adc[0];
	gyro[1] = _gyro_scale * gyro_adc[1];
	gyro[2] = _gyro_scale * gyro_adc[2];	
	return 0;
}
/**********************************************************************************************************
*函 数 名:  icm20602_get_accel
*功能说明:  读取加速度计的数据
*形    参:  accel 加速度计数据的数组指针
*返 回 值:  0 成功
**********************************************************************************************************/
uint8_t icm20602_get_accel(float *accel)
{
	int16_t accel_adc[3];
	if(icm20602_get_accel_adc(accel_adc))return 1;
	accel[0] = _accel_scale * accel_adc[0];
	accel[1] = _accel_scale * accel_adc[1];
	accel[2] = _accel_scale * accel_adc[2];	
	return 0;
}

float icm20602_get_temp()
{
	int16_t temp_adc;
	uint8_t buf[2];
	if(icm20602_read_buffer(ICM20_TEMP_OUT_H,buf,2))return 0.0f;

	temp_adc = (buf[0]<<8)+buf[1];

	return (25.0f + (float)temp_adc/326.8f);
}
/**********************************************************************************************************
*函 数 名:  icm20602_write_reg
*功能说明:  通过spi写寄存器
*形    参:  reg --- 寄存器
			val --- 数值
*返 回 值:  无
**********************************************************************************************************/
static uint8_t icm20602_write_reg(uint8_t reg,uint8_t val)
{	
	uint8_t send[2]={reg,val};
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(&hspi1,send,2,20) != HAL_OK)
		return 1;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);

	return 0;
}
/**********************************************************************************************************
*函 数 名:  icm20602_read_reg
*功能说明:  通过spi读取寄存器
*形    参:  reg --- 寄存器
*返 回 值:	读到的数值
**********************************************************************************************************/
static uint8_t icm20602_read_reg(uint8_t reg)
{
	uint8_t send=reg|0x80;
	uint8_t get;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(&hspi1,&send,1,10) != HAL_OK)
		return 1;	
	if(HAL_SPI_Receive(&hspi1,&get,1,10) != HAL_OK)
		return 1;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);

	return get;
}
/**********************************************************************************************************
*函 数 名:  icm20602_read_buffer
*功能说明:  通过spi读取寄存器数组
*形    参:  reg ------ 寄存器
			buffer --- 数组指针寄存器
			len ------ 长度
*返 回 值:  无
**********************************************************************************************************/
static uint8_t icm20602_read_buffer(uint8_t reg,uint8_t *buffer,uint8_t len)
{
	uint8_t send=reg|0x80;

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&send,1,5);	
	HAL_SPI_Receive(&hspi1,buffer,len,30);	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);

	return 0;
}
/**********************************************************************************************************
*函 数 名: icm_offset_call
*功能说明: 延时500ms，计算磁力计的偏差
*形    参: offset 偏差的数组
*返 回 值: 无
**********************************************************************************************************/
void icm_offset_call(float *offset)
{
	int i;
	float gyro[3];
	float gyro_sum[3];
	for(i=0;i<100;i++)
	{
		icm20602_get_gyro(gyro);
		gyro_sum[0] += gyro[0];
		gyro_sum[1] += gyro[1];
		gyro_sum[2] += gyro[2];
		
		HAL_Delay(5);
	}
	offset[0] = gyro_sum[0] / 100;
	offset[1] = gyro_sum[1] / 100;
	offset[2] = gyro_sum[2] / 100;
}
