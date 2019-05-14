/**********************************************************************************************************
 * @文件     timer.c
 * @说明     调用梯度下降和卡尔曼函数解算姿态
 * @版本  	 V1.0
 * @作者     林一成
 * @日期     2019.01
**********************************************************************************************************/
#include "timer.h"
#include "icm20602.h"
#include "ahrs.h"
#include "ist8310.h"
#include "math.h"
#include "kalman_filter.h"
#include "can.h"

extern TIM_HandleTypeDef htim2;
/**
  * @brief  使用TIM2做微秒级延时
  * @param  None
  * @retval None
  */
void delay_us(int tim)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);   //htim2
	__HAL_TIM_ENABLE(&htim2);
	while(__HAL_TIM_GET_COUNTER(&htim2) < (48*tim));  //48MHz
	__HAL_TIM_DISABLE(&htim2);
}

/**
  * @brief  TIMER中断回调函数
  * @param  None
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==(&htim2))
    {
	}
}
/************************
**Attitude solution part.
************************/
#define T 0.005f
extern float gyro_bias[3];
short ms5;

float icm_gyro[3];//陀螺仪数据
float icm_accel[3];//加速度计数据
extern magnetometer_t mag_data;//磁力计数据
//陀螺仪纯粹积分得到的角度
float pure_gyro_pitch;
float pure_gyro_yaw;
//加速度计计算出来的角度，假设只有重力加速度
float pure_acc_pitch;
float acc_inti_norm = 0;
//磁力计计算出来的角度，梯度下降法
extern struct attitude mag_attitude;//通过磁场计算出来的姿态
//yaw轴互补滤波参数
float k_filter = 0.02;//互补滤波系数
float combAngle = 0;  //互补滤波角度
float last_magYaw = 0;
//pitch轴互补滤波参数
float k_filter_pitch = 0.01;//互补滤波系数
float combAngle_pitch = 0;  //互补滤波角度
float last_acc_pitch = 0;
//yaw轴组合角度
float spliceYaw = 0;//组合角度
double last_yaw_angle = 0 ;
double last_yaw_angle2 = 0 ;
short spliceFlag = 0;
short spliceCnt = 0;
short breakCnt = 0;
double spliceThrehold = 0.06;
// yaw 时间参数
short break_gyro_yaw = 5;//避免抖动
short comb_time_yaw = 100;//进入互补滤波范围
short kalman_time_yaw = 5000;//进入卡尔曼滤波范围
float yaw_mag_bias = 0;
// pitch 时间参数
//英雄
// 10 3000
short comb_time_pitch = 10;//进入互补滤波范围
short kalman_time_pitch = 3000;//进入卡尔曼滤波范围
//pitch轴组合角度


float splicePitch = 0;//组合角度
double last_pitch_angle = 0 ;
double last_pitch_angle2 = 0 ;
short spliceFlag_pitch = 0;
short spliceCnt_pitch = 0;
double spliceThrehold_pitch = 0.06;//步兵

//pitch 时间参数

//double spliceThrehold_pitch = 1;//哨兵
//卡尔曼滤波角度
float pitch_angle;
float yaw_angle;   
float pitch_angle_dot;    				
float yaw_angle_dot;
//初始化标志位
uint8_t begin_flag = 0;
extern int8_t IMU_init_flag;
float delta_pitch;
float acc_norm;
float k_com = 0.005;
float delta_yaw;
float init_flag = 0;
void HAL_SYSTICK_Callback(void)
{
	
	ms5 ++;		
	if(ms5 >= 5&&IMU_init_flag ==1 )//每5ms解算一次
	{
		
		icm20602_get_gyro(icm_gyro);
		icm20602_get_accel(icm_accel);
		//通过通过加速度计得到pitch角度信息
		pure_acc_pitch = -atan2((double)icm_accel[0], (double)icm_accel[2]) / 3.1415926f * 180.0f;

		pure_gyro_pitch += (icm_gyro[1] - gyro_bias[1]) * T / 3.1415926f * 180.0f;
		pure_gyro_yaw += (icm_gyro[2] - gyro_bias[2]) * T / 3.1415926f * 180.0f;
		//yaw轴互补滤波，为了保持连续性磁力计角度采取增量形式
//		combAngle = combAngle + (icm_gyro[2]-gyro_bias[2])*T/3.1415926f * 180.0f*(1 - k_filter) + (mag_attitude.yaw - last_magYaw)*k_filter;
//		last_magYaw = mag_attitude.yaw ;
		combAngle = (combAngle + (icm_gyro[2]-gyro_bias[2])*T/3.1415926f * 180.0f)*(1 - k_filter) + mag_attitude.yaw * k_filter;
		
		acc_norm = icm_accel[0]*icm_accel[0]+icm_accel[1]*icm_accel[1]+icm_accel[2]*icm_accel[2];
		if(acc_inti_norm<60)
		{//通常是80多
			acc_inti_norm = acc_norm;
		}
		k_filter_pitch = -k_com*fabsf(acc_norm-acc_inti_norm)+0.02;//自适应参数
		if(k_filter_pitch<0)//限制幅度
			k_filter_pitch = 0;
		else if(k_filter_pitch>0.02)
		{
			k_filter_pitch = 0.02;
		}
		combAngle_pitch = (combAngle_pitch +(icm_gyro[1]-gyro_bias[1])*T/3.1415926f * 180.0f)*(1 - k_filter_pitch) + (pure_acc_pitch)*k_filter_pitch;
		last_acc_pitch = pure_acc_pitch ;
		
		//卡尔曼
		Kalman_Filter_pitch(pure_acc_pitch, icm_gyro[1]);
		Kalman_Filter_yaw(mag_attitude.yaw, icm_gyro[2]);
		//由于快速旋转时磁力计数据抖动很大且没有进行校准
		//动态时YAW轴采用校准后的陀螺仪数据，静止时采用卡尔曼滤波后的数据
		if((icm_gyro[2]>spliceThrehold||icm_gyro[2]<-spliceThrehold))
		{//如果在动态情况下
			if(breakCnt>break_gyro_yaw)
			{
				spliceFlag = 1;
				spliceYaw += (icm_gyro[2] - gyro_bias[2]) * T / 3.1415926f * 180.0f;
				spliceCnt = 0;
			}
			else 
			{
				spliceFlag = -1;
				spliceYaw += (combAngle - last_yaw_angle);
			}
			breakCnt ++;
		}
		else if(icm_gyro[2]<spliceThrehold&&icm_gyro[2]>-spliceThrehold)
		{//静态情况下
			spliceCnt ++;
			if(spliceCnt>comb_time_yaw&&spliceCnt<kalman_time_yaw)
			{//维持一段时间静态才使用互补数据避免频繁跳动
				spliceFlag = -1;
				spliceYaw += (combAngle - last_yaw_angle);
			}
			else if(spliceCnt>=kalman_time_yaw)
			{
				spliceFlag = -2;
	
				spliceYaw += (yaw_angle - last_yaw_angle2);
			}
			else
			{
				spliceFlag = 1;
				spliceYaw += (icm_gyro[2] - gyro_bias[2]) * T / 3.1415926f * 180.0f;
			
			}
			breakCnt = 0;
		}
		else 
		{
			spliceFlag = 0;
			spliceYaw += (icm_gyro[2] - gyro_bias[2]) * T / 3.1415926f * 180.0f;
		}
		last_yaw_angle = combAngle;
		last_yaw_angle2 = yaw_angle;
		//动态时pitch轴采用校准后的陀螺仪数据，静止时采用卡尔曼滤波后的数据
		//由于动态下加速度计数据不准
		
		
//		if(icm_gyro[1]>spliceThrehold_pitch||icm_gyro[1]<-spliceThrehold_pitch)
//		{//动态情况下
//			if(spliceFlag_pitch!=1)
//			{
//				delta_pitch = splicePitch - combAngle_pitch;
//				spliceFlag_pitch = 1;
//			}
//			splicePitch = (combAngle_pitch + delta_pitch);
//			spliceCnt_pitch = 0;
//		}
//		else if((icm_gyro[1]<spliceThrehold_pitch)&&(icm_gyro[1]>-spliceThrehold_pitch)&&
//			((combAngle_pitch - last_pitch_angle)<spliceThrehold_pitch)&&(combAngle_pitch - last_pitch_angle)>-spliceThrehold_pitch)
//		{
//			spliceCnt_pitch ++;
//			if(spliceCnt_pitch >comb_time_pitch&&spliceCnt_pitch<kalman_time_pitch)
//			{
//				if(spliceFlag_pitch!=-1)
//				{
//					delta_pitch = splicePitch - combAngle_pitch;
//					spliceFlag_pitch = -1;
//				}
//				splicePitch = (combAngle_pitch + delta_pitch);
//			}
//			else if(spliceCnt_pitch>=kalman_time_pitch)
//			{
//				if(spliceFlag_pitch!=-2)
//				{
//					delta_pitch = splicePitch - pitch_angle;
//					spliceFlag_pitch = -2;
//				}
//				
//				splicePitch = pitch_angle + delta_pitch;
//			}
//		}	
//		else
//		{
//			splicePitch -= (icm_gyro[1] - gyro_bias[1]) * T / 3.1415926f * 180.0f; 
//			spliceFlag_pitch = 0;
//			spliceCnt_pitch = 0;
//		}			
		last_pitch_angle = combAngle_pitch;
		last_pitch_angle2 = pitch_angle;
		if(init_flag == 0)
		{//初始化
			combAngle = mag_attitude.yaw;
			combAngle_pitch = pure_acc_pitch;
			yaw_angle = mag_attitude.yaw;
			acc_inti_norm = icm_accel[0]*icm_accel[0] + icm_accel[1]*icm_accel[1] +icm_accel[1]*icm_accel[1];
			init_flag = 1;
		}
		//发送can数据
		//可以选择不同的数据发送
		CANSendMag(&combAngle_pitch, &icm_gyro[1],0x100);//避免计速度计带来的点头现象
		CANSendMag(&spliceYaw,&icm_gyro[2],0x101);
		ms5 = 0;
	}
}
