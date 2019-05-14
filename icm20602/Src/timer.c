/**********************************************************************************************************
 * @�ļ�     timer.c
 * @˵��     �����ݶ��½��Ϳ���������������̬
 * @�汾  	 V1.0
 * @����     ��һ��
 * @����     2019.01
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
  * @brief  ʹ��TIM2��΢�뼶��ʱ
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
  * @brief  TIMER�жϻص�����
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

float icm_gyro[3];//����������
float icm_accel[3];//���ٶȼ�����
extern magnetometer_t mag_data;//����������
//�����Ǵ�����ֵõ��ĽǶ�
float pure_gyro_pitch;
float pure_gyro_yaw;
//���ٶȼƼ�������ĽǶȣ�����ֻ���������ٶ�
float pure_acc_pitch;
float acc_inti_norm = 0;
//�����Ƽ�������ĽǶȣ��ݶ��½���
extern struct attitude mag_attitude;//ͨ���ų������������̬
//yaw�ụ���˲�����
float k_filter = 0.02;//�����˲�ϵ��
float combAngle = 0;  //�����˲��Ƕ�
float last_magYaw = 0;
//pitch�ụ���˲�����
float k_filter_pitch = 0.01;//�����˲�ϵ��
float combAngle_pitch = 0;  //�����˲��Ƕ�
float last_acc_pitch = 0;
//yaw����ϽǶ�
float spliceYaw = 0;//��ϽǶ�
double last_yaw_angle = 0 ;
double last_yaw_angle2 = 0 ;
short spliceFlag = 0;
short spliceCnt = 0;
short breakCnt = 0;
double spliceThrehold = 0.06;
// yaw ʱ�����
short break_gyro_yaw = 5;//���ⶶ��
short comb_time_yaw = 100;//���뻥���˲���Χ
short kalman_time_yaw = 5000;//���뿨�����˲���Χ
float yaw_mag_bias = 0;
// pitch ʱ�����
//Ӣ��
// 10 3000
short comb_time_pitch = 10;//���뻥���˲���Χ
short kalman_time_pitch = 3000;//���뿨�����˲���Χ
//pitch����ϽǶ�


float splicePitch = 0;//��ϽǶ�
double last_pitch_angle = 0 ;
double last_pitch_angle2 = 0 ;
short spliceFlag_pitch = 0;
short spliceCnt_pitch = 0;
double spliceThrehold_pitch = 0.06;//����

//pitch ʱ�����

//double spliceThrehold_pitch = 1;//�ڱ�
//�������˲��Ƕ�
float pitch_angle;
float yaw_angle;   
float pitch_angle_dot;    				
float yaw_angle_dot;
//��ʼ����־λ
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
	if(ms5 >= 5&&IMU_init_flag ==1 )//ÿ5ms����һ��
	{
		
		icm20602_get_gyro(icm_gyro);
		icm20602_get_accel(icm_accel);
		//ͨ��ͨ�����ٶȼƵõ�pitch�Ƕ���Ϣ
		pure_acc_pitch = -atan2((double)icm_accel[0], (double)icm_accel[2]) / 3.1415926f * 180.0f;

		pure_gyro_pitch += (icm_gyro[1] - gyro_bias[1]) * T / 3.1415926f * 180.0f;
		pure_gyro_yaw += (icm_gyro[2] - gyro_bias[2]) * T / 3.1415926f * 180.0f;
		//yaw�ụ���˲���Ϊ�˱��������Դ����ƽǶȲ�ȡ������ʽ
//		combAngle = combAngle + (icm_gyro[2]-gyro_bias[2])*T/3.1415926f * 180.0f*(1 - k_filter) + (mag_attitude.yaw - last_magYaw)*k_filter;
//		last_magYaw = mag_attitude.yaw ;
		combAngle = (combAngle + (icm_gyro[2]-gyro_bias[2])*T/3.1415926f * 180.0f)*(1 - k_filter) + mag_attitude.yaw * k_filter;
		
		acc_norm = icm_accel[0]*icm_accel[0]+icm_accel[1]*icm_accel[1]+icm_accel[2]*icm_accel[2];
		if(acc_inti_norm<60)
		{//ͨ����80��
			acc_inti_norm = acc_norm;
		}
		k_filter_pitch = -k_com*fabsf(acc_norm-acc_inti_norm)+0.02;//����Ӧ����
		if(k_filter_pitch<0)//���Ʒ���
			k_filter_pitch = 0;
		else if(k_filter_pitch>0.02)
		{
			k_filter_pitch = 0.02;
		}
		combAngle_pitch = (combAngle_pitch +(icm_gyro[1]-gyro_bias[1])*T/3.1415926f * 180.0f)*(1 - k_filter_pitch) + (pure_acc_pitch)*k_filter_pitch;
		last_acc_pitch = pure_acc_pitch ;
		
		//������
		Kalman_Filter_pitch(pure_acc_pitch, icm_gyro[1]);
		Kalman_Filter_yaw(mag_attitude.yaw, icm_gyro[2]);
		//���ڿ�����תʱ���������ݶ����ܴ���û�н���У׼
		//��̬ʱYAW�����У׼������������ݣ���ֹʱ���ÿ������˲��������
		if((icm_gyro[2]>spliceThrehold||icm_gyro[2]<-spliceThrehold))
		{//����ڶ�̬�����
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
		{//��̬�����
			spliceCnt ++;
			if(spliceCnt>comb_time_yaw&&spliceCnt<kalman_time_yaw)
			{//ά��һ��ʱ�侲̬��ʹ�û������ݱ���Ƶ������
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
		//��̬ʱpitch�����У׼������������ݣ���ֹʱ���ÿ������˲��������
		//���ڶ�̬�¼��ٶȼ����ݲ�׼
		
		
//		if(icm_gyro[1]>spliceThrehold_pitch||icm_gyro[1]<-spliceThrehold_pitch)
//		{//��̬�����
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
		{//��ʼ��
			combAngle = mag_attitude.yaw;
			combAngle_pitch = pure_acc_pitch;
			yaw_angle = mag_attitude.yaw;
			acc_inti_norm = icm_accel[0]*icm_accel[0] + icm_accel[1]*icm_accel[1] +icm_accel[1]*icm_accel[1];
			init_flag = 1;
		}
		//����can����
		//����ѡ��ͬ�����ݷ���
		CANSendMag(&combAngle_pitch, &icm_gyro[1],0x100);//������ٶȼƴ����ĵ�ͷ����
		CANSendMag(&spliceYaw,&icm_gyro[2],0x101);
		ms5 = 0;
	}
}
