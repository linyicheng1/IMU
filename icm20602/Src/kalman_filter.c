/**********************************************************************************************************
 * @文件     madgwick.c
 * @说明     通过磁力计计算角度
 * @版本  	 V1.0
 * @作者     
 * @日期     2019.01
**********************************************************************************************************/

#include "kalman_filter.h"
#include "stm32f0xx_hal.h"
#include "ahrs.h"
float dt=0.005;   // 5ms    

float Q_angle_pitch=1	;// 过程噪声的协方差
float Q_gyro_pitch=1;//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle_pitch=5000;// 测量噪声的协方差 既测量偏差
            
char  C_0_pitch = 1;
float Q_bias_pitch, Angle_err_pitch;
float PCt_0_pitch, PCt_1_pitch, E_pitch;
float K_0_pitch, K_1_pitch, t_0_pitch, t_1_pitch;
float Pdot_pitch[4] ={0,0,0,0};
float PP_pitch[2][2] = { { 0, 0 },{ 0, 0 } };
uint8_t init_pitch_flag = 0;
extern uint8_t alige_flag;
float pitch_angle_est;
float K_bias = 1;
extern float gyro_bias[3];
extern float pure_gyro_pitch;
extern float splicePitch;//组合角度
extern float acc_inti_norm;
extern float icm_accel[3];//加速度计数据
extern struct attitude mag_attitude;//通过磁场计算出来的姿态
void KF_DeInit_pitch(void);
/**************************************************************************
函数功能：简易卡尔曼滤波
入口参数：加速度得到的角度值、角速度
返回  值：无
**************************************************************************/
void Kalman_Filter_pitch(float Accel,float Gyro)		
{

	{
		pitch_angle_est = pitch_angle - (Gyro - gyro_bias[1]) * dt/ 3.1415926f * 180.0f; //先验估计
		Pdot_pitch[0]=Q_angle_pitch - PP_pitch[0][1] - PP_pitch[1][0]; // Pk-先验估计误差协方差的微分

		Pdot_pitch[1]=-PP_pitch[1][1];
		Pdot_pitch[2]=-PP_pitch[1][1];
		Pdot_pitch[3]=Q_gyro_pitch;
		
		PP_pitch[0][0] += Pdot_pitch[0] * dt;   // Pk-先验估计误差协方差微分的积分
		PP_pitch[0][1] += Pdot_pitch[1] * dt;   // =先验估计误差协方差
		PP_pitch[1][0] += Pdot_pitch[2] * dt;
		PP_pitch[1][1] += Pdot_pitch[3] * dt;
			
		Angle_err_pitch = Accel - pitch_angle_est;	//zk-先验估计
		
		PCt_0_pitch = C_0_pitch * PP_pitch[0][0];
		PCt_1_pitch = C_0_pitch * PP_pitch[1][0];
		
		E_pitch = R_angle_pitch + C_0_pitch * PCt_0_pitch;
		
		K_0_pitch = PCt_0_pitch / E_pitch;
		K_1_pitch = PCt_1_pitch / E_pitch;
		
		t_0_pitch = PCt_0_pitch;
		t_1_pitch = C_0_pitch * PP_pitch[0][1];

		PP_pitch[0][0] -= K_0_pitch * t_0_pitch;		 //后验估计误差协方差
		PP_pitch[0][1] -= K_0_pitch * t_1_pitch;
		PP_pitch[1][0] -= K_1_pitch * t_0_pitch;
		PP_pitch[1][1] -= K_1_pitch * t_1_pitch;
			
		pitch_angle	= pitch_angle_est+ K_0_pitch * Angle_err_pitch;	 //后验估计
		Q_bias_pitch += K_1_pitch * Angle_err_pitch;	 //后验估计
		pitch_angle_dot   = Gyro - Q_bias_pitch;	 //输出值(后验估计)的微分=角速度

	}
}
void KF_DeInit_pitch(void)
{
	
	C_0_pitch = 1;
    Q_bias_pitch=0; Angle_err_pitch=0;
    PCt_0_pitch=0; PCt_1_pitch=0; E_pitch=0;
    K_0_pitch=0, K_1_pitch=0, t_0_pitch=0, t_1_pitch=0;
	for(int i = 0;i<4;i++)
	{
		Pdot_pitch[i] = 0;
	}
    PP_pitch[0][0] = 1;
	PP_pitch[0][1] = 0;
	PP_pitch[1][0] = 0;
	PP_pitch[1][0] = 1;
}

///////////////////////////////////////////////////////
float Q_angle_yaw=0.1;// 过程噪声的协方差
float Q_gyro_yaw=0.1;//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle_yaw=2000;// 测量噪声的协方差 既测量偏差
            
char  C_0_yaw = 1;
float Q_bias_yaw, Angle_err_yaw,Bias_err_yaw;
float PCt_0_yaw, PCt_1_yaw, E_yaw;
float K_0_yaw, K_1_yaw, t_0_yaw, t_1_yaw;
float Pdot_yaw[4] ={0,0,0,0};
float PP_yaw[2][2] = { { 0, 0 },{ 0, 0 } };
extern float pure_gyro_pitch;
float test_bias = 0;
/**************************************************************************
函数功能：简易卡尔曼滤波
入口参数：加速度得到的角度值、角速度
返回  值：无
**************************************************************************/
void Kalman_Filter_yaw(float Accel,float Gyro)		
{  
    test_bias = (Gyro - Q_bias_yaw)*1000;
	yaw_angle += (Gyro - Q_bias_yaw) * dt/ 3.1415926f * 180.0f; //先验估计
	Pdot_yaw[0]=Q_angle_yaw - PP_yaw[0][1] - PP_yaw[1][0]; // Pk-先验估计误差协方差的微分

	Pdot_yaw[1]=-PP_yaw[1][1];
	Pdot_yaw[2]=-PP_yaw[1][1];
	Pdot_yaw[3]=Q_gyro_yaw;
	
	PP_yaw[0][0] += Pdot_yaw[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP_yaw[0][1] += Pdot_yaw[1] * dt;   // =先验估计误差协方差
	PP_yaw[1][0] += Pdot_yaw[2] * dt;
	PP_yaw[1][1] += Pdot_yaw[3] * dt;
	Bias_err_yaw = 	Accel - yaw_angle - Angle_err_yaw;
	Angle_err_yaw = Accel - yaw_angle;	//zk-先验估计
	
	PCt_0_yaw = C_0_yaw * PP_yaw[0][0];
	PCt_1_yaw = C_0_yaw * PP_yaw[1][0];
	
	E_yaw = R_angle_yaw + C_0_yaw * PCt_0_yaw;
	
	K_0_yaw = PCt_0_yaw / E_yaw;
	K_1_yaw = PCt_1_yaw / E_yaw;
	
	t_0_yaw = PCt_0_yaw;
	t_1_yaw = C_0_yaw * PP_yaw[0][1];

	PP_yaw[0][0] -= K_0_yaw * t_0_yaw;		 //后验估计误差协方差
	PP_yaw[0][1] -= K_0_yaw * t_1_yaw;
	PP_yaw[1][0] -= K_1_yaw * t_0_yaw;
	PP_yaw[1][1] -= K_1_yaw * t_1_yaw;
		
	yaw_angle	+= K_0_yaw * Angle_err_yaw;	 //后验估计
	Q_bias_yaw	+= K_bias*K_1_yaw * Bias_err_yaw;	 //后验估计
	yaw_angle_dot   = Gyro - Q_bias_yaw;	 //输出值(后验估计)的微分=角速度
}

