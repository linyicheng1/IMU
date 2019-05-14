#include "ahrs.h"
#include "stdint.h"
#include "math.h"
#include "stm32f0xx_hal.h"

/* Private define ------------------------------------------------------------*/
float Kp = 100.0f;                        // proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki = 0.015f;                // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0025f                // half the sample period        : 0.005s/2=0.0025s
//float halfT;                      // half the sample period
#define ACCEL_1G 1000 //the acceleration of gravity is: 1000 mg

/* Private variables ---------------------------------------------------------*/
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;        // quaternion elements representing the estimated orientation
static float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error

/* Public variables ----------------------------------------------------------*/

float Magnetoresistor_mGauss_X = 0, Magnetoresistor_mGauss_Y = 0, Magnetoresistor_mGauss_Z = 0;//unit: milli-Gauss                                                                                                                                                                                                      
float Accelerate_mg_X, Accelerate_mg_Y, Accelerate_mg_Z;//unit: mg                                                               
float AngularRate_dps_X, AngularRate_dps_Y, AngularRate_dps_Z;//unit: dps: degree per second      

int16_t Magnetoresistor_X, Magnetoresistor_Y, Magnetoresistor_Z;                                                                                                                                                                                                      
uint16_t Accelerate_X = 0, Accelerate_Y = 0, Accelerate_Z = 0;                                                                                                                                                                                               
uint16_t AngularRate_X = 0, AngularRate_Y = 0, AngularRate_Z = 0;
float pitch, yaw, roll;
volatile uint32_t     last_update, now_update;  
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/*******************************************************************************
* Function Name  : AHRSupdate
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	// auxiliary variables to reduce number of repeated operations
	//先把这些变量计算出来以后可以调用
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;   
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;         
	
//	now_update  = HAL_GetTick(); //ms
//	halfT       = ((float)(now_update - last_update) / 2000.0f);
//	last_update = now_update;
	
	// normalise the measurements
	//归一化加速度值
	norm = sqrt(ax*ax + ay*ay + az*az);      
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
	//归一化磁场测量值
	norm = sqrt(mx*mx + my*my + mz*mz);         
	mx = mx / norm;
	my = my / norm;
	mz = mz / norm;         

	// compute reference direction of flux
	hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;        
    //计算加速度计和磁场的估计值
	// estimated direction of gravity and flux (v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
    //叉乘得到误差值
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) ;//+ (my*wz - mz*wy);
	ey = (az*vx - ax*vz) ;//+ (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) ;//+ (mx*wy - my*wx);

	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	// integrate quaternion rate and normalise
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
    //归一化
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	//解算出新的值
	pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
	roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
	yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
}
//#define Kp 0.0f                        // 这里的KpKi是用于调整加速度计修正陀螺仪的速度
//#define Ki 0.0                        
//#define halfT 0.0025f            // 采样周期的一半，用于求解四元数微分方程时计算角增量
//float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // 初始姿态四元数，由上篇博文提到的变换四元数公式得来
//float exInt = 0, eyInt = 0, ezInt = 0;    //当前加计测得的重力加速度在三轴上的分量
//                                //与用当前姿态计算得来的重力在三轴上的分量的误差的积分
//float pitch = 0;
//void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)//g表陀螺仪，a表加计
//{

//  float q0temp,q1temp,q2temp,q3temp;//四元数暂存变量，求解微分方程时要用
//  float norm; //矢量的模或四元数的范数
//  float vx, vy, vz;//当前姿态计算得来的重力在三轴上的分量
//  float ex, ey, ez;//当前加计测得的重力加速度在三轴上的分量
//              //与用当前姿态计算得来的重力在三轴上的分量的误差

//  // 先把这些用得到的值算好
//  float q0q0 = q0*q0;
//  float q0q1 = q0*q1;
//  float q0q2 = q0*q2;
//  float q1q1 = q1*q1;
//  float q1q3 = q1*q3;
//  float q2q2 = q2*q2;
//  float q2q3 = q2*q3;
//  float q3q3 = q3*q3;      
//  if(ax*ay*az==0)//加计处于自由落体状态时不进行姿态解算，因为会产生分母无穷大的情况
//        return;
//  norm = sqrt(ax*ax + ay*ay + az*az);//单位化加速度计，
//  ax = ax /norm;// 这样变更了量程也不需要修改KP参数，因为这里归一化了
//  ay = ay / norm;
//  az = az / norm;
//  //用当前姿态计算出重力在三个轴上的分量，
//  //参考坐标n系转化到载体坐标b系的用四元数表示的方向余弦矩阵第三列即是（博文一中有提到）
//  vx = 2*(q1q3 - q0q2);        
//  vy = 2*(q0q1 + q2q3);
//  vz = q0q0 - q1q1 - q2q2 + q3q3 ;
//  //计算测得的重力与计算得重力间的误差，向量外积可以表示这一误差
//  //原因我理解是因为两个向量是单位向量且sin0等于0
//  //不过要是夹角是180度呢~这个还没理解
//  ex = (ay*vz - az*vy) ;                                                                  
//  ey = (az*vx - ax*vz) ;
//  ez = (ax*vy - ay*vx) ;

//  exInt = exInt + ex * Ki;                                           //对误差进行积分
//  eyInt = eyInt + ey * Ki;
//  ezInt = ezInt + ez * Ki;
//  // adjusted gyroscope measurements
//  gx = gx + Kp*ex + exInt;  //将误差PI后补偿到陀螺仪，即补偿零点漂移
//  gy = gy + Kp*ey + eyInt;
//  gz = gz + Kp*ez + ezInt;    //这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减
//  //下面进行姿态的更新，也就是四元数微分方程的求解
//  q0temp=q0;//暂存当前值用于计算
//  q1temp=q1;//网上传的这份算法大多没有注意这个问题，在此更正
//  q2temp=q2;
//  q3temp=q3;
//  //采用一阶毕卡解法，相关知识可参见《惯性器件与惯性导航系统》P212
//  q0 = q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;
//  q1 = q1temp + (q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
//  q2 = q2temp + (q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
//  q3 = q3temp + (q0temp*gz + q1temp*gy -q2temp*gx)*halfT;
//  //单位化四元数在空间旋转时不会拉伸，仅有旋转角度，这类似线性代数里的正交变换
//  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//  q0 = q0 / norm;	
//  q1 = q1 / norm;
//  q2 = q2 / norm;
//  q3 = q3 / norm;
//  //四元数到欧拉角的转换，公式推导见博文一
//  //其中YAW航向角由于加速度计对其没有修正作用，因此此处直接用陀螺仪积分代替

//  pitch = asin(-2 * q1 * q3 + 2 * q0* q2)*57.3; // pitch
////  Q_ANGLE.X = atan2(2 * q2 * q3 + 2 * q0 * q1,-2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
//}
float clamp(float Value, float Min, float Max)
{
	if(Value > Max)
	{
		return Max;
	}
	else if(Value < Min)
	{
		return Min;
	}
	else
	{
		return Value;
	}
}
