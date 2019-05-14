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
	//�Ȱ���Щ������������Ժ���Ե���
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
	//��һ�����ٶ�ֵ
	norm = sqrt(ax*ax + ay*ay + az*az);      
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
	//��һ���ų�����ֵ
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
    //������ٶȼƺʹų��Ĺ���ֵ
	// estimated direction of gravity and flux (v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
    //��˵õ����ֵ
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
    //��һ��
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	//������µ�ֵ
	pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
	roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
	yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
}
//#define Kp 0.0f                        // �����KpKi�����ڵ������ٶȼ����������ǵ��ٶ�
//#define Ki 0.0                        
//#define halfT 0.0025f            // �������ڵ�һ�룬���������Ԫ��΢�ַ���ʱ���������
//float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // ��ʼ��̬��Ԫ��������ƪ�����ᵽ�ı任��Ԫ����ʽ����
//float exInt = 0, eyInt = 0, ezInt = 0;    //��ǰ�ӼƲ�õ��������ٶ��������ϵķ���
//                                //���õ�ǰ��̬��������������������ϵķ��������Ļ���
//float pitch = 0;
//void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)//g�������ǣ�a��Ӽ�
//{

//  float q0temp,q1temp,q2temp,q3temp;//��Ԫ���ݴ���������΢�ַ���ʱҪ��
//  float norm; //ʸ����ģ����Ԫ���ķ���
//  float vx, vy, vz;//��ǰ��̬��������������������ϵķ���
//  float ex, ey, ez;//��ǰ�ӼƲ�õ��������ٶ��������ϵķ���
//              //���õ�ǰ��̬��������������������ϵķ��������

//  // �Ȱ���Щ�õõ���ֵ���
//  float q0q0 = q0*q0;
//  float q0q1 = q0*q1;
//  float q0q2 = q0*q2;
//  float q1q1 = q1*q1;
//  float q1q3 = q1*q3;
//  float q2q2 = q2*q2;
//  float q2q3 = q2*q3;
//  float q3q3 = q3*q3;      
//  if(ax*ay*az==0)//�Ӽƴ�����������״̬ʱ��������̬���㣬��Ϊ�������ĸ���������
//        return;
//  norm = sqrt(ax*ax + ay*ay + az*az);//��λ�����ٶȼƣ�
//  ax = ax /norm;// �������������Ҳ����Ҫ�޸�KP��������Ϊ�����һ����
//  ay = ay / norm;
//  az = az / norm;
//  //�õ�ǰ��̬������������������ϵķ�����
//  //�ο�����nϵת������������bϵ������Ԫ����ʾ�ķ������Ҿ�������м��ǣ�����һ�����ᵽ��
//  vx = 2*(q1q3 - q0q2);        
//  vy = 2*(q0q1 + q2q3);
//  vz = q0q0 - q1q1 - q2q2 + q3q3 ;
//  //�����õ������������������������������Ա�ʾ��һ���
//  //ԭ�����������Ϊ���������ǵ�λ������sin0����0
//  //����Ҫ�Ǽн���180����~�����û���
//  ex = (ay*vz - az*vy) ;                                                                  
//  ey = (az*vx - ax*vz) ;
//  ez = (ax*vy - ay*vx) ;

//  exInt = exInt + ex * Ki;                                           //�������л���
//  eyInt = eyInt + ey * Ki;
//  ezInt = ezInt + ez * Ki;
//  // adjusted gyroscope measurements
//  gx = gx + Kp*ex + exInt;  //�����PI�󲹳��������ǣ����������Ư��
//  gy = gy + Kp*ey + eyInt;
//  gz = gz + Kp*ez + ezInt;    //�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�
//  //���������̬�ĸ��£�Ҳ������Ԫ��΢�ַ��̵����
//  q0temp=q0;//�ݴ浱ǰֵ���ڼ���
//  q1temp=q1;//���ϴ�������㷨���û��ע��������⣬�ڴ˸���
//  q2temp=q2;
//  q3temp=q3;
//  //����һ�ױϿ��ⷨ�����֪ʶ�ɲμ���������������Ե���ϵͳ��P212
//  q0 = q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;
//  q1 = q1temp + (q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
//  q2 = q2temp + (q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
//  q3 = q3temp + (q0temp*gz + q1temp*gy -q2temp*gx)*halfT;
//  //��λ����Ԫ���ڿռ���תʱ�������죬������ת�Ƕȣ����������Դ�����������任
//  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//  q0 = q0 / norm;	
//  q1 = q1 / norm;
//  q2 = q2 / norm;
//  q3 = q3 / norm;
//  //��Ԫ����ŷ���ǵ�ת������ʽ�Ƶ�������һ
//  //����YAW��������ڼ��ٶȼƶ���û���������ã���˴˴�ֱ���������ǻ��ִ���

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
