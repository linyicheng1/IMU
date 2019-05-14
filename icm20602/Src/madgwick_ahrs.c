/**********************************************************************************************************
 * @文件     madgwick.c
 * @说明     通过磁力计计算角度
 * @版本  	 V1.0
 * @作者     林一成
 * @日期     2019.01
**********************************************************************************************************/
#include "math.h"
#include "stdint.h"
#include "ahrs.h"

/**
  * @brief     通过磁力计计算出yaw轴角度
  * @param
  * @retval    
  */
/**********************************************************/
struct attitude mag_attitude;//通过磁场计算出来的姿态
float mag0[3];//初始状态下的磁场
float rotation_m[9];//旋转矩阵初始值为零
float rotation_m_result[9];//加上收敛区域判断后的旋转矩阵
float step_size = 0.01;//快速下降法的步长
float deltaF[9];//计算得到的函数的增量
float repeatTimesMAX = 200;
float err_now,err_next;
float _x,_y,_z;
float recipNorm;//归一化辅助变量
int32_t yaw_cycle  = 0;
float last_yaw;
float magYawCal(float mx,float my,float mz)
{
	
	err_now = 100000;//误差值，初始值默认很大
	err_next = 0;
	static uint8_t first_cal_flag = 0;//第一次进入的标志位
	float deltaF[9];//计算得到的函数的增量
	
	recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;//得到归一化后的磁场数据
	if(first_cal_flag==0&&mx!=0&&my!=0)
	{//记录初始状态下的磁场数据
		mag0[0] = mx;
		mag0[1] = my;
		mag0[2] = mz;
		first_cal_flag = 1;
	}
	//快速下降法计算得到增量
	for(int j=0;j<repeatTimesMAX;j++)
	{//目前假设两步就能收敛
//		_x = rotation_m[0]*mag0[0]+rotation_m[1]*mag0[1]+rotation_m[2]*mag0[2]-mx;
//		_y = rotation_m[3]*mag0[0]+rotation_m[4]*mag0[1]+rotation_m[5]*mag0[2]-my;
//		_z = rotation_m[6]*mag0[0]+rotation_m[7]*mag0[1]+rotation_m[8]*mag0[2]-mz;
      
		deltaF[0] = mag0[0]*_x;
		deltaF[1] = mag0[1]*_x;
		deltaF[2] = mag0[2]*_x;
		deltaF[3] = mag0[0]*_y;
		deltaF[4] = mag0[1]*_y;
		deltaF[5] = mag0[2]*_y;
		deltaF[6] = mag0[0]*_z;
		deltaF[7] = mag0[1]*_z;
		deltaF[8] = mag0[2]*_z;
		for(int i=0;i<9;i++)
		{
			rotation_m[i] -=step_size*deltaF[i];
		}
		_x = rotation_m[0]*mag0[0]+rotation_m[1]*mag0[1]+rotation_m[2]*mag0[2]-mx;
		_y = rotation_m[3]*mag0[0]+rotation_m[4]*mag0[1]+rotation_m[5]*mag0[2]-my;
		_z = rotation_m[6]*mag0[0]+rotation_m[7]*mag0[1]+rotation_m[8]*mag0[2]-mz;
		err_next = _x*_x+_y*_y+_z*_z;
//		if(err_next<err_now)
//		{
//			err_now = err_next;
			for(int i=0;i<9;i++)
			  rotation_m_result[i] = rotation_m[i];
//		}
//		else
//		{
//			break;
//		}
		if(err_now<0.005)
		{
			break;
		}
    }
	//计算得到欧拉角
	mag_attitude.roll = atan2(rotation_m_result[7],rotation_m_result[8])/ 3.1415926f * 180.0f;
	mag_attitude.pitch = atan2(-rotation_m_result[6],sqrt(rotation_m_result[8]*rotation_m_result[8]+rotation_m_result[7]*rotation_m_result[7]))/ 3.1415926f * 180.0f;
	float yaw_trans =  atan2(rotation_m_result[3],rotation_m_result[0])/ 3.1415926f * 180.0f;
	if(yaw_trans - last_yaw>200)//从-180变到180
	{
		yaw_cycle --;
	}
	else if(yaw_trans - last_yaw<-200)//从180变到-180
	{
		yaw_cycle++;
	}
	last_yaw = yaw_trans;
	mag_attitude.yaw = yaw_trans + yaw_cycle*360;
}

/**
  * @brief     通过磁力计计算出yaw轴角度，采用lm算法，算法有问题没有使用
  * @param
  * @retval    
  */
float err_lm[3];//当前估计值与实际值的误差
float normal_err_lm;//当前估计值与实际值的误差的平方和
float err_first_lm;//初始时误差值
float rotation_m_lm[9]={1,0,0,0,1,0,0,0,1};//求得的旋转矩阵，初始值为I
float rotation_m_lm_asm[9];//算法过程中的旋转矩阵估计值
uint8_t timeMAX = 5;//最大的迭代次数
uint8_t isConvergence = 1;//收敛的标志位
float lamda = 0.01;//阻尼系数lamda，暂定0.01
float delta_rotation[9];//每次迭代计算得到的增量
float H_lm[9][9];//海塞矩阵
float deltaF_lm[9];//计算得到的函数的增量
struct attitude mag_attitude_lm;//通过磁场计算出来的姿态
float _x1;
float _y1;
float _z1;
float magYawCal_LM(float mx,float my,float mz)
{
	float _x2;//局部变量
	float _y2;//局部变量
	float _z2;//局部变量
	float _xy;//局部变量
	float _xz;//局部变量
	float _yz;//局部变量
	float recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    _x1=mx*recipNorm;
    _y1=my*recipNorm;
    _z1=mz*recipNorm;//得到归一化后的磁场数据

	static uint8_t first_cal_flag_lm = 0;//第一次进入的标志位
	if(first_cal_flag_lm==0)
	{//记录初始状态下的磁场数据
		mag0[0] = _x1;
		mag0[1] = _y1;
		mag0[2] = _z1;
		first_cal_flag_lm = 1;
	}
	isConvergence = 1;
	lamda=0.01;
	for(int i=0;i<timeMAX;i++)
	{//最多循环timeMAX次
		if(isConvergence==1)
		{//如果没有收敛
			//计算误差值
			err_lm[0] = rotation_m_lm[0]*mag0[0]+rotation_m_lm[1]*mag0[1]+
			            rotation_m_lm[2]*mag0[2]-_x1;
			err_lm[1] = rotation_m_lm[3]*mag0[0]+rotation_m_lm[4]*mag0[1]+
			            rotation_m_lm[5]*mag0[2]-_y1;
			err_lm[2] = rotation_m_lm[6]*mag0[0]+rotation_m_lm[7]*mag0[1]+
			            rotation_m_lm[8]*mag0[2]-_z1;
			if(i==0)
	        {//第一次循环记录下初始误差值便于比较
				_x2 = err_lm[0]*err_lm[0];
				_y2 = err_lm[1]*err_lm[1];
				_z2 = err_lm[2]*err_lm[2];
				
				_xy = err_lm[0]*err_lm[1];
				_xz = err_lm[0]*err_lm[2];
				_yz = err_lm[1]*err_lm[2];
				
				err_first_lm = _x2+_y2+_z2;
	        }
			for(int i = 0;i<9;i++)
			{
				for(int j = 0;j<9;j++)
				{
					H_lm[i][j] = 0;
				}
			}
			//计算海塞矩阵
			H_lm[0][0] = lamda;
			H_lm[1][1] = lamda;
			H_lm[2][2] = lamda;
			H_lm[6][6] = lamda;
			H_lm[7][7] = lamda;
			H_lm[8][8] = lamda;
			
			H_lm[0][6] = _x2;
			H_lm[0][7] = _xy;
			H_lm[0][8] = _xz;
			H_lm[1][6] = _xy;
			H_lm[1][7] = _y2;
			H_lm[1][8] = _yz;
			H_lm[2][6] = _xz;
			H_lm[2][7] = _yz;
			H_lm[2][8] = _z2;
			
			H_lm[6][0] = _x2;
			H_lm[6][1] = _xy;
			H_lm[6][2] = _xz;
			H_lm[7][0] = _xy;
			H_lm[7][1] = _y2;
			H_lm[7][2] = _yz;
			H_lm[8][0] = _xz;
			H_lm[8][1] = _yz;
			H_lm[8][2] = _z2;
			
			H_lm[3][3] = lamda + _x2;
			H_lm[3][4] = _xy;
			H_lm[3][5] = _xz;
			H_lm[4][3] = _xy;
			H_lm[4][4] = lamda + _y2; 
			H_lm[4][5] = _yz;
            H_lm[5][3] = _xz;
            H_lm[5][4] = _yz;
			H_lm[5][5] = lamda + _z2;
			matrix_inv();
			//计算新的估计值
			float _x = rotation_m_lm[0]*mag0[0]+rotation_m_lm[1]*mag0[1]+rotation_m_lm[2]*mag0[2]-_x1;
		    float _y = rotation_m_lm[3]*mag0[0]+rotation_m_lm[4]*mag0[1]+rotation_m_lm[5]*mag0[2]-_y1;
		    float _z = rotation_m_lm[6]*mag0[0]+rotation_m_lm[7]*mag0[1]+rotation_m_lm[8]*mag0[2]-_z1;
		    deltaF_lm[0] = mag0[0]*_x;
		    deltaF_lm[1] = mag0[1]*_x;
			deltaF_lm[2] = mag0[2]*_x;
			deltaF_lm[3] = mag0[0]*_y;
			deltaF_lm[4] = mag0[1]*_y;
			deltaF_lm[5] = mag0[2]*_y;
			deltaF_lm[6] = mag0[0]*_z;
			deltaF_lm[7] = mag0[1]*_z;
			deltaF_lm[8] = mag0[2]*_z;
			for(int j=0;j<9;j++)
			{//矩阵相乘
				float temp_ = 0;
				for(int k=0;k<9;k++)
				{
					temp_+=H_lm[j][k]*deltaF_lm[k];
//					temp_=0.01*deltaF_lm[j];
				}
				delta_rotation[j] = temp_;
			}
			for(int j=0;j<9;j++)
			   rotation_m_lm_asm[j] = rotation_m_lm[j]-delta_rotation[j];
			//计算新的误差
			err_lm[0] = rotation_m_lm_asm[0]*mag0[0]+rotation_m_lm_asm[1]*mag0[1]+
			            rotation_m_lm_asm[2]*mag0[2]-_x1;
			err_lm[1] = rotation_m_lm_asm[3]*mag0[0]+rotation_m_lm_asm[4]*mag0[1]+
			            rotation_m_lm_asm[5]*mag0[2]-_y1;
			err_lm[2] = rotation_m_lm_asm[6]*mag0[0]+rotation_m_lm_asm[7]*mag0[1]+
			            rotation_m_lm_asm[8]*mag0[2]-_z1;
			normal_err_lm = err_lm[0]*err_lm[0]+err_lm[1]*err_lm[1]
				               +err_lm[2]*err_lm[2];
			//根据误差决定迭代方向
			if(normal_err_lm<err_first_lm)
			{//变小了继续迭代
				lamda=lamda/10;
				for(int j=0;j<9;j++)
				   rotation_m_lm[j] = rotation_m_lm_asm[j];
				err_first_lm = normal_err_lm;
				isConvergence=1;
			}
			else 
			{
				lamda = lamda*10;
				isConvergence=0;
                
		    }
		}
		
	}
   //计算得到欧拉角
	mag_attitude_lm.roll = atan2(rotation_m_lm[7],rotation_m_lm[8])/ 3.1415926f * 180.0f;
	mag_attitude_lm.pitch = atan2(-rotation_m_lm[6],sqrt(rotation_m_lm[8]*rotation_m_lm[8]+rotation_m_lm[7]*rotation_m_lm[7]))/ 3.1415926f * 180.0f;
	mag_attitude_lm.yaw = atan2(rotation_m_lm[3],rotation_m_lm[0])/ 3.1415926f * 180.0f;	
}
/**
  * @brief     Fast inverse square-root, to calculate 1/Sqrt(x)
               sizeof(long) must be 4 bytes.
  * @param[in] input:x
  * @retval    1/Sqrt(x)
  */
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
/**
  * @brief 矩阵求逆算法，LM算法使用     
  * @param 
  * @retval    
  */
int  matrix_inv(void)
{
	float tmp, tmp2, b_tmp[20], c_tmp[20];
	int k, k1, k2, k3, j, i, j2, i2, kme[20], kmf[20];
	i2 = j2 = 0;
 
	for (k = 0; k < 9; k++)  
	{
		tmp2 = 0.0;
		for (i = k; i < 9; i++)  
		{
			for (j = k; j < 9; j++)  
			{
				if (fabs(H_lm[i][j] ) <= fabs(tmp2)) 
					continue;
				tmp2 = H_lm[i][j];
				i2 = i;
				j2 = j;
			}  
		}
		if (i2 != k) 
		{
			for (j = 0; j < 9; j++)   
			{
				tmp = H_lm[i2][j];
				H_lm[i2][j] = H_lm[k][j];
				H_lm[k][j] = tmp;
			}
		}
		if (j2 != k) 
		{
			for (i = 0; i < 9; i++)  
			{
				tmp = H_lm[i][j2];
				H_lm[i][j2] = H_lm[i][k];
				H_lm[i][k] = tmp;
			}    
		}
		kme[k] = i2;
		kmf[k] = j2;
		for (j = 0; j < 9; j++)  
		{
			if (j == k)   
			{
				b_tmp[j] = 1.0 / tmp2;
				c_tmp[j] = 1.0;
			}
			else 
			{
				b_tmp[j] = -H_lm[k][j] / tmp2;
				c_tmp[j] = H_lm[j][k];
			}
			H_lm[k][j] = 0.0;
			H_lm[j][k] = 0.0;
		}
		for (i = 0; i < 9; i++)  
		{
			for (j = 0; j < 9; j++)  
			{
				H_lm[i][j] = H_lm[i][j] + c_tmp[i] * b_tmp[j];
			}  
		}
	}
	for (k3 = 0; k3 < 9;  k3++)   
	{
		k  = 9 - k3 - 1;
		k1 = kme[k];
		k2 = kmf[k];
		if (k1 != k)   
		{
			for (i = 0; i < 9; i++)  
			{
				tmp = H_lm[i][k1];
				H_lm[i][k1] = H_lm[i][k];
				H_lm[i][k] = tmp;
			}  
		}
		if (k2 != k)   
		{
			for(j = 0; j < 9; j++)  
			{
				tmp = H_lm[k2][j];
				H_lm[k2][j] = H_lm[k][j];
				H_lm[k][j] = tmp;
			}
		}
	}
	return (0);
}


