#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H
#include "stm32f0xx_hal.h"
extern float pitch_angle, pitch_angle_dot;
extern float yaw_angle,yaw_angle_dot;
//void Kalman_Filter(float Accel,float Gyro);		
//void KalmanFilter(float* am_angle_mat,float* gyro_angle_mat);
void Kalman_Filter_pitch(float Accel,float Gyro);
void Kalman_Filter_yaw(float Accel,float Gyro);	

void matrix_add(float ** A,float **B,float **C,uint8_t length);
//C = A - B
void matrix_sub(float ** A,float **B,float **C,uint8_t length);
//C = A * B
void matrix_mul(float ** A,float **B,float **C);
//     -1         
//B = A
void matrix_inv(float ** A,float **B);
//     T         
//B = A
void matrix_T(float ** A,float **B,uint8_t length);
#endif
