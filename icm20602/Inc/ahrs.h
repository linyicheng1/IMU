#ifndef __AHRS_H
#define __AHRS_H	
#include "main.h"
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
float inv_sqrt(float x);
void AHRSupdate_acc(float gx, float gy, float gz, float ax, float ay, float az);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
struct ahrs_sensor
{
  float ax;
  float ay;
  float az;

  float wx;
  float wy;
  float wz;

  float mx;
  float my;
  float mz;
};

struct attitude
{
  float roll;
  float pitch;
  float yaw;
};
float invSqrt(float x);

void madgwick_ahrs_update(struct ahrs_sensor *sensor, struct attitude *atti);
void madgwick_ahrs_updateIMU(struct ahrs_sensor *sensor, struct attitude *atti);

#endif 

