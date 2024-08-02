#ifndef __MPU6050_H
#define __MPU6050_H
extern float pitch, roll, yaw;
int MPU6050_DMP_Init(void);
int MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw);
void calibrate_quaternion_offset();
#endif
