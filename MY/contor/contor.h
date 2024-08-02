#ifndef _CONTOR_H
#define _CONTOR_H
#include "main.h"
#include "stdio.h"
#include "math.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "system.h"

/* USER CODE BEGIN Prototypes */
#define PWMA1   TIM10->CCR1 
#define PWMA2   TIM11->CCR1 

#define PWMB1   TIM9->CCR1 
#define PWMB2   TIM9->CCR2
#define Servo_PWM  TIM8->CCR1
#define SERVO_INIT 1500  //Servo zero point //������

void Drive_Motor(float Vx, float angle);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) ;
int Incremental_PI_A(float Encoder, float Target);
int Incremental_PI_B(float Encoder, float Target);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
void Set_Pwm(int motor_a,int motor_b,int servo);
void position_function(float u, float a, float l, float *x, float *y);

#endif