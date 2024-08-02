#ifndef SYSTEM_H
#define SYSTEM_H
#include "tim.h"
#include "stdio.h"
#include "usart.h"
#include "oled.h"
#include "sys.h"  


#define Akm_wheelspacing         0.162f
#define Akm_axlespacing          0.144f
//Motor_gear_ratio
#define   HALL_30F    30
//Number_of_encoder_lines
#define	  Hall_13           13
//Black tire, tank_car wheel diameter
#define	  Black_WheelDiameter   0.065
#define   EncoderMultiples  4
//编码器数据1s读取频率
#define   CONTROL_FREQUENCY 100
// #define   PI  3.1415926
//PID parameters of Speed control
//速度控制PID参数
extern float Velocity_KP,Velocity_KI; 

extern float t,dt,w;

extern float a,l;
//************ 小车相关变量 **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//编码器精度
extern float Encoder_precision; 
//Wheel circumference, unit: m
//轮子周长，单位：m
extern float Wheel_perimeter; 
//Drive wheel base, unit: m
//主动轮轮距，单位：m
extern float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//小车前后轴的轴距，单位：m
extern float Axle_spacing; 
//All-directional wheel turning radius, unit: m
//全向轮转弯半径，单位：m
extern float Omni_turn_radiaus; 
/************ 小车相关变量 **************************/


//Servo control PWM value, Ackerman car special
//舵机控制PWM值，阿克曼小车专用
extern int Servo;  
#define SERVO_INIT 1500  //Servo zero point //������


//Motor speed control related parameters of the structure
//电机速度控制相关参数结构体
typedef struct  
{
	float Encoder;     //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //电机PWM数值，控制电机实时速度
	float Target;      //Control the target speed of the motor //电机目标速度值，控制电机目标速度
	float Velocity_KP; //Speed control PID parameters //速度控制PID参数
	float	Velocity_KI; //Speed control PID parameters //速度控制PID参数
}Motor_parameter;

extern Motor_parameter MOTOR_A;
extern Motor_parameter MOTOR_B;

//Smoothed the speed of the three axes
//平滑处理后的三轴速度
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;
//Smooth_Control smooth_control;


typedef struct  
{
  float WheelSpacing;      //Wheelspacing, Mec_Car is half wheelspacing //�־� ���ֳ�Ϊ���־�
  float AxleSpacing;       //Axlespacing, Mec_Car is half axlespacing //��� ���ֳ�Ϊ�����	
  int GearRatio;           //Motor_gear_ratio //������ٱ�
  int EncoderAccuracy;     //Number_of_encoder_lines //����������(����������)
  float WheelDiameter;     //Diameter of driving wheel //������ֱ��		
}Robot_Parament_InitTypeDef;


void Robot_Init() ;

void println(const char *msg);
void print(const char *msg);
void print_int(uint8_t motor_speed);
void print_int32_t(int32_t motor_speed);
void print_float(float motor_speed);
#endif