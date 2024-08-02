#include "system.h"

Motor_parameter MOTOR_A, MOTOR_B;
Robot_Parament_InitTypeDef Robot_Parament;

float Velocity_KP = 700;
float Velocity_KI = 500;


//Servo control PWM value, Ackerman car special
//舵机控制PWM值，阿克曼小车专用
int Servo;  
/************ 小车相关变量 **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//编码器精度
float Encoder_precision; 
//Wheel circumference, unit: m
//轮子周长，单位：m
float Wheel_perimeter; 
//Drive wheel base, unit: m
//主动轮轮距，单位：m
float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//小车前后轴的轴距，单位：m
float Axle_spacing; 
//All-directional wheel turning radius, unit: m
//全向轮转弯半径，单位：m
float Omni_turn_radiaus; 
/************ 小车相关变量 **************************/
/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
函数功能：初始化小车参数
入口参数：轮距 轴距 自转半径 电机减速比 电机编码器精度 轮胎直径
返回  值：无
**************************************************************************/
void Robot_Init() // 
{
	//wheelspacing, Mec_Car is half wheelspacing
	//轮距 
  Robot_Parament.WheelSpacing=Akm_wheelspacing; 
	//axlespacing, Mec_Car is half axlespacing
  //轴距 
  Robot_Parament.AxleSpacing=Akm_axlespacing;   

	//motor_gear_ratio
	//电机减速比
  Robot_Parament.GearRatio=HALL_30F; 
	//Number_of_encoder_lines
  //编码器精度(编码器线数)	
  Robot_Parament.EncoderAccuracy=Hall_13;
	//Diameter of driving wheel
  //主动轮直径	
  Robot_Parament.WheelDiameter=Black_WheelDiameter;       
	
	//Encoder value corresponding to 1 turn of motor (wheel)
	//电机(车轮)转1圈对应的编码器数值
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
	//Driving wheel circumference
  //主动轮周长	
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI;
	//wheelspacing, Mec_Car is half wheelspacing
  //轮距 
   Wheel_spacing=Robot_Parament.WheelSpacing; 
  //axlespacing, Mec_Car is half axlespacing	
  //轴距
    Axle_spacing=Robot_Parament.AxleSpacing; 

}


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  // 使用 HAL 库的 UART 发�?�函�????
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void println(const char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg), HAL_MAX_DELAY);
  const char newline[] = "\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)newline, sizeof(newline), HAL_MAX_DELAY);
}
void print(const char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) + 2, HAL_MAX_DELAY);
  // const char newline[] = "\r\n";
  // HAL_UART_Transmit(&huart2, (uint8_t *)newline, sizeof(newline), HAL_MAX_DELAY);
}

void print_int(uint8_t motor_speed)
{
  char buffer[50];                    // 确保缓冲区足够大以容纳转换后的字符串
  sprintf(buffer, "%d", motor_speed); // 将整数转换为字符�??
  println(buffer);                    // 打印字符�??
}
void print_int32_t(int32_t motor_speed)
{
  char buffer[100];                   // 确保缓冲区足够大以容纳转换后的字符串
  sprintf(buffer, "%d", motor_speed); // 将整数转换为字符�??
  println(buffer);                    // 打印字符�??
}
void print_float(float motor_speed)
{
  char buffer[100];                     // 确保缓冲区足够大以容纳转换后的字符串
  sprintf(buffer, "%.2f", motor_speed); // 将浮点数转换为字符串，保留两位小�?
  println(buffer);                      // 打印字符�??
}