#include "contor.h"
#include "MPU6050.h"
// // 定义结构体来存储角速度和前进速度
// typedef struct {
//     float omega; // 角速度
//     float v;     // 前进速度
// } Velocity;

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx, float angle)
{   
    if(angle==0)angle=0.01;
    angle = target_limit_float(angle, -62, 62);
    float rad = angle * PI / 180.0; // 将角度转换为弧度
    // Ackerman car specific related variables //阿克曼小车专用相关变量
    float R, Ratio = 636.56, AngleR, Angle_Servo;

    R = Axle_spacing / tan(rad);
    // OLED_ShowFloat(R, 2, 75, 25);

    // Inverse kinematics //运动学逆解
    if (AngleR != 0)
    {
        MOTOR_A.Target = (Vx * (R - 0.5f * Wheel_spacing) / R/2);
        MOTOR_B.Target = (Vx * (R + 0.5f * Wheel_spacing) / R/2);
    }
    else
    {
        MOTOR_A.Target = Vx;
        MOTOR_B.Target = Vx;
    }

    // OLED_ShowFloat(MOTOR_A.Target, 1, 0, 50);
    // OLED_ShowFloat(MOTOR_B.Target, 1, 45, 50);
    //  OLED_Refresh_Gram();

    Servo = SERVO_INIT + angle * 100.0f / 9.0f;

    // Wheel (motor) target speed limit //车轮(电机)目标速度限幅
    MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -1.2, 1.2);
    MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -1.2, 1.2);
    Servo = target_limit_int(Servo, 800, 2200); // Servo PWM value limit //舵机PWM值限幅
}

/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
/**
 * @brief  根据得到的编码器脉冲值计算速度 单位:m/s
 * @retval 速度值
 */

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 定时器7中断回调函数，每10ms调用一次
// {
//     int encoderPulse[2] = {0};
//     //   float c_leftSpeed, c_rightSpeed;
//     if (htim == (&htim7))
//     {
//         encoderPulse[0] = (short)TIM2->CNT;
//         TIM2->CNT = 0; // 配合小车轮子运动方向，进行取反操作
//         encoderPulse[1] = (short)TIM3->CNT;
//         TIM3->CNT = 0;
//         MOTOR_A.Encoder = encoderPulse[0] * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision; // 获得当前的速度值
//         MOTOR_B.Encoder = -encoderPulse[1] * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;

//     }
// }

/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; // Calculate the deviation //计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16800)
        Pwm = 16800;
    if (Pwm < -16800)
        Pwm = -16800;
    Last_bias = Bias; // Save the last deviation //保存上一次偏差
                      // OLED_ShowFloat(Pwm, 1, 0, 0);

    // OLED_Refresh_Gram();
    return Pwm;
}
int Incremental_PI_B(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias = 0;
    Bias = Target - Encoder; // Calculate the deviation //计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16800)
        Pwm = 16800;
    if (Pwm < -16800)
        Pwm = -16800;
    // OLED_ShowFloat(Pwm, 1, 0, 20);
    Last_bias = Bias; // Save the last deviation //保存上一次偏差
    return Pwm;
}

/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert, float low, float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}
int target_limit_int(int insert, int low, int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a, int motor_b, int servo)
{
    // Servo control
    // 舵机控制
    Servo_PWM = servo;

    if (motor_a < 0)
        PWMA1 = 16800, PWMA2 = 16800 + motor_a;
    else
        PWMA2 = 16800, PWMA1 = 16800 - motor_a;

    if (motor_b < 0)
        PWMB1 = 16800, PWMB2 = 16800 + motor_b;
    else
        PWMB2 = 16800, PWMB1 = 16800 - motor_b;
}