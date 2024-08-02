/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "contor.h"
#include "MPU6050.h"
#include "oled.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart5.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart2; // 确保huart2在其他地方已正确初始化

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

volatile uint8_t uart5_data_buffer[DATA_SIZE];
volatile uint8_t uart5_data_index = 0;
volatile uint8_t data_ready_flag = 0;

int findMaxIndex(int dir[], int size)
{
  int maxIndex = 0;
  int maxValue = dir[0];

  for (int i = 1; i < size; i++)
  {
    if (dir[i] > maxValue)
    {
      maxValue = dir[i];
      maxIndex = i;
    }
  }

  return maxIndex;
}
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  // 处理栈溢出的代码
  for (;;)
    ; // 无限循环，防止继续执行
}

void vApplicationIdleHook(void)
{
  // 空闲任务钩子函数的代码
}

void vApplicationTickHook(void)
{
  // 系统时钟节拍钩子函数的代码
}

void vApplicationMallocFailedHook(void)
{
  // 动态内存分配失败钩子函数的代码
  for (;;)
    ; // 无限循环，防止继续执行
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// LED闪烁任务

// 定义任务句柄
TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;
TaskHandle_t Task3Handle = NULL;
TaskHandle_t StartupTaskHandle = NULL;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int direction_id;
PolarCoord direction;
int dir[5] = {0};
#define var_cir 8
#define var_line 4
int distance;
// 计算两点之间的斜率
float calculateSlope(CartesianPoint p1, CartesianPoint p2)
{
  if (p1.x == 0.0f || p2.x == 0.0f)
  {
    return 0.0f; // 避免除以零
  }
  return (p2.y - p1.y) / (p2.x - p1.x);
}

// 计算斜率的方差
float calculateVariance(float slopes[], int length)
{
  float sumSlopes = 0.0f;
  float meanSlope = 0.0f;
  float variance = 0.0f;

  for (int i = 0; i < length; i++)
  {
    sumSlopes += slopes[i];
  }
  meanSlope = sumSlopes / length;

  for (int i = 0; i < length; i++)
  {
    variance += powf(slopes[i] - meanSlope, 2);
  }
  return variance / length;
}

// 根据方差和角度决定方向
void determineDirection(float variance, float angle, int dir[])
{

  if (variance > var_cir)
  {
    if (angle >= 0 && angle <= 10)
    {
      dir[2]++;
    }
    else if (angle >= 350 && angle <= 360)
    {
      dir[2]++;
    }
    else if (angle > 10 && angle <= 30)
    {
      dir[3]++;
    }
    else if (angle > 30 && angle <= 100)
    {
      dir[4]++;
    }
    else if (angle >= 330 && angle < 350)
    {
      dir[1]++;
    }
    else if (angle >= 170 && angle < 330)
    {
      dir[0]++;
    }
  }


   else if (variance < var_line)
  {
    if (angle >= 0 && angle <= 10)
    {
      dir[2]--;
    }
    else if (angle >= 350 && angle <= 360)
    {
      dir[2]--;
    }
    else if (angle > 10 && angle <= 30)
    {
      dir[3]--;
    }
    else if (angle > 30 && angle <= 100)
    {
      dir[4]--;
    }
    else if (angle >= 330 && angle < 350)
    {
      dir[1]--;
    }
    else if (angle >= 170 && angle < 330)
    {
      dir[0]--;
    }
  }
}

void calcul_k(CartesianPoint cartesianData[], int n, int dir[])
{
  if (n < 4)
  {
    return; // 数组长度小于 4 时无法计算斜率方差
  }

  float slopes[3];           // 存储斜率
  float varianceHistory[10]; // 存储最近 10 次的方差
  int varianceIndex = 0;     // 当前方差在数组中的索引

  for (int i = 0; i < n - 2; i++)
  {
    // 检查坐标是否为 0
    if (cartesianData[i].x == 0.0f || cartesianData[i + 1].x == 0.0f || cartesianData[i].y == 0.0f || cartesianData[i + 1].y == 0.0f ||
        cartesianData[i + 2].x == 0.0f || cartesianData[i + 2].y == 0.0f || cartesianData[i + 3].x == 0.0f || cartesianData[i + 3].y == 0.0f)
    {
      continue; // 存在坐标为 0 的情况，跳过
    }

    // 计算斜率
    for (int k = 0; k <= 2; k++)
    {
      slopes[k] = calculateSlope(cartesianData[i + k], cartesianData[i + k + 1]);
    }

    // 计算方差
    float variance = calculateVariance(slopes, 3);

    // 将当前方差添加到历史记录中
    varianceHistory[varianceIndex % 10] = variance;
    varianceIndex++;

    // 计算平滑方差
    float smoothVariance = 0.0f;
    int count = (varianceIndex < 10) ? varianceIndex : 10;
    for (int j = 0; j < count; j++)
    {
      smoothVariance += varianceHistory[j];
    }
    smoothVariance /= count;

    // 打印调试信息
    print_float(smoothVariance);
    print_float(Dataprocess[i].angle);
    print_int32_t(Dataprocess[i].distance);
    println("");
    // 决定方向
    if (0 < Dataprocess[i].angle || Dataprocess[i].angle > 5)
      distance = Dataprocess[i].distance;
    determineDirection(smoothVariance, Dataprocess[i].angle, dir);
  }
}

// 任务1函数（优先级高）
void Task1(void *pvParameters)
{
  while (1)
  {
    PolarCoord polarCoord;
    Circle cir;

    if (data_ready_flag)
    {
      vTaskSuspend(Task3Handle);
      // 数据处理逻辑
      processData();
      // 转换为直角坐标
      CartesianPoint cartesianData[92];
      polarToCartesian(Dataprocess, cartesianData, 92);
      calcul_k(cartesianData, 92, dir);
      direction_id = findMaxIndex(dir, 5);
      for (int i = 0; i < 5; i++)
      {
        dir[i] = 0;
      }
      // 数据处理完毕，重置标志位
       vTaskResume(Task3Handle);
      vTaskDelay(pdMS_TO_TICKS(1000));
        Drive_Motor(0,0);
      data_ready_flag = 0;
      // 重新开启UART接收中断
      
      HAL_UART_Receive_IT(&huart5, Uart5_Receive_buf, sizeof(Uart5_Receive_buf));
    }
  }
}

// 任务2函数（优先级低）
void Task2(void *pvParameters)
{
  while (1)
  {
    // print_int(direction_id);
    OLED_ShowNumber(20, 20, direction_id);
    OLED_Refresh_Gram();
    if (distance < 300 && direction_id != 2)
    {
      Drive_Motor(-0.2, 0);
      // vTaskDelay(pdMS_TO_TICKS(1000));
    }
    else if (distance < 150 && direction_id == 2)
    {
      Drive_Motor(0, 0);
      // vTaskDelay(pdMS_TO_TICKS(2000));
    }
    else
    {
      switch (direction_id)
      {
      case 0:
        Drive_Motor(0.3, -60);
        // vTaskDelay(pdMS_TO_TICKS(1000));
      case 1:
        Drive_Motor(0.3, -40);
        // vTaskDelay(pdMS_TO_TICKS(1000));
        break;
      case 2:
        Drive_Motor(0.3, 0);
        // vTaskDelay(pdMS_TO_TICKS(1000));
        break;
      case 3:
        Drive_Motor(0.3, 40);
        // vTaskDelay(pdMS_TO_TICKS(1000));
        break;
      case 4:
        Drive_Motor(0.3, 60);
        // vTaskDelay(pdMS_TO_TICKS(1000));
        break;
      default:

        break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
  // Drive_Motor(0,0);
}

// 任务3函数（优先级低，与任务2相同）
void Task3(void *pvParameters)
{
  while (1)
  {
    Set_Pwm(Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target), Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target), Servo);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// 启动任务函数
void StartupTask(void *pvParameters)
{
  // 延时一段时间
  vTaskDelay(pdMS_TO_TICKS(2000));

  // 创建任务1，优先级为2，栈大小为1024字（可以根据需要调整）
  xTaskCreate(Task1, "Task 1", 1024, NULL, 2, &Task1Handle);

  // 创建任务2，优先级为1，栈大小为configMINIMAL_STACK_SIZE
  xTaskCreate(Task2, "Task 2", 256, NULL, 1, &Task2Handle);

  // 创建任务3，优先级为1，栈大小为configMINIMAL_STACK_SIZE
  xTaskCreate(Task3, "Task 3", 128, NULL, 1, &Task3Handle);

  // 删除启动任务
  vTaskDelete(NULL);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);      // motor_A
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);      // motor_A(pwm)
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);       // motor_B(pwm)
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);       // motor_B(pwm)
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);       // servo pwm
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // encoder_B
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // encoder_A
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim7); // T=10ms

  Robot_Init();
  OLED_Init();
  OLED_Clear();
  OLED_Display_On();

  MPU6050_DMP_Init();
  Set_Pwm(0, 0, 1500);
  HAL_Delay(100);

  // 创建启动任务
  xTaskCreate(StartupTask, "Startup Task", 128, NULL, 3, &StartupTaskHandle);

  // 启动调度器
  vTaskStartScheduler();

  // 如果调度器启动失败，则执行以下代码
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    // 检查标志位
    // PolarCoord polarCoord;
    // Circle cir;

    // if (data_ready_flag)
    // {
    //   // 数据处理逻辑
    //   processData();
    //   print("b");
    //   // 转换为直角坐标
    //   CartesianPoint cartesianData[80];
    //   polarToCartesian(Dataprocess, cartesianData, 80);
    //    calcul_k(cartesianData, 80,dir);
    //   direction_id = findMaxIndex(dir, 5);
    //   //print_int(direction_id);
    //   // 数据处理完毕，重置标志位
    //   data_ready_flag = 0;
    //   // 重新开启UART接收中断
    //   HAL_UART_Receive_IT(&huart5, Uart5_Receive_buf, sizeof(Uart5_Receive_buf));
    // }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  if (htim == (&htim7))
  {
    int encoderPulse[2] = {0};
    encoderPulse[0] = (short)TIM2->CNT;
    TIM2->CNT = 0; // 配合小车轮子运动方向，进行取反操作
    encoderPulse[1] = (short)TIM3->CNT;
    TIM3->CNT = 0;
    MOTOR_A.Encoder = encoderPulse[0] * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision; // 获得当前的速度值
    MOTOR_B.Encoder = -encoderPulse[1] * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
