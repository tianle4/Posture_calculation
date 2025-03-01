/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "bsp_dwt.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t velocity_x;
  int16_t velocity_y;
  int16_t velocity_z; 
  int16_t temperature;
}MPU6050;

typedef struct {
  float angle;      // 滤波后的角度
  float bias;       // 陀螺仪零漂
  float P[2][2];    // 估计误差协方差
  float Q_angle;    // 角度过程噪声协方差
  float Q_bias;     // 陀螺仪偏置过程噪声协方差
  float R_measure;  // 测量噪声协方差
} KalmanFilter;

typedef struct {
  float w,x,y,z;
}Quaternion;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_PI 3.1415926
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050 MPU6050_Data;

KalmanFilter rollKalman;
KalmanFilter yawKalman;
KalmanFilter pitchKalman;

Quaternion q;

uint32_t DWT_CNT;
float dt;

float roll, pitch, yaw;
float raw_roll, raw_pitch, raw_yaw;
float filtered_roll, filtered_pitch, filtered_yaw;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  DWT_Init(72);

  void MPU6050_Init(void);
  void MPU6050_read_data(void);

  void InitQuaternion(Quaternion *q);
  Quaternion multiplyQuaternions(const Quaternion *q1, const Quaternion *q2);
  void updateQuaternion(Quaternion *q, int16_t gx, int16_t gy, int16_t gz, float dt);
  void calculateAngles(Quaternion *q, float *roll, float *pitch, float *yaw);

  void kalmanInit(KalmanFilter *filter, float Q_angle, float Q_bias, float R_measure);
  kalmanInit(&rollKalman, 0.001f, 0.003f, 0.03f);
  kalmanInit(&pitchKalman, 0.001f, 0.003f, 0.03f);
  kalmanInit(&yawKalman, 0.001f, 0.003f, 0.03f);
  float kalmanUpdate(KalmanFilter *filter, float newAngle, float newRate, float dt);
  void calculateAccelAngles(float *accel_roll, float *accel_pitch); 
  
  q.w = 1.0f;
  q.x = 0.0f;
  q.y = 0.0f;
  q.z = 0.0f;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    dt = DWT_GetDeltaT(&DWT_CNT);
    MPU6050_read_data();
    
    updateQuaternion(&q,MPU6050_Data.gyro_x,MPU6050_Data.gyro_y,MPU6050_Data.gyro_z,dt);
    // calculateAngles(&q,&roll,&pitch,&yaw);

    // 计算加速度计角度
    float accel_roll, accel_pitch;
    calculateAccelAngles(&accel_roll, &accel_pitch);

    // 将陀螺仪原始数据转换为角速度(°/s)
    float gyro_factor = 131.0f; // 对于±250°/s量程
    float gyro_roll = MPU6050_Data.gyro_x / gyro_factor;
    float gyro_pitch = MPU6050_Data.gyro_y / gyro_factor;
    float gyro_yaw = MPU6050_Data.gyro_z / gyro_factor;

    // 应用卡尔曼滤波
    filtered_roll = kalmanUpdate(&rollKalman, accel_roll, gyro_roll, dt);
    filtered_pitch = kalmanUpdate(&pitchKalman, accel_pitch, gyro_pitch, dt);

    // 注意：yaw角不能从加速度计得到，需要磁力计
    // 这里简单地使用四元数计算的yaw值
    filtered_yaw = raw_yaw;

    // 使用滤波后的角度作为最终输出
    roll = filtered_roll;
    pitch = filtered_pitch;
    yaw = filtered_yaw;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(10);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void MPU6050_read_data(void)
{
  uint8_t readBuffer[14];
  uint8_t temp_data[2];

  HAL_I2C_Mem_Read(&hi2c1,MPU6050_Address,MPU6050_Accel_x,1,readBuffer,14,HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1,MPU6050_Address,MPU6050_Temp,1,temp_data,2,HAL_MAX_DELAY);
  MPU6050_Data.accel_x = (readBuffer[0] << 8) | readBuffer[1];
  MPU6050_Data.accel_y = (readBuffer[2] << 8) | readBuffer[3];
  MPU6050_Data.accel_z = (readBuffer[4] << 8) | readBuffer[5];
  MPU6050_Data.gyro_x = (readBuffer[8] << 8) | readBuffer[9];
  MPU6050_Data.gyro_y = (readBuffer[10] << 8) | readBuffer[11];
  MPU6050_Data.gyro_z = (readBuffer[12] << 8) | readBuffer[13];
  MPU6050_Data.temperature = (temp_data[0] << 8) | temp_data[1];
}

void MPU6050_Init(void)
{
  uint8_t Data;
  uint8_t check;

  HAL_I2C_Mem_Read(&hi2c1,MPU6050_Address,MPU6050_WHO_I_AM,1,&check,1,HAL_MAX_DELAY);
  if (check == MPU6050_Address)
  {
    Data = 0x00;//唤醒mpu6050
    HAL_I2C_Mem_Write(&hi2c1,MPU6050_Address,0x6b,1,&Data,1,HAL_MAX_DELAY);

    Data = 0x08;//角度寄存器，地址：0x1b
    HAL_I2C_Mem_Write(&hi2c1,MPU6050_Address,0x1b,1,&Data,1,HAL_MAX_DELAY);

    Data = 0x10;//加速度寄存器，地址：0x1c
    HAL_I2C_Mem_Write(&hi2c1,MPU6050_Address,0x1c,1,&Data,1,HAL_MAX_DELAY);
  }
  
}

void InitQuaternion(Quaternion *q)
{
  float norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
  q->w /= norm;
  q->x /= norm;
  q->y /= norm;
  q->z /= norm;
}

Quaternion multiplyQuaternions(const Quaternion *q1, const Quaternion *q2) 
{
  Quaternion output;
  output.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
  output.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
  output.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
  output.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
  return output;
}

void updateQuaternion(Quaternion *q, int16_t gx, int16_t gy, int16_t gz, float dt) 
{
  float norm;
  float half_dt = dt * 0.5;
  float wx = gx * half_dt;
  float wy = gy * half_dt;
  float wz = gz * half_dt;

  Quaternion q_temp;
  q_temp.w = 0;
  q_temp.x = wx;
  q_temp.y = wy;
  q_temp.z = wz;

  // 更新四元数
  *q = multiplyQuaternions(q, &q_temp);
  InitQuaternion(q);
}

void calculateAngles(Quaternion *q, float *roll, float *pitch, float *yaw) 
{
  *roll = atan2(2 * (q->w * q->x + q->y * q->z), 1 - 2 * (q->x * q->x + q->y * q->y));
  *pitch = asin(2 * (q->w * q->y - q->z * q->x));
  *yaw = atan2(2 * (q->w * q->z + q->x * q->y), 1 - 2 * (q->y * q->y + q->z * q->z));
}

void kalmanInit(KalmanFilter *filter, float Q_angle, float Q_bias, float R_measure) 
{
  filter->angle = 0.0f;
  filter->bias = 0.0f;
  
  filter->P[0][0] = 1.0f;
  filter->P[0][1] = 0.0f;
  filter->P[1][0] = 0.0f;
  filter->P[1][1] = 1.0f;
  
  filter->Q_angle = Q_angle;
  filter->Q_bias = Q_bias;
  filter->R_measure = R_measure;
}

float kalmanUpdate(KalmanFilter *filter, float newAngle, float newRate, float dt)
{
  // 预测步骤
  filter->angle += dt * (newRate - filter->bias);
  
  filter->P[0][0] += dt * (dt * filter->P[1][1] - filter->P[0][1] - filter->P[1][0] + filter->Q_angle);
  filter->P[0][1] -= dt * filter->P[1][1];
  filter->P[1][0] -= dt * filter->P[1][1];
  filter->P[1][1] += filter->Q_bias * dt;
  
  // 更新步骤
  float y = newAngle - filter->angle;
  float S = filter->P[0][0] + filter->R_measure;
  float K[2];
  K[0] = filter->P[0][0] / S;
  K[1] = filter->P[1][0] / S;
  
  filter->angle += K[0] * y;
  filter->bias += K[1] * y;
  
  float P00_temp = filter->P[0][0];
  float P01_temp = filter->P[0][1];
  
  filter->P[0][0] -= K[0] * P00_temp;
  filter->P[0][1] -= K[0] * P01_temp;
  filter->P[1][0] -= K[1] * P00_temp;
  filter->P[1][1] -= K[1] * P01_temp;
  
  return filter->angle;
}

// 从加速度计数据计算角度
void calculateAccelAngles(float *accel_roll, float *accel_pitch) 
{
  // 将原始整数值转换为物理量
  float accel_factor = 16384.0f; // 对于±2g量程
  float ax = MPU6050_Data.accel_x / accel_factor;
  float ay = MPU6050_Data.accel_y / accel_factor;
  float az = MPU6050_Data.accel_z / accel_factor;
  
  // 通过加速度计计算roll和pitch角度
  *accel_roll = atan2f(ay, az) * 180.0f / M_PI;
  *accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
}

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
