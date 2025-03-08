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

typedef struct{
  float angle;      
  float bias;     
  float P[2][2];   
  float Q_angle;   
  float Q_bias;     
  float measure;  
} KalmanFilter;

typedef struct{
  float w,x,y,z;
}Quaternion;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_PI 3.1415926 //圆周率
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
float raw_roll,raw_pitch,raw_yaw;
float filtered_roll, filtered_pitch, filtered_yaw;

float alpha = 0.98;
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

  void kalmanInit(KalmanFilter *filter, float Q_angle, float Q_bias, float measure);
  kalmanInit(&rollKalman, 0.001f, 0.003f, 0.03f);
  kalmanInit(&pitchKalman, 0.001f, 0.003f, 0.03f);
  kalmanInit(&yawKalman, 0.001f, 0.003f, 0.03f);
  float kalmanUpdate(KalmanFilter *filter, float newAngle, float newRate, float dt);
  void calculateAccelAngles(float *accel_roll, float *accel_pitch); 

  void complementaryFilter(float *angle, float accel_angle, float gyro_rate, float dt, float alpha);

  roll = pitch = yaw = 0.0f;
  q.w = 1.0f;
  q.x = q.y = q.z = 0.0f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    dt = DWT_GetDeltaT(&DWT_CNT);
    MPU6050_read_data();

    float gx_dps, gy_dps, gz_dps;
    convertGyroData(MPU6050_Data.gyro_x, MPU6050_Data.gyro_y, MPU6050_Data.gyro_z, 
                    &gx_dps, &gy_dps, &gz_dps);
    
    updateQuaternion(&q,MPU6050_Data.gyro_x,MPU6050_Data.gyro_y,MPU6050_Data.gyro_z,dt);

    calculateAngles(&q,&raw_pitch,&raw_roll,&raw_yaw);

    float accel_roll, accel_pitch;
    calculateAccelAngles(&accel_roll, &accel_pitch);

    float gyro_factor = 131.0f;
    float gyro_roll = MPU6050_Data.gyro_x / gyro_factor;
    float gyro_pitch = MPU6050_Data.gyro_y / gyro_factor;
    float gyro_yaw = MPU6050_Data.gyro_z / gyro_factor;

    filtered_roll = kalmanUpdate(&rollKalman, accel_roll, gyro_roll, dt);
    filtered_pitch = kalmanUpdate(&pitchKalman, accel_pitch, gyro_pitch, dt);
    complementaryFilter(&yaw, raw_yaw, gz_dps, dt, 0.95f);

    roll = filtered_roll;
    pitch = filtered_pitch;


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

void convertGyroData(int16_t gx, int16_t gy, int16_t gz, float *gx_dps, float *gy_dps, float *gz_dps) 
{
  // 根据MPU6050配置的灵敏度（+-250°/s对应131，+-500°/s对应65.5...）
  float gyro_factor = 131.0f;  // 对应±250°/s量程
  
  // 去除零偏（理想情况下应该通过校准获得）
  static float gyro_x_offset = 0;
  static float gyro_y_offset = 0;
  static float gyro_z_offset = 0;
  
  // 第一次调用时计算零偏（或者可以实现更复杂的校准程序）
  static int calibration_counter = 0;
  if (calibration_counter < 100) 
  {
    if (calibration_counter == 0) 
    {
      gyro_x_offset = 0;
      gyro_y_offset = 0;
      gyro_z_offset = 0;
    }
    gyro_x_offset += gx / gyro_factor;
    gyro_y_offset += gy / gyro_factor;
    gyro_z_offset += gz / gyro_factor;
    
    calibration_counter++;
    if (calibration_counter == 100) 
    {
      gyro_x_offset /= 100;
      gyro_y_offset /= 100;
      gyro_z_offset /= 100;
    }
  }
  
  // 转换为度/秒并去除零偏
  *gx_dps = gx / gyro_factor - gyro_x_offset;
  *gy_dps = gy / gyro_factor - gyro_y_offset;
  *gz_dps = gz / gyro_factor - gyro_z_offset;
}

void InitQuaternion(Quaternion *q) 
{
  float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
  if (norm < 1e-6f) 
  {
    q->w = 1.0f;
    q->x = q->y = q->z = 0.0f;
  } 
  else 
  {
    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
  }
}

Quaternion multiplyQuaternions(const Quaternion *q1, const Quaternion *q2)//四元数相乘
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
  float gx_dps, gy_dps, gz_dps;
  convertGyroData(gx, gy, gz, &gx_dps, &gy_dps, &gz_dps);
  
  // 转换为弧度/秒
  float gx_rad = gx_dps * M_PI / 180.0f;
  float gy_rad = gy_dps * M_PI / 180.0f;
  float gz_rad = gz_dps * M_PI / 180.0f;
  
  float half_dt = dt * 0.5f;
  float wx = gx_rad * half_dt;
  float wy = gy_rad * half_dt;
  float wz = gz_rad * half_dt;

  // 计算角增量的模长
  float angle_norm = sqrtf(wx*wx + wy*wy + wz*wz);
  
  Quaternion q_temp;
  
  if (angle_norm < 1e-6f) 
  {
    // 如果角增量很小，使用近似计算
    q_temp.w = 1.0f;
    q_temp.x = wx;
    q_temp.y = wy;
    q_temp.z = wz;
  } 
  else 
  {
    float half_angle = angle_norm;
    float sin_half_angle = sinf(half_angle) / angle_norm;
    
    q_temp.w = cosf(half_angle);
    q_temp.x = wx * sin_half_angle;
    q_temp.y = wy * sin_half_angle;
    q_temp.z = wz * sin_half_angle;
  }

  // 更新四元数
  *q = multiplyQuaternions(q, &q_temp);

  // 归一化四元数
  InitQuaternion(q);
}

void calculateAngles(Quaternion *q, float *roll, float *pitch, float *yaw) 
{
  *roll = atan2f(2.0f * (q->w * q->x + q->y * q->z), 
                1.0f - 2.0f * (q->x * q->x + q->y * q->y)) * 180.0f / M_PI;
  
  *pitch = asinf(2.0f * (q->w * q->y - q->z * q->x)) * 180.0f / M_PI;
  
  *yaw = atan2f(2.0f * (q->w * q->z + q->x * q->y),
                1.0f - 2.0f * (q->y * q->y + q->z * q->z)) * 180.0f / M_PI;
}

void kalmanInit(KalmanFilter *filter, float Q_angle, float Q_bias, float measure) //卡尔曼滤波初始化
{
  filter->angle = 0.0f;
  filter->bias = 0.0f;
  
  filter->P[0][0] = 1.0f;
  filter->P[0][1] = 0.0f;
  filter->P[1][0] = 0.0f;
  filter->P[1][1] = 1.0f;
  
  filter->Q_angle = Q_angle;
  filter->Q_bias = Q_bias;
  filter->measure = measure;
}

float kalmanUpdate(KalmanFilter *filter, float newAngle, float newRate, float dt) //卡尔曼滤波运算
{
  // 预测步骤
  filter->angle += dt * (newRate - filter->bias);
  
  filter->P[0][0] += dt * (dt * filter->P[1][1] - filter->P[0][1] - filter->P[1][0] + filter->Q_angle);
  filter->P[0][1] -= dt * filter->P[1][1];
  filter->P[1][0] -= dt * filter->P[1][1];
  filter->P[1][1] += filter->Q_bias * dt;
  
  // 更新步骤
  float y = newAngle - filter->angle;
  float S = filter->P[0][0] + filter->measure;
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

void calculateAccelAngles(float *accel_roll, float *accel_pitch) 
{
  // 将原始整数值转换为物理量
  float accel_factor = 16384.0f;
  float ax = MPU6050_Data.accel_x / accel_factor;
  float ay = MPU6050_Data.accel_y / accel_factor;
  float az = MPU6050_Data.accel_z / accel_factor;
  
  // 加速度计计算
  *accel_roll = atan2f(ay, az) * 180.0f / M_PI;
  *accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
}

// 添加互补滤波器函数
void complementaryFilter(float *angle, float accel_angle, float gyro_rate, float dt, float alpha) 
{
    *angle = alpha * (*angle + gyro_rate * dt) + (1.0f - alpha) * accel_angle;
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
