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
  float q0,q1,q2,q3;

}Quaternion;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050 MPU6050_Data;

Quaternion q = {1.0, 0.0, 0.0, 0.0};//初始化四元数
float Ki = 0;
float Kp = 1.0;
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;

uint32_t DWT_CNT;
float dt;

float roll, pitch, yaw;

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
  void MPU6050_Velocity(void);
  void QuaternionNormalize(Quaternion *q);
  void QuaternionUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt);
  void QuaternionToEuler(Quaternion *q, float *roll, float *pitch, float *yaw);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    dt = DWT_GetDeltaT(&DWT_CNT);
    MPU6050_read_data();
    MPU6050_Velocity();
    QuaternionUpdate(MPU6050_Data.gyro_x,MPU6050_Data.gyro_y,MPU6050_Data.gyro_z,
                     MPU6050_Data.accel_x,MPU6050_Data.accel_y,MPU6050_Data.accel_z,dt);
    QuaternionToEuler(&q,&roll,&pitch,&yaw);
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(100);
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
void MPU6050_Velocity(void)
{
  MPU6050_Data.velocity_x = MPU6050_Data.accel_x * dt;
  MPU6050_Data.velocity_y = MPU6050_Data.accel_y * dt;
  MPU6050_Data.velocity_z = MPU6050_Data.accel_z * dt;
}

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
void QuaternionNormalize(Quaternion *q) 
{
    float norm = sqrt(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
    q->q0 /= norm;
    q->q1 /= norm;
    q->q2 /= norm;
    q->q3 /= norm;
}

void QuaternionToEuler(Quaternion *q, float *roll, float *pitch, float *yaw) 
{
    *roll = atan2(2.0f * (q->q0 * q->q1 + q->q2 * q->q3), 1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2));
    *pitch = asin(2.0f * (q->q0 * q->q2 - q->q1 * q->q3));
    *yaw = atan2(2.0f * (q->q0 * q->q3 + q->q1 * q->q2), 1.0f - 2.0f * (q->q2 * q->q2 + q->q3 * q->q3));
}

void QuaternionUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) 
{
    float norm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 使用加速度计数据计算误差
    norm = sqrt(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    halfvx = q.q1 * q.q3 - q.q0 * q.q2;
    halfvy = q.q0 * q.q1 + q.q2 * q.q3;
    halfvz = q.q0 * q.q0 - 0.5f + q.q3 * q.q3;

    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // 积分误差
    integralFBx += Ki * halfex * dt;
    integralFBy += Ki * halfey * dt;
    integralFBz += Ki * halfez * dt;

    // 应用比例和积分控制
    gx = gx + Kp * halfex + integralFBx;
    gy = gy + Kp * halfey + integralFBy;
    gz = gz + Kp * halfez + integralFBz;

    // 归一化陀螺仪数据
    gx *= (1.0f / 131.0f);
    gy *= (1.0f / 131.0f);
    gz *= (1.0f / 131.0f);

    // 四元数微分方程
    qa = q.q0;
    qb = q.q1;
    qc = q.q2;
    q.q0 += (-qb * gx - qc * gy - q.q3 * gz) * 0.5f * dt;
    q.q1 += (qa * gx + qc * gz - q.q3 * gy) * 0.5f * dt;
    q.q2 += (qa * gy - qb * gz + q.q3 * gx) * 0.5f * dt;
    q.q3 += (qa * gz + qb * gy - qc * gx) * 0.5f * dt;

    // 归一化四元数
    QuaternionNormalize(&q);
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
