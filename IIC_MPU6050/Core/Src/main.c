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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int16_t accel_bias[3] = {0, 0, 0};
int16_t gyro_bias[3] = {0, 0, 0};

int16_t ax_data, ay_data, az_data;
int16_t gx_data, gy_data, gz_data;
float ax, ay, az;
float gx, gy, gz;

AngleProcessor angle_processer;

char message[100] = "";

float time_interval = 0.006f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim == &htim1){

		MPU_Get_Angle_KalmanFilter(accel_bias, gyro_bias, &angle_processer, time_interval);
		float roll = angle_processer.roll_hat;
		float pitch = angle_processer.pitch_hat;
		sprintf(message, "roll:%.1f, pitch:%.1f\n ", roll, pitch);
		HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message), 100);

	}
}
*/
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  AngleProcessor_Init(&angle_processer, 0.005, 0.05);
  MPU_Init();

  MPU_Calibrate_Accel_Data(accel_bias);

  char calibrate_accel_data[50] = "";
  sprintf(calibrate_accel_data, "ax:%d, ay:%d, az:%d;", accel_bias[0], accel_bias[1], accel_bias[2]);
  HAL_UART_Transmit(&huart2, (uint8_t*)calibrate_accel_data , strlen(calibrate_accel_data), HAL_MAX_DELAY);

  MPU_Calibrate_Gyro_Data(gyro_bias);

  char calibrate_gyro_data[50] = "";
  sprintf(calibrate_gyro_data, "gx:%d, gy:%d, gz:%d;", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
  HAL_UART_Transmit(&huart2, (uint8_t*)calibrate_gyro_data , strlen(calibrate_gyro_data), HAL_MAX_DELAY);

  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//-------------------------------------------------------
/*
	  MPU_Get_Accelerometer_Data(&ax_data, &ay_data, &az_data);
	  MPU_Get_Gyroscope_Data(&gx_data, &gy_data, &gz_data);

	  sprintf(message, "gyro: %d,%d,%d;\n accel: %.d,%d,%.d;\n ", gx_data, gy_data, gz_data, ax_data, ay_data, az_data);
	  HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message), HAL_MAX_DELAY);

	  HAL_Delay(500);
*/
//--------------------------------------------------------
/*
	  MPU_Get_Gyroscope(&gx, &gy, &gz, gyro_bias);
	  MPU_Get_Accelerometer(&ax, &ay, &az, accel_bias);

	  sprintf(message, "gyro(°/s): %.1f,%.1f,%.1f;\n accel(g): %.1f,%.1f,%.1f;\n ", gx, gy, gz, ax, ay, az);
	  HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message), HAL_MAX_DELAY);

	  HAL_Delay(500);
*/
//---------------------------------------------------------
	  MPU_Get_Angle_KalmanFilter(accel_bias, gyro_bias, &angle_processer, time_interval);
	  float roll = angle_processer.roll_hat;
	  float pitch = angle_processer.pitch_hat;
	  sprintf(message, "roll:%.1f, pitch:%.1f\n ", roll, pitch);
	  HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message), 50);

	  HAL_Delay(50);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
