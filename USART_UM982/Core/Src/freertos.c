/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "UM982.h"
#include "stdio.h"

#include "usart.h"

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
/* USER CODE BEGIN Variables */

osSemaphoreId_t osBinarySemaphore_UART;

uint8_t ReceiveDataEx[300];

MessageBufferHandle_t osMessageBuffer;

UART_HandleTypeDef huart_um982;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UM982Task */
osThreadId_t UM982TaskHandle;
const osThreadAttr_t UM982Task_attributes = {
  .name = "UM982Task",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void UM982Handler(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, ReceiveDataEx, sizeof(ReceiveDataEx));

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	osBinarySemaphore_UART = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	osMessageBuffer = xMessageBufferCreate(BUFFER_SIZE);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UM982Task */
  UM982TaskHandle = osThreadNew(UM982Handler, NULL, &UM982Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_UM982Handler */
/**
* @brief Function implementing the UM982HandlerTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UM982Handler */
void UM982Handler(void *argument)
{
  /* USER CODE BEGIN UM982Handler */
	uint8_t ReceiveData[RECEIVE_MAX_SIZE];
	uint8_t received_size;

  /* Infinite loop */
  for(;;)
  {

		osStatus_t status = osSemaphoreAcquire(osBinarySemaphore_UART, pdMS_TO_TICKS(100));

		if (status == osOK){

			HAL_GPIO_TogglePin(Green_LED_GPIO_Port, Green_LED_Pin);

			// RxBuffer Receive + Size Estimation
			received_size = xMessageBufferReceive(osMessageBuffer, ReceiveData, sizeof(ReceiveData), pdMS_TO_TICKS(100));

			// Data Processing
			if (received_size > 0){
				Andly_GPS(&Plane_Position, ReceiveData);
			}

			// TxBuffer Transmit
			HAL_UART_Transmit(&huart2, ReceiveData, received_size, 100);

			int print_size = snprintf(NULL, 0, "LAT:%.6f, LON:%.6f, ALT:%.2f, SAT:%d STS:%d, LEN:%.4f, HEAD:%.4f, PIT:%.4f\n",
						Plane_Position.latitude,
						Plane_Position.longitude,
						Plane_Position.altitude,
						Plane_Position.gps_num,
						Plane_Position.gps_status,
						Plane_Position.length,
						Plane_Position.heading,
						Plane_Position.pitch
						);

			char CharData[print_size];

			snprintf(CharData, sizeof(CharData), "LAT:%.6f, LON:%.6f, ALT:%.2f, SAT:%d STS:%d, LEN:%.4f, HEAD:%.4f, PIT:%.4f\n",
					Plane_Position.latitude,
					Plane_Position.longitude,
					Plane_Position.altitude,
					Plane_Position.gps_num,
					Plane_Position.gps_status,
					Plane_Position.length,
					Plane_Position.heading,
					Plane_Position.pitch
					);

			HAL_UART_Transmit(&huart2, (uint8_t *)CharData, sizeof(CharData), 100);
		} else {
//  		HAL_UART_Transmit(&huart2, (uint8_t *)"Semaphore Error\n", 17, 100);
		}
  }
  /* USER CODE END UM982Handler */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

