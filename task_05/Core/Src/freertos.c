/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
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

/* USER CODE END Variables */
/* Definitions for read_serial */
osThreadId_t read_serialHandle;
const osThreadAttr_t read_serial_attributes = {
  .name = "read_serial",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for backlight_timer */
osTimerId_t backlight_timerHandle;
const osTimerAttr_t backlight_timer_attributes = {
  .name = "backlight_timer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ReadSerialTask(void *argument);
void backLightTimerCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of backlight_timer */
  backlight_timerHandle = osTimerNew(backLightTimerCallback, osTimerOnce, NULL, &backlight_timer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of read_serial */
  read_serialHandle = osThreadNew(ReadSerialTask, NULL, &read_serial_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ReadSerialTask */
/**
  * @brief  Function implementing the read_serial thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ReadSerialTask */
void ReadSerialTask(void *argument)
{
  /* USER CODE BEGIN ReadSerialTask */
  /* Infinite loop */
  char byte;
  for(;;)
  {
    HAL_UART_Receive(&huart5, (uint8_t*)&byte, 1, 10);
    if (byte != 0) {
      // Just echo the characters
      HAL_UART_Transmit(&huart5, (uint8_t*)&byte, 1, 10);
      // Turn on when the keys are pressed
      HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 1);
      // set the timeout for 5 sec
      osTimerStart(backlight_timerHandle, 5000); // backlight timeout 5000 ms
      byte = 0;
    }
    osDelay(10);
  }
  /* USER CODE END ReadSerialTask */
}

/* backLightTimerCallback function */
void backLightTimerCallback(void *argument)
{
  /* USER CODE BEGIN backLightTimerCallback */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 0); // turn off the backlight after the timeout period
  /* USER CODE END backLightTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

