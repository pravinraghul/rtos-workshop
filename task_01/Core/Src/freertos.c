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
uint32_t delay_time = 1000;
/* USER CODE END Variables */
osThreadId led_control_tasHandle;
osThreadId uart_receive_taHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LedControlTask(void const * argument);
void UartReceiveTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of led_control_tas */
  osThreadDef(led_control_tas, LedControlTask, osPriorityNormal, 0, 128);
  led_control_tasHandle = osThreadCreate(osThread(led_control_tas), NULL);

  /* definition and creation of uart_receive_ta */
  osThreadDef(uart_receive_ta, UartReceiveTask, osPriorityNormal, 0, 128);
  uart_receive_taHandle = osThreadCreate(osThread(uart_receive_ta), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_LedControlTask */
/**
  * @brief  Function implementing the led_control_tas thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LedControlTask */
void LedControlTask(void const * argument)
{
  /* USER CODE BEGIN LedControlTask */
  /* Infinite loop */
  char msg[] = "toggle LED\r\n";
  for(;;)
  {
    HAL_UART_Transmit(&huart5, (uint8_t*)msg, sizeof(msg), 100);
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    osDelay(delay_time);
  }
  /* USER CODE END LedControlTask */
}

/* USER CODE BEGIN Header_UartReceiveTask */
/**
* @brief Function implementing the uart_receive_ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UartReceiveTask */
void UartReceiveTask(void const * argument)
{
  /* USER CODE BEGIN UartReceiveTask */
  /* Infinite loop */
  char buffer[30] = {0};
  uint8_t byte = 0;
  uint32_t local_time = 0;

  for(;;)
  {
    HAL_UART_Receive(&huart5, &byte, 1, 100);

    if (byte != 0) {
      // echo back
      HAL_UART_Transmit(&huart5, &byte, 1, 100);

      if ((char)byte == '\r') {
        delay_time = local_time;
        local_time = 0;

        // debug print
        int size = sprintf(buffer, "updated delay: %ld\r\n", delay_time);
        HAL_UART_Transmit(&huart5, (uint8_t*)buffer, size+2, 100);
      } else if ((char)byte >= '0' && (char)byte <= '9'){
        local_time = local_time * 10 + byte - 48;
      }

      // clear after used
      byte = 0;
    }

    osDelay(10);
  }
  /* USER CODE END UartReceiveTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
