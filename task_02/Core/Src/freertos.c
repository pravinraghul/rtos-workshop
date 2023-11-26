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
static char *msg_ptr = NULL;
static int size = 0;
static volatile uint8_t msg_flag = 0;

/* USER CODE END Variables */
osThreadId echo_back_taskHandle;
osThreadId input_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void EchoBackTask(void const * argument);
void InputTask(void const * argument);

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
  /* definition and creation of echo_back_task */
  osThreadDef(echo_back_task, EchoBackTask, osPriorityNormal, 0, 128);
  echo_back_taskHandle = osThreadCreate(osThread(echo_back_task), NULL);

  /* definition and creation of input_task */
  osThreadDef(input_task, InputTask, osPriorityNormal, 0, 128);
  input_taskHandle = osThreadCreate(osThread(input_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_EchoBackTask */
/**
  * @brief  Function implementing the echo_back_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_EchoBackTask */
void EchoBackTask(void const * argument)
{
  /* USER CODE BEGIN EchoBackTask */
  /* Infinite loop */
  for(;;)
  {
    if (msg_flag) {
      HAL_UART_Transmit(&huart5, (uint8_t*)msg_ptr, size, 100);
      vPortFree(msg_ptr);
      msg_ptr = NULL;
      msg_flag = 0;
    }

    osDelay(1);
  }
  /* USER CODE END EchoBackTask */
}

/* USER CODE BEGIN Header_InputTask */
/**
* @brief Function implementing the input_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InputTask */
void InputTask(void const * argument)
{
  /* USER CODE BEGIN InputTask */
  /* Infinite loop */
  char buf[50] = {0};
  int index = 0;
  uint8_t byte = 0;
  for(;;)
  {
    HAL_UART_Receive(&huart5, &byte, 1, 10);

    if (byte != 0) {
      HAL_UART_Transmit(&huart5, &byte, 1, 100);
      if (byte == '\r') {

        buf[index++] = '\r';
        buf[index++] = '\n';

        msg_ptr = (char *)pvPortMalloc(index * sizeof(char));
        if (msg_ptr == NULL) { // incase of not making memory
          continue;
        }

        // copy the data into the memory
        memcpy(msg_ptr, buf, index);
        size = index;

        // set the message flag true
        msg_flag = 1;

        // clear the buffer
        memset(buf, 0, size);
      } else {
        buf[index] = byte;
        index++;
      }

      byte = 0;
    }
    osDelay(10);
  }
  /* USER CODE END InputTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
