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
static uint32_t shared_buffer[10] = {0};
int count = 0;
/* USER CODE END Variables */
/* Definitions for producer */
osThreadId_t producerHandle;
const osThreadAttr_t producer_attributes = {
  .name = "producer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for consumer */
osThreadId_t consumerHandle;
const osThreadAttr_t consumer_attributes = {
  .name = "consumer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mutex01 */
osMutexId_t mutex01Handle;
const osMutexAttr_t mutex01_attributes = {
  .name = "mutex01"
};
/* Definitions for semFull */
osSemaphoreId_t semFullHandle;
const osSemaphoreAttr_t semFull_attributes = {
  .name = "semFull"
};
/* Definitions for semEmpty */
osSemaphoreId_t semEmptyHandle;
const osSemaphoreAttr_t semEmpty_attributes = {
  .name = "semEmpty"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void producerTask(void *argument);
void consumerTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of mutex01 */
  mutex01Handle = osMutexNew(&mutex01_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semFull */
  semFullHandle = osSemaphoreNew(1, 1, &semFull_attributes);

  /* creation of semEmpty */
  semEmptyHandle = osSemaphoreNew(1, 1, &semEmpty_attributes);

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
  /* creation of producer */
  producerHandle = osThreadNew(producerTask, NULL, &producer_attributes);

  /* creation of consumer */
  consumerHandle = osThreadNew(consumerTask, NULL, &consumer_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_producerTask */
/**
  * @brief  Function implementing the producer thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_producerTask */
void producerTask(void *argument)
{
  /* USER CODE BEGIN producerTask */
  /* Infinite loop */
  for(;;)
  {
    // Generated in the 500 ms interval
    osSemaphoreAcquire(semEmptyHandle, osWaitForever);
    if (count < 10) {
      shared_buffer[count] = osKernelGetTickCount();
      count++;
      HAL_UART_Transmit(&huart5, (uint8_t*)"Produced an item\r\n", 18, 100);
    } else {
      HAL_UART_Transmit(&huart5, (uint8_t*)"Waiting to be consumed\r\n", 22, 100);
    }
    osSemaphoreRelease(semFullHandle);
    osDelay(500); //
  }
  /* USER CODE END producerTask */
}

/* USER CODE BEGIN Header_consumerTask */
/**
* @brief Function implementing the consumer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_consumerTask */
void consumerTask(void *argument)
{
  /* USER CODE BEGIN consumerTask */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(semFullHandle, osWaitForever);
    if (count > 0) {
      int temp_value = shared_buffer[count];
      count--;
      HAL_UART_Transmit(&huart5, (uint8_t*)"Consumed an item\r\n", 18, 100);
    } else {
      HAL_UART_Transmit(&huart5, (uint8_t*)"Noitem to consume\r\n", 19, 100);
    }
    osSemaphoreRelease(semEmptyHandle);
    osDelay(1);
  }
  /* USER CODE END consumerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

