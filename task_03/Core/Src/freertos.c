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
#include <stdlib.h>
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
/* Definitions for led_control */
osThreadId_t led_controlHandle;
const osThreadAttr_t led_control_attributes = {
  .name = "led_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Queue01 */
osMessageQueueId_t Queue01Handle;
const osMessageQueueAttr_t Queue01_attributes = {
  .name = "Queue01"
};
/* Definitions for Queue02 */
osMessageQueueId_t Queue02Handle;
const osMessageQueueAttr_t Queue02_attributes = {
  .name = "Queue02"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ReadSerial(void *argument);
void LedControl(void *argument);

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

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Queue01 */
  Queue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &Queue01_attributes);

  /* creation of Queue02 */
  Queue02Handle = osMessageQueueNew (4, 16, &Queue02_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of read_serial */
  read_serialHandle = osThreadNew(ReadSerial, NULL, &read_serial_attributes);

  /* creation of led_control */
  led_controlHandle = osThreadNew(LedControl, NULL, &led_control_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ReadSerial */
/**
  * @brief  Function implementing the read_serial thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ReadSerial */
void ReadSerial(void *argument)
{
  /* USER CODE BEGIN ReadSerial */
  /* Infinite loop */
  char echo_buf[50] = {0};
  char byte = 0;
  int index = 0;
  char queue_buf[32] = {0};
  for(;;)
  {
    // Receive message from the queue02
    osStatus_t ret = osMessageQueueGet(Queue02Handle, queue_buf, NULL, 10);
    if (ret == osOK) {
      HAL_UART_Transmit(&huart5, (uint8_t*)queue_buf, sizeof(queue_buf), 100);
      memset(queue_buf, 0, sizeof(queue_buf));
    }
  
    // Receive message from the serial
    HAL_UART_Receive(&huart5, (uint8_t*)&byte, 1, 10);

    if (byte != 0) {
      HAL_UART_Transmit(&huart5, (uint8_t*)&byte, 1, 10);
      if (byte == '\r') {
          if (strncmp(echo_buf, "delay ", 6) == 0) {
            uint16_t delay = atoi(echo_buf+6);
            osMessageQueuePut(Queue01Handle, &delay, 0, 10);
          } else { // echo it back
            echo_buf[index++] = '\r';
            echo_buf[index++] = '\n';
            HAL_UART_Transmit(&huart5, (uint8_t*)echo_buf, index, 100);
          }
          memset(echo_buf, 0, index);
          index = 0;
      } else {
          echo_buf[index++] = byte;
      }
      byte = 0;
    }
  
    osDelay(10);
  }
  /* USER CODE END ReadSerial */
}

/* USER CODE BEGIN Header_LedControl */
/**
* @brief Function implementing the led_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedControl */
void LedControl(void *argument)
{
  /* USER CODE BEGIN LedControl */
  /* Infinite loop */
  uint32_t delay_time = 1000; // Default delay value
  uint32_t queue1_msg = 0;
  uint16_t led_blink_count = 1;
  char queue2_msg[32] = {0};
  char msg[] = "ledblink!\r\n";

  for(;;)
  {
    osStatus_t ret = osMessageQueueGet(Queue01Handle, &queue1_msg, NULL, 10);
    if (ret == osOK) {
      delay_time = queue1_msg;
      HAL_UART_Transmit(&huart5, (uint8_t*)"updated delay\r\n", 15, 100);
    }

    if (led_blink_count % 100 == 0) {
      memcpy(queue2_msg, msg, sizeof(msg));
      osMessageQueuePut(Queue02Handle, queue2_msg, 0, 100);
      memset(queue2_msg, 0, sizeof(queue2_msg));
    }

    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    led_blink_count++;
    osDelay(delay_time);
  }
  /* USER CODE END LedControl */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

