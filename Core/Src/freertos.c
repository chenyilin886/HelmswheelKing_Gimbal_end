/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "gimbal_c_api.h"
#include "usbd_cdc_if.h"//为了能调用 CDC_Transmit_FS
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GimbalTask */
osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTask_attributes = {
  .name = "GimbalTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for visionTxTask */
osThreadId_t visionTxTaskHandle;
const osThreadAttr_t visionTxTask_attributes = {
  .name = "visionTxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for visionTxQueue */
osMessageQueueId_t visionTxQueueHandle;
const osMessageQueueAttr_t visionTxQueue_attributes = {
  .name = "visionTxQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void gimbalTask(void *argument);
void StartVisionTxTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
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
  /* creation of visionTxQueue */
  visionTxQueueHandle = osMessageQueueNew (10, 29, &visionTxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of GimbalTask */
  GimbalTaskHandle = osThreadNew(gimbalTask, NULL, &GimbalTask_attributes);

  /* creation of visionTxTask */
  visionTxTaskHandle = osThreadNew(StartVisionTxTask, NULL, &visionTxTask_attributes);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_gimbalTask */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbalTask */
void gimbalTask(void *argument)
{
  /* USER CODE BEGIN gimbalTask */
  Gimbal_Init();
  osDelay(500);

  /* Infinite loop */
  for(;;)
  {
    Gimbal_Update();
    Gimbal_SendToChassis();
    osDelay(1);
  }
  /* USER CODE END gimbalTask */
}

/* USER CODE BEGIN Header_StartVisionTxTask */
/**
* @brief Function implementing the visionTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVisionTxTask */
void StartVisionTxTask(void *argument)
{
  /* USER CODE BEGIN StartVisionTxTask */
  // 准备一个 29 字节的本地数组，用来“接货”
  uint8_t tx_buf[29];
  uint8_t usb_ret;
  /* Infinite loop */
  for(;;)
  {
    // 1. 从队列中拿数据。osWaitForever 表示如果没有数据，任务就进入休眠，完全不占 CPU
    if (osMessageQueueGet(visionTxQueueHandle, tx_buf, NULL, osWaitForever) == osOK)
    {
      // 2. 拿到了数据，立刻尝试用 USB 发送
      usb_ret = CDC_Transmit_FS(tx_buf, 29);
      
      // 3. 安全的防丢包机制：如果此时 USB 硬件正忙，就稍等 1ms 再试一次，直到发出去为止
      // 注意：这里用 osDelay 是绝对安全的，因为这是个独立的普通优先级任务，绝不会卡死云台电机！
      while (usb_ret == USBD_BUSY)
      {
        osDelay(1);
        usb_ret = CDC_Transmit_FS(tx_buf, 29);
      }
    }
  }
  /* USER CODE END StartVisionTxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

