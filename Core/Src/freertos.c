/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tick.h"
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
extern volatile uint32_t task_tick;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for usbTask */
osThreadId_t usbTaskHandle;
const osThreadAttr_t usbTask_attributes = {
  .name = "usbTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for refereeTask */
osThreadId_t refereeTaskHandle;
const osThreadAttr_t refereeTask_attributes = {
  .name = "refereeTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for chassisTask */
osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for detectTask */
osThreadId_t detectTaskHandle;
const osThreadAttr_t detectTask_attributes = {
  .name = "detectTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for remoteControlTa */
osThreadId_t remoteControlTaHandle;
const osThreadAttr_t remoteControlTa_attributes = {
  .name = "remoteControlTa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gimbalTask */
osThreadId_t gimbalTaskHandle;
const osThreadAttr_t gimbalTask_attributes = {
  .name = "gimbalTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for buzzerTask */
osThreadId_t buzzerTaskHandle;
const osThreadAttr_t buzzerTask_attributes = {
  .name = "buzzerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for shootTask */
osThreadId_t shootTaskHandle;
const osThreadAttr_t shootTask_attributes = {
  .name = "shootTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for watchTask */
osThreadId_t watchTaskHandle;
const osThreadAttr_t watchTask_attributes = {
  .name = "watchTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for uiTask */
osThreadId_t uiTaskHandle;
const osThreadAttr_t uiTask_attributes = {
  .name = "uiTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for minigimbalTask */
osThreadId_t minigimbalTaskHandle;
const osThreadAttr_t minigimbalTask_attributes = {
  .name = "minigimbalTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for imuBinarySem01 */
osSemaphoreId_t imuBinarySem01Handle;
const osSemaphoreAttr_t imuBinarySem01_attributes = {
  .name = "imuBinarySem01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void led_task(void *argument);
extern void imu_task(void *argument);
extern void usb_task(void *argument);
extern void referee_task(void *argument);
extern void chassis_task(void *argument);
extern void detect_task(void *argument);
extern void remote_control_task(void *argument);
extern void gimbal_task(void *argument);
extern void buzzer_task(void *argument);
extern void shoot_task(void *argument);
extern void watch_task(void *argument);
extern void ui_task(void *argument);
extern void mini_gimbal_task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
	tick_init();
}

__weak unsigned long getRunTimeCounterValue(void)
{
	return task_tick;
}
/* USER CODE END 1 */

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

  /* Create the semaphores(s) */
  /* creation of imuBinarySem01 */
  imuBinarySem01Handle = osSemaphoreNew(1, 1, &imuBinarySem01_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(led_task, NULL, &ledTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(imu_task, NULL, &imuTask_attributes);

  /* creation of usbTask */
  usbTaskHandle = osThreadNew(usb_task, NULL, &usbTask_attributes);

  /* creation of refereeTask */
  refereeTaskHandle = osThreadNew(referee_task, NULL, &refereeTask_attributes);

  /* creation of chassisTask */
  chassisTaskHandle = osThreadNew(chassis_task, NULL, &chassisTask_attributes);

  /* creation of detectTask */
  detectTaskHandle = osThreadNew(detect_task, NULL, &detectTask_attributes);

  /* creation of remoteControlTa */
  remoteControlTaHandle = osThreadNew(remote_control_task, NULL, &remoteControlTa_attributes);

  /* creation of gimbalTask */
  gimbalTaskHandle = osThreadNew(gimbal_task, NULL, &gimbalTask_attributes);

  /* creation of buzzerTask */
  buzzerTaskHandle = osThreadNew(buzzer_task, NULL, &buzzerTask_attributes);

  /* creation of shootTask */
  shootTaskHandle = osThreadNew(shoot_task, NULL, &shootTask_attributes);

  /* creation of watchTask */
  watchTaskHandle = osThreadNew(watch_task, NULL, &watchTask_attributes);

  /* creation of uiTask */
  uiTaskHandle = osThreadNew(ui_task, NULL, &uiTask_attributes);

  /* creation of minigimbalTask */
  minigimbalTaskHandle = osThreadNew(mini_gimbal_task, NULL, &minigimbalTask_attributes);

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
	for (;;)
	{
		osThreadSuspend(osThreadGetId());
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

