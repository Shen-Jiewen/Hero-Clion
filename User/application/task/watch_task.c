//
// Created by Rick on 2025/1/4.
//

#include "main.h"
#include "cmsis_os.h"

extern IWDG_HandleTypeDef hiwdg1;

_Noreturn void watch_task(void* argument){

	// 喂狗任务,间隔20ms
	while (1){
		// 定时喂狗
		HAL_IWDG_Refresh(&hiwdg1);
		// 延时
		osDelay(20);
	}
}