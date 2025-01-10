//
// Created by Rick on 2025/1/4.
//

#include "main.h"
#include "cmsis_os.h"

_Noreturn void watch_task(__attribute__((unused)) void* argument){

	// 喂狗任务,间隔20ms
	while (1){
		// 延时
		osDelay(20);
	}
}