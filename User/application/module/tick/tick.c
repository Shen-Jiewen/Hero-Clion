//
// Created by Rick on 2024/12/30.
//

#include "tick.h"

extern TIM_HandleTypeDef htim24;

volatile uint32_t task_tick = 0;

static void tick_callback(void);

void tick_init(void){
	// 初始化定时器
	BSP_TIM_InitTypeDef tim_config = {
		.htim = &htim24,
		.period = 500,
		.callback = tick_callback
	};
	BSP_TIM_Init(&tim_config);
}

// 定时器回调函数
static void tick_callback(void){
	task_tick++;
}