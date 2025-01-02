//
// Created by Rick on 2024/12/30.
//

#include "tick.h"

extern TIM_HandleTypeDef htim24;

volatile uint32_t task_tick = 0;
volatile uint64_t imu_tick = 0;

static void tick_callback(void);

void tick_init(void){
	// 初始化定时器
	// 溢出时间为10微秒
	BSP_TIM_InitTypeDef tim_config = {
		.htim = &htim24,
		.period = 399,
		.callback = tick_callback
	};
	BSP_TIM_Init(&tim_config);
}

// 定时器回调函数
static void tick_callback(void){
	task_tick++;
	imu_tick++;
}