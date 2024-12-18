//
// Created by Rick on 2024/12/18.
//

#include "bsp_pwm.h"

extern TIM_HandleTypeDef htim12;

_Noreturn void buzzer_task(void* argument){
	// 配置 PWM
	PWM_InitTypeDef pwm_config = {
		.htim = &htim12,
		.channel = PWM_CHANNEL_2,
		.frequency = 4000,  // 1kHz
		.duty_cycle = 5000  // 50%
	};
	// 初始化 PWM
	BSP_PWM_Init(&pwm_config);

	while (1){
		// 设置占空比
		BSP_PWM_SetDutyCycle(&htim12, PWM_CHANNEL_2, 0);
	}
}