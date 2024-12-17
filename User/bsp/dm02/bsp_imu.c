//
// Created by Rick on 24-11-25.
//

#include "bsp_imu.h"

extern TIM_HandleTypeDef htim3;

void imu_pwm_start(void){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void imu_pwm_set(uint16_t pwm){
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm);
}