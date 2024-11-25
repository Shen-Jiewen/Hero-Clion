//
// Created by Rick on 24-11-25.
//

#include "bsp_imu.h"

extern TIM_HandleTypeDef htim3;
extern osSemaphoreId imuBinarySem01Handle;

void imu_pwm_start(void){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void imu_pwm_set(uint16_t pwm){
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm);
}

uint32_t imu_semaphore_wait(void){
	return osSemaphoreAcquire(imuBinarySem01Handle, osWaitForever);
}

/**
************************************************************************
* @brief:      	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* @param:       GPIO_Pin - 触发中断的GPIO引脚
* @retval:     	void
* @details:    	GPIO外部中断回调函数，处理加速度计和陀螺仪中断
************************************************************************
**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == ACC_INT_Pin)
	{
		osSemaphoreRelease(imuBinarySem01Handle);
	}
	else if(GPIO_Pin == GYRO_INT_Pin)
	{

	}
}