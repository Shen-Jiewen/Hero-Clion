//
// Created by Rick on 24-11-25.
//

#include "bsp_buzzer.h"

extern TIM_HandleTypeDef htim12;

/**
  * @brief          打开蜂鸣器并设置其频率和占空比
  * @author			Javen
  * @param[in]      psc: 定时器的预分频值，用于设置蜂鸣器的频率
  * @param[in]      pwm: 定时器的占空比，用于控制蜂鸣器的响度
  * @retval         无
  */
void buzzer_on(uint16_t psc, uint16_t pwm)
{
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
		__HAL_TIM_PRESCALER(&htim12, psc);
		__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, pwm);
}

/**
  * @brief          关闭蜂鸣器
  * @author         Javen
  * @retval         无
  */
void buzzer_off(void)
{
		__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, 0);
}

