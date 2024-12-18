/*******************************************************************************
 * @file: bsp_pwm.c
 * @author: Javen
 * @date: 2024年12月18日
 * @brief: BSP层PWM驱动实现
 * @note: 该文件实现了PWM驱动的初始化、占空比设置、启动和停止功能。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#include "bsp_pwm.h"

/**
 * @brief 初始化PWM
 * @param pwm_config PWM配置结构体指针
 */
void BSP_PWM_Init(PWM_InitTypeDef* pwm_config)
{
	if (pwm_config == NULL || pwm_config->htim == NULL)
	{
		return;
	}

	// 配置定时器
	pwm_config->htim->Init.Prescaler = (SystemCoreClock / pwm_config->frequency / 10000) - 1;
	pwm_config->htim->Init.Period = 10000 - 1;  // 10000对应100%占空比
	HAL_TIM_PWM_Init(pwm_config->htim);

	// 配置PWM通道
	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pwm_config->duty_cycle;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	switch (pwm_config->channel)
	{
	case PWM_CHANNEL_1:
		HAL_TIM_PWM_ConfigChannel(pwm_config->htim, &sConfigOC, TIM_CHANNEL_1);
		break;
	case PWM_CHANNEL_2:
		HAL_TIM_PWM_ConfigChannel(pwm_config->htim, &sConfigOC, TIM_CHANNEL_2);
		break;
	case PWM_CHANNEL_3:
		HAL_TIM_PWM_ConfigChannel(pwm_config->htim, &sConfigOC, TIM_CHANNEL_3);
		break;
	case PWM_CHANNEL_4:
		HAL_TIM_PWM_ConfigChannel(pwm_config->htim, &sConfigOC, TIM_CHANNEL_4);
		break;
	default:
		break;
	}

	// 启动PWM
	BSP_PWM_Start(pwm_config->htim, pwm_config->channel);
}

/**
 * @brief 设置PWM占空比
 * @param htim 定时器句柄
 * @param channel PWM通道
 * @param duty_cycle 占空比（0-10000，对应0%-100%）
 */
void BSP_PWM_SetDutyCycle(TIM_HandleTypeDef* htim, PWM_Channel_t channel, uint16_t duty_cycle)
{
	if (htim == NULL || duty_cycle > 10000)
	{
		return;
	}

	switch (channel)
	{
	case PWM_CHANNEL_1:
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, duty_cycle);
		break;
	case PWM_CHANNEL_2:
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, duty_cycle);
		break;
	case PWM_CHANNEL_3:
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, duty_cycle);
		break;
	case PWM_CHANNEL_4:
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, duty_cycle);
		break;
	default:
		break;
	}
}

/**
 * @brief 启动PWM
 * @param htim 定时器句柄
 * @param channel PWM通道
 */
void BSP_PWM_Start(TIM_HandleTypeDef* htim, PWM_Channel_t channel)
{
	if (htim == NULL)
	{
		return;
	}

	switch (channel)
	{
	case PWM_CHANNEL_1:
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
		break;
	case PWM_CHANNEL_2:
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
		break;
	case PWM_CHANNEL_3:
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
		break;
	case PWM_CHANNEL_4:
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
		break;
	default:
		break;
	}
}

/**
 * @brief 停止PWM
 * @param htim 定时器句柄
 * @param channel PWM通道
 */
void BSP_PWM_Stop(TIM_HandleTypeDef* htim, PWM_Channel_t channel)
{
	if (htim == NULL)
	{
		return;
	}

	switch (channel)
	{
	case PWM_CHANNEL_1:
		HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
		break;
	case PWM_CHANNEL_2:
		HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
		break;
	case PWM_CHANNEL_3:
		HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);
		break;
	case PWM_CHANNEL_4:
		HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_4);
		break;
	default:
		break;
	}
}

/**
 * @brief 设置PWM频率
 * @param htim 定时器句柄
 * @param frequency 频率（Hz）
 */
void BSP_PWM_SetFrequency(TIM_HandleTypeDef* htim, uint32_t frequency)
{
	if (htim == NULL || frequency == 0)
	{
		return;
	}

	// 计算定时器的周期
	uint32_t period = (SystemCoreClock / frequency) - 1;
	__HAL_TIM_SET_AUTORELOAD(htim, period);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, period / 2);  // 50%占空比
}

