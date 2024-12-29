/*******************************************************************************
 * @file: bsp_tim.c
 * @author: Javen
 * @date: 2024年12月18日
 * @brief: DM02开发板定时器驱动实现
 * @note: 该文件实现了定时器的初始化、启动、停止、周期设置和回调函数注册功能。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#include "bsp_tim.h"

// 定义最大定时器实例数量
#define DEVICE_TIM_CNT 5

// 定时器服务实例, 所有注册了定时器的模块信息会保存在这里
static uint8_t idx = 0;
static BSP_TIM_InitTypeDef tim_instance[DEVICE_TIM_CNT] = {0};

/**
 * @brief 初始化定时器实例
 * @param tim_config 定时器初始化结构体指针
 */
void BSP_TIM_Init(BSP_TIM_InitTypeDef* tim_config)
{
	if (idx >= DEVICE_TIM_CNT) // 超过最大实例数
		return;

	for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
		if (tim_instance[i].htim == tim_config->htim)
			return;

	BSP_TIM_InitTypeDef* instance = &tim_instance[idx++];  // 直接使用数组中的下一个元素
	memset(instance, 0, sizeof(BSP_TIM_InitTypeDef));  // 清空结构体内容

	instance->htim = tim_config->htim;
	instance->period = tim_config->period;
	instance->callback = tim_config->callback;

	// 配置定时器周期
	__HAL_TIM_SET_AUTORELOAD(tim_config->htim, tim_config->period - 1);

	// 启动定时器
	HAL_TIM_Base_Start_IT(tim_config->htim);
}

/**
 * @brief 启动定时器
 * @param htim 定时器句柄
 */
void BSP_TIM_Start(TIM_HandleTypeDef* htim)
{
	if (htim == NULL)
		return;

	HAL_TIM_Base_Start_IT(htim);
}

/**
 * @brief 停止定时器
 * @param htim 定时器句柄
 */
void BSP_TIM_Stop(TIM_HandleTypeDef* htim)
{
	if (htim == NULL)
		return;

	HAL_TIM_Base_Stop_IT(htim);
}

/**
 * @brief 设置定时器周期
 * @param htim 定时器句柄
 * @param period 定时器周期
 */
void BSP_TIM_SetPeriod(TIM_HandleTypeDef* htim, uint32_t period)
{
	if (htim == NULL)
		return;

	__HAL_TIM_SET_AUTORELOAD(htim, period - 1);
}

/**
 * @brief 注册定时器溢出回调函数
 * @param htim 定时器句柄
 * @param callback 回调函数指针
 */
void BSP_TIM_RegisterCallback(TIM_HandleTypeDef* htim, void (*callback)(void))
{
	for (uint8_t i = 0; i < idx; ++i)
	{
		if (tim_instance[i].htim == htim)
		{
			tim_instance[i].callback = callback;
			return;
		}
	}
}

/**
 * @brief 定时器溢出中断回调函数
 * @param htim 定时器句柄
 */
void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	for (uint8_t i = 0; i < idx; ++i)
	{
		if (htim == tim_instance[i].htim)
		{
			if (tim_instance[i].callback != NULL)
			{
				tim_instance[i].callback();
			}
			return;
		}
	}
}