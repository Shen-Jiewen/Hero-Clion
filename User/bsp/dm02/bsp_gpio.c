/*******************************************************************************
 * @file: bsp_gpio.c
 * @author: Javen
 * @date: 2024年12月18日
 * @brief: DM02开发板GPIO驱动
 * @note:
 * 1. 该驱动适用于DM02开发板，基于STM32 HAL库实现。
 * 2. 支持GPIO的输入和输出模式配置。
 * 3. 支持外部中断的回调函数注册和调用。
 * 4. 在使用该驱动前，需确保GPIO引脚已正确配置并初始化。
 * 5. 该驱动支持多GPIO引脚的配置和操作。
 * 6. 通过`BSP_GPIO_Init`函数可以初始化GPIO引脚。
 * 7. 通过`BSP_GPIO_RegisterCallback`函数可以注册外部中断的回调函数。
 * 8. 该驱动未涉及复杂的GPIO功能（如复用功能、模拟输入等），如需使用这些功能，需自行扩展。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#include "bsp_gpio.h"

// 定义回调函数数组
static BSP_GPIO_Callback_t gpio_callbacks[16] = {NULL};

/**
 * @brief: 初始化GPIO引脚
 * @param: gpio_config - GPIO初始化结构体指针
 * @retval: void
 */
void BSP_GPIO_Init(BSP_GPIO_InitTypeDef* gpio_config)
{
	GPIO_InitTypeDef init_struct;

	// 配置GPIO模式
	if (gpio_config->mode == BSP_GPIO_MODE_INPUT)
	{
		init_struct.Mode = GPIO_MODE_INPUT;
	}
	else if (gpio_config->mode == BSP_GPIO_MODE_OUTPUT)
	{
		init_struct.Mode = GPIO_MODE_OUTPUT_PP;
	}
	else if (gpio_config->mode == BSP_GPIO_MODE_EXTI)
	{
		init_struct.Mode = GPIO_MODE_IT_RISING; // 默认上升沿触发
	}

	// 配置上拉/下拉
	init_struct.Pull = gpio_config->pull;

	// 配置GPIO速度
	init_struct.Speed = gpio_config->speed;

	// 配置GPIO引脚
	init_struct.Pin = gpio_config->pin;

	// 初始化GPIO
	HAL_GPIO_Init(gpio_config->port, &init_struct);

	// 如果模式为外部中断，配置中断
	if (gpio_config->mode == BSP_GPIO_MODE_EXTI)
	{
		// @TODO 如果中断函数中调用到FreeRTOS的ISR的API, 需要配置优先级小于FreeRTOS的最大优先级, 以免ISR中调用FreeRTOS的API导致优先级错误
		HAL_NVIC_SetPriority(gpio_config->irqn, 10, 0);
		HAL_NVIC_EnableIRQ(gpio_config->irqn);
	}
}

/**
 * @brief: 注册GPIO外部中断回调函数
 * @param: port - GPIO端口
 * @param: pin - GPIO引脚
 * @param: callback - 回调函数指针
 * @retval: void
 */
void BSP_GPIO_RegisterCallback(uint16_t pin, BSP_GPIO_Callback_t callback)
{
	uint8_t index = __builtin_ctz(pin); // 获取引脚索引
	gpio_callbacks[index] = callback;
}

/**
 * @brief: 写GPIO引脚状态
 * @param: port - GPIO端口
 * @param: pin - GPIO引脚
 * @param: state - GPIO引脚状态（GPIO_PIN_RESET或GPIO_PIN_SET）
 * @retval: void
 */
void BSP_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state)
{
	HAL_GPIO_WritePin(port, pin, state);
}

/**
 * @brief: 读GPIO引脚状态
 * @param: port - GPIO端口
 * @param: pin - GPIO引脚
 * @retval: GPIO_PinState - GPIO引脚状态（GPIO_PIN_RESET或GPIO_PIN_SET）
 */
GPIO_PinState BSP_GPIO_ReadPin(GPIO_TypeDef* port, uint16_t pin)
{
	return HAL_GPIO_ReadPin(port, pin);
}

/**
 * @brief: GPIO中断回调处理函数
 * @param: GPIO_Pin - 触发中断的GPIO引脚
 * @retval: void
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t pin_index = __builtin_ctz(GPIO_Pin); // 获取引脚索引

	// 检查回调函数是否已注册
	if (gpio_callbacks[pin_index] != NULL)
	{
		gpio_callbacks[pin_index](); // 调用对应的回调函数
	}
}