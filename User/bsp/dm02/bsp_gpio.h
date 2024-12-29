/*******************************************************************************
 * @file: bsp_gpio.h
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

#ifndef DM_02_HERO_USER_BSP_DM02_BSP_GPIO_H_
#define DM_02_HERO_USER_BSP_DM02_BSP_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// 定义GPIO模式
typedef enum
{
	BSP_GPIO_MODE_INPUT = 0,   			// 输入模式
	BSP_GPIO_MODE_OUTPUT,      			// 输出模式
	BSP_GPIO_MODE_EXTI,        			// 外部中断模式
	BSP_GPIO_NOPULL = GPIO_NOPULL, 		// 无上拉/下拉
	BSP_GPIO_PULLUP = GPIO_PULLUP, 		// 上拉
	BSP_GPIO_PULLDOWN = GPIO_PULLDOWN,	// 下拉
} BSP_GPIO_Mode_t;

// 定义GPIO初始化结构体
typedef struct
{
	GPIO_TypeDef* port;    // GPIO端口
	uint16_t pin;          // GPIO引脚
	BSP_GPIO_Mode_t mode;  // GPIO模式
	uint8_t pull;          // 上拉/下拉配置
	uint8_t speed;         // GPIO速度
	IRQn_Type irqn;        // 中断号（仅在外部中断模式下使用）
} BSP_GPIO_InitTypeDef;

// 定义回调函数类型
typedef void (*BSP_GPIO_Callback_t)(void);

// 函数声明
void BSP_GPIO_Init(BSP_GPIO_InitTypeDef* gpio_config);
void BSP_GPIO_RegisterCallback(uint16_t pin, BSP_GPIO_Callback_t callback);
void BSP_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state);
GPIO_PinState BSP_GPIO_ReadPin(GPIO_TypeDef* port, uint16_t pin);

#ifdef __cplusplus
}
#endif

#endif //DM_02_HERO_USER_BSP_DM02_BSP_GPIO_H_
