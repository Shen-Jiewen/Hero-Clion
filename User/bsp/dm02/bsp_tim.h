/*******************************************************************************
 * @file: bsp_tim.h
 * @author: Javen
 * @date: 2024年12月18日
 * @brief: DM02开发板定时器驱动
 * @note:
 * 1. 该驱动适用于DM02开发板，基于STM32 HAL库实现。
 * 2. 支持定时器的启动、停止、周期设置和回调函数注册。
 * 3. 在使用该驱动前，需确保定时器已正确配置并初始化。
 * 4. 通过`BSP_TIM_Init`函数可以初始化定时器实例。
 * 5. 通过`BSP_TIM_Start`和`BSP_TIM_Stop`函数可以启动和停止定时器。
 * 6. 通过`BSP_TIM_SetPeriod`函数可以动态调整定时器的周期。
 * 7. 通过`BSP_TIM_RegisterCallback`函数可以注册定时器溢出回调函数。
 * 8. 该驱动未涉及复杂的定时器功能（如PWM、输入捕获等），如需使用这些功能，需自行扩展。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#ifndef DM_02_HERO_USER_BSP_DM02_BSP_TIM_H_
#define DM_02_HERO_USER_BSP_DM02_BSP_TIM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <string.h>

// 定义定时器初始化结构体
typedef struct
{
	TIM_HandleTypeDef* htim;  // 定时器句柄
	uint32_t period;          // 定时器周期
	void (*callback)(void);   // 定时器溢出回调函数
} BSP_TIM_InitTypeDef;

// 函数声明
void BSP_TIM_Init(BSP_TIM_InitTypeDef* tim_config);
void BSP_TIM_Start(TIM_HandleTypeDef* htim);
void BSP_TIM_Stop(TIM_HandleTypeDef* htim);
void BSP_TIM_SetPeriod(TIM_HandleTypeDef* htim, uint32_t period);
void BSP_TIM_RegisterCallback(TIM_HandleTypeDef* htim, void (*callback)(void));

void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif

#endif // DM_02_HERO_USER_BSP_DM02_BSP_TIM_H_