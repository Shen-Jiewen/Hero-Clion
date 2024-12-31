/*******************************************************************************
 * @file: bsp_pwm.h
 * @author: Javen
 * @date: 2024年12月18日
 * @brief: DM02开发板PWM驱动
 * @note:
 * 1. 该驱动适用于DM02开发板，基于STM32 HAL库实现。
 * 2. PWM频率和占空比的设置范围如下：
 *    - 频率：根据定时器的时钟源和分频系数，可以设置不同的频率范围。
 *    - 占空比：占空比的范围为0-10000，其中0对应0%占空比，10000对应100%占空比。
 * 3. 在使用该驱动前，需确保定时器已正确配置并初始化。
 * 4. PWM通道的选择应与硬件连接的引脚一致，避免出现输出错误。
 * 5. 该驱动支持多通道PWM输出，但需注意定时器的资源限制。
 * 6. 在调用PWM初始化函数`BSP_PWM_Init`时，需传入一个`PWM_InitTypeDef`结构体指针，
 *    该结构体包含了定时器句柄、PWM通道、频率和占空比等信息。
 * 7. 通过`BSP_PWM_SetDutyCycle`函数可以动态调整PWM的占空比，但需确保占空比值在有效范围内。
 * 8. 使用`BSP_PWM_Start`和`BSP_PWM_Stop`函数可以分别启动和停止PWM输出。
 * 9. 该驱动未涉及PWM的死区时间设置，如需使用死区时间功能，需自行扩展。
 * 10. 该驱动仅提供基本的PWM控制功能，如需更复杂的功能（如PWM中断、互补输出等），
 *     需根据具体需求进行扩展。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#ifndef BSP_PWM_H_
#define BSP_PWM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// 定义PWM通道
typedef enum
{
	PWM_CHANNEL_1 = 0,
	PWM_CHANNEL_2,
	PWM_CHANNEL_3,
	PWM_CHANNEL_4
} PWM_Channel_t;

// 定义PWM初始化结构体
typedef struct
{
	TIM_HandleTypeDef* htim;  // 定时器句柄
	PWM_Channel_t channel;    // PWM通道
	uint32_t frequency;       // PWM频率（Hz）
	uint16_t duty_cycle;      // PWM占空比（0-10000，对应0%-100%）
} PWM_InitTypeDef;

// 函数声明
void BSP_PWM_Init(PWM_InitTypeDef* pwm_config);
void BSP_PWM_SetDutyCycle(TIM_HandleTypeDef* htim, PWM_Channel_t channel, uint16_t duty_cycle);
void BSP_PWM_Start(TIM_HandleTypeDef* htim, PWM_Channel_t channel);
void BSP_PWM_Stop(TIM_HandleTypeDef* htim, PWM_Channel_t channel);
void BSP_PWM_SetFrequency(TIM_HandleTypeDef* htim, uint32_t frequency);

#ifdef __cplusplus
}
#endif

#endif //BSP_PWM_H_
