//
// Created by Rick on 2024/12/18.
//

#include "cmsis_os.h"
#include "bsp_pwm.h"
#include "detect.h"

#define BUZZER

// 定义蜂鸣器开关宏
#ifdef BUZZER
#define BUZZER_OFF() BSP_PWM_SetDutyCycle(&htim12, PWM_CHANNEL_2, 0);
#define BUZZER_ON() BSP_PWM_SetDutyCycle(&htim12, PWM_CHANNEL_2, 3000);
#else
#define BUZZER_OFF()
#define BUZZER_ON()
#endif

// 静态函数声明
static void buzzer_warn_error(uint8_t error_count);
static void buzzer_play_startup_sound(void);

// 外部定时器句柄
extern TIM_HandleTypeDef htim12;

// 本地错误列表指针
const error_t* error_list_local;

/**
 * @brief 蜂鸣器任务函数，用于检测错误并控制蜂鸣器
 * @param argument 任务参数（未使用）
 */
_Noreturn void buzzer_task(__attribute__((unused)) void* argument)
{
	// 配置 PWM
	PWM_InitTypeDef pwm_config = {
		.htim = &htim12,
		.channel = PWM_CHANNEL_2,
		.frequency = 4000,  // 频率为 4kHz
		.duty_cycle = 0     // 占空比为 0%
	};

	// 初始化 PWM
	BSP_PWM_Init(&pwm_config);

	// 开启声音
	buzzer_play_startup_sound();

	// 重新初始化
	BSP_PWM_Init(&pwm_config);

	// 初始化错误状态变量
	static uint8_t current_error = 0;
	static uint8_t last_error = 0;
	static int32_t error_index = 0;

	// 获取错误列表指针
	error_list_local = get_error_list_point();

	while (1)
	{
		// 查找错误
		current_error = 0;  // 重置当前错误状态
		for (error_index = 0; error_index < REFEREE_TOE; error_index++)
		{
			if (error_list_local[error_index].enable == 0)
			{
				continue;  // 如果错误未启用，跳过
			}
			if (error_list_local[error_index].error_exist)
			{
				current_error = 1;  // 发现错误
				break;
			}
		}

		// 如果没有错误且上次有错误，关闭蜂鸣器
		if (current_error == 0 && last_error != 0)
		{
			BUZZER_OFF()
		}

		// 如果有错误，发出警告
		if (current_error)
		{
			buzzer_warn_error(error_index + 1);
		}

		// 更新上次错误状态
		last_error = current_error;

		// 任务延时
		osDelay(10);
	}
}

/**
 * @brief 蜂鸣器警告错误函数，根据错误数量发出警告
 * @param error_count 错误数量
 */
static void buzzer_warn_error(uint8_t error_count)
{
	static uint8_t show_count = 0;
	static uint8_t stop_count = 100;

	if (show_count == 0 && stop_count == 0)
	{
		show_count = error_count;  // 设置显示的错误数量
		stop_count = 100;          // 重置停止计数
	}
	else if (show_count == 0)
	{
		stop_count--;  // 减少停止计数
		BUZZER_OFF() // 关闭蜂鸣器
	}
	else
	{
		static uint8_t tick = 0;
		tick++;

		if (tick < 50)
		{ // NOLINT(*-branch-clone)
			BUZZER_OFF() // 关闭蜂鸣器
		}
		else if (tick < 100)
		{
			BUZZER_ON() // 打开蜂鸣器
		}
		else
		{
			tick = 0;       // 重置 tick
			show_count--;   // 减少显示的错误数量
		}
	}
}

/**
 * @brief 播放开机声音，模拟DJI电机启动时的声音
 */
static void buzzer_play_startup_sound(void)
{
	// 定义频率和持续时间数组
	const uint16_t frequencies[] = {
		5211, 1636, 1335, 6373, 1765, 2411
	};

	const uint16_t durations[] = {
		100, 100, 300, 200, 200, 100
	};

	// 播放声音
	for (int i = 0; i < sizeof(frequencies) / sizeof(frequencies[0]); i++)
	{
		BSP_PWM_SetFrequency(&htim12, frequencies[i]); // 设置频率
		BSP_PWM_SetDutyCycle(&htim12, PWM_CHANNEL_2, 3000); // 设置占空比
		osDelay(durations[i]); // 持续指定时间
		BSP_PWM_SetDutyCycle(&htim12, PWM_CHANNEL_2, 0); // 关闭蜂鸣器
		osDelay(10); // 添加短暂间隔
	}
}