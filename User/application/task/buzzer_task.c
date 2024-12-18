//
// Created by Rick on 2024/12/18.
//

#include "cmsis_os.h"
#include "bsp_pwm.h"
#include "detect.h"

//#define BUZZER_ON

#ifdef BUZZER_ON
#define buzzer_off() BSP_PWM_SetDutyCycle(&htim12, PWM_CHANNEL_2, 0)
#define buffer_on() BSP_PWM_SetDutyCycle(&htim12, PWM_CHANNEL_2, 3000)
#else
#define buzzer_off()
#define buffer_on()
#endif

static void buzzer_warn_error(uint8_t num);
extern TIM_HandleTypeDef htim12;

const error_t *error_list_test_local;

_Noreturn void buzzer_task(void* argument){
	// 配置 PWM
	PWM_InitTypeDef pwm_config = {
		.htim = &htim12,
		.channel = PWM_CHANNEL_2,
		.frequency = 4000,  // 1kHz
		.duty_cycle = 0  // 0%
	};
	// 初始化 PWM
	BSP_PWM_Init(&pwm_config);
	// 初始化错误列表
	static uint8_t error, last_error;
	static uint16_t error_num;
	error_list_test_local = get_error_list_point();

	while (1){
		//find error
		for(error_num = 0; error_num < REFEREE_TOE; error_num++)
		{
			if(error_list_test_local[error_num].enable == 0)
			{
				continue;
			}
			if(error_list_test_local[error_num].error_exist)
			{
				error = 1;
				break;
			}
		}

		//no error, stop buzzer
		if(error == 0 && last_error != 0)
		{
			buzzer_off();
		}
		//have error
		if(error)
		{
			buzzer_warn_error(error_num+1);
		}

		last_error = error;
		osDelay(10);
	}
}

static void buzzer_warn_error(uint8_t num)
{
	static uint8_t show_num = 0;
	static uint8_t stop_num = 100;
	if(show_num == 0 && stop_num == 0)
	{
		show_num = num;
		stop_num = 100;
	}
	else if(show_num == 0)
	{
		stop_num--;
		buzzer_off();
	}
	else
	{
		static uint8_t tick = 0;
		tick++;
		if(tick < 50)
		{
			buzzer_off();
		}
		else if(tick < 100)
		{
			buffer_on();
		}
		else
		{
			tick = 0;
			show_num--;
		}
	}
}