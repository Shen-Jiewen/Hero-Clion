//
// Created by Rick on 2024/12/18.
//

#include "song.h"
#include "main.h"
#include "cmsis_os.h"

// 播放歌曲
void Song_Play(TIM_HandleTypeDef* htim, PWM_Channel_t channel, const Note_TypeDef* song, uint16_t length)
{
	for (uint16_t i = 0; i < length; i++)
	{
		// 设置PWM频率
		PWM_InitTypeDef pwm_config = {
			.htim = htim,
			.channel = channel,
			.frequency = song[i].note,
			.duty_cycle = 5000  // 50%占空比
		};
		BSP_PWM_Init(&pwm_config);

		// 启动PWM输出
		BSP_PWM_Start(htim, channel);

		// 延时音符持续时间
		Song_Delay(song[i].duration);

		// 停止PWM输出
		BSP_PWM_Stop(htim, channel);

		// 延时音符之间的间隔（可选）
		Song_Delay(100); // 100ms间隔
	}
}

// 延时函数（简单实现，实际项目中应使用系统提供的延时函数）
void Song_Delay(uint16_t ms)
{
	osDelay(ms);
}