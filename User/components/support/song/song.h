//
// Created by Rick on 2024/12/18.
//

#ifndef DM_02_HERO_USER_COMPONENTS_SUPPORT_SONG_SONG_H_
#define DM_02_HERO_USER_COMPONENTS_SUPPORT_SONG_SONG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_pwm.h"

// 定义音符频率（单位：Hz）
typedef enum
{
	NOTE_C4  = 262,  // 中音C
	NOTE_D4  = 294,  // 中音D
	NOTE_E4  = 330,  // 中音E
	NOTE_F4  = 349,  // 中音F
	NOTE_G4  = 392,  // 中音G
	NOTE_A4  = 440,  // 中音A
	NOTE_B4  = 494,  // 中音B
	NOTE_C5  = 523,  // 高音C
	// 可以根据需要添加更多音符
} Note_t;

// 定义音符结构体
typedef struct
{
	Note_t note;       // 音符频率
	uint16_t duration; // 音符持续时间（单位：毫秒）
} Note_TypeDef;

// 函数声明
void Song_Play(TIM_HandleTypeDef* htim, PWM_Channel_t channel, const Note_TypeDef* song, uint16_t length);
void Song_Delay(uint16_t ms);

#ifdef __cplusplus
}
#endif

#endif //DM_02_HERO_USER_COMPONENTS_SUPPORT_SONG_SONG_H_
