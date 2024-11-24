//
// Created by Rick on 24-11-25.
//

#ifndef DM_02_HERO_USER_BSP_DM02_BSP_WS2812_H_
#define DM_02_HERO_USER_BSP_DM02_BSP_WS2812_H_

#include "main.h"

// 定义宏来表示预定义颜色的 RGB 值
#define WS2812_COLOR_OFF      0, 0, 0
#define WS2812_COLOR_RED      255, 0, 0
#define WS2812_COLOR_GREEN    0, 255, 0
#define WS2812_COLOR_BLUE     0, 0, 255
#define WS2812_COLOR_YELLOW   255, 255, 0
#define WS2812_COLOR_CYAN     0, 255, 255
#define WS2812_COLOR_MAGENTA  255, 0, 255
#define WS2812_COLOR_WHITE    255, 255, 255

// 调用宏实现颜色设置
#define WS2812_SetColor(color_macro) ws2812_set_color(color_macro)

void ws2812_set_color(uint8_t r, uint8_t g, uint8_t b);
void HSV_to_RGB(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b);

#endif //DM_02_HERO_USER_BSP_DM02_BSP_WS2812_H_
