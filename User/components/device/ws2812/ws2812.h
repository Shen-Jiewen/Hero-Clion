//
// Created by Rick on 2024/12/16.
//

#ifndef DM_02_HERO_USER_COMPONENTS_DEVICE_WS2812_WS2812_H_
#define DM_02_HERO_USER_COMPONENTS_DEVICE_WS2812_WS2812_H_

#include "main.h"

#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);

#endif //DM_02_HERO_USER_COMPONENTS_DEVICE_WS2812_WS2812_H_
