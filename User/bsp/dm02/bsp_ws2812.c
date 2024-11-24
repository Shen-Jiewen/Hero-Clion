//
// Created by Rick on 24-11-25.
//

#include "bsp_ws2812.h"

// 定义 WS2812 使用的 SPI 外设
extern SPI_HandleTypeDef hspi6;

#define WS2812_SPI_UNIT hspi6
// 定义 WS2812 的逻辑电平
#define WS2812_HighLevel 0xF8  // 高电平，基于时序要求调整值
#define WS2812_LowLevel  0xC0  // 低电平，基于时序要求调整值

/**
 * @brief WS2812 设置颜色
 * @param r 红色通道值（0~255）
 * @param g 绿色通道值（0~255）
 * @param b 蓝色通道值（0~255）
 *
 * @note 每次调用会发送 24 位数据流控制一个灯珠的颜色，GRB 顺序。
 *       如果控制多个灯珠，需要多次调用，并保证发送后尾部有足够的复位信号。
 */
void ws2812_set_color(uint8_t r, uint8_t g, uint8_t b) {
	uint8_t txbuf[24]; // 用于存储 24 位 GRB 数据的 SPI 数据缓冲区
	uint8_t reset_signal = 0x00; // 复位信号

	// 将 RGB 转换为 GRB 顺序，并填充到 SPI 数据缓冲区
	for (int i = 0; i < 8; i++) {
		txbuf[7 - i]  = ((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel; // 绿色
		txbuf[15 - i] = ((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel; // 红色
		txbuf[23 - i] = ((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel; // 蓝色
	}

	// 发送复位信号
	HAL_SPI_Transmit(&WS2812_SPI_UNIT, &reset_signal, 1, HAL_MAX_DELAY);

	// 等待 SPI 空闲
	while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);

	// 发送颜色数据
	HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, HAL_MAX_DELAY);

	// 确保尾部信号足够长（复位信号，确保灯珠正确更新）
	for (int i = 0; i < 50; i++) {
		HAL_SPI_Transmit(&WS2812_SPI_UNIT, &reset_signal, 1, HAL_MAX_DELAY);
	}

	HAL_Delay(1);
}

/**
 * @brief 将 HSV 转换为 RGB 值
 * @param h 色相（Hue），取值范围 0-360
 * @param s 饱和度（Saturation），取值范围 0-1
 * @param v 亮度（Value），取值范围 0-1
 * @param r 输出红色分量（取值范围 0-255）
 * @param g 输出绿色分量（取值范围 0-255）
 * @param b 输出蓝色分量（取值范围 0-255）
 */
void HSV_to_RGB(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b) {
	float c = v * s; // 色彩饱和度
	float x = c * (1 - fabsf(fmodf(h / 60.0, 2) - 1)); // NOLINT(*-narrowing-conversions)
	float m = v - c;
	float r_temp, g_temp, b_temp;

	if (h >= 0 && h < 60) {
		r_temp = c; g_temp = x; b_temp = 0;
	} else if (h >= 60 && h < 120) {
		r_temp = x; g_temp = c; b_temp = 0;
	} else if (h >= 120 && h < 180) {
		r_temp = 0; g_temp = c; b_temp = x;
	} else if (h >= 180 && h < 240) {
		r_temp = 0; g_temp = x; b_temp = c;
	} else if (h >= 240 && h < 300) {
		r_temp = x; g_temp = 0; b_temp = c;
	} else {
		r_temp = c; g_temp = 0; b_temp = x;
	}

	*r = (uint8_t)((r_temp + m) * 255);
	*g = (uint8_t)((g_temp + m) * 255);
	*b = (uint8_t)((b_temp + m) * 255);
}