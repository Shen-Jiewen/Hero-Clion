#include <sys/cdefs.h>
//
// Created by Rick on 24-11-25.
//

#include "main.h"
#include "ws2812.h"
#include "FreeRTOS.h"
#include "bsp_adc.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"

uint8_t r = 1;
uint8_t g = 1;
uint8_t b = 1;

fp32 temperature = 0.f;
fp32 voltage_in = 0.f;

uint8_t psc = 23;

_Noreturn void log_task(void *argument){
	while(1)
	{
		WS2812_Ctrl(r, g, b);
		r++;
		g += 5;
		b += 10;
		vTaskDelay(100);
		// 获取电压
		init_vrefint_reciprocal();
		// 获取温度
		temperature = get_temperature();
		// 获取电源电压
		voltage_in = get_battery_voltage();

		// 开启蜂鸣器
		buzzer_on(psc, 50);
	}
}