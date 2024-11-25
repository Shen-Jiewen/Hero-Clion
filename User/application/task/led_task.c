#include <sys/cdefs.h>
//
// Created by Rick on 24-11-25.
//

#include "main.h"
#include "FreeRTOS.h"
#include "bsp_ws2812.h"
#include "cmsis_os.h"

_Noreturn void led_task(void* argument)
{
	while (1)
	{
		// 设置颜色
		WS2812_SetColor(WS2812_COLOR_YELLOW);
		HAL_Delay(100);
	}
}