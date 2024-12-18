#include <sys/cdefs.h>
//
// Created by Rick on 24-11-25.
//

#include "cmsis_os.h"
#include "ws2812.h"

uint8_t red = 1;
uint8_t green = 1;
uint8_t blue = 1;

_Noreturn void led_task(void* argument)
{
	while (1)
	{
		// 调用WS2812_Ctrl函数控制LED的颜色
		WS2812_Ctrl(red, green, blue);

		// 增加红色、绿色和蓝色的值
		red += 1;        // 红色增加1
		green += 5;      // 绿色增加5
		blue += 10;      // 蓝色增加10

		// 短时间延时，防止LED闪烁过快
		osDelay(1);

		// 在不同的时间点对颜色进行更大的调整
		red++;           // 再次增加红色值
		green++;         // 再次增加绿色值
		blue++;          // 再次增加蓝色值

		// 延时100毫秒，控制LED颜色变化速度
		osDelay(100);
	}
}
