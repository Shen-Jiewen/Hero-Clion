#include <sys/cdefs.h>
//
// Created by Rick on 24-11-25.
//

#include "main.h"
#include "FreeRTOS.h"
#include "bsp_adc.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "chassis_module.h"

fp32 temperature = 0.f;
fp32 voltage_in = 0.f;
uint8_t psc = 23;

_Noreturn void log_task(void* argument)
{
	// 获取对象
	chassis_control_t* chassis_control = get_chassis_control_point();

	while (1)
	{
//		// 获取电压
//		init_vrefint_reciprocal();
//		// 获取温度
//		temperature = get_temperature();
//		// 获取电源电压
//		voltage_in = get_battery_voltage();

		// CAN发送数据
		chassis_control->CAN_cmd_chassis(10, 10, 10, 10);
		vTaskDelay(100);
	}
}