//
// Created by Rick on 2024/12/10.
//

#include "cmsis_os.h"
#include "dt7.h"
#include "detect.h"
#include "bsp_uart.h"

// 定义接收缓冲区
uint8_t sbus_buffer[RC_FRAME_LENGTH] = {0};

extern UART_HandleTypeDef huart5;

static void rc_callback(void);

_Noreturn void remote_control_task(void* argument)
{
	// 初始化串口
	BSP_UART_InitTypeDef uart_config = {
		.huart = &huart5,                	// 串口句柄
		.recv_buff = sbus_buffer,        	// 接收缓冲区
		.recv_buff_size = RC_FRAME_LENGTH, 	// 接收缓冲区大小
		.callback = rc_callback      		// 接收完成回调函数
	};
	BSP_UART_Init(&uart_config);

	while (1)
	{
		osThreadSuspend(osThreadGetId());
	}
}

static void rc_callback(void)
{
	// 解析数据
	sbus_to_dt7(get_dt7_point(), sbus_buffer);
	// 掉线检测
	detect_hook(DBUS_TOE);
}