//
// Created by Rick on 2024/12/10.
//

#include "cmsis_os.h"
#include "dt7.h"
#include "detect.h"
#include "bsp_usart.h"

extern UART_HandleTypeDef huart5;
usart_instance_t* sbusInstance;

static void rc_callback(void);

_Noreturn void remote_control_task(void* argument)
{
	// 开启串口
	sbusInstance = usart_register(&((usart_init_config_s){ RC_FRAME_LENGTH, &huart5, rc_callback }));

	while (1)
	{
		osThreadSuspend(NULL);
	}
}

static void rc_callback(void)
{
	// 解析数据
	sbus_to_dt7(get_dt7_point(), sbusInstance->recv_buff);
	// 掉线检测
	detect_hook(DBUS_TOE);
}