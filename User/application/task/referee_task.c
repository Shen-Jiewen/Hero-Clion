#include <sys/cdefs.h>
//
// Created by Rick on 2024/12/6.
//

#include "main.h"
#include "cmsis_os.h"
#include "referee.h"
#include "protocol.h"
#include "fifo.h"
#include "detect.h"
#include "bsp_usart.h"

static void referee_callback(void);

#define USART_RX_BUF_LENGTH     512
#define REFEREE_FIFO_BUF_LENGTH 1024

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

// 声明UART句柄
extern UART_HandleTypeDef huart1;
usart_instance_t *refereeInstance;

_Noreturn void referee_task(void *argument){
	// 裁判系统结构体初始化
	init_referee_struct_data();
	// 接收FIFO初始化
	fifo_s_init(&referee_fifo,  referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
	// 开启串口
	refereeInstance = usart_register(&((usart_init_config_s){USART_RX_BUF_LENGTH, &huart1, referee_callback}));

	while (1){
		// 串口数据解包
		referee_unpack_fifo_data(&referee_unpack_obj, &referee_fifo);
		osDelay(100);
	}
}

static void referee_callback(void)
{
	// 放到fifo中处理
	fifo_s_puts(&referee_fifo, (char *)refereeInstance->recv_buff, refereeInstance->recv_buff_size);
	// 掉线检测
	detect_hook(REFEREE_TOE);
}