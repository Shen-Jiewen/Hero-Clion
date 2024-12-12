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

#define USART_RX_BUF_LENGTH     512
#define REFEREE_FIFO_BUF_LENGTH 1024

uint8_t usart1_buf[2][USART_RX_BUF_LENGTH];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

// 声明UART句柄
extern UART_HandleTypeDef huart1;

_Noreturn void referee_task(void *argument){
	// 裁判系统结构体初始化
	init_referee_struct_data();
	// 接收FIFO初始化
	fifo_s_init(&referee_fifo,  referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);

	while (1){
		// 串口数据解包
		referee_unpack_fifo_data(&referee_unpack_obj, &referee_fifo);
		vTaskDelay(100);
	}
}

static void referee_callback(UART_HandleTypeDef *huart, uint16_t Size)
{
	// 掉线检测
	detect_hook(REFEREE_TOE);
}