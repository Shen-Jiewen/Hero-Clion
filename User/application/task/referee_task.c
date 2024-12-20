#include "main.h"
#include "cmsis_os.h"
#include "referee.h"
#include "protocol.h"
#include "fifo.h"
#include "detect.h"
#include "bsp_uart.h"

static void referee_callback(void);

#define USART_RX_BUF_LENGTH     512
#define REFEREE_FIFO_BUF_LENGTH 1024

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

// 定义接收缓冲区
uint8_t referee_buffer[USART_RX_BUF_LENGTH];

// 声明UART句柄
extern UART_HandleTypeDef huart1;

_Noreturn void referee_task(void *argument){
	// 裁判系统结构体初始化
	init_referee_struct_data();
	// 接收FIFO初始化
	fifo_s_init(&referee_fifo,  referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
	// 初始化串口
	BSP_UART_InitTypeDef uart_config = {
		.huart = &huart1,                		// 串口句柄
		.recv_buff = referee_buffer,        	// 接收缓冲区
		.recv_buff_size = USART_RX_BUF_LENGTH, 	// 接收缓冲区大小
		.callback = referee_callback       		// 接收完成回调函数
	};
	BSP_UART_Init(&uart_config);

	while (1){
		// 串口数据解包
		referee_unpack_fifo_data(&referee_unpack_obj, &referee_fifo);
		osDelay(100);
	}
}

static void referee_callback(void)
{
	// 放到fifo中处理
	fifo_s_puts(&referee_fifo, (char *)referee_buffer, USART_RX_BUF_LENGTH);
	// 掉线检测
	detect_hook(REFEREE_TOE);
}