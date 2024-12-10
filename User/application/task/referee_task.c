#include <sys/cdefs.h>
//
// Created by Rick on 2024/12/6.
//

#include "main.h"
#include "cmsis_os.h"
#include "referee.h"
#include "protocol.h"
#include "fifo.h"
#include "dma_buffer.h"
#include "detect.h"

static void referee_full_callback(UART_HandleTypeDef *huart);

#define USART_RX_BUF_LENGTH     512
#define REFEREE_FIFO_BUF_LENGTH 1024

uint8_t usart1_buf[2][USART_RX_BUF_LENGTH];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

// 声明UART句柄
extern UART_HandleTypeDef huart1;
// 定义双缓冲区对象
dma_buffer_t referee_dma_buffer;

_Noreturn void referee_task(void *argument){
	// 裁判系统结构体初始化
	init_referee_struct_data();
	// 接收FIFO初始化
	fifo_s_init(&referee_fifo,  referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
	// 初始化缓冲区
	if(dma_buffer_init(&referee_dma_buffer, &huart1, usart1_buf[0], usart1_buf[1], USART_RX_BUF_LENGTH) != HAL_OK){
		Error_Handler();
	}
	// 注册完全传输回调
	dma_buffer_register_full_callback(&referee_dma_buffer, referee_full_callback);
	if(dma_buffer_start_receive(&referee_dma_buffer) != HAL_OK){
		Error_Handler();
	}

	while (1){
		// 串口数据解包
		referee_unpack_fifo_data(&referee_unpack_obj, &referee_fifo);
		vTaskDelay(100);
	}
}

/**
 * @brief 完全传输完成回调函数
 *
 * 当 UART DMA 接收完成时，此函数被调用。它执行以下操作：
 * 1. 清除 UART 的奇偶校验错误标志。
 * 2. 禁用 DMA 接收，以安全地访问 DMA 寄存器。
 * 3. 计算本次接收的数据长度。
 * 4. 重置 DMA 计数器，为下一次接收做准备。
 * 5. 切换到另一个缓冲区并重新启动 DMA 接收。
 * 6. 启用 DMA 接收。
 * 7. 将接收到的数据写入 FIFO 队列以供后续处理。
 *
 * @param huart 指向 UART 句柄的指针。
 */
static void referee_full_callback(UART_HandleTypeDef *huart)
{
	// 静态变量用于存储本次接收的数据长度
	static uint16_t this_time_rx_len = 0;

	// 清除 UART 的奇偶校验错误标志
	__HAL_UART_CLEAR_PEFLAG(huart);

	// 禁用 DMA 接收，确保安全访问 DMA 寄存器
	__HAL_DMA_DISABLE(huart->hdmarx);

	// 计算本次接收的数据长度
	this_time_rx_len = USART_RX_BUF_LENGTH - __HAL_DMA_GET_COUNTER(huart->hdmarx);

	// 重置 DMA 计数器，为下一次接收做准备
	__HAL_DMA_SET_COUNTER(huart->hdmarx, USART_RX_BUF_LENGTH);

	// 获取对应 UART 的双缓冲对象
	dma_buffer_t *dma_buffer = dma_buffer_get_by_uart(huart);
	if (dma_buffer != NULL)
	{
		// 切换到另一个缓冲区并重新启动 DMA 接收
		HAL_StatusTypeDef status = dma_buffer_switch_buffer(dma_buffer);
		if (status != HAL_OK)
		{
			// 如果切换缓冲区失败，调用错误处理函数
			Error_Handler();
			return;
		}
		else
		{
			// 重新启用 DMA 接收
			__HAL_DMA_ENABLE(huart->hdmarx);
		}
	}
	else
	{
		// 如果未找到对应的双缓冲对象，调用错误处理函数
		Error_Handler();
		return;
	}

	// 将接收到的数据写入 FIFO 队列
	if (dma_buffer->current_buffer_index == 0)
	{
		// 当前使用的是 buffer1，将数据写入 FIFO
		fifo_s_puts(&referee_fifo, (char *)usart1_buf[0], this_time_rx_len);
	}
	else if (dma_buffer->current_buffer_index == 1)
	{
		// 当前使用的是 buffer2，将数据写入 FIFO
		fifo_s_puts(&referee_fifo, (char *)usart1_buf[1], this_time_rx_len);
	}
	// 掉线检测
	detect_hook(REFEREE_TOE);
}