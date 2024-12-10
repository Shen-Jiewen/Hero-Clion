//
// Created by Rick on 2024/12/10.
//

#include "cmsis_os.h"
#include "bsp_rc.h"
#include "dt7.h"
#include "dma_buffer.h"
#include "detect.h"

#define REMOTE_CONTROL_INIT_TIME 20

static void rc_full_callback(UART_HandleTypeDef *huart);

DT7_ctrl_t *dt7_ctrl;

// 接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
extern UART_HandleTypeDef huart5;
// 定义双缓冲区对象
dma_buffer_t rc_dma_buffer;

_Noreturn void remote_control_task(void* argument)
{
	// 等待其他设备上线
	vTaskDelay(REMOTE_CONTROL_INIT_TIME);
	// 获取遥控器对象
	dt7_ctrl = get_dt7_point();
	// 初始化外设
	dt7_init(sbus_rx_buf[0], sbus_rx_buf[1], RC_FRAME_LENGTH);
	// 初始化缓冲区
	if(dma_buffer_init(&rc_dma_buffer, &huart5, sbus_rx_buf[0], sbus_rx_buf[1], RC_FRAME_LENGTH) != HAL_OK){
		Error_Handler();
	}
	// 注册完全传输回调
	dma_buffer_register_full_callback(&rc_dma_buffer, rc_full_callback);
	if(dma_buffer_start_receive(&rc_dma_buffer) != HAL_OK){
		Error_Handler();
	}

	while (1) {
		vTaskSuspend(NULL);
	}
}

/**
 * @brief          接收并处理 RC 数据帧的回调函数
 * @param[in]      huart: UART 句柄，用于标识接收到数据的 UART 外设
 * @note           该函数在 UART 接收中断服务程序中被调用，用于接收 RC 数据帧并处理数据。
 *                 主要功能包括：
 *                 1. 清除 UART 的奇偶校验错误标志；
 *                 2. 计算本次接收的数据长度；
 *                 3. 切换双缓冲区进行数据接收；
 *                 4. 处理接收到的数据并将其写入 FIFO 队列；
 *                 5. 调用掉线检测。
 * @retval         none
 */
static void rc_full_callback(UART_HandleTypeDef *huart)
{
	// 静态变量用于存储本次接收的数据长度
	static uint16_t this_time_rx_len = 0;

	// 清除 UART 的奇偶校验错误标志
	__HAL_UART_CLEAR_PEFLAG(huart);

	// 禁用 DMA 接收，确保安全访问 DMA 寄存器
	__HAL_DMA_DISABLE(huart->hdmarx);

	// 计算本次接收的数据长度
	this_time_rx_len = RC_FRAME_LENGTH - __HAL_DMA_GET_COUNTER(huart->hdmarx);

	// 重置 DMA 计数器，为下一次接收做准备
	__HAL_DMA_SET_COUNTER(huart->hdmarx, RC_FRAME_LENGTH);

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
		sbus_to_dt7(sbus_rx_buf[0]);
	}
	else if (dma_buffer->current_buffer_index == 1)
	{
		// 当前使用的是 buffer2，将数据写入 FIFO
		sbus_to_dt7(sbus_rx_buf[1]);
	}
	// 掉线检测
	detect_hook(DBUS_TOE);
}
