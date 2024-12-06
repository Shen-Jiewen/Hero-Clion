//
// Created by Rick on 2024/11/29.
//

#include "bsp_rc.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

/**
 * @brief  初始化 UART5 接收 DMA 配置。
 *
 * @param  rx_buf     接收数据缓冲区的指针。
 * @param  dma_buf_num  DMA 缓冲区的大小（即要接收的字节数）。
 *
 * @note   该函数启动了 UART5 的 DMA 接收，并将数据存储到指定的缓冲区。
 */
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num){
	// 配置双缓冲,DMA将在两个缓冲区之间轮换
	HAL_DMAEx_MultiBufferStart_IT(&hdma_uart5_rx, (uint32_t)&huart5.Instance->RDR,(uint32_t)rx1_buf, (uint32_t)rx2_buf, dma_buf_num);
	// 启动 UART5 DMA 接收操作，将接收到的数据存储到 rx_buf 缓冲区中
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx1_buf, dma_buf_num);
	// 清除IDLE标志
	__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	// 使能IDLE中断
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
}

/**
 * @brief  禁用 UART5 的 DMA 接收功能。
 *
 * 该函数禁用 DMA 通道，以便在重新配置或停止 DMA 时使用。
 */
void RC_unable(void){
	// 禁用 UART5 的 DMA 接收
	__HAL_DMA_DISABLE(&hdma_uart5_rx);
}

/**
 * @brief  重新启动 UART5 的 DMA 接收。
 *
 * @param  dma_buf_num  新的 DMA 缓冲区大小，表示接收的数据字节数。
 *
 * @note   该函数会禁用 UART5 和 DMA 接收，然后重新配置 DMA 缓冲区大小，最后启用 UART5 和 DMA 进行接收。
 */
void RC_restart(uint16_t dma_buf_num){
	// 禁用 UART5 外设，确保 DMA 配置期间不会发生接收中断
	__HAL_UART_DISABLE(&huart5);

	// 禁用 DMA 通道，停止当前 DMA 接收操作
	__HAL_DMA_DISABLE(&hdma_uart5_rx);

	// 设置 DMA 接收的计数器，更新传输字节数
	// 使用宏 __HAL_DMA_SET_COUNTER 来更新剩余传输字节数（NDTR 或 CNDTR）
	__HAL_DMA_SET_COUNTER(&hdma_uart5_rx, dma_buf_num);

	// 重新启用 UART5 外设，准备接收数据
	__HAL_UART_ENABLE(&huart5);

	// 重新启用 DMA 接收通道，开始新的 DMA 数据接收
	__HAL_DMA_ENABLE(&hdma_uart5_rx);
}

/**
  * @brief  切换 UART DMA 接收缓冲区
  * @param  rx_buf:        指向新的接收缓冲区的指针
  * @param  dma_buf_num:   DMA 接收的缓冲区大小（即每个缓冲区的数据长度）
  * @retval None
  *
  * 该函数在接收到 UART 数据时，切换 DMA 的接收缓冲区。
  * 通过调用 `HAL_UARTEx_ReceiveToIdle_DMA`，重新配置 DMA 接收新的数据到指定的缓冲区。
  */
void RC_change_dma_memory(uint8_t *rx_buf, uint16_t dma_buf_num)
{
	// 启动 DMA 接收操作，将数据接收存储到新的缓冲区 rx_buf 中
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf, dma_buf_num);
}

