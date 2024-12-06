//
// Created by Rick on 2024/12/6.
//

#include "bsp_usart.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

/**
  * @brief          初始化 USART1 的 DMA 接收，并配置 IDLE 中断
  * @param[in]      rx1_buf: 第一个接收缓冲区的指针，用于存储接收到的数据
  * @param[in]      rx2_buf: 第二个接收缓冲区的指针，用于存储接收到的数据
  * @param[in]      dma_buf_num: 每次 DMA 接收的缓冲区大小，通常为缓冲区的字节数
  * @retval         none
  *
  * 该函数初始化 USART1 的 DMA 接收操作，启动接收过程，并配置 IDLE 中断。
  * 使用双缓冲区模式（rx1_buf 和 rx2_buf）以实现数据接收的连续性。
  * 具体步骤包括：
  * 1. 使用 `HAL_DMAEx_MultiBufferStart_IT` 启动 DMA 双缓冲区接收。
  * 2. 启动 UART DMA 接收操作，并将数据存储到第一个缓冲区（rx1_buf）。
  * 3. 清除 UART 的 IDLE 标志，以便正确处理接收到的数据。
  * 4. 使能 UART 的 IDLE 中断，方便在接收完成后进行中断处理。
  */
void usart1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num){
	// 启动 USART1 DMA 双缓冲区接收，并启用中断
	HAL_DMAEx_MultiBufferStart_IT(&hdma_usart1_rx, (uint32_t)&huart1.Instance->RDR, (uint32_t)rx1_buf, (uint32_t)rx2_buf, dma_buf_num);

	// 启动 UART1 DMA 接收操作，将接收到的数据存储到 rx1_buf 缓冲区中
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buf, dma_buf_num);

	// 清除 IDLE 标志位，以便接下来的接收操作不会被误触发
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);

	// 使能 UART1 的 IDLE 中断，以便在接收完成后进行相关中断处理
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}
