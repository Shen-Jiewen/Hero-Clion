//
// Created by Rick on 2024/12/3.
//

#include "bsp_callback.h"

// FDCAN
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

// Remote Control
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

// Referee
extern UART_HandleTypeDef huart1;
extern dma_buffer_t referee_dma_buffer;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8]; // 数据缓存

	// 获取接收对象
	chassis_control_t* chassis_control = get_chassis_control_point();

	// 检查触发回调的具体 FDCAN 实例
	if (hfdcan == &hfdcan1)
	{
		// FDCAN1 的接收处理
		if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
		{
			if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
			{
				// 底盘电机获取信息
				chassis_control->CAN_rec_chassis(RxHeader.Identifier, RxData);
			}
		}
	}
	else if (hfdcan == &hfdcan2)
	{
		// FDCAN2 的接收处理
		if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
		{
			if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
			{
				// 处理 FDCAN2 的接收到的消息

			}
		}
	}
	else if (hfdcan == &hfdcan3)
	{
		// FDCAN3 的接收处理
		if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
		{
			if (HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
			{
				// 处理 FDCAN3 的接收到的消息

			}
		}
	}
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	static uint16_t current_rx_len = 0;

	if (huart == &huart5) // 确保是你需要处理的 UART
	{
		// 清除 IDLE 标志
		__HAL_UART_CLEAR_IDLEFLAG(huart);

		if(HAL_DMA_GetState(&hdma_uart5_rx) == HAL_DMA_STATE_READY){
			if(get_dt7_dma_memory() == DT7_DMA_MEMORY_0){
				// 当前 DMA 缓冲区 0 已经接收完成
				// 失效DMA
				__HAL_DMA_DISABLE(&hdma_uart5_rx);
				// 获取接收数据长度,长度 = 设定长度 - 剩余长度
				current_rx_len = SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);
				// 重新设定数据长度
				__HAL_DMA_SET_COUNTER(&hdma_uart5_rx, SBUS_RX_BUF_NUM);
				// 切换缓冲区
				DT7_change_dma_memory(DT7_DMA_MEMORY_1);
				// 使能DMA
				__HAL_DMA_ENABLE(&hdma_uart5_rx);
				// 解析数据
				if(current_rx_len == RC_FRAME_LENGTH){
					sbus_to_dt7(DT7_DMA_MEMORY_0);
				}
			}
			else if(get_dt7_dma_memory() == DT7_DMA_MEMORY_1){
				// 当前 DMA 缓冲区 1 已经接收完成
				// 失效DMA
				__HAL_DMA_DISABLE(&hdma_uart5_rx);
				// 获取接收数据长度,长度 = 设定长度 - 剩余长度
				current_rx_len = SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);
				// 重新设定数据长度
				__HAL_DMA_SET_COUNTER(&hdma_uart5_rx, SBUS_RX_BUF_NUM);
				// 切换缓冲区
				DT7_change_dma_memory(DT7_DMA_MEMORY_0);
				// 使能DMA
				__HAL_DMA_ENABLE(&hdma_uart5_rx);
				// 解析数据
				if(current_rx_len == RC_FRAME_LENGTH){
					sbus_to_dt7(DT7_DMA_MEMORY_1);
				}
			}
		}
	}
	else if(huart == &huart1){
		// 裁判系统数据解析
		referee_dma_buffer.full_complete_callback(huart);
	}
}
