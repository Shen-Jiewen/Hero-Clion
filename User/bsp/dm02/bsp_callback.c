//
// Created by Rick on 2024/12/3.
//

#include "bsp_callback.h"

// FDCAN
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

// Remote Control
extern void rc_callback(UART_HandleTypeDef *huart, uint16_t Size);
extern uint8_t sbus_rx_buf[RC_FRAME_LENGTH];

// Referee


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