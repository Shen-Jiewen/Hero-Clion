//
// Created by Rick on 24-11-25.
//

#include "bsp_uart.h"

extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart10;

HAL_StatusTypeDef uart1_send_data(uint8_t* pData, uint16_t Size)
{
	return HAL_UART_Transmit_DMA(&huart1, pData, Size);
}

HAL_StatusTypeDef uart7_send_data(uint8_t* pData, uint16_t Size)
{
	return HAL_UART_Transmit_DMA(&huart7, pData, Size);
}

HAL_StatusTypeDef uart10_send_data(uint8_t* pData, uint16_t Size)
{
	return HAL_UART_Transmit_DMA(&huart10, pData, Size);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart->Instance == USART1)
	{

	}
	else if (huart->Instance == UART7)
	{

	}
	else if (huart->Instance == USART10)
	{

	}
	else
	{

	}
}
