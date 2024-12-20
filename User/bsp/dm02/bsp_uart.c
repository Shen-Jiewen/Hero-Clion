/*******************************************************************************
 * @file: bsp_uart.c
 * @author: Javen
 * @date: 2024年12月17日
 * @brief: DM02开发板串口驱动
 * @note: 串口驱动，支持阻塞、中断、DMA发送模式
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#include "bsp_uart.h"
#include <string.h>

// 定义最大串口实例数量
#define DEVICE_USART_CNT 5

// 串口服务实例, 所有注册了串口的模块信息会保存在这里
static uint8_t idx = 0;
static BSP_UART_InitTypeDef usart_instance[DEVICE_USART_CNT] = {0};

/**
 * @brief 启动串口服务，传入一个USART实例用于启动串口的工作
 * @param instance 串口实例，启动相应串口的服务
 */
static void BSP_UART_ServiceInit(BSP_UART_InitTypeDef* instance)
{
	HAL_UARTEx_ReceiveToIdle_DMA(instance->huart, instance->recv_buff, instance->recv_buff_size);
	// 关闭DMA半传输中断，防止两次进入HAL_UARTEx_RxEventCallback()
	__HAL_DMA_DISABLE_IT(instance->huart->hdmarx, DMA_IT_HT);
}

/**
 * @brief 初始化串口实例
 * @param uart_config 传入串口初始化结构体，包含串口的配置项
 */
void BSP_UART_Init(BSP_UART_InitTypeDef* uart_config)
{
	if (idx >= DEVICE_USART_CNT) // 超过最大实例数
		return;

	for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
		if (usart_instance[i].huart == uart_config->huart)
			return;

	BSP_UART_InitTypeDef* instance = &usart_instance[idx++];  // 直接使用数组中的下一个元素
	memset(instance, 0, sizeof(BSP_UART_InitTypeDef));  // 清空结构体内容

	instance->huart = uart_config->huart;
	instance->recv_buff = uart_config->recv_buff;          // 配置接收缓冲区数组
	instance->recv_buff_size = uart_config->recv_buff_size;
	instance->callback = uart_config->callback;

	BSP_UART_ServiceInit(instance);
}

/**
 * @brief 发送一帧数据，传入串口实例、发送缓冲区及数据长度进行发送
 * @param huart 串口句柄
 * @param send_buf 待发送数据的缓冲区
 * @param send_size 发送数据的字节数
 * @param mode 发送模式，可以选择阻塞模式、IT模式或DMA模式
 */
void BSP_UART_Send(UART_HandleTypeDef* huart, uint8_t* send_buf, uint16_t send_size, BSP_UART_Mode_t mode)
{
	switch (mode)
	{
	case BSP_UART_MODE_BLOCKING:
		HAL_UART_Transmit(huart, send_buf, send_size, HAL_MAX_DELAY);
		break;
	case BSP_UART_MODE_IT:
		HAL_UART_Transmit_IT(huart, send_buf, send_size);
		break;
	case BSP_UART_MODE_DMA:
		HAL_UART_Transmit_DMA(huart, send_buf, send_size);
		break;
	default:
		return; // 非法模式，直接返回
	}
}

/**
 * @brief 注册接收完成回调函数
 * @param huart 串口句柄
 * @param callback 回调函数指针
 */
void BSP_UART_RegisterCallback(UART_HandleTypeDef* huart, void (*callback)(void))
{
	for (uint8_t i = 0; i < idx; ++i)
	{
		if (usart_instance[i].huart == huart)
		{
			usart_instance[i].callback = callback;
			return;
		}
	}
}

/**
 * @brief 每次DMA/IDLE中断发生时，都会调用此函数。对于每个UART实例会调用对应的回调进行进一步的处理
 * @param huart 发生中断的串口
 * @param Size 此次接收到的总数，暂时没用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
	for (uint8_t i = 0; i < idx; ++i)
	{
		// 找到正在处理的实例
		if (huart == usart_instance[i].huart)
		{
			// 调用回调函数
			if (usart_instance[i].callback != NULL)
			{
				usart_instance[i].callback();
			}
			HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i].huart, usart_instance[i].recv_buff, usart_instance[i].recv_buff_size);
			__HAL_DMA_DISABLE_IT(usart_instance[i].huart->hdmarx, DMA_IT_HT);
			return; // 退出循环
		}
	}
}

/**
 * @brief 当串口发送/接收出现错误时，会调用此函数，重新启动接收
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
	for (uint8_t i = 0; i < idx; ++i)
	{
		if (huart == usart_instance[i].huart)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i].huart, usart_instance[i].recv_buff, usart_instance[i].recv_buff_size);
			__HAL_DMA_DISABLE_IT(usart_instance[i].huart->hdmarx, DMA_IT_HT);
			return;  // 发生错误时重新启动接收
		}
	}
}