/*******************************************************************************
 * @file: bsp_uart.h
 * @author: Javen
 * @date: 2024年12月17日
 * @brief: DM02开发板串口驱动
 * @note: 串口驱动，支持阻塞、中断、DMA发送模式
 *
 * @copyright: Copyright (c) 2022 | basic_framework_MC02
 * @license: MIT
 ******************************************************************************/

#include "bsp_usart.h"

/* usart服务实例, 所有注册了usart的模块信息会保存在这里 */
static uint8_t idx;
static usart_instance_t usart_instance[DEVICE_USART_CNT] = {0};

/**
 * @brief 启动串口服务，传入一个USART实例用于启动串口的工作
 * @param _instance 串口实例，启动相应串口的服务
 */
void usart_service_init(usart_instance_t *_instance)
{
	HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
	// 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
	// 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
	// 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
	__HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

/**
 * @brief 注册一个串口实例，并返回串口实例指针
 * @param init_config 传入串口初始化结构体，包含串口的配置项
 * @return usart_instance_t* 返回创建的串口实例指针，如果初始化失败则返回NULL
 */
usart_instance_t *usart_register(usart_init_config_s *init_config)
{
	if (idx >= DEVICE_USART_CNT) // 超过最大实例数
		return NULL; // 返回NULL表示超过最大实例数

	for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
		if (usart_instance[i].usart_handle == init_config->usart_handle)
			return NULL; // 返回NULL表示已经注册过

	usart_instance_t *instance = &usart_instance[idx++];  // 直接使用数组中的下一个元素
	memset(instance, 0, sizeof(usart_instance_t));  // 清空结构体内容

	instance->usart_handle = init_config->usart_handle;
	instance->recv_buff_size = init_config->recv_buff_size;
	instance->module_callback = init_config->module_callback;

	usart_service_init(instance);
	return instance;
}

/**
 * @brief 发送一帧数据，传入串口实例、发送缓冲区及数据长度进行发送
 * @param _instance 串口实例，指定要发送数据的串口
 * @param send_buf 待发送数据的缓冲区
 * @param send_size 发送数据的字节数
 * @param mode 发送模式，可以选择阻塞模式、IT模式或DMA模式
 */
void usart_send(usart_instance_t *_instance, uint8_t *send_buf, uint16_t send_size, usart_transfer_mode mode)
{
	switch (mode)
	{
	case USART_TRANSFER_BLOCKING:
		HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
		break;
	case USART_TRANSFER_IT:
		HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
		break;
	case USART_TRANSFER_DMA:
		HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
		break;
	default:
		return; // 非法模式，直接返回
	}
}

/**
 * @brief 判断串口是否准备好，用于IT/DMA模式下的连续或异步发送
 * @param _instance 要判断的串口实例
 * @return uint8_t 返回1表示串口准备好发送，返回0表示串口正在忙
 */
uint8_t usart_is_ready(usart_instance_t *_instance)
{
	if (_instance->usart_handle->gState | HAL_UART_STATE_BUSY_TX)
		return 0;  // 串口忙碌，返回0
	else
		return 1;  // 串口空闲，返回1
}

/**
 * @brief 每次DMA/IDLE中断发生时，都会调用此函数。对于每个UART实例会调用对应的回调进行进一步的处理
 * @note 通过__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT)关闭DMA半传输中断，防止两次进入HAL_UARTEx_RxEventCallback()
 * @param huart 发生中断的串口
 * @param Size 此次接收到的总数，暂时没用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	for (uint8_t i = 0; i < idx; ++i)
	{
		// 找到正在处理的实例
		if (huart == usart_instance[i].usart_handle)
		{ // 调用回调函数
			if (usart_instance[i].module_callback != NULL)
			{
				usart_instance[i].module_callback();
				memset(usart_instance[i].recv_buff, 0, Size); // 接收结束后清空缓冲区，对于变长数据是必要的
			}
			HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i].usart_handle, usart_instance[i].recv_buff, usart_instance[i].recv_buff_size);
			__HAL_DMA_DISABLE_IT(usart_instance[i].usart_handle->hdmarx, DMA_IT_HT);
			return; // 退出循环
		}
	}
}

/**
 * @brief 当串口发送/接收出现错误时，会调用此函数，重新启动接收
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	for (uint8_t i = 0; i < idx; ++i)
	{
		if (huart == usart_instance[i].usart_handle)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i].usart_handle, usart_instance[i].recv_buff, usart_instance[i].recv_buff_size);
			__HAL_DMA_DISABLE_IT(usart_instance[i].usart_handle->hdmarx, DMA_IT_HT);
			return;  // 发生错误时重新启动接收
		}
	}
}
