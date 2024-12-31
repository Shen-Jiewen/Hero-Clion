/*******************************************************************************
 * @file: bsp_uart.h
 * @author: Javen
 * @date: 2024年12月17日
 * @brief: DM02开发板串口驱动
 * @note:
 * 1. 该驱动适用于DM02开发板，基于STM32 HAL库实现。
 * 2. 支持阻塞、中断、DMA三种发送模式。
 * 3. 在使用该驱动前，需确保串口已正确配置并初始化。
 * 4. 通过`BSP_UART_Init`函数可以初始化串口实例。
 * 5. 通过`BSP_UART_Send`函数可以发送数据，支持阻塞、中断和DMA模式。
 * 6. 通过`BSP_UART_RegisterCallback`函数可以注册接收完成回调函数。
 * 7. 该驱动未涉及复杂的串口功能（如硬件流控、奇偶校验等），如需使用这些功能，需自行扩展。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#ifndef BSP_UART_H_
#define BSP_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// 定义串口传输模式
typedef enum
{
	BSP_UART_MODE_BLOCKING = 0,  // 阻塞模式
	BSP_UART_MODE_IT,            // 中断模式
	BSP_UART_MODE_DMA            // DMA模式
} BSP_UART_Mode_t;

// 定义串口初始化结构体
typedef struct
{
	UART_HandleTypeDef* huart;  // 串口句柄
	uint8_t* recv_buff;          // 接收缓冲区数组
	uint16_t recv_buff_size;     // 接收缓冲区大小
	void (*callback)(void);      // 接收完成回调函数
} BSP_UART_InitTypeDef;

// 函数声明
void BSP_UART_Init(BSP_UART_InitTypeDef* uart_config);
void BSP_UART_Send(UART_HandleTypeDef* huart, uint8_t* send_buf, uint16_t send_size, BSP_UART_Mode_t mode);
void BSP_UART_RegisterCallback(UART_HandleTypeDef* huart, void (*callback)(void));

#ifdef __cplusplus
}
#endif

#endif // BSP_UART_H_