/*******************************************************************************
 * @file: bsp_uart.h
 * @author: Javen
 * @date: 2024年12月17日
 * @brief: DM02开发板串口驱动
 * @note: 串口驱动，支持阻塞、中断、DMA发送模式
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#ifndef DM_02_HERO_USER_BSP_DM02_BSP_USART_H_
#define DM_02_HERO_USER_BSP_DM02_BSP_USART_H_

#include "main.h"
#include "string.h"

#define DEVICE_USART_CNT 8     // MC02串口数量8个，其中
#define USART_RXBUFF_LIMIT 512 // 如果协议需要更大的buff,请修改这里

// 模块回调函数,用于解析协议
typedef void (*usart_module_callback)();

/* 发送模式枚举 */
typedef enum
{
	USART_TRANSFER_NONE = 0,  // 不使用任何发送模式
	USART_TRANSFER_BLOCKING,  // 阻塞模式，发送完成后才返回
	USART_TRANSFER_IT,        // 中断模式，使用中断通知发送完成
	USART_TRANSFER_DMA,       // DMA模式，使用DMA进行高速数据传输
} usart_transfer_mode;

// 串口实例结构体,每个module都要包含一个实例。
// 由于串口是独占的点对点通信,所以不需要考虑多个module同时使用一个串口的情况。
// 每个串口实例包含了接收缓冲区、缓冲区大小、串口句柄以及解析数据的回调函数。
typedef struct
{
	uint8_t recv_buff[USART_RXBUFF_LIMIT];  // 预先定义的最大缓冲区大小，接收的数据将存储在此缓冲区中
	uint8_t recv_buff_size;                 // 当前接收到的一包数据的大小
	UART_HandleTypeDef *usart_handle;       // 对应的USART硬件句柄，指向硬件资源
	usart_module_callback module_callback;  // 解析收到的数据的回调函数
} usart_instance_t;

/* usart 初始化配置结构体 */
// 用户初始化串口时，需要传入该结构体，定义串口的配置选项，如接收缓冲区大小、回调函数等。
typedef struct
{
	uint16_t recv_buff_size;               // 模块接收一包数据的大小
	UART_HandleTypeDef *usart_handle;      // 对应的USART硬件句柄
	usart_module_callback module_callback; // 解析收到的数据的回调函数
} usart_init_config_s;

/**
 * @brief 注册一个串口实例，并返回一个串口实例指针
 *
 * @param init_config 传入串口初始化结构体，包含串口的配置项
 *
 * @return usart_instance_t* 返回创建的串口实例指针，如果初始化失败则返回NULL
 */
usart_instance_t *usart_register(usart_init_config_s *init_config);

/**
 * @brief 启动串口服务，传入一个USART实例用于启动串口的工作。
 * 该函数通常用于处理回调丢失的情况，启动后台服务。
 *
 * @param _instance 串口实例，启动相应串口的服务
 */
void usart_service_init(usart_instance_t *_instance);

/**
 * @brief 发送一帧数据，传入串口实例、发送缓冲区及数据长度进行发送
 *
 * @note 在短时间内连续调用此接口，若采用IT/DMA模式，可能会导致上次的发送未完成，新的发送任务被取消。
 *       若希望连续使用DMA/IT进行发送，请配合`usart_is_ready()`函数使用，或自行为模块实现一个发送队列。
 *
 * @param _instance 串口实例，指定要发送数据的串口
 * @param send_buf 待发送数据的缓冲区
 * @param send_size 发送数据的字节数
 * @param mode 发送模式，可以选择阻塞模式、IT模式或DMA模式
 */
void usart_send(usart_instance_t *_instance, uint8_t *send_buf, uint16_t send_size, usart_transfer_mode mode);

#endif // DM_02_HERO_USER_BSP_DM02_BSP_USART_H_
