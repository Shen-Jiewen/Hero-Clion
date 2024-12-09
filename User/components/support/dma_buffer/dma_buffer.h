//
// Created by Rick on 2024/12/9.
//

#ifndef DM_02_HERO_USER_COMPONENTS_SUPPORT_DMA_BUFFER_DMA_BUFFER_H_
#define DM_02_HERO_USER_COMPONENTS_SUPPORT_DMA_BUFFER_DMA_BUFFER_H_

#include "stm32h7xx_hal.h"

// 双缓冲状态枚举
typedef enum {
	DMA_BUFFER_IDLE_e = 0,
	DMA_BUFFER_BUSY_e
} dma_buffer_state_e;

// 双缓冲对象结构体
typedef struct {
	UART_HandleTypeDef *huart;                                   // 串口句柄
	uint8_t *buffer1;                                            // 缓冲区1
	uint8_t *buffer2;                                            // 缓冲区2
	uint32_t buffer_size;                                        // 缓冲区大小
	dma_buffer_state_e state;                                    // 当前状态
	void (*half_complete_callback)(UART_HandleTypeDef *huart);   // 半传输回调
	void (*full_complete_callback)(UART_HandleTypeDef *huart);   // 完全传输回调
	uint8_t current_buffer_index;                                // 当前缓冲区索引（0 或 1）
} dma_buffer_t;

// 初始化双缓冲
HAL_StatusTypeDef dma_buffer_init(dma_buffer_t *dma_buffer, UART_HandleTypeDef *huart,
	uint8_t *buffer1, uint8_t *buffer2, uint32_t buffer_size);

// 启动双缓冲接收
HAL_StatusTypeDef dma_buffer_start_receive(dma_buffer_t *dma_buffer);

// 注册完全传输回调函数
void dma_buffer_register_full_callback(dma_buffer_t *dma_buffer,
	void (*full_callback)(UART_HandleTypeDef *huart));

// 获取对应 UART 的双缓冲对象
dma_buffer_t* dma_buffer_get_by_uart(UART_HandleTypeDef *huart);

// 切换缓冲区并重新启动 DMA 接收
HAL_StatusTypeDef dma_buffer_switch_buffer(dma_buffer_t *dma_buffer);

#endif //DM_02_HERO_USER_COMPONENTS_SUPPORT_DMA_BUFFER_DMA_BUFFER_H_
