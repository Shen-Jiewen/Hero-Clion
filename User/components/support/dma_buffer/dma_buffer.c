//
// Created by Rick on 2024/12/9.
//
// 文件说明：
// 该文件实现了一个 UART DMA 双缓冲区管理系统，用于在 DMA 接收时自动切换缓冲区。
// 在接收到的数据达到半个缓冲区或整个缓冲区时，通过回调函数通知用户。
// 用户可以通过注册回调函数，处理 DMA 缓冲区传输完成的事件。
// 该系统支持最大 10 个 DMA 缓冲区对象的管理，并可以在运行时动态添加和管理多个缓冲区。

// 作者: Javen
// 日期: 2024/12/9

#include "dma_buffer.h"

/* 最大的 DMA 缓冲区对象数量 */
#define DMA_BUFFER_MAX_COUNT 10  // 根据需要调整

/* 静态数组用于存储指向 DMA 缓冲区对象的指针 */
static dma_buffer_t* dma_buffers_local[DMA_BUFFER_MAX_COUNT];

/* 当前 DMA 缓冲区对象的数量 */
static uint32_t dma_buffer_count_local = 0;

/**
 * @brief 初始化一个 UART DMA 双缓冲区
 *
 * 该函数初始化一个 `dma_buffer_t` 结构体，并将其添加到内部缓冲区列表中以进行管理。
 *
 * @param dma_buffer 指向要初始化的 `dma_buffer_t` 结构体的指针
 * @param huart      指向 UART 句柄的指针
 * @param buffer1    指向第一个接收缓冲区的指针
 * @param buffer2    指向第二个接收缓冲区的指针
 * @param buffer_size 每个接收缓冲区的大小（字节）
 *
 * @return 如果初始化成功返回 `HAL_OK`，否则返回 `HAL_ERROR`
 */
HAL_StatusTypeDef dma_buffer_init(dma_buffer_t *dma_buffer, UART_HandleTypeDef *huart,
	uint8_t *buffer1, uint8_t *buffer2, uint32_t buffer_size)
{
	if (dma_buffer == NULL || huart == NULL || buffer1 == NULL || buffer2 == NULL || buffer_size == 0) {
		return HAL_ERROR;
	}

	/* 初始化 dma_buffer 结构体 */
	dma_buffer->huart = huart;
	dma_buffer->buffer1 = buffer1;
	dma_buffer->buffer2 = buffer2;
	dma_buffer->buffer_size = buffer_size;
	dma_buffer->state = DMA_BUFFER_IDLE_e;
	dma_buffer->half_complete_callback = NULL;
	dma_buffer->full_complete_callback = NULL;
	dma_buffer->current_buffer_index = 0; // 初始使用 buffer1

	/* 将 dma_buffer 添加到全局缓冲区列表中 */
	if (dma_buffer_count_local < DMA_BUFFER_MAX_COUNT) {
		dma_buffers_local[dma_buffer_count_local++] = dma_buffer;
	} else {
		/* 超出最大缓冲区对象数量 */
		return HAL_ERROR;
	}

	return HAL_OK;
}

/**
 * @brief 启动指定 DMA 缓冲区的 DMA 接收
 *
 * 该函数启动指定 `dma_buffer_t` 对象的 DMA 接收过程。
 *
 * @param dma_buffer 指向要启动接收的 `dma_buffer_t` 结构体的指针
 *
 * @return 如果 DMA 接收成功启动，返回 `HAL_OK`；
 *         如果缓冲区当前忙，返回 `HAL_BUSY`；
 *         如果发生其他错误，返回 `HAL_ERROR`
 */
HAL_StatusTypeDef dma_buffer_start_receive(dma_buffer_t *dma_buffer)
{
	if (dma_buffer == NULL || dma_buffer->huart == NULL) {
		return HAL_ERROR;
	}

	if (dma_buffer->state != DMA_BUFFER_IDLE_e) {
		return HAL_BUSY;
	}

	/* 将状态设置为 BUSY */
	dma_buffer->state = DMA_BUFFER_BUSY_e;

	/* 启动 UART 接收，使用 DMA 方式接收到 buffer1 */
	return HAL_UARTEx_ReceiveToIdle_DMA(dma_buffer->huart, dma_buffer->buffer1, dma_buffer->buffer_size);
}

/**
 * @brief 注册半传输完成的回调函数
 *
 * 该函数用于注册在 DMA 接收达到一半时调用的回调函数。
 *
 * @param dma_buffer   指向 `dma_buffer_t` 结构体的指针
 * @param half_callback 函数指针，指向用户定义的半传输完成回调函数
 */
void dma_buffer_register_half_callback(dma_buffer_t *dma_buffer, void (*half_callback)(UART_HandleTypeDef *huart))
{
	if (dma_buffer == NULL) {
		return;
	}

	dma_buffer->half_complete_callback = half_callback;
}

/**
 * @brief 注册完全传输完成的回调函数
 *
 * 该函数用于注册在 DMA 接收完成时调用的回调函数。
 *
 * @param dma_buffer   指向 `dma_buffer_t` 结构体的指针
 * @param full_callback 函数指针，指向用户定义的完全传输完成回调函数
 */
void dma_buffer_register_full_callback(dma_buffer_t *dma_buffer, void (*full_callback)(UART_HandleTypeDef *huart))
{
	if (dma_buffer == NULL) {
		return;
	}

	dma_buffer->full_complete_callback = full_callback;
}

/**
 * @brief 根据 UART 句柄获取对应的 DMA 缓冲区对象
 *
 * 该函数在内部缓冲区列表中查找与给定 UART 句柄关联的 `dma_buffer_t` 对象。
 *
 * @param huart 指向 UART 句柄的指针
 *
 * @return 如果找到对应的 `dma_buffer_t` 对象，返回其指针；否则返回 NULL
 */
dma_buffer_t* dma_buffer_get_by_uart(UART_HandleTypeDef *huart)
{
	for (uint32_t i = 0; i < dma_buffer_count_local; i++) {
		if (dma_buffers_local[i]->huart == huart) {
			return dma_buffers_local[i];
		}
	}
	return NULL;
}

/**
 * @brief 切换当前缓冲区并重新启动 DMA 接收
 *
 * 该函数切换当前正在使用的缓冲区索引，并在新缓冲区上重新启动 DMA 接收。
 *
 * @param dma_buffer 指向 `dma_buffer_t` 结构体的指针
 *
 * @return 如果缓冲区切换和 DMA 重新启动成功，返回 `HAL_OK`；
 *         否则返回 `HAL_ERROR`
 */
HAL_StatusTypeDef dma_buffer_switch_buffer(dma_buffer_t *dma_buffer)
{
	if (dma_buffer == NULL || dma_buffer->huart == NULL) {
		return HAL_ERROR;
	}

	/* 切换缓冲区索引 */
	dma_buffer->current_buffer_index = 1 - dma_buffer->current_buffer_index;

	/* 根据当前缓冲区索引选择新的缓冲区 */
	uint8_t *new_buffer = (dma_buffer->current_buffer_index == 0) ?
						  dma_buffer->buffer1 :
						  dma_buffer->buffer2;

	/* 在新的缓冲区上重新启动 DMA 接收 */
	HAL_StatusTypeDef status = HAL_UART_Receive_DMA(dma_buffer->huart, new_buffer, dma_buffer->buffer_size);
	if (status == HAL_OK) {
		return HAL_OK;
	} else {
		/* 如果 DMA 重新启动失败，将缓冲区状态设置为 IDLE */
		dma_buffer->state = DMA_BUFFER_IDLE_e;
		return HAL_ERROR;
	}
}
