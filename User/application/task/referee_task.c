#include <sys/cdefs.h>
//
// Created by Rick on 2024/12/6.
//

#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "referee.h"
#include "protocol.h"
#include "fifo.h"

#define USART_RX_BUF_LENGTH     512
#define REFEREE_FIFO_BUF_LENGTH 1024

uint8_t usart1_buf[2][USART_RX_BUF_LENGTH];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

_Noreturn void referee_task(void *argument){
	init_referee_struct_data();
	fifo_s_init(&referee_fifo,  referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
	usart1_init(usart1_buf[0], usart1_buf[1], USART_RX_BUF_LENGTH);

	while (1){
		referee_unpack_fifo_data(&referee_unpack_obj, &referee_fifo);
		vTaskDelay(10);
	}
}