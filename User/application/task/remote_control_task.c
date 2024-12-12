//
// Created by Rick on 2024/12/10.
//

#include "cmsis_os.h"
#include "bsp_rc.h"
#include "dt7.h"
#include "detect.h"

extern long long uart5_count;

#define REMOTE_CONTROL_INIT_TIME 20

DT7_ctrl_t *dt7_ctrl;

// 接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];
extern UART_HandleTypeDef huart5;

_Noreturn void remote_control_task(void* argument)
{
	// 获取遥控器对象
	dt7_ctrl = get_dt7_point();
	// 初始化外设
	dt7_init(sbus_rx_buf, SBUS_RX_BUF_NUM);

	while (1) {
		vTaskSuspend(NULL);
	}
}

void rc_callback(UART_HandleTypeDef *huart, uint16_t Size)
{
	sbus_to_dt7(sbus_rx_buf);
	detect_hook(DBUS_TOE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, SBUS_RX_BUF_NUM * 2);
}
