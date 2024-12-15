//
// Created by Rick on 24-11-25.
//

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "vofa.h"
#include "Fusion.h"
#include "dt7.h"

long long uart5_count = 0;
extern FusionEuler euler;

_Noreturn void usb_task(void* argument){
	DT7_ctrl_t *usb_dt7_ctrl = get_dt7_point();

	// 初始化
	while (1){
		vofa_send_data(0, euler.angle.pitch);
		vofa_send_data(1, euler.angle.yaw);
		vofa_send_data(2, euler.angle.roll);
		vofa_send_data(3, usb_dt7_ctrl->rc.ch[0]);
		vofa_send_data(4, usb_dt7_ctrl->rc.ch[1]);
		vofa_send_data(5, usb_dt7_ctrl->rc.ch[2]);
		vofa_send_data(6, usb_dt7_ctrl->rc.ch[3]);
		vofa_send_data(7, usb_dt7_ctrl->rc.ch[4]);
		vofa_send_data(8, usb_dt7_ctrl->rc.s[0]);
		vofa_send_data(9, usb_dt7_ctrl->rc.s[1]);
		vofa_sendframetail();
		osDelay(10);
	}
}