//
// Created by Rick on 24-11-25.
//

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "vofa.h"
#include "Fusion.h"

extern FusionEuler euler;

_Noreturn void usb_task(void* argument){

	// 初始化
	while (1){
		vofa_send_data(0, euler.angle.pitch);
		vofa_send_data(1, euler.angle.yaw);
		vofa_send_data(2, euler.angle.roll);
		vofa_sendframetail();
		vTaskDelay(10);
	}
}