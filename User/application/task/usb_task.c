//
// Created by Rick on 24-11-25.
//

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "vofa.h"
#include "Fusion.h"
#include "imu.h"

extern FusionEuler euler;

_Noreturn void usb_task(void* argument){
	// 初始化
	imu_t* imu_handle =  get_imu_point();
	while (1){
		vofa_send_data(0, imu_handle->gyro[0]);
		vofa_send_data(1, imu_handle->gyro[1]);
		vofa_send_data(2, imu_handle->gyro[2]);
		vofa_send_data(3, imu_handle->accel[0]);
		vofa_send_data(4, imu_handle->accel[1]);
		vofa_send_data(5, imu_handle->accel[2]);
		vofa_send_data(6, euler.angle.pitch);
		vofa_send_data(7, euler.angle.yaw);
		vofa_send_data(8, euler.angle.roll);
		vofa_sendframetail();
		vTaskDelay(10);
	}
}