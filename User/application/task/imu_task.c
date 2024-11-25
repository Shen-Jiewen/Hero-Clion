//
// Created by Rick on 24-11-25.
//

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "bsp_imu.h"
#include "imu_module.h"

_Noreturn void imu_task(void *argument)
{
	osDelay(500);
	imu_pwm_start();
	// 获取IMU指针对象
	imu_handle_t* imu_handle =  get_imu_point();
	while (BMI088_init()){
		;
	}

	while (1){
		imu_semaphore_wait();
		// 获取传感器数据
		BMI088_read(imu_handle->gyro, imu_handle->accel, &(imu_handle->temp));
	}
}