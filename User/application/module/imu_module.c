//
// Created by Rick on 24-11-25.
//

#include "imu_module.h"

imu_handle_t imu_handle;

imu_handle_t* get_imu_point(void){
	return &imu_handle;
}

imu_handle_t imu_handle  = {
	.angle = 0,
	.gyro = 0,
	.accel = 0,
	.temp = 0,
};