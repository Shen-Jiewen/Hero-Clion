//
// Created by Rick on 24-11-25.
//

#include "imu.h"

imu_t imu;

imu_t* get_imu_point(void){
	return &imu;
}

imu_t imu  = {
	.angle = 0,
	.gyro = 0,
	.accel = 0,
	.temp = 0,
};