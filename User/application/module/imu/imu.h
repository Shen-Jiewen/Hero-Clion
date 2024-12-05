//
// Created by Rick on 24-11-25.
//

#ifndef DM_02_HERO_USER_APPLICATION_MODULE_IMU_MODULE_H_
#define DM_02_HERO_USER_APPLICATION_MODULE_IMU_MODULE_H_

#include "main.h"
#include "struct_typedef.h"

typedef struct {
	fp32 angle[3];
	fp32 gyro[3];
	fp32 accel[3];
	fp32 temp;
}imu_t;

imu_t* get_imu_point(void);

#endif //DM_02_HERO_USER_APPLICATION_MODULE_IMU_MODULE_H_
