//
// Created by Rick on 2024/12/10.
//

#ifndef DM_02_HERO_USER_APPLICATION_MODULE_IMU_IMU_H_
#define DM_02_HERO_USER_APPLICATION_MODULE_IMU_IMU_H_

#include "main.h"
#include "struct_typedef.h"
#include "fusion.h"
#define INS_TASK_INIT_TIME 7

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2


extern fp32* get_INS_angle_point(void);
extern fp32* get_gyro_data_point(void);



#endif //DM_02_HERO_USER_APPLICATION_MODULE_IMU_IMU_H_
