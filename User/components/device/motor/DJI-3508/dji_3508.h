//
// Created by Rick on 24-11-26.
//

#ifndef DM_02_HERO_USER_COMPONENTS_DEVICE_MOTOR_DJI_3508_DJI_3508_H_
#define DM_02_HERO_USER_COMPONENTS_DEVICE_MOTOR_DJI_3508_DJI_3508_H_

#include "main.h"
#include "struct_typedef.h"

typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperature;
	int16_t last_ecd;
} motor_3508_measure_t;

typedef struct
{
	const motor_3508_measure_t *motor_3508_measure;
	fp32 accel;
	fp32 speed;
	fp32 speed_set;
	int16_t give_current;
} motor_3508_t;

void get_motor_3508_measure(motor_3508_measure_t *ptr, const uint8_t *data);

#endif //DM_02_HERO_USER_COMPONENTS_DEVICE_MOTOR_DJI_3508_DJI_3508_H_
