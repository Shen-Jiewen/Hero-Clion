//
// Created by Rick on 24-11-26.
//

#ifndef DM_02_HERO_USER_COMPONENTS_DEVICE_MOTOR_DJI_3508_DJI_3508_H_
#define DM_02_HERO_USER_COMPONENTS_DEVICE_MOTOR_DJI_3508_DJI_3508_H_

#include "main.h"
#include "struct_typedef.h"
#define DETECT
#ifdef DETECT
#include "detect.h"
#endif

typedef enum
{
	CAN_CHASSIS_ALL_ID = 0x200,
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	CAN_3508_M3_ID = 0x203,
	CAN_3508_M4_ID = 0x204,
	CAN_3508_M5_ID = 0x205,
	CAN_3508_M6_ID = 0x206,
	CAN_3508_M7_ID = 0x207,
	CAN_3508_M8_ID = 0x208,
} motor_3508_id_e;

typedef struct __attribute__((packed))
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperature;
	uint16_t last_ecd;
} motor_3508_measure_t;

typedef struct
{
	const motor_3508_measure_t* motor_3508_measure;
	fp32 accel;
	fp32 speed;
	fp32 speed_set;
	int16_t give_current;
} motor_3508_t;

void get_motor_3508_measure(motor_3508_measure_t* ptr, const uint8_t* data);

const motor_3508_measure_t* get_motor_3508_measure_point(uint8_t i);
void motor_3505_can_callback(uint32_t can_id, const uint8_t* rx_data);

#endif //DM_02_HERO_USER_COMPONENTS_DEVICE_MOTOR_DJI_3508_DJI_3508_H_
