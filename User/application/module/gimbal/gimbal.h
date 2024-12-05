//
// Created by Rick on 2024/12/5.
//

#ifndef DM_02_HERO_USER_APPLICATION_MODULE_GIMBAL_GIMBAL_H_
#define DM_02_HERO_USER_APPLICATION_MODULE_GIMBAL_GIMBAL_H_

#include "main.h"
#include "struct_typedef.h"
#include "dji_3508.h"
#include "user_lib.h"
#include "pid.h"

typedef enum
{
	GIMBAL_MOTOR_RAW = 0, //电机原始值控制
	GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
	GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
	fp32 kp;
	fp32 ki;
	fp32 kd;

	fp32 set;
	fp32 get;
	fp32 err;

	fp32 max_out;
	fp32 max_iout;

	fp32 Pout;
	fp32 Iout;
	fp32 Dout;

	fp32 out;
} gimbal_PID_t;

typedef struct
{
	const motor_3508_measure_t *gimbal_motor_measure;
	gimbal_PID_t gimbal_motor_absolute_angle_pid;
	gimbal_PID_t gimbal_motor_relative_angle_pid;
	pid_type_def gimbal_motor_gyro_pid;
	gimbal_motor_mode_e gimbal_motor_mode;
	gimbal_motor_mode_e last_gimbal_motor_mode;
	uint16_t offset_ecd;
	fp32 max_relative_angle; //rad
	fp32 min_relative_angle; //rad

	fp32 relative_angle;     //rad
	fp32 relative_angle_set; //rad
	fp32 absolute_angle;     //rad
	fp32 absolute_angle_set; //rad
	fp32 motor_gyro;         //rad/s
	fp32 motor_gyro_set;
	fp32 motor_speed;
	fp32 raw_cmd_current;
	fp32 current_set;
	int16_t given_current;
} gimbal_motor_t;

#endif //DM_02_HERO_USER_APPLICATION_MODULE_GIMBAL_GIMBAL_H_
