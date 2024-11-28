//
// Created by Rick on 24-11-26.
//

#ifndef DM_02_HERO_USER_APPLICATION_MODULE_CHASSIS_H_
#define DM_02_HERO_USER_APPLICATION_MODULE_CHASSIS_H_

#include "main.h"
#include "struct_typedef.h"
#include "dji_3508.h"
#include "pid.h"

typedef enum
{
	CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //chassis will follow yaw gimbal motor relative angle.底盘会跟随云台相对角度
	CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //chassis will have yaw angle(chassis_yaw) close-looped control.底盘有底盘角度控制闭环
	CHASSIS_VECTOR_NO_FOLLOW_YAW,       //chassis will have rotation speed control. 底盘有旋转速度控制
	CHASSIS_VECTOR_RAW,                 //control-current will be sent to CAN bus derectly.
} chassis_mode_e;

/* CAN send and receive ID */
typedef enum
{
	CAN_CHASSIS_ALL_ID = 0x200,
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	CAN_3508_M3_ID = 0x203,
	CAN_3508_M4_ID = 0x204,

	CAN_YAW_MOTOR_ID = 0x205,
	CAN_PIT_MOTOR_ID = 0x206,
	CAN_Down_TRIGGER_MOTOR_ID = 0x207,
	CAN_GIMBAL_ALL_ID = 0x1FF,
	SUPER_CAP_ID = 0x329,
} can_msg_id_e;


typedef struct {
	chassis_mode_e chassis_mode;
	chassis_mode_e last_chassis_mode;
	motor_3508_t motor_chassis[4];
	pid_type_def motor_speed_pid;
	pid_type_def chassis_angle_pid;

	// 通信接口定义
	void (*CAN_cmd_chassis)(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
	void (*CAN_rec_chassis)(uint32_t can_id, const int16_t *rx_data);
}chassis_control_t;

extern chassis_control_t* get_chassis_control_point(void);

#endif //DM_02_HERO_USER_APPLICATION_MODULE_CHASSIS_H_
