//
// Created by Rick on 2024/12/3.
//

#ifndef DM_02_HERO_USER_BSP_DM02_BSP_CALLBACK_H_
#define DM_02_HERO_USER_BSP_DM02_BSP_CALLBACK_H_

#include "main.h"
#include "struct_typedef.h"
// 底盘
#include "chassis.h"
#include "bsp_can.h"
// 遥控器
#include "dt7.h"
#include "bsp_rc.h"
// 裁判系统
#include "fifo.h"

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

#endif //DM_02_HERO_USER_BSP_DM02_BSP_CALLBACK_H_
