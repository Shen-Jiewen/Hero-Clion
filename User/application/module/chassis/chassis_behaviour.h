//
// Created by Rick on 2024/12/4.
//

#ifndef DM_02_HERO_USER_APPLICATION_MODULE_CHASSIS_CHASSIS_BEHAVIOUR_H_
#define DM_02_HERO_USER_APPLICATION_MODULE_CHASSIS_CHASSIS_BEHAVIOUR_H_

#include "main.h"
#include "struct_typedef.h"
#include "arm_math.h"
#include "chassis.h"
#include "dt7.h"
#include "user_lib.h"

#define CHASSIS_OPEN_RC_SCALE 10		  // 在  CHASSIS_OPEN 模式下,遥控器乘以该比例发送到can上

typedef enum {
	CHASSIS_ZERO_FORCE,                   // 底盘无力，类似没上电
	CHASSIS_NO_MOVE,                      // 底盘停止，保持不动
	CHASSIS_FOLLOW_GIMBAL_YAW,			  // 底盘跟随云台偏航，适用于步兵模式
	CHASSIS_NO_FOLLOW_YAW,                // 底盘不跟随偏航角，角度开环控制，轮速闭环控制
	CHASSIS_GYRO_MODE,					  // 底盘小陀螺模式，底盘旋转，平移跟随云台
	CHASSIS_OPEN,                         // 遥控器输入乘比例因子后，直接发送电流值到CAN总线
} chassis_behaviour_e;

extern void chassis_behaviour_mode_set(chassis_control_t *chassis_move_mode);

extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_control_t *chassis_move_rc_to_vector);

#endif //DM_02_HERO_USER_APPLICATION_MODULE_CHASSIS_CHASSIS_BEHAVIOUR_H_
