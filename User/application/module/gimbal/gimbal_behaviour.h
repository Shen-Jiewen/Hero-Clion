//
// Created by Rick on 2024/12/10.
//

#ifndef GIMBAL_BEHAVIOUR_H_
#define GIMBAL_BEHAVIOUR_H_

#include "main.h"
#include "struct_typedef.h"
#include "detect.h"
#include "gimbal.h"

typedef enum
{
	GIMBAL_ZERO_FORCE = 0,
	GIMBAL_INIT,
	GIMBAL_CALI,
	GIMBAL_ABSOLUTE_ANGLE,
	GIMBAL_RELATIVE_ANGLE,
	GIMBAL_MOTIONLESS,
	GIMBAL_AUTO,						//云台自瞄
} gimbal_behaviour_e;

extern void gimbal_behaviour_set(gimbal_control_t *gimbal_mode_set);
extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);
extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set);
gimbal_behaviour_e* get_gimbal_behaviour_point(void);
#endif //GIMBAL_BEHAVIOUR_H_
