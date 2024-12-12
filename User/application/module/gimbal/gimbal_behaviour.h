//
// Created by Rick on 2024/12/10.
//

#ifndef DM_02_HERO_USER_APPLICATION_MODULE_GIMBAL_GIMBAL_BEHAVIOUR_H_
#define DM_02_HERO_USER_APPLICATION_MODULE_GIMBAL_GIMBAL_BEHAVIOUR_H_

#include "main.h"
#include "struct_typedef.h"
#include "gimbal.h"
#include "detect.h"

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

#endif //DM_02_HERO_USER_APPLICATION_MODULE_GIMBAL_GIMBAL_BEHAVIOUR_H_
