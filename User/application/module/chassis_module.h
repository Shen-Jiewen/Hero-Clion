//
// Created by Rick on 24-11-26.
//

#ifndef DM_02_HERO_USER_APPLICATION_MODULE_CHASSIS_MODULE_H_
#define DM_02_HERO_USER_APPLICATION_MODULE_CHASSIS_MODULE_H_

#include "main.h"



typedef struct {

	// 通信接口定义
	void (*CAN_cmd_chassis)(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
}chassis_control_t;

extern chassis_control_t* get_chassis_control_point(void);

#endif //DM_02_HERO_USER_APPLICATION_MODULE_CHASSIS_MODULE_H_
