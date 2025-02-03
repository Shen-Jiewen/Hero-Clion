//
// Created by Du.X.Y on 2025/1/10.
//

#ifndef MINI_GIMBAL_H
#define MINI_GIMBAL_H


#include "dt7.h"

#define MINI_GIMBAL_TASK_INIT_TIME 200


typedef struct
{
    const RC_ctrl_t* shoot_rc_ctrl;                 //指向发射机构遥控控制输入的常量指针
}mini_gimbal_control_t;

mini_gimbal_control_t* get_mini_gimbal_control_point(void);

#endif //MINI_GIMBAL_H
