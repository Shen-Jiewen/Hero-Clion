//
// Created by Du.X.Y on 2025/1/10.
//

#include "mini_gimbal.h"

static mini_gimbal_control_t mini_gimbal_control;

/**
 * @brief  返回小云台控制结构体指针
 * @return  &mini_gimbal_control
 */
mini_gimbal_control_t* get_mini_gimbal_control_point(void)
{
    return &mini_gimbal_control;
}