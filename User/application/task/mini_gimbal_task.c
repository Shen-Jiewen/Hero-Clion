//
// Created by Du.X.Y on 2025/2/3.
//
#include "mini_gimbal.h"
#include "cmsis_os.h"

mini_gimbal_control_t* mini_gimbal_control;

_Noreturn void mini_gimbal_task(__attribute__((unused)) void* argument)
{
    //上电后初始化小云台
    osDelay(MINI_GIMBAL_TASK_INIT_TIME);
    //获取小云台机构对象
    mini_gimbal_control = get_mini_gimbal_control_point();

    while (1)
    {

    }
}