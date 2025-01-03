//
// Created by Rick on 2024/12/20.
//

#include "shoot.h"

static shoot_control_t shoot_control;

shoot_control_t* get_shoot_control_point(void)
{
    return &shoot_control;
}

void shoot_init(shoot_control_t* shoot_init)
{
    //定义各个PID参数
    static const fp32 Down_trigger_angle_pid[3] =
        { Down_TRIGGER_ANGLE_PID_KP, Down_TRIGGER_ANGLE_PID_KI, Down_TRIGGER_ANGLE_PID_KD};
    static const fp32 Outside_shoot_speed_pid[3] =
        {OUTSIDE_SHOOT_SPEED_PID_KP,OUTSIDE_SHOOT_SPEED_PID_KI,OUTSIDE_SHOOT_SPEED_PID_KD};
    static const fp32 Inside_shoot_speed_pid[3] =
        {INSIDE_SHOOT_SPEED_PID_KP,INSIDE_SHOOT_SPEED_PID_KI,INSIDE_SHOOT_SPEED_PID_KD};

    //获取遥控器指针
    shoot_init ->shoot_rc_ctrl = get_dt7_point();

    //初始化拨盘电机类型和测量数据
    shoot_init->trigger_motor.motor_type = MOTOR_TYPE_4310;
    shoot_init->trigger_motor.motor_measure.motor_4310 = get_dm_4310_measure_point(1);
}