//
// Created by Rick on 2024/12/20.
//

#include "shoot.h"

#include "referee.h"

static shoot_control_t shoot_control;

/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

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

    //初始化摩擦轮电机的测量数据
    for (uint8_t i = 0; i < 4; i++) {
        shoot_init->friction_motor[i].motor_3508_measure = get_motor_3508_measure_point(1,i);
    }

    //初始状态机为发射停止
    shoot_init->shoot_mode = SHOOT_STOP;

    //初始化拨弹电机PID
    PID_init(&shoot_init->trigger_motor.trigger_angle_pid,
        PID_POSITION,
        Down_trigger_angle_pid,
        Down_TRIGGER_ANGLE_PID_MAX_OUT,
        Down_TRIGGER_ANGLE_PID_MAX_IOUT);

    //初始化外侧摩擦轮电机PID
    for (uint8_t i = 0; i < 2; i++) {
        PID_init(&shoot_init->friction_speed_pid[i],
    PID_POSITION,
    Outside_shoot_speed_pid,
    OUTSIDE_SHOOT_SPEED_PID_MAX_OUT,
    OUTSIDE_SHOOT_SPEED_PID_MAX_IOUT);
    }

    //初始化外侧摩擦轮电机PID
    for (uint8_t i = 3; i < 5; i++) {
        PID_init(&shoot_init->friction_speed_pid[i],
    PID_POSITION,
    Outside_shoot_speed_pid,
    OUTSIDE_SHOOT_SPEED_PID_MAX_OUT,
    OUTSIDE_SHOOT_SPEED_PID_MAX_IOUT);
    }

    //数据更新
    shoot_feedback_update();
    //初始化发射标志位
    shoot_init->shoot_flag = 0;

    //初始化四个摩擦轮电机的速度和目标速度
    for (uint8_t i = 0; i < 4; i++) {
        shoot_init->friction_motor[i].speed = 0.0f;
        shoot_init->friction_motor[i].speed_set = 0.0f;
    }

    //初始化拨盘电机
    shoot_init->trigger_motor.give_current = 0;
    shoot_init->trigger_motor.angle_set = shoot_init->trigger_motor.motor_measure.motor_4310->position;
    shoot_init->trigger_motor.speed_set = 0;

    //获取当前弹速限制以及热量限制
    get_shoot_speed_limit(&shoot_init->last_shoot_speed_limit);
    get_shoot_speed_limit(&shoot_init->shoot_speed_limit);
    get_shoot_heat0_limit_and_heat0(&shoot_init->heat_limit,&shoot_init->heat);

    // 拨弹电机使能
    DM4310_MotorEnable(1);
}

/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void) {

    //更新拨弹电机数据
    shoot_control.trigger_motor.motor_measure.motor_4310 = get_dm_4310_measure_point(1);

    //更新摩擦轮电机速度
    for (uint8_t i = 0; i < 4; i++) {
        shoot_control.friction_motor[i].speed = shoot_control.friction_motor[i].motor_3508_measure->speed_rpm * M3508_MOTOR_RPM_TO_VECTOR;
    }

    //记录上次的键盘状态
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;

    //判断鼠标状态
    shoot_control.press_l = shoot_control.shoot_rc_ctrl->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc_ctrl->mouse.press_r;
}
