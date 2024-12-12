//
// Created by Rick on 2024/12/10.
//

#include "gimbal_behaviour.h"

static void gimbal_behaviour_set(gimbal_control_t *gimbal_mode_set);

//云台行为状态机
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;
static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;  //记录上次的行为模式

/**
  * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
  * @param[out]     gimbal_mode_set: 云台数据指针
  * @retval         none
  */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
	if (gimbal_mode_set == NULL)
	{
		return;
	}
	//云台行为状态机设置
	gimbal_behaviour_set(gimbal_mode_set);

	//根据云台行为状态机设置电机状态机
	if (gimbal_behaviour == GIMBAL_ZERO_FORCE || gimbal_behaviour == GIMBAL_CALI)
	{
		gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
		gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
	}
	else if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_RELATIVE_ANGLE || gimbal_behaviour == GIMBAL_MOTIONLESS)
	{
		gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
		gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
	}
	else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE || gimbal_behaviour == GIMBAL_AUTO)
	{
		gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
		gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
	}
}

/**
  * @brief          云台行为状态机设置.
  * @param[in]      gimbal_mode_set: 云台数据指针
  * @retval         none
  */
static void gimbal_behaviour_set(gimbal_control_t *gimbal_mode_set)
{
	//校准行为，return 不会设置其他的模式
	if (gimbal_behaviour == GIMBAL_CALI && gimbal_mode_set->gimbal_cali.step != GIMBAL_CALI_END_STEP)
	{
		return;
	}
	//如果外部使得校准步骤从0 变成 start，则进入校准模式
	if (gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_START_STEP && !toe_is_error(DBUS_TOE))
	{
		gimbal_behaviour = GIMBAL_CALI;
		return;
	}

	//初始化模式判断是否到达中值位置
	if (gimbal_behaviour == GIMBAL_INIT)
	{
		static uint16_t init_time = 0;
		static uint16_t init_stop_time = 0;
		init_time++;

		if ((fabsf(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
			fabsf(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
		{

			if (init_stop_time < GIMBAL_INIT_STOP_TIME)
			{
				init_stop_time++;
			}
		}
		else
		{

			if (init_time < GIMBAL_INIT_TIME)
			{
				init_time++;
			}
		}

		//超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
		if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
			!switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) && !toe_is_error(DBUS_TOE))
		{
			return;
		}
		else
		{
			init_stop_time = 0;
			init_time = 0;
		}
	}

	//开关控制 云台状态
	//遥控器右拨杆拨到下边,云台为无力模式
	if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
	{
		gimbal_behaviour = GIMBAL_ZERO_FORCE;
	}
		//遥控器右拨杆拨到中间,云台为跟随模式(即底盘跟随云台转动)
	else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
	{
		//跟随模式下按下鼠标右键可以开启自瞄
		if( gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r && last_gimbal_behaviour != GIMBAL_AUTO) //鼠标右键开启自瞄
		{
			gimbal_behaviour = GIMBAL_AUTO;
		}
		else if (gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r ==0 && last_gimbal_behaviour == GIMBAL_AUTO)
		{
			gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
		}
	}

	if( toe_is_error(DBUS_TOE))
	{
		gimbal_behaviour = GIMBAL_ZERO_FORCE;
	}
	last_gimbal_behaviour = gimbal_behaviour;

	//判断进入init状态机
	{
		last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
		if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
		{
			gimbal_behaviour = GIMBAL_INIT;
		}
		last_gimbal_behaviour = gimbal_behaviour;
	}
}