//
// Created by Rick on 2024/12/10.
//

#include "gimbal_behaviour.h"

/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
  * @brief          通过判断角速度来判断云台是否到达极限位置
  * @param          对应轴的角速度，单位rad/s
  * @param          计时时间，到达GIMBAL_CALI_STEP_TIME的时间后归零
  * @param          记录的角度 rad
  * @param          反馈的角度 rad
  * @param          记录的编码值 raw
  * @param          反馈的编码值 raw
  * @param          校准的步骤 完成一次 加一
  */
#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

static bool handle_calibration_mode(gimbal_control_t *gimbal_mode_set);
static void handle_initialization_mode(gimbal_control_t *gimbal_mode_set);
static void handle_gimbal_switch_control(gimbal_control_t *gimbal_mode_set);

static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, __attribute__((unused)) gimbal_control_t *gimbal_control_set);
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, __attribute__((unused)) gimbal_control_t *gimbal_control_set);

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
  * @brief          云台行为状态机设置，根据遥控器开关和校准状态更新云台的行为状态.
  * @param[in]      gimbal_mode_set: 云台数据指针，包含云台各类状态数据
  * @retval         none
  */
void gimbal_behaviour_set(gimbal_control_t *gimbal_mode_set)
{
	// 校准模式处理，如果满足条件，返回，不继续处理其他模式
	if (handle_calibration_mode(gimbal_mode_set))
	{
		return;
	}

	// 云台初始化模式处理
	if (gimbal_behaviour == GIMBAL_INIT)
	{
		handle_initialization_mode(gimbal_mode_set);
	}

	// 云台开关控制状态
	handle_gimbal_switch_control(gimbal_mode_set);

	// 错误处理，若遇到系统错误，切换到无力模式
	if (toe_is_error(DBUS_TOE))
	{
		gimbal_behaviour = GIMBAL_ZERO_FORCE;
	}

	// 如果当前状态为无力模式，且下一状态不是无力模式，则进入初始化状态
	if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
	{
		gimbal_behaviour = GIMBAL_INIT;
	}

	// 更新状态信息
	last_gimbal_behaviour = gimbal_behaviour;
}

/**
  * @brief          处理云台的校准模式，若在校准模式下，依据状态更新行为
  * @param[in]      gimbal_mode_set: 云台数据指针，包含云台的校准状态
  * @retval         true: 如果已经进入校准模式或不再执行其他模式，false: 否则
  */
static bool handle_calibration_mode(gimbal_control_t *gimbal_mode_set)
{
	// 校准行为时，若校准未结束，直接返回，不设置其他模式
	if (gimbal_behaviour == GIMBAL_CALI && gimbal_mode_set->gimbal_cali.step != GIMBAL_CALI_END_STEP)
	{
		return true;
	}

	// 外部指令将校准步骤从0变为开始，进入校准模式
	if (gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_START_STEP && !toe_is_error(DBUS_TOE))
	{
		gimbal_behaviour = GIMBAL_CALI;
		return true;
	}

	return false;
}

/**
  * @brief          处理云台初始化模式，更新初始化时间并判断是否结束初始化
  * @param[in]      gimbal_mode_set: 云台数据指针，包含云台的角度信息
  * @retval         none
  */
static void handle_initialization_mode(gimbal_control_t *gimbal_mode_set)
{
	static uint16_t init_time = 0;       // 初始化持续时间
	static uint16_t init_stop_time = 0;  // 初始化停止时间

	init_time++;  // 增加初始化时间计数

	// 检查云台是否到达目标位置，判断是否停止
	if (fabsf(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
		fabsf(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR)
	{
		if (init_stop_time < GIMBAL_INIT_STOP_TIME)
		{
			init_stop_time++;  // 云台已停稳，增加停止时间
		}
	}
	else
	{
		if (init_time < GIMBAL_INIT_TIME)
		{
			init_time++;  // 如果未达到目标角度，增加初始化时间
		}
	}

	// 如果初始化时间超时或停止时间达到上限，退出初始化状态
	if (init_time >= GIMBAL_INIT_TIME || init_stop_time >= GIMBAL_INIT_STOP_TIME)
	{
		init_stop_time = 0;
		init_time = 0;
	}

	// 若遥控器没有切换到关闭模式，且无错误，则继续初始化
	if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
		!switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) && !toe_is_error(DBUS_TOE))
	{
		return;
	}
}

/**
  * @brief          处理遥控器开关控制云台的行为模式（无力模式、跟随模式、自瞄模式等）
  * @param[in]      gimbal_mode_set: 云台数据指针，包含遥控器控制信号
  * @retval         none
  */
static void handle_gimbal_switch_control(gimbal_control_t *gimbal_mode_set)
{
	// 遥控器右拨杆拨到下边，云台进入无力模式
	if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
	{
		gimbal_behaviour = GIMBAL_ZERO_FORCE;
	}

	// 遥控器右拨杆拨到中间，云台进入跟随模式
	else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
	{
		// 默认是绝对角度控制
		gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
		// 在跟随模式下按下鼠标右键时，切换到自瞄模式
		if (gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r)
		{
			gimbal_behaviour = GIMBAL_AUTO;
		}
	}
}

/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
  * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
  * @param[in]      gimbal_mode_set:云台数据指针
  * @retval         none
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{

	if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
	{
		return;
	}

	if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
	{
		gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
	}
	else if (gimbal_behaviour == GIMBAL_INIT)
	{
		gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
	}
	else if (gimbal_behaviour == GIMBAL_CALI)
	{
		gimbal_cali_control(add_yaw, add_pitch, gimbal_control_set);
	}
	else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
	{
#ifdef feedforward_test
		gimbal_feedforward_control(add_yaw, add_pitch, gimbal_control_set);
#else
		gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
#endif
	}
	else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
	{
		gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
	}
	else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
	{
		gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
	}
	else if (gimbal_behaviour == GIMBAL_AUTO)
	{
//		gimbal_autoangel_control(add_yaw, add_pitch, gimbal_control_set);
	}
}

/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, __attribute__((unused)) gimbal_control_t *gimbal_control_set)
{
	*yaw = 0.0f;
	*pitch = 0.0f;
}

/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @author         RM
  * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
	//初始化状态控制量计算
	if (fabsf(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
	{
		*pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
		*yaw = 0.0f;
	}
	else
	{
		*pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
		*yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
	}
}

/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[out]     yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[out]     pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
	static uint32_t cali_time = 0;

	if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP)
	{

		*pitch = GIMBAL_CALI_MOTOR_SET;
		*yaw = 0;

		//判断陀螺仪数据， 并记录最大最小角度数据
		gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
			gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
			get_gimbal_motor_ecd(&gimbal_control_set->gimbal_pitch_motor), gimbal_control_set->gimbal_cali.step)
	}
	else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP)
	{
		*pitch = -GIMBAL_CALI_MOTOR_SET;
		*yaw = 0;

		gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
			gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
			get_gimbal_motor_ecd(&gimbal_control_set->gimbal_pitch_motor), gimbal_control_set->gimbal_cali.step)
	}
	else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP)
	{
		*pitch = 0;
		*yaw = GIMBAL_CALI_MOTOR_SET;

		gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
			gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
			get_gimbal_motor_ecd(&gimbal_control_set->gimbal_yaw_motor), gimbal_control_set->gimbal_cali.step)
	}

	else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP)
	{
		*pitch = 0;
		*yaw = -GIMBAL_CALI_MOTOR_SET;

		gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
			gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
			get_gimbal_motor_ecd(&gimbal_control_set->gimbal_yaw_motor), gimbal_control_set->gimbal_cali.step)
	}
	else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
	{
		cali_time = 0;
	}
}

/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
	static int16_t yaw_channel = 0, pitch_channel = 0;

	rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND)
	rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND)

	*yaw = (fp32)yaw_channel * YAW_RC_SEN - (fp32)gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
	*pitch = (fp32)pitch_channel * PITCH_RC_SEN + (fp32)gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;

}

/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
	static int16_t yaw_channel = 0, pitch_channel = 0;

	rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND)
	rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND)

	*yaw = (fp32)yaw_channel * YAW_RC_SEN - (fp32)gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
	*pitch = (fp32)pitch_channel * PITCH_RC_SEN + (fp32)gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;
}

/**
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, __attribute__((unused)) gimbal_control_t *gimbal_control_set)
{
	*yaw = 0.0f;
	*pitch = 0.0f;
}

/**
 * @brief          返回云台状态机指针，目前用于发射机构判断是否自瞄以及是否无力
 * @param[in]      none
 * @return         云台状态机指针
 */
bool_t gimbal_cmd_to_chassis_stop(void)
{
	if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
  * @brief          返回云台状态机指针，目前用于发射机构判断是否自瞄以及是否无力
  * @author         Du.X.Y
  * @param[in]      none
  * @param[out]      gimbal_behaviour:云台状态机指针
  * @retval         none
  */
gimbal_behaviour_e* get_gimbal_behaviour_point(void) {
	return &gimbal_behaviour;
}
