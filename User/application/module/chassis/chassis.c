//
// Created by Rick on 24-11-26.
//

#include "chassis.h"
#include "chassis_behaviour.h"

extern FDCAN_HandleTypeDef hfdcan1;
static chassis_control_t chassis_control;

#define rc_deadband_limit(input, output, deadline) \
    (output) = ((input) > (deadline) || (input) < -(deadline)) ? (input) : 0;

#define DEADLINE_VX (CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
#define DEADLINE_VY (CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)

/**
  * @brief          检查并应用遥控器的死区限制
  *
  * @param[in]      chassis_move_rc_to_vector: 包含遥控器输入信号的控制结构体
  * @param[in,out]  vx_channel: 水平通道（X轴）原始输入值，经过死区限制后的值
  * @param[in,out]  vy_channel: 垂直通道（Y轴）原始输入值，经过死区限制后的值
  * @retval         none
  */
inline static void apply_rc_deadband_limit(chassis_control_t* chassis_move_rc_to_vector,
	int16_t* vx_channel,
	int16_t* vy_channel);

/**
  * @brief          应用键盘控制，覆盖遥控器输入
  *
  * @param[in]      chassis_move_rc_to_vector: 包含遥控器和键盘输入信号的控制结构体
  * @param[in,out]  vx_set_channel: 水平速度通道值，根据键盘控制进行调整
  * @param[in,out]  vy_set_channel: 垂直速度通道值，根据键盘控制进行调整
  * @retval         none
  */
inline static void apply_keyboard_control(chassis_control_t* chassis_move_rc_to_vector,
	fp32* vx_set_channel,
	fp32* vy_set_channel);

/**
  * @brief          处理底盘速度限制与平滑控制
  *
  * @param[in]      chassis_move_rc_to_vector: 包含底盘速度命令的控制结构体
  * @param[in,out]  vx_set_channel: 水平速度通道值，经过滤波和平滑处理
  * @param[in,out]  vy_set_channel: 垂直速度通道值，经过滤波和平滑处理
  * @retval         none
  */
inline static void apply_smooth_control(chassis_control_t* chassis_move_rc_to_vector,
	const fp32* vx_set_channel,
	const fp32* vy_set_channel);

/**
 * @brief          计算目标相对角度
 * @param[in]      vx_set: x轴速度
 * @param[in]      vy_set: y轴速度
 * @param[in]      mov_mag: 速度向量的模长
 * @param[in]      current_yaw: 当前云台角度
 * @retval         relative_angle: 计算后的相对角度
 */
inline static fp32 calculate_relative_angle(fp32 vx_set, fp32 vy_set, fp32 mov_mag, fp32 current_yaw);

/**
 * @brief          在原始模式下直接设置底盘速度
 * @param[in]      vx_set: x轴速度
 * @param[in]      vy_set: y轴速度
 * @param[in]      angle_set: 角速度
 * @param[out]     chassis_move_control: 底盘控制结构体指针
 * @retval         none
 */
inline static void chassis_raw_mode(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_control_t* chassis_move_control);

/**
 * @brief          跟随云台的角度进行底盘控制
 * @param[in]      vx_set: x轴速度
 * @param[in]      vy_set: y轴速度
 * @param[in]      angle_set: 目标角度
 * @param[out]     chassis_move_control: 底盘控制结构体指针
 * @retval         none
 */
inline static void chassis_follow_gimbal_yaw(fp32 vx_set,
	fp32 vy_set,
	fp32 angle_set,
	chassis_control_t* chassis_move_control);

/**
 * @brief          不跟随云台的角速度控制模式
 * @param[in]      vx_set: x轴速度
 * @param[in]      vy_set: y轴速度
 * @param[in]      angle_set: 目标角速度
 * @param[out]     chassis_move_control: 底盘控制结构体指针
 * @retval         none
 */
inline static void chassis_no_follow_yaw(fp32 vx_set,
	fp32 vy_set,
	fp32 angle_set,
	chassis_control_t* chassis_move_control);

inline static void chassis_vector_to_mecanum_wheel_speed(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4]);

/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[in]      chassis_move_rc_to_vector: "chassis_control" 变量指针
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32* vx_set, fp32* vy_set, chassis_control_t* chassis_move_rc_to_vector)
{
	// 参数检查
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
	{
		return;
	}

	// 初始化通道值
	int16_t vx_channel = 0, vy_channel = 0;
	fp32 vx_set_channel = 0.0f, vy_set_channel = 0.0f;

	// 应用死区限制
	apply_rc_deadband_limit(chassis_move_rc_to_vector, &vx_channel, &vy_channel);

	// 根据遥控器输入计算速度
	vx_set_channel = (fp32)vx_channel * CHASSIS_VX_RC_SEN;
	vy_set_channel = (fp32)vy_channel * -CHASSIS_VY_RC_SEN;

	// 应用键盘控制（优先级覆盖遥控器）
	apply_keyboard_control(chassis_move_rc_to_vector, &vx_set_channel, &vy_set_channel);

	// 应用平滑控制：低通滤波处理
	apply_smooth_control(chassis_move_rc_to_vector, &vx_set_channel, &vy_set_channel);

	// 最终速度输出
	*vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	*vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

/**
  * @brief          检查并应用遥控器的死区限制
  *
  * @param[in]      chassis_move_rc_to_vector: 包含遥控器输入信号的控制结构体
  * @param[in,out]  vx_channel: 水平通道（X轴）原始输入值，经过死区限制后的值
  * @param[in,out]  vy_channel: 垂直通道（Y轴）原始输入值，经过死区限制后的值
  * @retval         none
  */
inline static void apply_rc_deadband_limit(chassis_control_t* chassis_move_rc_to_vector,
	int16_t* vx_channel,
	int16_t* vy_channel)
{
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], *vx_channel, CHASSIS_RC_DEADLINE)
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], *vy_channel, CHASSIS_RC_DEADLINE)
}

/**
  * @brief          应用键盘控制，覆盖遥控器输入
  *
  * @param[in]      chassis_move_rc_to_vector: 包含遥控器和键盘输入信号的控制结构体
  * @param[in,out]  vx_set_channel: 水平速度通道值，根据键盘控制进行调整
  * @param[in,out]  vy_set_channel: 垂直速度通道值，根据键盘控制进行调整
  * @retval         none
  */
inline static void apply_keyboard_control(chassis_control_t* chassis_move_rc_to_vector,
	fp32* vx_set_channel,
	fp32* vy_set_channel)
{
	// 键盘前进/后退控制
	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
	{
		*vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
	{
		*vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
	}

	// 键盘左转/右转控制
	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
	{
		*vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
	{
		*vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
	}
}

/**
  * @brief          处理底盘速度限制与平滑控制
  *
  * @param[in]      chassis_move_rc_to_vector: 包含底盘速度命令的控制结构体
  * @param[in,out]  vx_set_channel: 水平速度通道值，经过滤波和平滑处理
  * @param[in,out]  vy_set_channel: 垂直速度通道值，经过滤波和平滑处理
  * @retval         none
  */
inline static void apply_smooth_control(chassis_control_t* chassis_move_rc_to_vector,
	const fp32* vx_set_channel,
	const fp32* vy_set_channel)
{
	// 应用一阶低通滤波
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, *vx_set_channel);
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, *vy_set_channel);

	// 判断是否接近零速，若是则直接停止
	if (*vx_set_channel < DEADLINE_VX && *vx_set_channel > -DEADLINE_VX)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
	}

	if (*vy_set_channel < DEADLINE_VY && *vy_set_channel > -DEADLINE_VY)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
	}
}

/**
  * @brief          初始化"chassis_control"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_control_t"变量指针.
  * @retval         none
  */
void chassis_init(chassis_control_t* chassis_move_init)
{
	// 检查输入指针是否为空
	if (chassis_move_init == NULL)
	{
		return;
	}

	//底盘速度环pid值
	const static fp32
		motor_speed_pid[3] = { M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD };

	//底盘角度pid值
	const static fp32 chassis_yaw_pid[3] =
		{ CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD };

	const static fp32 chassis_x_order_filter[1] = { CHASSIS_ACCEL_X_NUM };
	const static fp32 chassis_y_order_filter[1] = { CHASSIS_ACCEL_Y_NUM };

	//底盘开机状态为原始
	chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
	//获取遥控器指针
	chassis_move_init->chassis_RC = get_dt7_point();
	//获取云台电机数据指针
	chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
	chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();

	// 初始化PID参数
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_move_init->motor_chassis[i].motor_3508_measure = get_motor_3508_measure_point(0,i);
		PID_init(&chassis_move_init->motor_speed_pid[i],
			PID_POSITION,
			motor_speed_pid,
			M3505_MOTOR_SPEED_PID_MAX_OUT,
			M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
	//初始化角度PID
	PID_init(&chassis_move_init->chassis_angle_pid,
		PID_POSITION,
		chassis_yaw_pid,
		CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT,
		CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

	//用一阶滤波代替斜波函数生成
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

	//最大 最小速度
	chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

	//更新一下数据
	chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_control_t"变量指针.
  * @retval         none
  */
void chassis_set_mode(chassis_control_t* chassis_move_mode)
{
	if (chassis_move_mode == NULL)
	{
		return;
	}

	chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_control_t"变量指针.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_control_t* chassis_move_transit)
{
	// 检查输入指针是否为空
	if (chassis_move_transit == NULL)
	{
		return;
	}

	// 模式未改变
	if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
	{
		return;
	}

	//切入跟随云台模式
	if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
		&& chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
	}

		//切入不跟随云台模式
	else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW)
		&& chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
	{
		chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
	}

	// 更新模式
	chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_control_t"变量指针.
  * @retval         none
  */
void chassis_feedback_update(chassis_control_t* chassis_move_update)
{
	// 检查输入指针是否为空
	if (chassis_move_update == NULL)
	{
		return;
	}

	// 更新电机速度
	for (uint8_t i = 0; i < 4; i++)
	{
		// 更新电机速度，加速度是速度PID的微分
		chassis_move_update->motor_chassis[i].speed =
			CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * (fp32)chassis_move_update->motor_chassis[i].motor_3508_measure->speed_rpm;
		chassis_move_update->motor_chassis[i].accel =
			chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
	}

	//更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
	chassis_move_update->vx =
		(-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed
			+ chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed)
			* MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_move_update->vy =
		(-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed
			+ chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed)
			* MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	chassis_move_update->wz =
		(-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed
			- chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed)
			* MOTOR_SPEED_TO_CHASSIS_SPEED_WZ;
}

/**
 * @brief          设置底盘控制设置值，根据不同模式处理底盘运动控制
 * @param[out]     chassis_move_control: "chassis_control_t"变量指针.
 * @retval         none
 */
void chassis_set_control(chassis_control_t* chassis_move_control)
{
	// 检查输入指针是否为空
	if (chassis_move_control == NULL)
	{
		return;
	}

	fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
	// 获取三个控制设置值
	chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

	// 根据不同模式设置底盘控制值
	switch (chassis_move_control->chassis_mode)
	{
	case CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW:
		chassis_follow_gimbal_yaw(vx_set, vy_set, angle_set, chassis_move_control);
		break;
	case CHASSIS_VECTOR_NO_FOLLOW_YAW:
		chassis_no_follow_yaw(vx_set, vy_set, angle_set, chassis_move_control);
		break;
	case CHASSIS_VECTOR_RAW:
		chassis_raw_mode(vx_set, vy_set, angle_set, chassis_move_control);
		break;
	}
}

/**
 * @brief          跟随云台的角度进行底盘控制
 * @param[in]      vx_set: x轴速度
 * @param[in]      vy_set: y轴速度
 * @param[in]      angle_set: 目标角度
 * @param[out]     chassis_move_control: 底盘控制结构体指针
 * @retval         none
 */
static void chassis_follow_gimbal_yaw(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_control_t* chassis_move_control)
{
	fp32 sin_yaw, cos_yaw;
	// 旋转控制底盘速度方向，保证前进方向是云台方向
	sin_yaw = sinf(-chassis_move_control->chassis_yaw_motor->relative_angle);
	cos_yaw = sinf(-chassis_move_control->chassis_yaw_motor->relative_angle);
	chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
	chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;

	// 设置控制相对云台角度
	chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
	// 计算旋转PID角速度
	chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid,
		chassis_move_control->chassis_yaw_motor->relative_angle,
		chassis_move_control->chassis_relative_angle_set);

	// 速度限幅
	chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set,
		chassis_move_control->vx_min_speed,
		chassis_move_control->vx_max_speed);
	chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set,
		chassis_move_control->vy_min_speed,
		chassis_move_control->vy_max_speed);
}

/**
 * @brief          不跟随云台的角速度控制模式
 * @param[in]      vx_set: x轴速度
 * @param[in]      vy_set: y轴速度
 * @param[in]      angle_set: 目标角速度
 * @param[out]     chassis_move_control: 底盘控制结构体指针
 * @retval         none
 */
static void chassis_no_follow_yaw(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_control_t* chassis_move_control)
{
	// 恢复原始的速度数值
	vx_set /= CHASSIS_VX_RC_SEN;
	vy_set /= -CHASSIS_VY_RC_SEN;

	// 计算速度向量的模长
	fp32 mov_mag = (fp32)sqrt(vx_set * vx_set + vy_set * vy_set);
	fp32 relative_angle;

	// 计算目标相对角度
	relative_angle =
		calculate_relative_angle(vx_set, vy_set, mov_mag, chassis_move_control->chassis_yaw_motor->relative_angle);

	// 限制角度范围 [0, 2*PI]
	loop_fp32_constrain(relative_angle, 0, 2 * PI);

	// 计算新的 vx_set 和 vy_set
	vx_set = mov_mag * sinf(relative_angle) * CHASSIS_VX_RC_SEN;
	vy_set = mov_mag * cosf(relative_angle) * -CHASSIS_VY_RC_SEN;

	// 设置角速度
	chassis_move_control->wz_set = angle_set;

	// 速度限幅
	chassis_move_control->vx_set =
		fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
	chassis_move_control->vy_set =
		fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
}

/**
 * @brief          原始模式下直接设置底盘速度
 * @param[in]      vx_set: x轴速度
 * @param[in]      vy_set: y轴速度
 * @param[in]      angle_set: 角速度
 * @param[out]     chassis_move_control: 底盘控制结构体指针
 * @retval         none
 */
static void chassis_raw_mode(fp32 vx_set, fp32 vy_set, fp32 angle_set, chassis_control_t* chassis_move_control)
{
	chassis_move_control->vx_set = vx_set;
	chassis_move_control->vy_set = vy_set;
	chassis_move_control->wz_set = angle_set;

	// 设置慢速控制值为 0
	chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
	chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
}

/**
 * @brief          计算目标相对角度
 * @param[in]      vx_set: x轴速度
 * @param[in]      vy_set: y轴速度
 * @param[in]      mov_mag: 速度向量的模长
 * @param[in]      current_yaw: 当前云台角度
 * @retval         relative_angle: 计算后的相对角度
 */
static fp32 calculate_relative_angle(fp32 vx_set, fp32 vy_set, fp32 mov_mag, fp32 current_yaw)
{
	fp32 relative_angle = 0.0f;

	if (vx_set == 0.0f && vy_set == 0.0f)
	{
		relative_angle = current_yaw;  // 没有速度，角度保持不变
	}
	else
	{
		if (vx_set > 0.0f && vy_set > 0.0f)
			relative_angle = asinf(vx_set / mov_mag);
		else if (vx_set > 0.0f && vy_set == 0.0f)
			relative_angle = 0.5f * PI;
		else if (vx_set > 0.0f && vy_set < 0.0f)
			relative_angle = PI - asinf(vx_set / mov_mag);
		else if (vx_set == 0.0f && vy_set < 0.0f)
			relative_angle = PI;
		else if (vx_set < 0.0f && vy_set < 0.0f)
			relative_angle = PI + asinf(-vx_set / mov_mag);
		else if (vx_set < 0.0f && vy_set == 0.0f)
			relative_angle = 1.5f * PI;
		else if (vx_set < 0.0f && vy_set > 0.0f)
			relative_angle = 2 * PI - asinf(-vx_set / mov_mag);
	}

	return relative_angle;
}

/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_control_t"变量指针.
  * @retval         none
  */
void chassis_control_loop(chassis_control_t* chassis_move_control_loop)
{
	fp32 max_vector = 0.0f, vector_rate;
	fp32 temp;
	fp32 wheel_speed[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	uint8_t i = 0;

	// 麦轮运动分解
	chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
		chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

	// raw控制
	if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
	{

		for (i = 0; i < 4; i++)
		{
			chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
		}
		//raw控制直接返回
		return;
	}

	// 计算轮子控制最大速度，并限制其最大速度
	for (i = 0; i < 4; i++)
	{
		chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
		temp = fabsf(chassis_move_control_loop->motor_chassis[i].speed_set);
		if (max_vector < temp)
		{
			max_vector = temp;
		}
	}

	if (max_vector > MAX_WHEEL_SPEED)
	{
		vector_rate = MAX_WHEEL_SPEED / max_vector;
		for (i = 0; i < 4; i++)
		{
			chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
		}
	}

	//计算pid
	for (i = 0; i < 4; i++)
	{
		PID_calc(&chassis_move_control_loop->motor_speed_pid[i],
			chassis_move_control_loop->motor_chassis[i].speed,
			chassis_move_control_loop->motor_chassis[i].speed_set);
	}

	//赋值电流值
	for (i = 0; i < 4; i++)
	{
		chassis_move_control_loop->motor_chassis[i].give_current =
			(int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
	}
}

/**
  * @brief          四个麦轮速度是通过三个参数计算出来的
  * @param[in]      vx_set: 纵向速度
  * @param[in]      vy_set: 横向速度
  * @param[in]      wz_set: 旋转速度
  * @param[out]     wheel_speed: 四个麦轮速度
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set,
	const fp32 vy_set,
	const fp32 wz_set,
	fp32 wheel_speed[4])
{
	//because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
	//旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
	wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

static void FDCAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	FDCAN_TxHeaderTypeDef txHeader;
	uint8_t txData[8]; // 数据缓存

	// 配置 FDCAN 消息头
	txHeader.Identifier = CAN_CHASSIS_ALL_ID;  // 示例标准 ID (根据实际需求修改)
	txHeader.IdType = FDCAN_STANDARD_ID;       // 标准帧 (11位 ID)
	txHeader.TxFrameType = FDCAN_DATA_FRAME;   // 数据帧
	txHeader.DataLength = FDCAN_DLC_BYTES_8;   // 数据长度为 8 字节
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;    // 无速率切换（经典 CAN）
	txHeader.FDFormat = FDCAN_FD_CAN;           // 使用经典 CAN 帧格式
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 无错误
	txHeader.MessageMarker = 0;

	// 填充发送数据
	txData[0] = motor1 >> 8;   // Motor1 高位
	txData[1] = motor1 & 0xFF; // Motor1 低位
	txData[2] = motor2 >> 8;   // Motor2 高位
	txData[3] = motor2 & 0xFF; // Motor2 低位
	txData[4] = motor3 >> 8;   // Motor3 高位
	txData[5] = motor3 & 0xFF; // Motor3 低位
	txData[6] = motor4 >> 8;   // Motor4 高位
	txData[7] = motor4 & 0xFF; // Motor4 低位

	// 将消息添加到发送 FIFO 队列
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK)
	{

	}
}

chassis_control_t* get_chassis_control_point(void)
{
	return &chassis_control;
}

static chassis_control_t chassis_control = {
	.CAN_cmd_chassis = FDCAN_cmd_chassis,
};