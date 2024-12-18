//
// Created by Rick on 2024/12/5.
//

#include "gimbal.h"
#include "gimbal_behaviour.h"

extern FDCAN_HandleTypeDef hfdcan2;
static gimbal_control_t gimbal_control;

static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);

/**
  * @brief          初始化云台控制结构体"gimbal_control"变量，设置PID控制器、遥控器、云台电机、陀螺仪数据指针等。
  * @param[out]     init: 指向 "gimbal_control" 结构体的指针，初始化该结构体中的各个成员。
  * @retval         none
  *
  * 该函数初始化云台的控制系统，包括：
  * 1. 设置PID控制器的参数，包括俯仰角和偏航角的绝对、相对角度PID、角速度PID等。
  * 2. 获取并设置陀螺仪和惯性导航系统（INS）的角度和陀螺仪数据。
  * 3. 获取遥控器数据指针并初始化云台电机。
  * 4. 初始化电机的控制模式及相关PID。
  * 5. 清除所有PID控制器。
  * 6. 设置电机的目标角度和速度。
  */
void gimbal_init(gimbal_control_t *init)
{
	// 定义各个PID参数
	static const fp32 Yaw_absolute_angle_pid[3] = {YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD};
	static const fp32 Yaw_relative_angle_pid[3] = {YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD};
	static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
	static const fp32 Pitch_absolute_angle_pid[3] = {PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD};
	static const fp32 Pitch_relative_angle_pid[3] = {PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD};
	static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};

	// 初始化云台电机测量数据
	init->gimbal_pitch_motor.gimbal_motor_measure = get_motor_6020_measure_point(0);
	init->gimbal_yaw_motor.gimbal_motor_measure = get_motor_6020_measure_point(1);
	// 获取陀螺仪数据和INS角度数据的指针
	init->gimbal_INT_angle_point = get_INS_angle_point();  // 获取惯性导航系统（INS）角度数据指针
	init->gimbal_INT_gyro_point = get_gyro_data_point();   // 获取陀螺仪数据指针

	// 获取遥控器数据指针
	init->gimbal_rc_ctrl = get_dt7_point();  // 获取遥控器数据指针

	// 目前自瞄角度数据的获取暂时注释掉
	// init->gimbal_auto_ctrl = get_AUTOshoot_point();

	// 初始化云台电机模式，设定初始值为原始模式
	init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
	init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

	// 初始化偏航角电机的PID控制器
	PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, PID_POSITION, Yaw_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
	PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, PID_POSITION, Yaw_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT);
	PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);

	// 初始化俯仰角电机的PID控制器
	PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PID_POSITION, Pitch_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT);
	PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PID_POSITION, Pitch_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);
	PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

	// 清除偏航电机的PID控制器状态
	PID_clear(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);
	PID_clear(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);
	PID_clear(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid);

	// 清除俯仰电机的PID控制器状态
	PID_clear(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid);
	PID_clear(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid);
	PID_clear(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid);


	// 设置电机的初始目标角度和速度（默认为当前角度和速度）
	init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
	init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
	init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

	init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
	init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
	init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
}


/**
  * @brief          更新底盘测量数据，包括电机速度、欧拉角度和机器人速度。
  *                 本函数通过读取云台电机的角度数据、角速度数据及传感器测量值来更新
  *                 云台电机的反馈信息。
  * @param[out]     feedback_update: 指向 `gimbal_control_t` 类型结构体的指针，该结构体存储
  *                                 云台电机相关的各种控制和测量数据。
  * @retval         none
  *
  * @note           该函数主要用于更新云台电机的绝对角度、相对角度、陀螺仪数据等信息。
  *                 其中，俯仰角和偏航角的更新有所不同。通过 `motor_ecd_to_angle_change` 函数
  *                 来计算相对角度，并根据是否定义了 `PITCH_TURN` 和 `YAW_TURN` 来决定角度的
  *                 方向。此外，计算偏航角的角速度时需要考虑俯仰角的影响。
  */
void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
	// 如果传入的 feedback_update 指针为空，直接返回
	if (feedback_update == NULL)
	{
		return;
	}

	// 更新俯仰电机的绝对角度
	feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

	// 更新俯仰电机的相对角度
#if PITCH_TURN
	// 如果启用了 PITCH_TURN，将相对角度计算为负值
        feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(
            feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
            feedback_update->gimbal_pitch_motor.offset_ecd
        );
#else
	// 否则使用正值计算相对角度
	feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(
		feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
		feedback_update->gimbal_pitch_motor.offset_ecd
	);
#endif

	// 更新俯仰电机的陀螺仪数据（角速度）
	feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

	// 更新偏航电机的绝对角度
	feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

	// 更新偏航电机的相对角度
#if YAW_TURN
	// 如果启用了 YAW_TURN，将相对角度计算为负值
	feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(
		feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
		feedback_update->gimbal_yaw_motor.offset_ecd
	);
#else
	// 否则使用正值计算相对角度
        feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(
            feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
            feedback_update->gimbal_yaw_motor.offset_ecd
        );
#endif

	// 更新偏航电机的陀螺仪数据（角速度）
	// 计算时考虑俯仰角度对偏航角速度的影响
	feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) *
		(*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
		- arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) *
			(*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
}

/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
	int32_t relative_ecd = ecd - offset_ecd;
	if (relative_ecd > HALF_ECD_RANGE)
	{
		relative_ecd -= ECD_RANGE;
	}
	else if (relative_ecd < -HALF_ECD_RANGE)
	{
		relative_ecd += ECD_RANGE;
	}

	return (fp32)relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
void gimbal_set_mode(gimbal_control_t *set_mode)
{
	if (set_mode == NULL)
	{
		return;
	}
	gimbal_behaviour_mode_set(set_mode);
}

/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
	if (gimbal_mode_change == NULL)
	{
		return;
	}

	//yaw电机状态机切换保存数据
	if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
	{
		gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = (fp32)gimbal_mode_change->gimbal_yaw_motor.given_current;
	}
	else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
	}
	else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
		gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
	}
	gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

	//pitch电机状态机切换保存数据
	if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
	{
		gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = (fp32)gimbal_mode_change->gimbal_pitch_motor.given_current;
	}
	else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
	}
	else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
		gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
	}

	gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}

/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
void gimbal_set_control(gimbal_control_t *set_control)
{
	if (set_control == NULL)
	{
		return;
	}

	fp32 add_yaw_angle = 0.0f;
	fp32 add_pitch_angle = 0.0f;

	gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);

	//yaw电机模式控制
	if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
	{
		//raw模式下，直接发送控制值
		set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
	}
	else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		//gyro模式下，陀螺仪角度控制
		gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
	}
	else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
		//enconde模式下，电机编码角度控制
		gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
	}

	//pitch电机模式控制
	if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
	{
		//raw模式下，直接发送控制值
		set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
	}
	else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		//gyro模式下，陀螺仪角度控制
		gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
	}
	else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
		//enconde模式下，电机编码角度控制
		gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
	}
}
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
	if (gimbal_motor == NULL)
	{
		return;
	}
	gimbal_motor->absolute_angle_set = rad_format(gimbal_motor->absolute_angle_set + add);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
	if (gimbal_motor == NULL)
	{
		return;
	}
	gimbal_motor->relative_angle_set += add;
	//是否超过最大 最小值
	if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
	{
		gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
	}
	else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
	{
		gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
	}
}

/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
void gimbal_control_loop(gimbal_control_t *control_loop)
{
	if (control_loop == NULL)
	{
		return;
	}

	if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
	{
		gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
	}
	else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
	}
	else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
		gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
	}

	if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
	{
		gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
	}
	else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
	}
	else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
		gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
	}
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
	if (gimbal_motor == NULL)
	{
		return;
	}
	gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
	gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
	if (gimbal_motor == NULL)
	{
		return;
	}

	gimbal_motor->motor_gyro_set = PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set);
	gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
	//控制值赋值
	gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
	if (gimbal_motor == NULL)
	{
		return;
	}

	//角度环，速度环串级pid调试
	gimbal_motor->motor_gyro_set = PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set);
	gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
	//控制值赋值
	gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

static void FDCAN_cmd_gimbal(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	FDCAN_TxHeaderTypeDef txHeader;
	uint8_t txData[8]; // 数据缓存

	// 配置 FDCAN 消息头
	txHeader.Identifier = CAN_GIMBAL_ALL_ID;  // 示例标准 ID (根据实际需求修改)
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
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, txData) != HAL_OK)
	{

	}
}

gimbal_control_t * get_gimbal_control_point(void)
{
	return &gimbal_control;
}

static gimbal_control_t gimbal_control = {
	.CAN_cmd_gimbal = FDCAN_cmd_gimbal,
};

