//
// Created by Rick on 24-11-26.
//

#include "chassis.h"

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
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[in]      chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32* vx_set, fp32* vy_set, chassis_control_t* chassis_move_rc_to_vector)
{
	// 参数检查
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
	{
		return;
	}

	int16_t vx_channel = 0, vy_channel = 0;      // 初始化通道值
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
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_control_t *chassis_move_init)
{
	if (chassis_move_init == NULL)
	{
		return;
	}

	//chassis motor speed PID
	//底盘速度环pid值
	const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

	//chassis angle PID
	//底盘角度pid值
	const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
	const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
	uint8_t i;

	//in beginning， chassis mode is raw
	//底盘开机状态为原始
	chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
	//get remote control point
	//获取遥控器指针
	chassis_move_init->chassis_RC = get_dt7_point();
	//get gyro sensor euler angle point
	//获取陀螺仪姿态角指针
//	chassis_move_init->chassis_INS_angle = get_INS_angle_point();
	//get gimbal motor data point
	//获取云台电机数据指针
//	chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
//	chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();

	// 初始化PID参数
	for (i = 0; i < 4; i++)
	{
		PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
	//初始化角度PID
	PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

	//用一阶滤波代替斜波函数生成
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

	//最大 最小速度
	chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

	chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

	//更新一下数据
//	chassis_feedback_update(chassis_move_init);
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
//		Error_Handler();
	}
}

static void FDCAN_rec_chassis(uint32_t can_id, const uint8_t* rx_data)
{
	switch (can_id)
	{
	case CAN_3508_M1_ID:
		get_motor_3508_measure((motor_3508_measure_t*)(&chassis_control.motor_chassis[0].motor_3508_measure), rx_data);
		break;
	case CAN_3508_M2_ID:
		get_motor_3508_measure((motor_3508_measure_t*)(&chassis_control.motor_chassis[1].motor_3508_measure), rx_data);
		break;
	case CAN_3508_M3_ID:
		get_motor_3508_measure((motor_3508_measure_t*)(&chassis_control.motor_chassis[2].motor_3508_measure), rx_data);
		break;
	case CAN_3508_M4_ID:
		get_motor_3508_measure((motor_3508_measure_t*)(&chassis_control.motor_chassis[3].motor_3508_measure), rx_data);
		break;
	default:
		break;
	}
}

chassis_control_t* get_chassis_control_point(void)
{
	return &chassis_control;
}

static chassis_control_t chassis_control = {
	.CAN_cmd_chassis = FDCAN_cmd_chassis,
	.CAN_rec_chassis = FDCAN_rec_chassis,
};