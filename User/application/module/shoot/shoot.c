//
// Created by Rick on 2024/12/20.
//

#include "shoot.h"

#include "fdcan.h"
#include "gimbal_behaviour.h"
#include "referee.h"

static shoot_control_t shoot_control;
static gimbal_behaviour_e* gimbal_behaviour;

static void trigger_motor_turn_back(void);
static void DM_shoot_control(void);
static void Down_shoot_bullet_control(void);

/**
  * @brief          返回射击控制结构体指针
  * @retval         void
  */
shoot_control_t* get_shoot_control_point(void)
{
	return &shoot_control;
}

/**
  * @brief          射击初始化
  * @param			shoot_init
  * @param[in]      shoot_control_t结构体指针
  * @retval         void
  */
void shoot_init(shoot_control_t* shoot_init)
{

	if (shoot_init == NULL)
	{
		return;
	}

	//定义各个PID参数
	static const fp32 Down_trigger_angle_pid[3] =
		{ Down_TRIGGER_ANGLE_PID_KP, Down_TRIGGER_ANGLE_PID_KI, Down_TRIGGER_ANGLE_PID_KD };
	static const fp32 Outside_shoot_speed_pid[3] =
		{ OUTSIDE_SHOOT_SPEED_PID_KP, OUTSIDE_SHOOT_SPEED_PID_KI, OUTSIDE_SHOOT_SPEED_PID_KD };
	static const fp32 Inside_shoot_speed_pid[3] =
		{ INSIDE_SHOOT_SPEED_PID_KP, INSIDE_SHOOT_SPEED_PID_KI, INSIDE_SHOOT_SPEED_PID_KD };

	//获取遥控器指针
	shoot_init->shoot_rc_ctrl = get_dt7_point();

	//获取云台行为模式指针
	gimbal_behaviour = get_gimbal_behaviour_point();

	//初始化拨盘电机类型和测量数据
	shoot_init->trigger_motor.motor_type = MOTOR_4310;
	shoot_init->trigger_motor.motor_measure.motor_4310 = get_dm_4310_measure_point(1);

	//初始化摩擦轮电机的测量数据
	for (uint8_t i = 0; i < 4; i++)
	{
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
	for (uint8_t i = 0; i < 2; i++)
	{
		PID_init(&shoot_init->friction_speed_pid[i],
			PID_POSITION,
			Outside_shoot_speed_pid,
			OUTSIDE_SHOOT_SPEED_PID_MAX_OUT,
			OUTSIDE_SHOOT_SPEED_PID_MAX_IOUT);
	}

	//初始化内侧摩擦轮电机PID
	for (uint8_t i = 2; i < 4; i++)
	{
		PID_init(&shoot_init->friction_speed_pid[i],
			PID_POSITION,
			Inside_shoot_speed_pid,
			OUTSIDE_SHOOT_SPEED_PID_MAX_OUT,
			OUTSIDE_SHOOT_SPEED_PID_MAX_IOUT);
	}

	//数据更新
	shoot_feedback_update(shoot_init);
	//初始化发射标志位
	shoot_init->fric_flag = 0;

	//初始化四个摩擦轮电机的速度和目标速度
	for (uint8_t i = 0; i < 4; i++)
	{
		shoot_init->friction_motor[i].speed = 0.0f;
		shoot_init->friction_motor[i].speed_set = 0.0f;
	}

	//初始化拨盘电机
	shoot_init->trigger_motor.give_current = 0;
	shoot_init->trigger_motor.angle_set = shoot_init->trigger_motor.motor_measure.motor_4310->position;
	shoot_init->trigger_motor.speed_set = 0;

	shoot_init->outside_fric_speed_set = 0.0f;
	shoot_init->inside_fric_speed_set = 0.0f;

	//获取当前弹速限制以及热量限制
	get_shoot_speed_limit(&shoot_init->last_shoot_speed_limit);
	get_shoot_speed_limit(&shoot_init->shoot_speed_limit);
	get_shoot_heat0_limit_and_heat0(&shoot_init->heat_limit, &shoot_init->heat);

	// 拨弹电机使能
	DM4310_MotorEnable(1);
}

/**
  * @brief          射击数据更新
  * @param			shoot_feedback
  * @retval         void
  */
void shoot_feedback_update(shoot_control_t* shoot_feedback)
{

	if (shoot_feedback == NULL)
	{
		return;
	}

	//更新拨弹电机数据
	shoot_feedback->trigger_motor.motor_measure.motor_4310 = get_dm_4310_measure_point(1);

	//更新摩擦轮电机速度
	for (uint8_t i = 0; i < 4; i++)
	{
		shoot_feedback->friction_motor[i].speed =
				(float)shoot_feedback->friction_motor[i].motor_3508_measure->speed_rpm * M3508_MOTOR_RPM_TO_VECTOR;
	}

	//记录上次的键盘状态
	shoot_feedback->last_press_l = shoot_feedback->press_l;
	shoot_feedback->last_press_r = shoot_feedback->press_r;

	//判断鼠标状态
	shoot_feedback->press_l = shoot_feedback->shoot_rc_ctrl->mouse.press_l;
	shoot_feedback->press_r = shoot_feedback->shoot_rc_ctrl->mouse.press_r;
}

/**
  * @brief 射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
    *	实际上只用到了三个状态机模式
    *	SHOOT_STOP，初始停止模式，摩擦轮和拨弹电机都不转动
    *	SHOOT_READY_FRIC，摩擦轮开启模式，前后四个摩擦轮转动
    *	SHOOT_READY_BULLET，准备射击模式，摩擦轮转动稳定会跳到下一个模式
    *	SHOOT_BULLET，射击模式，摩擦轮和拨弹电机转动，弹丸被发射出去
  * @param			set_mode
  * @retval         void
  */
void shoot_set_mode(shoot_control_t* set_mode)
{
	static int8_t last_s = RC_SW_UP;

	//左拨杆上拨判断，一次开启，再次关闭
	if (switch_is_up(set_mode->shoot_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL])
		&& !switch_is_up(last_s)
		&& set_mode->shoot_mode == SHOOT_STOP)
	{
		set_mode->shoot_mode = SHOOT_READY_FRIC;
	}

	//拨杆居中，Q键开启摩擦轮，E键关闭摩擦轮
	if (switch_is_mid(set_mode->shoot_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL])
		&& (set_mode->shoot_rc_ctrl->key.v & SHOOT_ON_KEYBOARD)
		&& set_mode->shoot_mode == SHOOT_STOP)
	{
		set_mode->shoot_mode = SHOOT_READY_FRIC;
	}
	else if (switch_is_mid(set_mode->shoot_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL])
		&& (set_mode->shoot_rc_ctrl->key.v & SHOOT_OFF_KEYBOARD)
		&& set_mode->shoot_mode != SHOOT_STOP)
	{
		set_mode->shoot_mode = SHOOT_STOP;
	}

	//摩擦轮开启模式下，且四个摩擦轮转速与对应目标值误差不大，进入准备发射环节
	if (set_mode->shoot_mode == SHOOT_READY_FRIC
		&& fabs(set_mode->friction_motor[0].speed - set_mode->friction_motor[0].speed_set) < 0.1f
		&& fabs(set_mode->friction_motor[1].speed - set_mode->friction_motor[1].speed_set) < 0.1f
		&& fabs(set_mode->friction_motor[2].speed - set_mode->friction_motor[2].speed_set) < 0.1f
		&& fabs(set_mode->friction_motor[3].speed - set_mode->friction_motor[3].speed_set) < 0.1f) {
		//Ready_Bullet为拨弹停，此时摩擦轮准备完毕
		set_mode->shoot_mode = SHOOT_READY_BULLET;
	}

		//遥控器左拨杆下拨，或者按下鼠标左键代表发射（允许拨弹电机拨弹）
	else if (set_mode->shoot_mode == SHOOT_READY_BULLET
		&& ((switch_is_down(set_mode->shoot_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL])) || set_mode->press_l))
	{
		set_mode->shoot_mode = SHOOT_BULLET;
	}
		//左拨杆没有下拨，或者没有按下鼠标左键表示处于准备发射阶段
	else if (set_mode->shoot_mode == SHOOT_READY
		&& (!switch_is_down(set_mode->shoot_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL])) || set_mode->press_l == 0)
	{
		set_mode->shoot_mode = SHOOT_READY_BULLET;
	}
	//  现在还没有写自瞄没有接口先注释掉
	// //开启自瞄并且找到目标后允许拨弹
	// else if (shoot_control.shoot_mode == SHOOT_READY_BULLET
	//     && *gimbal_behaviour == GIMBAL_AUTO
	//     && Get_Shoot_Brief() == 1) {
	//     shoot_control.shoot_mode = SHOOT_BULLET;
	// }

	//获取枪口热量限制和当前热量
	get_shoot_heat0_limit_and_heat0(&set_mode->heat_limit, &set_mode->heat);

	//若发生错误或者枪口热量超限，退出射击模式，进入准备射击
	if (!toe_is_error(REFEREE_TOE)
		&& (set_mode->heat + SHOOT_HEAT_NEED_VALUE > set_mode->heat_limit))
	{
		if (set_mode->shoot_mode == SHOOT_BULLET)
		{
			set_mode->shoot_mode = SHOOT_READY_BULLET;
		}
	}

	//若升级导致射速上限提高，就关闭发射
	if (set_mode->shoot_speed_limit != set_mode->last_shoot_speed_limit)
	{
		set_mode->shoot_mode = SHOOT_STOP;
	}

	//若云台无力，关闭发射
	if (*gimbal_behaviour == GIMBAL_ZERO_FORCE)
	{
		set_mode->shoot_mode = SHOOT_STOP;
	}

	//设置摩擦轮标志位
	if (set_mode->shoot_mode >= SHOOT_READY_FRIC)
	{
		set_mode->fric_flag = 1;
	}
	else
	{
		set_mode->fric_flag = 0;
	}

	last_s = (int8_t)set_mode->shoot_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL];
}

/**
  * @brief          设置发射机构控制量
  * @param			set_control
  * @retval         void
  */
void shoot_set_control(shoot_control_t* set_control)
{

	if (set_control == NULL)
	{
		return;
	}

	//发射停止，目标值置0
	if (set_control->shoot_mode == SHOOT_STOP)
	{
		set_control->trigger_motor.speed_set = 0.0f;
		set_control->outside_fric_speed_set = 0.0f;
		set_control->inside_fric_speed_set = 0.0f;
	}
	else if (set_control->shoot_mode == SHOOT_READY_FRIC)
	{
		//更新拨弹电机速度
		set_control->trigger_motor.speed_set = 0.0f;
	}
	else if (set_control->shoot_mode == SHOOT_READY_BULLET)
	{
		//设置摩擦轮电机的速度
		set_control->outside_fric_speed_set = -OUTSIDE_SPEED_SET;
		set_control->inside_fric_speed_set = -INSIDE_SPEED_SET;
		//单发标志位，拨动一次或鼠标按下一次只可以发射一发
		set_control->move_flag = 1;
	}
	else if (set_control->shoot_mode == SHOOT_BULLET)
	{
		//角度环限幅
		set_control->trigger_motor.trigger_angle_pid.max_out = Down_TRIGGER_ANGLE_PID_MAX_OUT;
		set_control->trigger_motor.trigger_angle_pid.max_iout = Down_TRIGGER_ANGLE_PID_MAX_IOUT;

		//射击控制
		Down_shoot_bullet_control();
	}
}

static void Down_shoot_bullet_control(void)
{

	//热量限制
//    if (shoot_control.heat_limit - shoot_control.heat >= 100) {
	DM_shoot_control();
//    }
	if (shoot_control.shoot_mode == SHOOT_BULLET
		&& (!(switch_is_down(shoot_control.shoot_rc_ctrl->rc.s[SHOOT_RC_MODE_CHANNEL]))
			&& (shoot_control.press_l == 0)))
	{
		shoot_control.shoot_mode = SHOOT_READY_BULLET;
	}
}

/**
 * @brief 拨弹电机拨弹转动控制
 * @param void
 */
static void DM_shoot_control(void)
{
	static int count = 0;
	count++;

	if (shoot_control.shoot_mode == SHOOT_BULLET)
	{
		if (shoot_control.move_flag == 1
			&& fabs(shoot_control.trigger_motor.motor_measure.motor_4310->position - Down_TRIGGER_POSITION) < 0.04f)
		{
			//每次循环保存一次零点，使电机转动绝对位置近似于转动相对位置
			//如果在射击模式下就向拨弹电机发送数据，每次转六十度
			uint8_t Data_Save_zero[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };    //达妙电机保存零点命令
			shoot_control.move_flag = 0;
			DM4310_SendStdData(&hfdcan2, Down_TRIGGER_MOTOR_ID, Data_Save_zero);
		}
		if (count == 5)
		{
			trigger_motor_turn_back();
			count = 0;
		}
		shoot_control.move_flag = 0;
	}

}

/**
 * @brief 拨弹电机防堵转
 * @param void
 */
static void trigger_motor_turn_back(void)
{
	if (fabsf(shoot_control.trigger_motor.motor_measure.motor_4310->torque) > 10)
	{
		DM4310_PosSpeed_CtrlMotor(&hfdcan2, Down_TRIGGER_MOTOR_ID, (float)0.1, (float)0.5);
	}
	else
	{
		DM4310_PosSpeed_CtrlMotor(&hfdcan2, Down_TRIGGER_MOTOR_ID, Down_TRIGGER_POSITION, -2);
	}
}

void shoot_control_loop(shoot_control_t* control_loop)
{

	if (control_loop == NULL)
	{
		return;
	}

	if (control_loop->shoot_mode == SHOOT_STOP)
	{
		control_loop->trigger_motor.give_current = (int16_t)0.0f;

		//关闭摩擦轮
		if (fabs(control_loop->friction_motor[0].speed < 0.1f)
			&& fabs(control_loop->friction_motor[1].speed < 0.1f)
			&& fabs(control_loop->friction_motor[2].speed < 0.1f)
			&& fabs(control_loop->friction_motor[3].speed < 0.1f))
		{
			for (uint8_t i = 0; i < 4; i++)
			{
				control_loop->friction_motor[i].give_current = (int16_t)0.0f;
			}
		}
		else
		{
			control_loop->friction_motor[0].give_current = (int16_t)control_loop->friction_speed_pid[0].out;
			control_loop->friction_motor[1].give_current = (int16_t)-control_loop->friction_speed_pid[1].out;
			control_loop->friction_motor[2].give_current = (int16_t)control_loop->friction_speed_pid[2].out;
			control_loop->friction_motor[3].give_current = (int16_t)-control_loop->friction_speed_pid[3].out;
		}
	}
	else
	{
		//PID计算
		//下拨弹电机电流赋值
		control_loop->trigger_motor.give_current = (int16_t)control_loop->trigger_motor.trigger_angle_pid.out;

		//摩擦轮电流赋值
		control_loop->friction_motor[0].give_current = (int16_t)control_loop->friction_speed_pid[0].out;
		control_loop->friction_motor[1].give_current = (int16_t)-control_loop->friction_speed_pid[1].out;
		control_loop->friction_motor[2].give_current = (int16_t)control_loop->friction_speed_pid[2].out;
		control_loop->friction_motor[3].give_current = (int16_t)-control_loop->friction_speed_pid[3].out;
		//如果状态机更改
		if (control_loop->shoot_mode < SHOOT_READY_BULLET)
		{
			control_loop->trigger_motor.give_current = (int16_t)0.0f;
		}
	}

	//PID计算
	PID_calc(&control_loop->friction_speed_pid[0],
		control_loop->friction_motor[0].speed,
		control_loop->outside_fric_speed_set);

	PID_calc(&control_loop->friction_speed_pid[1],
		control_loop->friction_motor[1].speed,
		control_loop->outside_fric_speed_set);

	PID_calc(&control_loop->friction_speed_pid[2],
		control_loop->friction_motor[2].speed,
		control_loop->inside_fric_speed_set);
	PID_calc(&control_loop->friction_speed_pid[3],
		control_loop->friction_motor[3].speed,
		control_loop->inside_fric_speed_set);
}

static void FDCAN_cmd_shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
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
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, txData) != HAL_OK)
	{

	}
}

static shoot_control_t shoot_control = {
	.CAN_cmd_shoot = FDCAN_cmd_shoot,
};