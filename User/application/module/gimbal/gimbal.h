//
// Created by Rick on 2024/12/5.
//

#ifndef GIMBAL_H_
#define GIMBAL_H_

#include "main.h"
#include "struct_typedef.h"
#include "dji_3508.h"
#include "dji_6020.h"
#include "dm_4310.h"
#include "dm_4310_v41.h"
#include "dt7.h"
#include "user_lib.h"
#include "arm_math.h"
#include "pid.h"
#include "imu.h"

//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP        8000.0f//1500
#define PITCH_SPEED_PID_KI        28.0f//5
#define PITCH_SPEED_PID_KD        0.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP        10000.0f//3600
#define YAW_SPEED_PID_KI        20.0f//20
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f

//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 25.5f  //15
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP        26.0f  //28（26）
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.3f  //0.3
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 100.0f   //  15
#define PITCH_ENCODE_RELATIVE_PID_KI 0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP        8.0f  //8
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 2000
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0

//掉头180 按键
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//掉头云台速度
#define TURN_SPEED    0.04f
//测试按键尚未使用
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   10

//遥控器通道转换成电机角度
#define YAW_RC_SEN    (-0.000005f)
#define PITCH_RC_SEN  (-0.000004f) //6f

#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00005f // 0.00015

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 1

//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

#define PITCH_TURN  0//1
#define YAW_TURN    1//0

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.005f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET   20000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

typedef struct
{
	fp32 max_yaw;              // 最大偏航角，单位弧度。
	fp32 min_yaw;              // 最小偏航角，单位弧度。
	fp32 max_pitch;            // 最大俯仰角，单位弧度。
	fp32 min_pitch;            // 最小俯仰角，单位弧度。
	uint16_t max_yaw_ecd;      // 对应最大偏航位置的编码器计数。
	uint16_t min_yaw_ecd;      // 对应最小偏航位置的编码器计数。
	uint16_t max_pitch_ecd;    // 对应最大俯仰位置的编码器计数。
	uint16_t min_pitch_ecd;    // 对应最小俯仰位置的编码器计数。
	uint8_t step;              // 校准步骤的步长，决定校准步骤的粒度。
} gimbal_step_cali_t;

/**
 * @brief 定义通用电机字段枚举类型
 */
typedef enum
{
	MOTOR_FIELD_TEMPERATURE,  // 电机温度（通用字段）
	MOTOR_FIELD_LAST_ECD      // 上一次的编码器数据（通用字段）
} motor_common_field_t;

/**
 * @brief 定义电机控制模式的枚举类型
 */
typedef enum
{
	MOTOR_TYPE_6020,
	MOTOR_TYPE_4310,
	MOTOR_TYPE_UNKNOWN
} motor_type_e;

/**
 * @brief 定义电机控制数据结构体
 */
typedef struct
{
	motor_type_e motor_type;                                // 电机类型
	union
	{
		const motor_6020_measure_t* motor_6020;            // 指向6020电机测量数据的常量指针
		const motor_4310_v41_measure_t* motor_4310;        // 指向4310电机测量数据的常量指针
	} motor_measure;

	pid_type_def gimbal_motor_absolute_angle_pid;      // 绝对角度PID控制器
	pid_type_def gimbal_motor_relative_angle_pid;      // 相对角度PID控制器
	pid_type_def gimbal_motor_gyro_pid;                // 陀螺仪PID控制器

	gimbal_motor_mode_e gimbal_motor_mode;             // 当前电机控制模式
	gimbal_motor_mode_e last_gimbal_motor_mode;        // 上一次电机控制模式

	fp32 offset_ecd;                                   // 编码器偏移值

	fp32 max_relative_angle;                           // 允许的最大相对角度，单位：弧度 (rad)
	fp32 min_relative_angle;                           // 允许的最小相对角度，单位：弧度 (rad)

	fp32 relative_angle;                               // 当前相对角度，单位：弧度 (rad)
	fp32 relative_angle_set;                           // 设定相对角度，单位：弧度 (rad)
	fp32 absolute_angle;                               // 当前绝对角度，单位：弧度 (rad)
	fp32 absolute_angle_set;                           // 设定绝对角度，单位：弧度 (rad)
	fp32 motor_gyro;                                   // 电机陀螺仪测量值，单位：弧度/秒 (rad/s)
	fp32 motor_gyro_set;                               // 陀螺仪设定值，单位：弧度/秒 (rad/s)
	fp32 motor_speed;                                  // 电机速度，单位：弧度/秒 (rad/s)
	fp32 raw_cmd_current;                              // 原始电流命令值
	fp32 current_set;                                  // 电流设定值
	int32_t given_current;                             // 实际给定电流值
} gimbal_motor_t;

typedef struct
{
	const RC_ctrl_t* gimbal_rc_ctrl;        // 指向云台遥控控制输入的常量指针。
	const fp32* gimbal_INT_angle_point;     // 指向内部角度传感器数据点的常量指针，单位弧度。
	const fp32* gimbal_INT_gyro_point;      // 指向内部陀螺仪传感器数据点的常量指针，单位弧度每秒。
	gimbal_motor_t gimbal_yaw_motor;        // 控制云台偏航电机的数据和控制变量的结构体。
	gimbal_motor_t gimbal_pitch_motor;      // 控制云台俯仰电机的数据和控制变量的结构体。
	gimbal_step_cali_t gimbal_cali;         // 云台步进位置的校准数据。

	// 通信接口定义
	void (* CAN_cmd_gimbal)(uint16_t canid, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
} gimbal_control_t;

void gimbal_init(gimbal_control_t* init);
void gimbal_feedback_update(gimbal_control_t* feedback_update);
void gimbal_set_mode(gimbal_control_t* set_mode);
void gimbal_mode_change_control_transit(gimbal_control_t* gimbal_mode_change);
void gimbal_set_control(gimbal_control_t* set_control);
void gimbal_control_loop(gimbal_control_t* control_loop);

fp32 get_gimbal_motor_ecd(const gimbal_motor_t* motor);

gimbal_control_t* get_gimbal_control_point(void);
gimbal_motor_t* get_yaw_motor_point(void);
gimbal_motor_t* get_pitch_motor_point(void);

#endif //GIMBAL_H_
