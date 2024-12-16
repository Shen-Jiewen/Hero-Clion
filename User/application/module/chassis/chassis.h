//
// Created by Rick on 24-11-26.
//

#ifndef DM_02_HERO_USER_APPLICATION_MODULE_CHASSIS_H_
#define DM_02_HERO_USER_APPLICATION_MODULE_CHASSIS_H_

#include "main.h"
#include "struct_typedef.h"
// 设备
#include "gimbal.h"
#include "dji_3508.h"
#include "dt7.h"
#include "detect.h"
// 板载层
#include "bsp_callback.h"
// 库
#include "pid.h"
#include "user_lib.h"
#include "arm_math.h"

// 任务开始时空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

// 控制前后速度的遥控器通道号
#define CHASSIS_X_CHANNEL 1

// 控制左右速度的遥控器通道号
#define CHASSIS_Y_CHANNEL 0

// 在某些模式下，遥控器可以控制旋转速度
#define CHASSIS_WZ_CHANNEL 2

// 选择底盘模式的遥控器通道号
#define CHASSIS_MODE_CHANNEL 0

// 前进摇杆值（最大660）转化为车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f

// 左右摇杆值（最大660）转化为车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f

// 跟随底盘yaw角模式下，遥控器的yaw遥杆（最大660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

// 不跟随云台时，遥控器的yaw遥杆（最大660）转化为车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

// 摇杆死区范围
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f   //4.0
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f  //2.0

//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.8f  //1.5

#define CHASSIS_WZ_SET_SCALE 0.1f

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 40.0f   //30
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 3.0f   //2.0
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
	CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   // 底盘会跟随云台相对角度
	CHASSIS_VECTOR_NO_FOLLOW_YAW,       // 底盘具有旋转速度控制
	CHASSIS_VECTOR_RAW,                 // 控制电流将直接发送到CAN总线
} chassis_mode_e;

typedef struct {
	const RC_ctrl_t *chassis_RC;
	const gimbal_motor_t *chassis_yaw_motor;
	const gimbal_motor_t *chassis_pitch_motor;
	chassis_mode_e chassis_mode;
	chassis_mode_e last_chassis_mode;
	motor_3508_t motor_chassis[4];
	pid_type_def motor_speed_pid[4];
	pid_type_def chassis_angle_pid;

	first_order_filter_type_t chassis_cmd_slow_set_vx;  // 使用一阶低通滤波减缓设定值
	first_order_filter_type_t chassis_cmd_slow_set_vy;  // 使用一阶低通滤波减缓设定值

	fp32 vx;                          // 底盘速度 前进方向 前为正，单位 m/s
	fp32 vy;                          // 底盘速度 左右方向 左为正  单位 m/s
	fp32 wz;                          // 底盘旋转角速度，逆时针为正 单位 rad/s
	fp32 vx_set;                      // 底盘设定速度 前进方向 前为正，单位 m/s
	fp32 vy_set;                      // 底盘设定速度 左右方向 左为正，单位 m/s
	fp32 wz_set;                      // 底盘设定旋转角速度，逆时针为正 单位 rad/s
	fp32 chassis_relative_angle;      // 底盘与云台的相对角度，单位 rad
	fp32 chassis_relative_angle_set;  // 设置相对云台控制角度
	fp32 chassis_yaw_set;

	fp32 vx_max_speed;  // 前进方向最大速度 单位m/s
	fp32 vx_min_speed;  // 后退方向最大速度 单位m/s

	fp32 vy_max_speed;  // 左方向最大速度 单位m/s
	fp32 vy_min_speed;  // 右方向最大速度 单位m/s
	fp32 chassis_yaw;   // 陀螺仪和云台电机叠加的yaw角度
	fp32 chassis_pitch; // 陀螺仪和云台电机叠加的pitch角度
	fp32 chassis_roll;  // 陀螺仪和云台电机叠加的roll角度

	uint8_t gyroscape_flag;		//小陀螺开启标志位
	uint8_t high_speed_flag;	//提速标志位
	uint8_t auto_flag;			//自瞄标志位

	// 通信接口定义
	void (*CAN_cmd_chassis)(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
	void (*CAN_rec_chassis)(uint32_t can_id, const uint8_t* rx_data);
} chassis_control_t;

extern void chassis_init(chassis_control_t *chassis_move_init);
extern void chassis_set_mode(chassis_control_t *chassis_move_mode);
extern void chassis_mode_change_control_transit(chassis_control_t *chassis_move_transit);
extern void chassis_feedback_update(chassis_control_t *chassis_move_update);
extern void chassis_set_control(chassis_control_t* chassis_move_control);
extern void chassis_control_loop(chassis_control_t *chassis_move_control_loop);

extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_control_t *chassis_move_rc_to_vector);
extern chassis_control_t* get_chassis_control_point(void);

#endif //DM_02_HERO_USER_APPLICATION_MODULE_CHASSIS_H_
