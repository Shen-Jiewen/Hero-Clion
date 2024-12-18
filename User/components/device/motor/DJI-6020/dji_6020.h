/*******************************************************************************
 * @file: dji_6020.h
 * @author: Javen
 * @date: 2024年12月18日
 * @brief: DJI 6020电机驱动头文件
 * @note: 该文件定义了DJI 6020电机的相关数据结构、枚举类型和函数声明。
 *        支持CAN通信解析、电机控制模式切换以及PID控制器配置。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#ifndef DM_02_HERO_USER_COMPONENTS_DEVICE_MOTOR_DJI_6020_DJI_6020_H_
#define DM_02_HERO_USER_COMPONENTS_DEVICE_MOTOR_DJI_6020_DJI_6020_H_

#include "main.h"
#include "struct_typedef.h"
#include "pid.h"
#include "detect.h"

/**
 * @brief 定义电机CAN ID的枚举类型
 */
typedef enum
{
	CAN_GIMBAL_ALL_ID = 0x1FF,  // 所有电机的CAN ID
	CAN_YAW_MOTOR_ID = 0x205,   // Yaw电机的CAN ID
	CAN_PIT_MOTOR_ID = 0x206,   // Pitch电机的CAN ID
} motor_6020_id_e;

/**
 * @brief 定义电机控制模式的枚举类型
 */
typedef enum
{
	GIMBAL_MOTOR_RAW = 0,       // 电机原始值控制模式
	GIMBAL_MOTOR_GYRO,          // 电机陀螺仪角度控制模式
	GIMBAL_MOTOR_ENCONDE        // 电机编码器角度控制模式
} gimbal_motor_mode_e;

/**
 * @brief 定义电机测量数据结构体
 */
typedef struct
{
	uint16_t ecd;               // 编码器数据
	int16_t speed_rpm;          // 电机转速，单位：转/分钟
	int16_t given_current;      // 给定电流值
	uint8_t temperature;        // 电机温度
	int16_t last_ecd;           // 上一次的编码器数据
} motor_6020_measure_t;

/**
 * @brief 定义电机控制数据结构体
 */
typedef struct
{
	const motor_6020_measure_t* gimbal_motor_measure;  // 指向电机测量数据的常量指针

	pid_type_def gimbal_motor_absolute_angle_pid;      // 绝对角度PID控制器
	pid_type_def gimbal_motor_relative_angle_pid;      // 相对角度PID控制器
	pid_type_def gimbal_motor_gyro_pid;                // 陀螺仪PID控制器

	gimbal_motor_mode_e gimbal_motor_mode;             // 当前电机控制模式
	gimbal_motor_mode_e last_gimbal_motor_mode;        // 上一次电机控制模式

	uint16_t offset_ecd;                               // 编码器偏移值

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

/**
 * @brief 获取指定索引的motor_6020_measure_t结构体指针。
 *
 * @param i 数组索引，范围为0到7。
 * @return 指向指定索引的motor_6020_measure_t结构体的指针。
 */
const motor_6020_measure_t * get_motor_6020_measure_point(uint8_t i);

/**
 * @brief CAN回调函数，处理接收到的CAN消息。
 *
 * @param can_id CAN消息的ID。
 * @param rx_data 指向接收到的CAN数据的字节数组的指针。
 */
void motor_6020_can_callback(uint32_t can_id, const uint8_t* rx_data);

#endif //DM_02_HERO_USER_COMPONENTS_DEVICE_MOTOR_DJI_6020_DJI_6020_H_