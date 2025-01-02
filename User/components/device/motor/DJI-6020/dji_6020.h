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

#ifndef DJI_6020_H_
#define DJI_6020_H_

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

#endif //DJI_6020_H_