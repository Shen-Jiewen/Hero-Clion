/*******************************************************************************
 * @file: dji_3508.h
 * @author: Javen
 * @date: 2024年12月18日
 * @brief: DJI 3508电机驱动头文件
 * @note: 该文件定义了DJI 3508电机的相关数据结构、枚举类型和函数声明。
 *        支持CAN通信解析、电机数据解析以及电机控制。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#ifndef DM_02_HERO_USER_COMPONENTS_DEVICE_MOTOR_DJI_3508_DJI_3508_H_
#define DM_02_HERO_USER_COMPONENTS_DEVICE_MOTOR_DJI_3508_DJI_3508_H_

#include "main.h"
#include "struct_typedef.h"
#include "detect.h"

/**
 * @brief 定义电机CAN ID的枚举类型
 */
typedef enum
{
	CAN_CHASSIS_ALL_ID = 0x200,  // 所有电机的CAN ID
	CAN_3508_M1_ID = 0x201,      // 电机1的CAN ID
	CAN_3508_M2_ID = 0x202,      // 电机2的CAN ID
	CAN_3508_M3_ID = 0x203,      // 电机3的CAN ID
	CAN_3508_M4_ID = 0x204,      // 电机4的CAN ID
	CAN_3508_M5_ID = 0x205,      // 电机5的CAN ID
	CAN_3508_M6_ID = 0x206,      // 电机6的CAN ID
	CAN_3508_M7_ID = 0x207,      // 电机7的CAN ID
	CAN_3508_M8_ID = 0x208,      // 电机8的CAN ID
} motor_3508_id_e;

/**
 * @brief 定义电机测量数据结构体
 * @note 该结构体用于存储电机的编码器数据、转速、给定电流、温度以及上一次的编码器数据。
 */
typedef struct __attribute__((packed))
{
	uint16_t ecd;               // 编码器数据
	int16_t speed_rpm;          // 电机转速，单位：转/分钟
	int16_t given_current;      // 给定电流值
	uint8_t temperature;        // 电机温度
	uint16_t last_ecd;          // 上一次的编码器数据
} motor_3508_measure_t;

/**
 * @brief 定义电机控制数据结构体
 * @note 该结构体用于存储电机的测量数据指针、加速度、速度、设定速度、给定电流等信息。
 */
typedef struct
{
	const motor_3508_measure_t* motor_3508_measure;  // 指向电机测量数据的常量指针
	fp32 accel;                                      // 电机加速度
	fp32 speed;                                      // 电机当前速度
	fp32 speed_set;                                  // 电机设定速度
	int16_t give_current;                            // 实际给定电流值
} motor_3508_t;

/**
 * @brief 获取指定索引的motor_3508_measure_t结构体指针。
 *
 * @param i 数组索引，范围为0到1。0代表CAN1,1代表CAN2 ,j数组索引，范围为0到7
 * @return 指向指定索引的motor_3508_measure_t结构体的指针。
 */
const motor_3508_measure_t* get_motor_3508_measure_point(uint8_t i,uint8_t j);
/**
 * @brief CAN回调函数，处理接收到的CAN消息。
 *
 * @param can_id CAN消息的ID。
 * @param rx_data 指向接收到的CAN数据的字节数组的指针。
 */
void motor_3508_can_callback(uint32_t can_id, const uint8_t* rx_data);
void shoot_3508_can_callback(uint32_t can_id, const uint8_t* rx_data);
#endif //DM_02_HERO_USER_COMPONENTS_DEVICE_MOTOR_DJI_3508_DJI_3508_H_