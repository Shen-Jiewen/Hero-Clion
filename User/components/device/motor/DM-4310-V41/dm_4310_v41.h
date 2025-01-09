/*******************************************************************************
 * @file: dm_4310_v41.h
 * @author: Javen
 * @date: 2024年1月4日
 * @brief: DM 4310电机驱动
 * @note: 该文件实现了DM 4310电机的CAN通信解析和数据处理功能。
 *        支持8个电机的数据解析，包括编码器计数、转速、电流和温度。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#ifndef DM_4310_V41_H_
#define DM_4310_V41_H_

#include "main.h"
#include "struct_typedef.h"
#include "detect.h"

typedef enum
{
	CAN_4310_v41_ALL_ID = 0x4FE,		// 所有电机的ID
	FDCAN_DM4310_V41_M1_ID = 0x305,   	// 电机1的ID
	FDCAN_DM4310_V41_M2_ID = 0x306,   	// 电机2的ID
	FDCAN_DM4310_V41_M3_ID = 0x307,   	// 电机3的ID
	FDCAN_DM4310_V41_M4_ID = 0x308,   	// 电机4的ID
} dm_4310_v41_id_e;

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
	uint8_t error;              // 错误码
} motor_4310_v41_measure_t;

/**
 * @brief 获取指定索引的`motor_4310_v41_measure_t`结构体指针。
 *
 * @param i 数组索引，范围为0到7。
 * @return 指向指定索引的`motor_4310_v41_measure_t`结构体的指针。
 */
const motor_4310_v41_measure_t * get_motor_4310_v41_measure_point(uint8_t i);

/**
 * @brief CAN回调函数，处理接收到的CAN消息。
 *
 * @param can_id CAN消息的ID。
 * @param rx_data 指向接收到的CAN数据的字节数组的指针。
 */
void motor_4310_v41_can_callback(uint32_t can_id, const uint8_t* rx_data);

#endif //DM_4310_V41_H_
