/*******************************************************************************
 * @file: dji_3508.c
 * @author: Javen
 * @date: 2024年12月18日
 * @brief: DJI 3508电机驱动
 * @note: 3508电机驱动, 支持CAN通信, 电机数据解析, 电机控制
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#include "dji_3508.h"

// 静态数组，用于存储两组8个DJI 3508电机的测量数据
//二维数组方便存放不同CAN上相同id的3508
static motor_3508_measure_t motor_3508_measure[2][8] = { 0 };

/**
 * @brief 解析CAN数据并填充到motor_3508_measure_t结构体中。
 *
 * 该函数接收一个指向motor_3508_measure_t结构体的指针和一个包含原始CAN数据的字节数组。
 * 从数据中提取编码器计数（ecd）、转速（RPM）、给定电流和温度，并将其存储到结构体中。
 *
 * @param ptr 指向motor_3508_measure_t结构体的指针，用于存储解析后的数据。
 * @param data 指向包含原始CAN数据的字节数组的指针。
 */
void motor_3508_measure_parse(motor_3508_measure_t *ptr, const uint8_t *data) {
	ptr->last_ecd = ptr->ecd; // 存储上一次的编码器计数
	ptr->ecd = (uint16_t)((data[0] << 8) | data[1]); // 提取当前编码器计数
	ptr->speed_rpm = (uint16_t)((data[2] << 8) | data[3]); // 提取转速（RPM） NOLINT(*-narrowing-conversions)
	ptr->given_current = (uint16_t)((data[4] << 8) | data[5]); // 提取给定电流 NOLINT(*-narrowing-conversions)
	ptr->temperature = data[6]; // 提取温度
}

/**
 * @brief 获取指定索引的motor_3508_measure_t结构体指针。
 *
 * 该函数返回指向motor_3508_measure数组中指定索引的结构体指针。
 *
 * @param i 数组索引，范围为0到1，分别代表CAN1和CAN2，J，数组索引，范围为0到7
 * @return 指向指定索引的motor_3508_measure_t结构体的指针。
 */
const motor_3508_measure_t* get_motor_3508_measure_point(uint8_t i,uint8_t j){
	return &motor_3508_measure[i][j];
}

/**
 * @brief CAN回调函数，处理接收到的CAN消息。
 *
 * 该函数根据CAN消息的ID，解析`底盘`对应电机的数据，并调用检测钩子函数。
 * 如果CAN ID不在预期范围内，则调用错误处理函数。
 *
 * @param can_id CAN消息的ID。
 * @param rx_data 指向接收到的CAN数据的字节数组的指针。
 */
void motor_3508_can_callback(uint32_t can_id, const uint8_t* rx_data)
{
	switch (can_id)
	{
	case CAN_3508_M1_ID:
		motor_3508_measure_parse(&motor_3508_measure[0][0], rx_data); // 解析电机1的数据
		detect_hook(CHASSIS_MOTOR1_TOE); // 调用检测钩子函数
		break;
	case CAN_3508_M2_ID:
		motor_3508_measure_parse(&motor_3508_measure[0][1], rx_data); // 解析电机2的数据
		detect_hook(CHASSIS_MOTOR2_TOE); // 调用检测钩子函数
		break;
	case CAN_3508_M3_ID:
		motor_3508_measure_parse(&motor_3508_measure[0][2], rx_data); // 解析电机3的数据
		detect_hook(CHASSIS_MOTOR3_TOE); // 调用检测钩子函数
		break;
	case CAN_3508_M4_ID:
		motor_3508_measure_parse(&motor_3508_measure[0][3], rx_data); // 解析电机4的数据
		detect_hook(CHASSIS_MOTOR4_TOE); // 调用检测钩子函数
		break;
	case CAN_3508_M5_ID:
		motor_3508_measure_parse(&motor_3508_measure[0][4], rx_data); // 解析电机5的数据
		break;
	case CAN_3508_M6_ID:
		motor_3508_measure_parse(&motor_3508_measure[0][5], rx_data); // 解析电机6的数据
		break;
	case CAN_3508_M7_ID:
		motor_3508_measure_parse(&motor_3508_measure[0][6], rx_data); // 解析电机7的数据
		break;
	case CAN_3508_M8_ID:
		motor_3508_measure_parse(&motor_3508_measure[0][7], rx_data); // 解析电机8的数据
		break;
	default:
		break;
	}
}


/**
 * @brief CAN回调函数，处理接收到的CAN消息。
 *
 * 该函数根据CAN消息的ID，解析`发射机构`对应电机的数据，并调用检测钩子函数。
 * 如果CAN ID不在预期范围内，则调用错误处理函数。
 * @param can_id
 * @param rx_data
 */
void shoot_3508_can_callback(uint32_t can_id, const uint8_t* rx_data) {
	switch (can_id) {
		case CAN_3508_M1_ID:
			motor_3508_measure_parse(&motor_3508_measure[1][0], rx_data); // 解析电机1的数据
		break;
		case CAN_3508_M2_ID:
			motor_3508_measure_parse(&motor_3508_measure[1][1], rx_data); // 解析电机2的数据
		break;
		case CAN_3508_M3_ID:
			motor_3508_measure_parse(&motor_3508_measure[1][2], rx_data); // 解析电机3的数据
		break;
		case CAN_3508_M4_ID:
			motor_3508_measure_parse(&motor_3508_measure[1][3], rx_data); // 解析电机4的数据
		break;
		case CAN_3508_M5_ID:
			motor_3508_measure_parse(&motor_3508_measure[1][4], rx_data); // 解析电机5的数据
		break;
		case CAN_3508_M6_ID:
			motor_3508_measure_parse(&motor_3508_measure[1][5], rx_data); // 解析电机6的数据
		break;
		case CAN_3508_M7_ID:
			motor_3508_measure_parse(&motor_3508_measure[1][6], rx_data); // 解析电机7的数据
		break;
		case CAN_3508_M8_ID:
			motor_3508_measure_parse(&motor_3508_measure[1][7], rx_data); // 解析电机8的数据
		break;
		default:
			break;
	}
}