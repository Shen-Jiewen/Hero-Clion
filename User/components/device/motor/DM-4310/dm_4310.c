/*******************************************************************************
 * @file: dm_4310.c
 * @author: Javen
 * @date: 2024年12月30日
 * @brief: DM 4310电机驱动头文件
 * @note: 该文件定义了DM 4310电机的相关数据结构、枚举类型和函数声明。
 *        支持CAN通信解析、电机控制模式切换以及PID控制器配置。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/


#include "dm_4310.h"

// 静态数组，用于存储3个DM 4310电机的测量数据
static motor_4310_measure_t dm_4310_measure[4] = { 0};

/**
 * @brief 解析FDCAN数据并填充到motor_4310_measure_t结构体中。
 *
 * @param ptr 指向motor_4310_measure_t结构体的指针。
 * @param data 指向包含原始FDCAN数据的字节数组的指针。
 */
void dm_4310_measure_parse(motor_4310_measure_t* ptr, const uint8_t* data) {
	ptr->ID = (data[0] & 0xF);                      // 提取低 4 位作为 ID
	ptr->error = (data[0] >> 4);                    // 提取高 4 位作为错误码
	ptr->p_int = (data[1] << 8) | data[2];          // 组合成 16 位位置整数
	ptr->v_int = (data[3] << 4) | (data[4] >> 4);   // 组合成 12 位速度整数
	ptr->t_int = ((data[4] & 0xF) << 8) | data[5];  // 组合成 12 位扭矩整数
	ptr->position = DM4310_UintToFloat(ptr->p_int, P_MIN, P_MAX, 16); // 转换为浮点数位置，单位：弧度 (rad)
	ptr->velocity = DM4310_UintToFloat(ptr->v_int, V_MIN, V_MAX, 12); // 转换为浮点数速度，单位：弧度/秒 (rad/s)
	ptr->torque = DM4310_UintToFloat(ptr->t_int, T_MIN, T_MAX, 12);   // 转换为浮点数扭矩，单位：牛·米 (N·m)
	ptr->T_mos = (float)data[6];                    // MOSFET 温度，单位：摄氏度 (°C)
	ptr->T_motor = (float)data[7];                  // 电机线圈温度，单位：摄氏度 (°C)
}

/**
 * @brief 获取指定索引的motor_4310_measure_t结构体指针。
 *
 * @param i 数组索引，范围为0到2。
 * @return 指向指定索引的motor_4310_measure_t结构体的指针。
 */
const motor_4310_measure_t* get_dm_4310_measure_point(uint8_t i)
{
	return &dm_4310_measure[i];
}

/**
 * @brief 发送标准ID的数据帧。
 *
 * @param hfdcan FDCAN的句柄。
 * @param ID 数据帧ID。
 * @param pData 数组指针。
 * @param Len 字节数0~8。
 */
uint8_t DM4310_SendStdData(FDCAN_HandleTypeDef* hfdcan, uint16_t ID, uint8_t* pData)
{
	FDCAN_TxHeaderTypeDef Tx_Header = {
		.Identifier = ID,
		.IdType = FDCAN_STANDARD_ID,
		.TxFrameType = FDCAN_DATA_FRAME,
		.DataLength = FDCAN_DLC_BYTES_8,
		.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
		.BitRateSwitch = FDCAN_BRS_OFF,
		.FDFormat = FDCAN_CLASSIC_CAN,
		.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
		.MessageMarker = 0
	};

	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &Tx_Header, pData) != HAL_OK)
	{
		return 1; // 发送失败
	}
	return 0; // 发送成功
}

/**
 * @brief 使能电机。
 */
void DM4310_MotorEnable(uint8_t index)
{
	uint8_t Data_Enable[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
	switch (index)
	{
	case 0:
		DM4310_SendStdData(DM_CAN, FDCAN_DM4310_M1_SLAVE_ID, Data_Enable);
		break;
	case 1:
		DM4310_SendStdData(DM_CAN, FDCAN_DM4310_M2_SLAVE_ID, Data_Enable);
		break;
	case 2:
		DM4310_SendStdData(DM_CAN, FDCAN_DM4310_M3_SLAVE_ID, Data_Enable);
		break;
	case 3:
		DM4310_SendStdData(DM_CAN, FDCAN_DM4310_M4_SLAVE_ID, Data_Enable);
		break;
	default:
		break;
	}
}

/**
 * @brief MIT模式控制电机。
 *
 * @param hfdcan FDCAN的句柄。
 * @param id 数据帧的ID。
 * @param pos 位置给定。
 * @param _vel 速度给定。
 * @param KP 位置比例系数。
 * @param KD 位置微分系数。
 * @param torq 转矩给定值。
 */
void DM4310_MIT_CtrlMotor(FDCAN_HandleTypeDef* hfdcan,
	uint16_t id,
	float pos,
	float vel,
	float KP,
	float KD,
	float _torq)
{
	FDCAN_TxHeaderTypeDef Tx_Header = {
		.Identifier = id,
		.IdType = FDCAN_STANDARD_ID,
		.TxFrameType = FDCAN_DATA_FRAME,
		.DataLength = FDCAN_DLC_BYTES_8,
		.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
		.BitRateSwitch = FDCAN_BRS_OFF,
		.FDFormat = FDCAN_CLASSIC_CAN,
		.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
		.MessageMarker = 0
	};

	uint16_t pos_tmp = DM4310_FloatToUint(pos, P_MIN, P_MAX, 16);
	uint16_t vel_tmp = DM4310_FloatToUint(vel, V_MIN, V_MAX, 12);
	uint16_t kp_tmp = DM4310_FloatToUint(KP, KP_MIN, KP_MAX, 12);
	uint16_t kd_tmp = DM4310_FloatToUint(KD, KD_MIN, KD_MAX, 12);
	uint16_t tor_tmp = DM4310_FloatToUint(_torq, T_MIN, T_MAX, 12);

	uint8_t Tx_Data[8] = {
		(pos_tmp >> 8),
		pos_tmp,
		(vel_tmp >> 4),
		((vel_tmp & 0xF) << 4) | (kp_tmp >> 8),
		kp_tmp,
		(kd_tmp >> 4),
		((kd_tmp & 0xF) << 4) | (tor_tmp >> 8),
		tor_tmp
	};

	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &Tx_Header, Tx_Data);
}

/**
 * @brief 位置速度模式控制电机。
 *
 * @param hfdcan FDCAN的句柄。
 * @param id 数据帧的ID。
 * @param _pos 位置给定。
 * @param _vel 速度给定。
 */
void DM4310_PosSpeed_CtrlMotor(FDCAN_HandleTypeDef* hfdcan, uint16_t id, float _pos, float _vel)
{
	FDCAN_TxHeaderTypeDef Tx_Header = {
		.Identifier = id,
		.IdType = FDCAN_STANDARD_ID,
		.TxFrameType = FDCAN_DATA_FRAME,
		.DataLength = FDCAN_DLC_BYTES_8,
		.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
		.BitRateSwitch = FDCAN_BRS_OFF,
		.FDFormat = FDCAN_CLASSIC_CAN,
		.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
		.MessageMarker = 0
	};

	uint8_t* pbuf = (uint8_t*)&_pos;
	uint8_t* vbuf = (uint8_t*)&_vel;

	uint8_t Tx_Data[8] = {
		pbuf[0], pbuf[1], pbuf[2], pbuf[3],
		vbuf[0], vbuf[1], vbuf[2], vbuf[3]
	};

	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &Tx_Header, Tx_Data);
}

/**
 * @brief 速度模式控制电机。
 *
 * @param hfdcan FDCAN的句柄。
 * @param ID 数据帧的ID。
 * @param _vel 速度给定。
 */
void DM4310_Speed_CtrlMotor(FDCAN_HandleTypeDef* hfdcan, uint16_t ID, float _vel)
{
	FDCAN_TxHeaderTypeDef Tx_Header = {
		.Identifier = ID,
		.IdType = FDCAN_STANDARD_ID,
		.TxFrameType = FDCAN_DATA_FRAME,
		.DataLength = FDCAN_DLC_BYTES_4,
		.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
		.BitRateSwitch = FDCAN_BRS_OFF,
		.FDFormat = FDCAN_CLASSIC_CAN,
		.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
		.MessageMarker = 0
	};

	uint8_t* vbuf = (uint8_t*)&_vel;

	uint8_t Tx_Data[4] = {
		vbuf[0], vbuf[1], vbuf[2], vbuf[3]
	};

	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &Tx_Header, Tx_Data);
}

/**
 * @brief 将无符号整数转换为浮点数。
 *
 * @param x_int 要转换的无符号整数。
 * @param x_min 目标浮点数的最小值。
 * @param x_max 目标浮点数的最大值。
 * @param bits 无符号整数的位数。
 * @return 转换后的浮点数。
 */
inline float DM4310_UintToFloat(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

/**
 * @brief 将浮点数转换为无符号整数。
 *
 * @param x 要转换的浮点数。
 * @param x_min 浮点数的最小值。
 * @param x_max 浮点数的最大值。
 * @param bits 无符号整数的位数。
 * @return 转换后的无符号整数。
 */
inline int DM4310_FloatToUint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	return (int)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief CAN回调函数，处理接收到的CAN消息。
 *
 * 该函数根据CAN消息的ID，解析对应电机的数据，并调用检测钩子函数。
 * 如果CAN ID不在预期范围内，则调用错误处理函数。
 *
 * @param can_id CAN消息的ID。
 * @param rx_data 指向接收到的CAN数据的字节数组的指针。
 */
void motor_4310_can_callback(uint32_t can_id, const uint8_t* rx_data)
{
	switch (can_id)
	{
	case FDCAN_DM4310_M1_SLAVE_ID:
		dm_4310_measure_parse(&dm_4310_measure[0], rx_data);
		detect_hook(PITCH_GIMBAL_MOTOR_TOE);
		break;
	case FDCAN_DM4310_M2_SLAVE_ID:
		dm_4310_measure_parse(&dm_4310_measure[1], rx_data);
		break;
	case FDCAN_DM4310_M3_SLAVE_ID:
		dm_4310_measure_parse(&dm_4310_measure[2], rx_data);
		break;
	case FDCAN_DM4310_M4_SLAVE_ID:
		dm_4310_measure_parse(&dm_4310_measure[3], rx_data);
		break;
	default:
		break;
	}
}
