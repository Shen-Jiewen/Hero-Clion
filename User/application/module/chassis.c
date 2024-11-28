//
// Created by Rick on 24-11-26.
//

#include "chassis.h"

extern FDCAN_HandleTypeDef hfdcan1;

static chassis_control_t chassis_control;

static void FDCAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4){
	FDCAN_TxHeaderTypeDef txHeader;
	uint8_t txData[8]; // 数据缓存

	// 配置 FDCAN 消息头
	txHeader.Identifier = CAN_CHASSIS_ALL_ID;  // 示例标准 ID (根据实际需求修改)
	txHeader.IdType = FDCAN_STANDARD_ID;       // 标准帧 (11位 ID)
	txHeader.TxFrameType = FDCAN_DATA_FRAME;   // 数据帧
	txHeader.DataLength = FDCAN_DLC_BYTES_8;   // 数据长度为 8 字节
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;    // 无速率切换（经典 CAN）
	txHeader.FDFormat = FDCAN_FD_CAN;     	   // 使用经典 CAN 帧格式
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
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK) {
		// 如果发送失败，处理错误
		Error_Handler();
	}
}

static void CAN_rec_chassis(uint32_t can_id, const uint8_t* rx_data){
	switch (can_id)
	{
	case CAN_3508_M1_ID:
		get_motor_3508_measure((motor_3508_measure_t *)(&chassis_control.motor_chassis[0].motor_3508_measure), rx_data);
		break;
	case CAN_3508_M2_ID:
		get_motor_3508_measure((motor_3508_measure_t *)(&chassis_control.motor_chassis[1].motor_3508_measure), rx_data);
		break;
	case CAN_3508_M3_ID:
		get_motor_3508_measure((motor_3508_measure_t *)(&chassis_control.motor_chassis[2].motor_3508_measure), rx_data);
		break;
	case CAN_3508_M4_ID:
		get_motor_3508_measure((motor_3508_measure_t *)(&chassis_control.motor_chassis[3].motor_3508_measure), rx_data);
		break;
	default:
		break;
	}
}

chassis_control_t* get_chassis_control_point(void){
	return &chassis_control;
}

static chassis_control_t chassis_control = {
	.CAN_cmd_chassis = FDCAN_cmd_chassis
};