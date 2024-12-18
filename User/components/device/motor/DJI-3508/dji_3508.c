//
// Created by Rick on 24-11-26.
//

#include "dji_3508.h"

static motor_3508_measure_t motor_3508_measure[8] = {0};

void get_motor_3508_measure(motor_3508_measure_t *ptr, const uint8_t *data) {
	ptr->last_ecd = ptr->ecd; // NOLINT(*-narrowing-conversions)
	ptr->ecd = (uint16_t)((data[0] << 8) | data[1]);
	ptr->speed_rpm = (uint16_t)((data[2] << 8) | data[3]); // NOLINT(*-narrowing-conversions)
	ptr->given_current = (uint16_t)((data[4] << 8) | data[5]); // NOLINT(*-narrowing-conversions)
	ptr->temperature = data[6];
}

const motor_3508_measure_t* get_motor_3508_measure_point(uint8_t i){
	return &motor_3508_measure[i];
}

void motor_3505_can_callback(uint32_t can_id, const uint8_t* rx_data)
{
	switch (can_id)
	{
	case CAN_3508_M1_ID:
		get_motor_3508_measure(&motor_3508_measure[0], rx_data);
		detect_hook(CHASSIS_MOTOR1_TOE);
		break;
	case CAN_3508_M2_ID:
		get_motor_3508_measure(&motor_3508_measure[1], rx_data);
		detect_hook(CHASSIS_MOTOR2_TOE);
		break;
	case CAN_3508_M3_ID:
		get_motor_3508_measure(&motor_3508_measure[2], rx_data);
		detect_hook(CHASSIS_MOTOR3_TOE);
		break;
	case CAN_3508_M4_ID:
		get_motor_3508_measure(&motor_3508_measure[3], rx_data);
		detect_hook(CHASSIS_MOTOR4_TOE);
		break;
	case CAN_3508_M5_ID:
		get_motor_3508_measure(&motor_3508_measure[4], rx_data);
		break;
	case CAN_3508_M6_ID:
		get_motor_3508_measure(&motor_3508_measure[5], rx_data);
		break;
	case CAN_3508_M7_ID:
		get_motor_3508_measure(&motor_3508_measure[6], rx_data);
		break;
	case CAN_3508_M8_ID:
		get_motor_3508_measure(&motor_3508_measure[7], rx_data);
		break;
	default:
		Error_Handler();
		break;
	}
}
