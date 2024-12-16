//
// Created by Rick on 24-11-26.
//

#include "dji_3508.h"

void get_motor_3508_measure(motor_3508_measure_t *ptr, const uint8_t *data) {
	ptr->last_ecd = ptr->ecd; // NOLINT(*-narrowing-conversions)
	ptr->ecd = (uint16_t)((data[0] << 8) | data[1]);
	ptr->speed_rpm = (uint16_t)((data[2] << 8) | data[3]); // NOLINT(*-narrowing-conversions)
	ptr->given_current = (uint16_t)((data[4] << 8) | data[5]); // NOLINT(*-narrowing-conversions)
	ptr->temperature = data[6];
}