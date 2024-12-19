#include "FreeRTOS.h"
#include "Fusion.h"
#include "semphr.h"
#include "imu.h"

#define SAMPLE_RATE 1000

imu_control_t *imu_control;

_Noreturn void imu_task(void *argument) {
	imu_control = get_imu_control_point();
	imu_control_init(imu_control);
	imu_hardware_init();

	while (1) {
		if (xSemaphoreTake(imu_control->xAccSemaphore, portMAX_DELAY) == pdTRUE &&
			xSemaphoreTake(imu_control->xGyroSemaphore, portMAX_DELAY) == pdTRUE) {

			imu_data_update(imu_control);
			imu_calibration_init(imu_control);
			imu_ahrs_update(imu_control);
			imu_statistics_update(imu_control);
		}
	}
}
