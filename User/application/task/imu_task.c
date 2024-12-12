#include <sys/cdefs.h>
//
// Created by Rick on 24-11-25.
//

#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "bsp_imu.h"
#include "Fusion.h"

#define SAMPLE_RATE 1000

FusionAhrs ahrs;
FusionOffset offset;
FusionEuler euler;

FusionVector gyroscope;        // replace this with actual gyroscope data in degrees/s
FusionVector accelerometer;    // replace this with actual accelerometer data in g

float temperature = 0;

_Noreturn void imu_task(void *argument)
{
	imu_pwm_start();
	while (BMI088_init()){

	}

	// 定义传感器校准参数
	const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
	const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
	const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
	const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
	//姿态解算初始化
	FusionOffsetInitialise(&offset, SAMPLE_RATE);
	FusionAhrsInitialise(&ahrs);    // Initialise algorithms
	// Set AHRS algorithm settings
	const FusionAhrsSettings settings = {
		.convention = FusionConventionNwu,
		.gain = 10.0f,
		.gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
		.accelerationRejection = 10.0f,
		.magneticRejection = 10.0f,
		.recoveryTriggerPeriod = 2 * SAMPLE_RATE
	};
	FusionAhrsSetSettings(&ahrs, &settings);

	while (1){
		// 获取传感器数据
		BMI088_read(gyroscope.array, accelerometer.array, &temperature);

		// 传感器数据校准
		gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
		accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

		// 更新校准
		gyroscope = FusionOffsetUpdate(&offset, gyroscope);

		// 数据更新
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, (fp32)0.001);
		euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

		// 延时
		osDelay(1);
	}
}