#include "imu.h"
#include "BMI088driver.h"
#include "bsp_imu.h"
#include "bsp_gpio.h"
#include "cmsis_os.h"

#define SAMPLE_RATE 1000

// 全局IMU控制指针
static imu_control_t imu_control_instance;

// 中断回调函数声明
static void acc_int_callback(void);
static void gyro_int_callback(void);

/**
 * @brief 初始化IMU控制结构体
 *
 * @param imu_control IMU控制结构体指针
 */
void imu_control_init(imu_control_t *imu_control) {
	// 初始化信号量
	imu_control->xAccSemaphore = xSemaphoreCreateBinary();
	imu_control->xGyroSemaphore = xSemaphoreCreateBinary();

	// 初始化时间戳
	imu_control->last_update_time = 0;
	imu_control->last_second_time = 0;
	imu_control->solve_count = 0;
	imu_control->solves_per_second = 0;

	// 初始化IMU相关变量
	FusionOffsetInitialise(&imu_control->offset, SAMPLE_RATE);
	FusionAhrsInitialise(&imu_control->ahrs);

	// 设置AHRS算法设置
	const FusionAhrsSettings settings = {
		.convention = FusionConventionNwu,
		.gain = 10.0f,
		.gyroscopeRange = 2000.0f, /* 替换为实际陀螺仪范围，单位：°/s */
		.accelerationRejection = 10.0f,
		.magneticRejection = 10.0f,
		.recoveryTriggerPeriod = 2 * SAMPLE_RATE
	};
	FusionAhrsSetSettings(&imu_control->ahrs, &settings);
}

/**
 * @brief 获取IMU控制指针
 *
 * @return imu_control_t* IMU控制结构体指针
 */
imu_control_t *get_imu_control_point(void) {
	return &imu_control_instance;
}

/**
 * @brief 初始化IMU硬件
 */
void imu_hardware_init(void) {
	imu_pwm_start();
	imu_gpio_init();
	while (BMI088_init()) {
		// 等待IMU初始化完成
	}
}

/**
 * @brief 初始化IMU校准参数
 *
 * @param imu_control IMU控制结构体指针
 */
void imu_calibration_init(imu_control_t *imu_control) {
	const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
	const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
	const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
	const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};

	// 传感器数据校准
	imu_control->gyroscope = FusionCalibrationInertial(imu_control->gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
	imu_control->accelerometer = FusionCalibrationInertial(imu_control->accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
}

/**
 * @brief 更新IMU数据
 *
 * @param imu_control IMU控制结构体指针
 */
void imu_data_update(imu_control_t *imu_control) {
	// 获取当前时间戳
	TickType_t current_time = xTaskGetTickCount();
	imu_control->delta_time = (float)(current_time - imu_control->last_update_time) / (float)configTICK_RATE_HZ;
	imu_control->last_update_time = current_time;

	// 更新校准
	imu_control->gyroscope = FusionOffsetUpdate(&imu_control->offset, imu_control->gyroscope);
}

/**
 * @brief 更新AHRS算法
 *
 * @param imu_control IMU控制结构体指针
 */
void imu_ahrs_update(imu_control_t *imu_control) {
	FusionAhrsUpdateNoMagnetometer(&imu_control->ahrs, imu_control->gyroscope, imu_control->accelerometer, imu_control->delta_time);
	imu_control->euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&imu_control->ahrs));
}

/**
 * @brief 更新统计信息
 *
 * @param imu_control IMU控制结构体指针
 */
void imu_statistics_update(imu_control_t *imu_control) {
	imu_control->solve_count++;

	// 每秒钟更新一次解算次数
	TickType_t current_time = xTaskGetTickCount();
	if (current_time - imu_control->last_second_time >= configTICK_RATE_HZ) {
		imu_control->solves_per_second = imu_control->solve_count;
		imu_control->solve_count = 0;
		imu_control->last_second_time = current_time;
	}
}

/**
 * @brief 加速度计中断回调函数
 */
static void acc_int_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// 读取加速度计数据
	BMI088_read_accel(imu_control_instance.accelerometer.array);

	// 在中断中释放信号量
	xSemaphoreGiveFromISR(imu_control_instance.xAccSemaphore, &xHigherPriorityTaskWoken);

	// 如果需要，触发上下文切换
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief 陀螺仪中断回调函数
 */
static void gyro_int_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// 读取陀螺仪数据
	BMI088_read_gyro(imu_control_instance.gyroscope.array);

	// 在中断中释放信号量
	xSemaphoreGiveFromISR(imu_control_instance.xGyroSemaphore, &xHigherPriorityTaskWoken);

	// 如果需要，触发上下文切换
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief GPIO初始化函数
 */
void imu_gpio_init(void) {
	// 配置加速度计中断引脚
	BSP_GPIO_InitTypeDef acc_int_config = {
		.port = ACC_INT_GPIO_Port,
		.pin = ACC_INT_Pin,
		.mode = BSP_GPIO_MODE_EXTI,
		.pull = GPIO_NOPULL,
		.speed = GPIO_SPEED_FREQ_LOW,
		.irqn = ACC_INT_EXTI_IRQn
	};
	BSP_GPIO_Init(&acc_int_config);

	// 配置陀螺仪中断引脚
	BSP_GPIO_InitTypeDef gyro_int_config = {
		.port = GYRO_INT_GPIO_Port,
		.pin = GYRO_INT_Pin,
		.mode = BSP_GPIO_MODE_EXTI,
		.pull = GPIO_NOPULL,
		.speed = GPIO_SPEED_FREQ_LOW,
		.irqn = GYRO_INT_EXTI_IRQn
	};
	BSP_GPIO_Init(&gyro_int_config);

	// 注册回调函数
	BSP_GPIO_RegisterCallback(ACC_INT_Pin, acc_int_callback);
	BSP_GPIO_RegisterCallback(GYRO_INT_Pin, gyro_int_callback);
}