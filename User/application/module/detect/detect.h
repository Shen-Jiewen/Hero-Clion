//
// Created by Rick on 2024/12/10.
//

#ifndef DM_02_HERO_USER_APPLICATION_MODULE_DETECT_DETECT_H_
#define DM_02_HERO_USER_APPLICATION_MODULE_DETECT_DETECT_H_

#include "main.h"
#include "struct_typedef.h"

#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10

//错误码以及对应设备顺序
enum errorList
{
	DBUS_TOE = 0,
	CHASSIS_MOTOR1_TOE,
	CHASSIS_MOTOR2_TOE,
	CHASSIS_MOTOR3_TOE,
	CHASSIS_MOTOR4_TOE,
	YAW_GIMBAL_MOTOR_TOE,
	PITCH_GIMBAL_MOTOR_TOE,
	DOWN_TRIGGER_MOTOR_TOE,
	BOARD_GYRO_TOE,
	BOARD_ACCEL_TOE,
	BOARD_MAG_TOE,
	REFEREE_TOE,
	RM_IMU_TOE,
	OLED_TOE,
	ERROR_LIST_LENGHT,
};

typedef struct __attribute((packed))
{
	uint32_t new_time;
	uint32_t last_time;
	uint32_t lost_time;
	uint32_t work_time;
	uint16_t set_offline_time : 12;
	uint16_t set_online_time : 12;
	uint8_t enable : 1;
	uint8_t priority : 4;
	uint8_t error_exist : 1;
	uint8_t is_lost : 1;
	uint8_t data_is_error : 1;

	fp32 frequency;
	bool_t (*data_is_error_fun)(void);
	void (*solve_lost_fun)(void);
	void (*solve_data_error_fun)(void);
} error_t;


extern void detect_init(uint32_t time);

/**
  * @brief          获取设备对应的错误状态
  * @param[in]      toe:设备目录
  * @retval         true(错误) 或者false(没错误)
  */
extern bool_t toe_is_error(uint8_t err);

/**
  * @brief          记录时间
  * @param[in]      toe:设备目录
  * @retval         none
  */
extern void detect_hook(uint8_t toe);

/**
  * @brief          得到错误列表
  * @param[in]      none
  * @retval         error_list的指针
  */
extern error_t *get_error_list_point(void);

#endif //DM_02_HERO_USER_APPLICATION_MODULE_DETECT_DETECT_H_
