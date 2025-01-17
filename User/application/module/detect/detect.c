//
// Created by Rick on 2024/12/10.
//

#include "detect.h"

static error_t error_list[ERROR_LIST_LENGHT + 1];

void detect_init(uint32_t time)
{
	//设置离线时间，上线稳定工作时间，优先级 offlineTime online time priority
	uint16_t set_item[ERROR_LIST_LENGHT][3] =
		{
			{30, 40, 15},   //SBUS
			{10, 10, 11},   //motor1
			{10, 10, 10},   //motor2
			{10, 10, 9},    //motor3
			{10, 10, 8},    //motor4
			{2, 3, 14},     //yaw
			{2, 3, 13},     //pitch
			{10, 10, 12},   //trigger
			{2, 3, 7},      //board gyro
			{5, 5, 7},      //board accel
			{40, 200, 7},   //board mag
			{100, 100, 5},  //referee
			{10, 10, 7},    //rm imu
			{100, 100, 1},  //oled
		};

	for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
	{
		error_list[i].set_offline_time = set_item[i][0];
		error_list[i].set_online_time = set_item[i][1];
		error_list[i].priority = set_item[i][2];
		error_list[i].data_is_error_fun = NULL;
		error_list[i].solve_lost_fun = NULL;
		error_list[i].solve_data_error_fun = NULL;

		error_list[i].enable = 1;
		error_list[i].error_exist = 1;
		error_list[i].is_lost = 1;
		error_list[i].data_is_error = 1;
		error_list[i].frequency = 0.0f;
		error_list[i].new_time = time;
		error_list[i].last_time = time;
		error_list[i].lost_time = time;
		error_list[i].work_time = time;
	}

	error_list[OLED_TOE].data_is_error_fun = NULL;
	error_list[OLED_TOE].solve_lost_fun = NULL;
	error_list[OLED_TOE].solve_data_error_fun = NULL;
}

/**
  * @brief          记录时间
  * @param[in]      toe:设备目录
  * @retval         none
  */
void detect_hook(uint8_t toe)
{
	error_list[toe].last_time = error_list[toe].new_time;
	error_list[toe].new_time = xTaskGetTickCount();

	if (error_list[toe].is_lost)
	{
		error_list[toe].is_lost = 0;
		error_list[toe].work_time = error_list[toe].new_time;
	}

	if (error_list[toe].data_is_error_fun != NULL)
	{
		if (error_list[toe].data_is_error_fun())
		{
			error_list[toe].error_exist = 1;
			error_list[toe].data_is_error = 1;

			if (error_list[toe].solve_data_error_fun != NULL)
			{
				error_list[toe].solve_data_error_fun();
			}
		}
		else
		{
			error_list[toe].data_is_error = 0;
		}
	}
	else
	{
		error_list[toe].data_is_error = 0;
	}
}

/**
  * @brief          获取设备对应的错误状态
  * @param[in]      toe:设备目录
  * @retval         true(错误) 或者false(没错误)
  */
bool_t toe_is_error(uint8_t toe)
{
	return (error_list[toe].error_exist == 1);
}

error_t *get_error_list_point(void)
{
	return error_list;
}

