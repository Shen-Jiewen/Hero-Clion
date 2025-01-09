//
// Created by Rick on 2024/12/10.
//

#include "chassis.h"
#include "cmsis_os.h"
#include "detect.h"

chassis_control_t *chassis_control;

_Noreturn void chassis_task(__attribute__((unused)) void* argument)
{
	// 空闲一段时间
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	// 获取底盘对象
	chassis_control = get_chassis_control_point();
	// 底盘初始化
	chassis_init(chassis_control);
	// 判断底盘都在线
	if (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE)
		|| toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
	{
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}

	while (1)
	{
		// 设置底盘的控制模式
		chassis_set_mode(chassis_control);
		// 模式切换数据保存
		chassis_mode_change_control_transit(chassis_control);
		// 底盘数据更新
		chassis_feedback_update(chassis_control);
		// 底盘控制量设置
		chassis_set_control(chassis_control);
		// 底盘控制PID计算
		chassis_control_loop(chassis_control);
		// 确保至少一个电机在线,这样CAN控制包也可以被接收到
		if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE)
			&& toe_is_error(CHASSIS_MOTOR4_TOE)))
		{
			// 当遥控器掉线的时候,发送给底盘电机零电流
			if (toe_is_error(DBUS_TOE))
			{
				chassis_control->CAN_cmd_chassis(0, 0, 0, 0);
			}
			else
			{
//				chassis_control->CAN_cmd_chassis(chassis_control->motor_chassis[0].give_current,
//					chassis_control->motor_chassis[1].give_current,
//					chassis_control->motor_chassis[2].give_current,
//					chassis_control->motor_chassis[3].give_current);
			}
		}
		osDelay(CHASSIS_CONTROL_TIME_MS);
	}
}