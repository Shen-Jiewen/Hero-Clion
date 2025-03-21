//
// Created by Rick on 2024/12/20.
//

#include "cmsis_os.h"
#include "shoot.h"

shoot_control_t* shoot_control;

_Noreturn void shoot_task(__attribute__((unused)) void* argument)
{
	//等待上电后一段时间使能拨盘达妙电机
	osDelay(SHOOT_TASK_INIT_TIME);
	//获取发射机构对象
	shoot_control = get_shoot_control_point();
	//发射机构初始化
	shoot_init(shoot_control);

	//判断是否允许开始任务循环
	while (toe_is_error(DOWN_TRIGGER_MOTOR_TOE)||
		toe_is_error(DBUS_TOE) ||
		toe_is_error(FRIC_MOTOR1_TOE) ||
		toe_is_error(FRIC_MOTOR2_TOE) ||
		toe_is_error(FRIC_MOTOR3_TOE) ||
		toe_is_error(FRIC_MOTOR4_TOE)
		) {
		osDelay(SHOOT_CONTROL_TIME);
		shoot_feedback_update(shoot_control);
	}
	while (1)
	{
		//设置发射机构状态机
		shoot_set_mode(shoot_control);

		//更新发射机构数据
		shoot_feedback_update(shoot_control);

		//设置发射机构控制量
		shoot_set_control(shoot_control);

		//发射机构PID计算
		shoot_control_loop(shoot_control);

		// 发送CAN数据
		if (!toe_is_error(DOWN_TRIGGER_MOTOR_TOE)) {
			if (toe_is_error(DBUS_TOE)) {
				shoot_control->CAN_cmd_shoot(0,0,0,0);
			}
			else {
				shoot_control->CAN_cmd_shoot(shoot_control->friction_motor[0].give_current,
				shoot_control->friction_motor[1].give_current,
				shoot_control->friction_motor[2].give_current,
				shoot_control->friction_motor[3].give_current);
			}
		}
	}
}