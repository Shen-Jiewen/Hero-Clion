//
// Created by Rick on 2024/12/20.
//

#include "cmsis_os.h"
#include "shoot.h"

shoot_control_t* shoot_control;

_Noreturn void shoot_task(void* argument)
{
	//等待上电后一段时间使能拨盘达妙电机
	osDelay(SHOOT_TASK_INIT_TIME);
	//获取发射机构对象
	shoot_control = get_shoot_control_point();
	//发射机构初始化
	shoot_init(shoot_control);

	//判断电机是否在线
	while (toe_is_error(DOWN_TRIGGER_MOTOR_TOE)) {
		osDelay(SHOOT_CONTROL_TIME);
		shoot_feedback_update(shoot_control);
	}
	while (1)
	{
		//设置发射机构状态机
		shoot_set_mode();

		//更新发射机构数据
		shoot_feedback_update(shoot_control);

		//设置发射机构控制量
		shoot_set_control(shoot_control);
	}
}