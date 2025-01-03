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

	while (1)
	{


	}
}