//
// Created by Rick on 2024/12/10.
//

#include "detect.h"
#include "cmsis_os.h"

error_t *error_list;

_Noreturn void detect_task(void *argument){
	static uint32_t system_time;
	system_time = xTaskGetTickCount();
	// detect初始化
	detect_init(system_time);
	// 获取错误检测列表对象
	error_list = get_error_list_point();
	// 空闲一段时间,等待设备和其他任务初始化
	vTaskDelay(DETECT_TASK_INIT_TIME);

	while (1){
		static uint8_t error_num_display = 0;
		system_time = xTaskGetTickCount();

		error_num_display = ERROR_LIST_LENGHT;
		error_list[ERROR_LIST_LENGHT].is_lost = 0;
		error_list[ERROR_LIST_LENGHT].error_exist = 0;

		for(int i = 0; i < ERROR_LIST_LENGHT; i++){
			// 未使能,跳过
			if(error_list[i].enable == 0){
				continue;
			}

			// 判断掉线
			if(system_time - error_list[i].new_time > error_list[i].set_offline_time)
			{
				if (error_list[i].error_exist == 0)
				{
					// 记录错误以及掉线时间
					error_list[i].is_lost = 1;
					error_list[i].error_exist = 1;
					error_list[i].lost_time = system_time;
				}
				// 判断错误优先级,保存优先级最高的错误码
				if (error_list[i].priority > error_list[error_num_display].priority)
				{
					error_num_display = i;
				}

				error_list[ERROR_LIST_LENGHT].is_lost = 1;
				error_list[ERROR_LIST_LENGHT].error_exist = 1;
				// 如果提供解决函数,运行解决函数
				if (error_list[i].solve_lost_fun != NULL)
				{
					error_list[i].solve_lost_fun();
				}
			}else if(system_time - error_list[i].work_time < error_list[i].set_online_time){
				// 刚刚上线,可能存在数据不稳定,只记录报错，不记录设备丢失
				error_list[i].is_lost = 0;
				error_list[i].error_exist = 1;
			}else{
				error_list[i].is_lost = 0;
				// 判断是否存在错误
				if(error_list[i].data_is_error != 0){
					error_list[i].error_exist = 1;
				}else{
					error_list[i].error_exist = 0;
				}
				// 计算频率
				if(error_list[i].new_time > error_list[i].last_time){
					error_list[i].frequency = configTICK_RATE_HZ / (fp32)(error_list[i].new_time - error_list[i].last_time);
				}
			}
		}
		vTaskDelay(DETECT_CONTROL_TIME);
	}
}