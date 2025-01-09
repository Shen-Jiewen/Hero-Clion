//
// Created by Rick on 2025/1/4.
//

#include "main.h"
#include "cmsis_os.h"

_Noreturn void ui_task(__attribute__((unused)) void* argument){
	while (1){
		osDelay(200);
	}
}