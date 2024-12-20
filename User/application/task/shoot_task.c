//
// Created by Rick on 2024/12/20.
//

#include "cmsis_os.h"
#include "shoot.h"

_Noreturn void shoot_task(void* argument) {
	while (1){
		osDelay(100);
	}
}