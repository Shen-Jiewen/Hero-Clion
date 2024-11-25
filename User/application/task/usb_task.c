//
// Created by Rick on 24-11-25.
//

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "vofa.h"

_Noreturn void usb_task(void* argument){
	while (1){
		vofa_start();
	}
}