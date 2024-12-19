//
// Created by Rick on 24-11-25.
//

#include "cmsis_os.h"
#include "vofa.h"
#include "Fusion.h"
#include "dt7.h"
#include "chassis.h"
#include "imu.h"


_Noreturn void usb_task(void* argument){
	RC_ctrl_t *usb_dt7_ctrl = get_dt7_point();
	imu_control_t *imu_control = get_imu_control_point();
	const motor_6020_measure_t *motor0 = get_motor_6020_measure_point(1);

	// 初始化
	while (1){
		vofa_send_data(0, imu_control->euler.angle.pitch);
		vofa_send_data(1, imu_control->euler.angle.yaw);
		vofa_send_data(2, imu_control->euler.angle.roll);
		vofa_send_data(3, usb_dt7_ctrl->rc.ch[0]);
		vofa_send_data(4, usb_dt7_ctrl->rc.ch[1]);
		vofa_send_data(5, usb_dt7_ctrl->rc.ch[2]);
		vofa_send_data(6, usb_dt7_ctrl->rc.ch[3]);
		vofa_send_data(7, usb_dt7_ctrl->rc.ch[4]);
		vofa_send_data(8, usb_dt7_ctrl->rc.s[0]);
		vofa_send_data(9, usb_dt7_ctrl->rc.s[1]);
		vofa_send_data(10, motor0->ecd);
		vofa_send_data(11, motor0->last_ecd);
		vofa_send_data(12, motor0->temperature);
		vofa_send_data(13, motor0->speed_rpm);
		vofa_send_data(14, motor0->given_current);
		vofa_send_data(15, imu_control->solves_per_second);
		vofa_sendframetail();
		osDelay(10);
	}
}