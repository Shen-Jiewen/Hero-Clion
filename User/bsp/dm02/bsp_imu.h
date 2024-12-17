//
// Created by Rick on 24-11-25.
//

#ifndef DM_02_HERO_USER_BSP_DM02_BSP_IMU_H_
#define DM_02_HERO_USER_BSP_DM02_BSP_IMU_H_

#include "main.h"
#include "struct_typedef.h"

extern void imu_pwm_start(void);
extern void imu_pwm_set(uint16_t pwm);
extern uint32_t imu_semaphore_wait(void);

#endif //DM_02_HERO_USER_BSP_DM02_BSP_IMU_H_
