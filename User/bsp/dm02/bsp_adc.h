//
// Created by Rick on 24-11-24.
//

#ifndef DM_02_HERO_USER_BSP_DM02_BSP_ADC_H_
#define DM_02_HERO_USER_BSP_DM02_BSP_ADC_H_

#include "main.h"
#include "struct_typedef.h"

extern void init_vrefint_peciprocal(void);
extern fp32 get_template(void);
extern fp32 get_battery_voltage(void);
extern uint8_t get_hardware_version(void);

#endif //DM_02_HERO_USER_BSP_DM02_BSP_ADC_H_
