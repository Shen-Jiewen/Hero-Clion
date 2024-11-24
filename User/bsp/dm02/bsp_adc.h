//
// Created by Rick on 24-11-24.
//

#ifndef DM_02_HERO_USER_BSP_DM02_BSP_ADC_H_
#define DM_02_HERO_USER_BSP_DM02_BSP_ADC_H_

#include "main.h"
#include "struct_typedef.h"

extern void init_vrefint_reciprocal(void);
extern fp32 get_temperature(void);
extern fp32 get_battery_voltage(void);

#endif //DM_02_HERO_USER_BSP_DM02_BSP_ADC_H_
