//
// Created by Rick on 24-11-24.
//

#ifndef BSP_ADC_H_
#define BSP_ADC_H_

#include "main.h"
#include "struct_typedef.h"

extern void init_vrefint_reciprocal(void);
extern fp32 get_temperature(void);
extern fp32 get_battery_voltage(void);

#endif //BSP_ADC_H_
