//
// Created by Rick on 24-11-24.
//

#include "bsp_adc.h"

extern ADC_HandleTypeDef  hadc1;

// 根据VREFINT参考电压获取到的校准值
volatile fp32 voltage_vrefint_proportion = 0.f;

static uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch){
	static ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ch;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;



	return (uint16_t)HAL_ADC_GetValue(ADCx);
}