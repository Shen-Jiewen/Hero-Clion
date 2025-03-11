//
// Created by Rick on 24-11-24.
//

#include "bsp_adc.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

// 根据VREFINT参考电压获取到的校准值
volatile uint32_t vref_voltage = 3300;

/**
  * @brief          对指定ADC的指定通道进行采样
  * @author         Javen
  * @param[in]      adc句柄
  * @param[in]      adc通道号
  * @retval         返回获取到的采样值
  */
static uint32_t adcx_get_chx_value(ADC_HandleTypeDef* ADCx, uint32_t ch, uint32_t sample_time)
{
	static ADC_ChannelConfTypeDef sConfig = { 0 };
	uint32_t adcValue = 0;

	sConfig.Channel = ch;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = sample_time;                // 推荐采用较长的采样时间
	sConfig.SingleDiff = ADC_SINGLE_ENDED;            // 单端模式
	sConfig.OffsetNumber = ADC_OFFSET_NONE;        // 无偏移
	sConfig.Offset = 0;

	if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	// 启动ADC转换
	HAL_ADC_Start(ADCx);
	// 以阻塞的方式等待转换完成,超时时间设置为10ms
	if (HAL_ADC_PollForConversion(ADCx, 10) == HAL_OK)
	{
		adcValue = HAL_ADC_GetValue(ADCx);
	}
	// 关闭ADC转换
	HAL_ADC_Stop(ADCx);

	return adcValue;
}

/**
  * @brief          对内部参考电压电压 VREFINT 进行 adc 采样将其作为校准值
  * @author         Javen
  * @retval         返回空
  */
void init_vrefint_reciprocal(void)
{
	uint8_t i = 0;
	uint32_t vref_adc = 0;
	uint32_t vref_cal = *VREFINT_CAL_ADDR;

	vref_adc = adcx_get_chx_value(&hadc3, ADC_CHANNEL_VREFINT, ADC3_SAMPLETIME_640CYCLES_5);

	vref_voltage = (vref_cal * VREFINT_CAL_VREF) / vref_adc; // NOLINT(*-narrowing-conversions)
}

/**
  * @brief          通过 ADC 获得板载的温度传感器的温度值
  * @author         Javen
  * @retval         返回经过校准值校验的板载温度传感器温度
  */
fp32 get_temperature(void)
{
	uint16_t adcx = 0;
	uint16_t ts_cal1 = (*TEMPSENSOR_CAL1_ADDR); // 校准值 @ 30°C
	uint16_t ts_cal2 = (*TEMPSENSOR_CAL2_ADDR); // 校准值 @ 110°C

	adcx = adcx_get_chx_value(&hadc3, ADC_CHANNEL_TEMPSENSOR, ADC_SAMPLETIME_387CYCLES_5);
	// 计算温度值（线性插值公式）
//	temperature = ((fp32)adcx - ts_cal1) * (110.0f - 30.0f) / (ts_cal2 - ts_cal1) + 30.0f; // NOLINT(*-narrowing-conversions)
	fp32 temperature = __HAL_ADC_CALC_TEMPERATURE_TYP_PARAMS( // NOLINT(*-integer-division)
		4300, // Avg_Slope: µV/°C
		760, // V30: 30°C 时的典型电压（单位：mV）
		30, // 校准温度（30°C）
		vref_voltage, // 当前参考电压（单位：mV）
		adcx, // 温度传感器的 ADC 值
		ADC_RESOLUTION_12B // ADC 分辨率（这里是 12 位）
	);

	return temperature;
}

/**
  * @brief          通过 ADC 对经过了分压电路的电池电压值进行采样
  * @author         Javen
  * @retval         返回经过校准值校验的电池电压
  */
fp32 get_battery_voltage(void)
{
	fp32 voltage;
	uint16_t adcx = 0;

	adcx = adcx_get_chx_value(&hadc1, ADC_CHANNEL_4, ADC_SAMPLETIME_810CYCLES_5);

	//校准
	voltage = (fp32)(adcx * 3.3f / 65535) * 11.00009f; // NOLINT(*-narrowing-conversions)

	return voltage;
}

