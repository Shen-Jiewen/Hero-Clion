#include "user_lib.h"
#include "arm_math.h"

/**
 * @brief 快速平方根倒数计算（牛顿迭代法）
 * @param num 输入的浮点数
 * @return 平方根倒数的近似值
 */
fp32 invSqrt(fp32 num)
{
	static const fp32 HALF = 0.5f;
	static const fp32 MAGIC_NUMBER = 1.5f;

	fp32 x = num;
	long i = *(long *)&x;
	i = 0x5f3759df - (i >> 1);
	x = *(fp32 *)&i;
	x *= MAGIC_NUMBER - (HALF * num * x * x);

	return x;
}

/**
 * @brief 斜波函数初始化
 * @param ramp_source_type 斜波函数结构体指针
 * @param frame_period 控制周期（单位：秒）
 * @param max 最大值限制
 * @param min 最小值限制
 */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
	ramp_source_type->frame_period = frame_period;
	ramp_source_type->max_value = max;
	ramp_source_type->min_value = min;
	ramp_source_type->input = 0.0f;
	ramp_source_type->out = 0.0f;
}

/**
 * @brief 斜波函数计算，根据输入值进行线性变化
 * @param ramp_source_type 斜波函数结构体指针
 * @param input 输入值，单位为 /s
 */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
{
	ramp_source_type->input = input;
	ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;

	ramp_source_type->out = fp32_constrain(
		ramp_source_type->out,
		ramp_source_type->min_value,
		ramp_source_type->max_value
	);
}

/**
 * @brief 一阶低通滤波初始化
 * @param first_order_filter_type 滤波器结构体指针
 * @param frame_period 控制周期（单位：秒）
 * @param num 滤波参数
 */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
	first_order_filter_type->frame_period = frame_period;
	first_order_filter_type->num[0] = num[0];
	first_order_filter_type->input = 0.0f;
	first_order_filter_type->out = 0.0f;
}

/**
 * @brief 一阶低通滤波计算
 * @param first_order_filter_type 滤波器结构体指针
 * @param input 输入值
 */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
	fp32 denominator = first_order_filter_type->num[0] + first_order_filter_type->frame_period;
	fp32 output_coeff = first_order_filter_type->num[0] / denominator;
	fp32 input_coeff = first_order_filter_type->frame_period / denominator;

	first_order_filter_type->input = input;
	first_order_filter_type->out = output_coeff * first_order_filter_type->out + input_coeff * input;
}

/**
 * @brief 绝对值限幅
 * @param num 待限幅的数值指针
 * @param Limit 限幅值
 */
void abs_limit(fp32 *num, fp32 Limit)
{
	*num = (*num > Limit) ? Limit :
		   (*num < -Limit) ? -Limit : *num;
}

/**
 * @brief 获取数值符号
 * @param value 输入值
 * @return 符号（1.0f 或 -1.0f）
 */
fp32 sign(fp32 value)
{
	return (value >= 0.0f) ? 1.0f : -1.0f;
}

/**
 * @brief 浮点数死区处理
 * @param Value 输入值
 * @param minValue 最小值
 * @param maxValue 最大值
 * @return 处理后的值
 */
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
	return ((Value < maxValue && Value > minValue) ? 0.0f : Value);
}

/**
 * @brief 16位整数死区处理
 * @param Value 输入值
 * @param minValue 最小值
 * @param maxValue 最大值
 * @return 处理后的值
 */
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
	if (Value < maxValue && Value > minValue)
	{
		return 0;
	}
	else
	{
		return Value;
	}
}

/**
 * @brief 浮点数限幅
 * @param Value 输入值
 * @param minValue 最小值
 * @param maxValue 最大值
 * @return 限幅后的值
 */
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
	return (Value < minValue) ? minValue :
		   (Value > maxValue) ? maxValue : Value;
}

/**
 * @brief 16位整数限幅
 * @param Value 输入值
 * @param minValue 最小值
 * @param maxValue 最大值
 * @return 限幅后的值
 */
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
	if (Value > maxValue)
	{
		if (Value < minValue)
		{
			return minValue;
		}
		else
		{
			return maxValue;
		}
	}
	else
	{
		if (Value < minValue)
		{
			return minValue;
		}
		else
		{
			return Value;
		}
	}
}

/**
 * @brief 浮点数循环限幅
 * @param Input 输入值
 * @param minValue 最小值
 * @param maxValue 最大值
 * @return 循环限幅后的值
 */
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
	// 快速处理无效区间情况
	if (maxValue < minValue) return Input;

	// 计算区间长度
	fp32 len = maxValue - minValue;

	// 先减去最小值归一化，取模后再加回最小值
	Input = fmodf(Input - minValue, len) + minValue;

	// 处理负数情况
	if (Input < minValue) Input += len;

	return Input;
}

/**
 * @brief 角度格式化到-180~180范围
 * @param Ang 输入角度
 * @return 格式化后的角度
 */
fp32 theta_format(fp32 Ang)
{
	return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}