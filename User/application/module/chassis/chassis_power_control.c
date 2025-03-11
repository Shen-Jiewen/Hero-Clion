/**
******************************************************************************
  * @file    chassis_power_control.c
  * @brief   底盘功率控制模块实现
  * @note
  * 使用说明：
  * 1. 初始化要求：
  *    - 调用前需初始化各电机CAN接口和PID控制器
  *    - 配置power_buffer_pid参数（通常为PI控制器）
  *    - 设置正确的能量阈值和功率加成参数
  *
  * 2. 接口说明：
  *    - chassis_power_control()为主入口函数，需在控制循环中周期性调用
  *    - 需实现get_chassis_power_and_buffer()获取当前功率状态
  *    - 需对接电容控制模块CAP_CAN_DataSend()
  *
  * 3. 参数整定指南：
  *    - PID参数：根据系统响应调节功率缓冲PID，建议Kp=0.5, Ki=0.1
  *    - 功率缓冲目标POWER_BUFFER_TARGET：通常设为5-10%最大功率
  *    - 能量阈值（CAP_ENERGY_xx_THRES）：根据电容特性设置
  *    - 功率加成配置：根据比赛策略调整各能量区间的加成值
  *    - 电机功率模型参数：需通过实验标定
  *
  * 4. 工作流程：
  *    获取功率状态 → 计算输入功率 → 电容控制 → 确定最大功率 → 功率分配
  ******************************************************************************
  */

#include "chassis_power_control.h"
#include "referee.h"

extern FDCAN_HandleTypeDef hfdcan1;

/* 常量定义 ----------------------------------------------------------------*/
#define MAX_MOTOR_TORQUE          16000.0f    // 电机最大扭矩输出值
#define POWER_BUFFER_TARGET       50.0f       // 功率缓冲目标值
#define CAP_ENERGY_HIGH_THRES     60.0f       // 高能量阈值（%）
#define CAP_ENERGY_MID_THRES      40.0f       // 中能量阈值（%）
#define CAP_ENERGY_LOW_THRES      30.0f       // 低能量阈值（%）

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

// 不同能量状态下的功率加成配置
typedef struct {
    float normal_mode_add; // 正常模式功率加成
    float boost_mode_add; // 暴走模式功率加成
} PowerBoostConfig;

static const PowerBoostConfig POWER_BOOST_CONFIG[] = {
    [0] = {50.0f, 80.0f}, // 高能量区间配置
    [1] = {30.0f, 40.0f}, // 中高能量区间配置
    [2] = {10.0f, 10.0f}, // 中低能量区间配置
    [3] = {0.0f, 0.0f} // 低能量区间配置
};

/* 私有函数声明 ------------------------------------------------------------*/
static float calculate_input_power(float max_power_limit, float chassis_power);

static float determine_chassis_max_power(float input_power, uint8_t cap_state, float cap_energy);

static void calculate_motor_powers(chassis_control_t *control, float *powers, float *total_power);

static void scale_motor_powers(chassis_control_t *control, const float *powers, float total_power, float max_power);

/**
  * @brief 底盘功率主控制函数
  * @param chassis_power_control 底盘控制结构体指针
  */
void chassis_power_control(chassis_control_t *chassis_power_control) {
    /* 初始化局部变量 */
    float chassis_power = 0.0f;
    float chassis_power_buffer = 0.0f;
    float max_power_limit = 40.0f;
    float initial_power_per_motor[4] = {0};
    float total_motor_power = 0.0f;

    /* 步骤1：获取底盘功率状态 */
    get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer, &max_power_limit);

    /* 步骤2：计算输入功率 */
    float input_power = calculate_input_power(max_power_limit, chassis_power);

    /* 步骤3：电容器控制 */
    CAP_CAN_DataSend(&hfdcan1, input_power, ENABLE_CAP);

    /* 步骤4：确定底盘最大功率 */
    uint8_t boost_mode = (chassis_power_control->chassis_RC->key.v & KEY_SPEED_UP) ? 1 : 0;
    float chassis_max_power = determine_chassis_max_power(
        input_power,
        boost_mode,
        CAP_CANData.cap_energy
    );

    /* 步骤5：计算电机功率需求 */
    calculate_motor_powers(chassis_power_control, initial_power_per_motor, &total_motor_power);

    /* 步骤6：功率限制与分配 */
    if (total_motor_power > chassis_max_power) {
        scale_motor_powers(chassis_power_control, initial_power_per_motor, total_motor_power, chassis_max_power);
    }
}

/**
  * @brief 计算输入功率
  * @param max_power_limit 最大功率限制
  * @param chassis_power 当前底盘功率
  * @return 计算后的输入功率
  */
static float calculate_input_power(float max_power_limit, float chassis_power) {
    const float POWER_FILTER_GAIN = 0.5f;

    if (chassis_power < 0.9f * max_power_limit) {
        return (1.0f - POWER_FILTER_GAIN) * (max_power_limit - chassis_power) + max_power_limit;
    }
    return POWER_FILTER_GAIN * (max_power_limit - chassis_power) + max_power_limit;
}

/**
  * @brief 确定底盘最大可用功率
  * @param input_power 基础输入功率
  * @param cap_state 电容器状态（暴走模式）
  * @param cap_energy 电容器剩余能量（%）
  * @return 最大可用功率
  */
static float determine_chassis_max_power(float input_power, uint8_t cap_state, float cap_energy) {
    uint8_t energy_level = 0;

    /* 确定能量区间 */
    if (cap_energy > CAP_ENERGY_HIGH_THRES) {
        energy_level = 0;
    } else if (cap_energy > CAP_ENERGY_MID_THRES) {
        energy_level = 1;
    } else if (cap_energy > CAP_ENERGY_LOW_THRES) {
        energy_level = 2;
    } else {
        energy_level = 3;
    }

    /* 获取功率加成配置 */
    float power_boost = cap_state
                            ? POWER_BOOST_CONFIG[energy_level].boost_mode_add
                            : POWER_BOOST_CONFIG[energy_level].normal_mode_add;

    return input_power + power_boost;
}

/**
  * @brief 计算各电机初始功率需求
  * @param control 底盘控制结构体指针
  * @param powers 电机功率数组（输出）
  * @param total_power 总功率（输出）
  */
static void calculate_motor_powers(chassis_control_t *control, float *powers, float *total_power) {
    const float TORQUE_COEFF = 1.99688994e-6f;
    const float K2 = 1.453e-07f;
    const float A = 1.23e-07f;
    const float CONST_TERM = 4.081f;

    *total_power = 0.0f;

    for (uint8_t i = 0; i < 4; i++) {
        motor_3508_t *motor = &control->motor_chassis[i];
        pid_type_def *pid = &control->motor_speed_pid[i];

        /* 计算单电机功率：P = τ*ω + k2*ω² + a*τ² + const */
        powers[i] = pid->out * TORQUE_COEFF * motor->speed
                    + K2 * motor->speed * motor->speed
                    + A * pid->out * pid->out
                    + CONST_TERM;

        /* 忽略负功率（暂态过程） */
        if (powers[i] > 0.0f) {
            *total_power += powers[i];
        }
    }
}

/**
  * @brief 缩放电机功率并进行扭矩计算
  * @param control 底盘控制结构体指针
  * @param powers 原始功率数组
  * @param total_power 原始总功率
  * @param max_power 允许的最大功率
  */
static void scale_motor_powers(chassis_control_t *control, const float *powers, float total_power, float max_power) {
    const float TORQUE_COEFF = 1.99688994e-6f;
    const float K2 = 1.453e-07f;
    const float A = 1.23e-07f;

    float scale_factor = max_power / total_power;

    for (uint8_t i = 0; i < 4; i++) {
        pid_type_def *pid = &control->motor_speed_pid[i];
        motor_3508_t *motor = &control->motor_chassis[i];

        /* 计算缩放后功率 */
        float scaled_power = powers[i] * scale_factor;
        if (scaled_power < 0.0f) continue;

        /* 构建二次方程：aτ² + (k1ω)τ + (k2ω² - P_scaled + C) = 0 */
        float b = TORQUE_COEFF * motor->speed;
        float c = K2 * motor->speed * motor->speed - scaled_power + 4.081f;

        /* 根据原始扭矩方向选择解 */
        float discriminant = b * b - 4 * A * c;
        if (discriminant < 0.0f) {
            pid->out = 0.0f; // 无效解处理
            continue;
        }

        float sqrt_d = sqrtf(discriminant);
        float temp = (pid->out > 0) ? (-b + sqrt_d) / (2 * A) : (-b - sqrt_d) / (2 * A);

        /* 扭矩限幅 */
        temp = CLAMP(temp, -MAX_MOTOR_TORQUE, MAX_MOTOR_TORQUE);
        pid->out = temp;
    }
}
