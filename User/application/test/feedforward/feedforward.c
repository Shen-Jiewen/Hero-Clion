/**
 * @file feedforward.c
 * @brief 云台前馈控制模块实现文件
 * @details 该模块实现了云台的 yaw 和 pitch 轴角度控制，并记录了电流值。
 * @author YourName
 * @date 2025-01-11
 */

#include "feedforward.h"

// 控制周期 2ms
#define CONTROL_PRECISION 0.002f

// 角度变化间隔 5s
#define ANGLE_CHANGE_INTERVAL 5000

// 角度变化步长，直接使用弧度制
#define YAW_ANGLE_CHANGE_STEP 0.001745f   // Yaw 每次角度变化0.1度，直接使用弧度值 (0.1 * π / 180 ≈ 0.001745)
#define PITCH_ANGLE_CHANGE_STEP 0.001745f // Pitch 每次角度变化0.1度，直接使用弧度值 (0.1 * π / 180 ≈ 0.001745)

// 起始角度和末尾角度，直接使用弧度制
#define YAW_START_ANGLE 0.0f        // Yaw 起始角度0度，直接使用弧度值
#define YAW_END_ANGLE 0.0f          // Yaw 目标角度0度，直接使用弧度值
#define PITCH_START_ANGLE 0.5236f   // Pitch 起始角度30度，直接使用弧度值 (30 * π / 180 ≈ 0.5236)
#define PITCH_END_ANGLE (-0.5236f)  // Pitch 目标角度-30度，直接使用弧度值 (-30 * π / 180 ≈ -0.5236)

// 静态全局变量，记录当前电流值
static fp32 gimbal_yaw_current = 0.f;
static fp32 gimbal_pitch_current = 0.f;

/**
 * @brief 角度控制结构体
 * @details 用于记录每个轴的时间、当前角度、目标角度和步长。
 */
typedef struct {
    uint32_t time;       // 当前时间
    fp32 current_angle;  // 当前角度
    fp32 target_angle;   // 目标角度
    fp32 step;           // 角度变化步长
} angle_control_t;

// Yaw 轴角度控制变量
static angle_control_t yaw_control = {0, YAW_START_ANGLE, YAW_END_ANGLE, YAW_ANGLE_CHANGE_STEP};

// Pitch 轴角度控制变量
static angle_control_t pitch_control = {0, PITCH_START_ANGLE, PITCH_END_ANGLE, PITCH_ANGLE_CHANGE_STEP};

/**
 * @brief 云台前馈控制函数
 * @details 根据时间间隔更新 yaw 和 pitch 角度，并记录电流值。
 * @param yaw 指向 yaw 角度值的指针，用于输出当前 yaw 角度
 * @param pitch 指向 pitch 角度值的指针，用于输出当前 pitch 角度
 * @param gimbal_control_set 云台控制设定值结构体指针
 */
void gimbal_feedforward_control(fp32 *yaw, fp32 *pitch, const gimbal_control_t *gimbal_control_set)
{
    // 每次进入函数，时间增加2ms
    yaw_control.time += 2;
    pitch_control.time += 2;

    // Yaw 角度变化逻辑
    if (yaw_control.time >= ANGLE_CHANGE_INTERVAL)
    {
        yaw_control.time = 0; // 重置时间

        // 更新 Yaw 角度
        yaw_control.current_angle += yaw_control.step;

        // 如果角度超过目标角度，则重置为起始角度
        if (yaw_control.current_angle > yaw_control.target_angle)
        {
            yaw_control.current_angle = YAW_START_ANGLE;
        }

        // 直接输出 Yaw 角度（已经是弧度制）
        *yaw = yaw_control.current_angle;

        // 记录当前 Yaw 电流值到静态全局变量中
        gimbal_yaw_current = gimbal_control_set->gimbal_yaw_motor.current_set;
    }
    else
    {
        // 保持当前 Yaw 角度
        *yaw = yaw_control.current_angle;
    }

    // Pitch 角度变化逻辑
    if (pitch_control.time >= ANGLE_CHANGE_INTERVAL)
    {
        pitch_control.time = 0; // 重置时间

        // 更新 Pitch 角度
        pitch_control.current_angle += pitch_control.step;

        // 如果角度超过目标角度，则重置为起始角度
        if (pitch_control.current_angle > pitch_control.target_angle)
        {
            pitch_control.current_angle = PITCH_START_ANGLE;
        }

        // 直接输出 Pitch 角度（已经是弧度制）
        *pitch = pitch_control.current_angle;

        // 记录当前 Pitch 电流值到静态全局变量中
        gimbal_pitch_current = gimbal_control_set->gimbal_pitch_motor.current_set;
    }
    else
    {
        // 保持当前 Pitch 角度
        *pitch = pitch_control.current_angle;
    }
}

/**
 * @brief 获取 Yaw 轴电流值
 * @details 返回当前 Yaw 轴的电流值。
 * @return Yaw 轴当前电流值
 */
fp32 get_gimbal_yaw_current(void)
{
    return gimbal_yaw_current;
}

/**
 * @brief 获取 Pitch 轴电流值
 * @details 返回当前 Pitch 轴的电流值。
 * @return Pitch 轴当前电流值
 */
fp32 get_gimbal_pitch_current(void)
{
    return gimbal_pitch_current;
}