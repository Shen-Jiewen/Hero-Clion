/**
 * @file feedforward.c
 * @brief 云台前馈控制模块实现文件
 * @details 该模块实现了云台的 yaw 和 pitch 轴角度控制，并记录了电流值。
 * @author YourName
 * @date 2025-01-11
 */

#include "feedforward.h"

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
    // 每次进入函数，时间增加1ms
    yaw_control.time += 1;
    pitch_control.time += 1;

    // Yaw 角度变化逻辑
    // if (yaw_control.time >= ANGLE_CHANGE_INTERVAL)
    // {
    //     yaw_control.time = 0; // 重置时间
    //
    //     // 记录当前 Yaw 电流值到静态全局变量中
    //     gimbal_yaw_current = gimbal_control_set->gimbal_yaw_motor.current_set;
    //
    //     // 根据目标角度和当前角度的大小关系决定步长方向
    //     if (yaw_control.target_angle > yaw_control.current_angle) {
    //         yaw_control.current_angle += yaw_control.step;
    //         if (yaw_control.current_angle > yaw_control.target_angle) {
    //             yaw_control.current_angle = yaw_control.target_angle; // 到达目标角度后保持
    //         }
    //     } else {
    //         yaw_control.current_angle -= yaw_control.step;
    //         if (yaw_control.current_angle < yaw_control.target_angle) {
    //             yaw_control.current_angle = yaw_control.target_angle; // 到达目标角度后保持
    //         }
    //     }
    //
    //     // 直接输出 Yaw 角度（已经是弧度制）
    //     *yaw = yaw_control.current_angle;
    // }
    // else
    // {
    //     // 保持当前 Yaw 角度
    //     *yaw = yaw_control.current_angle;
    // }

    // Pitch 角度变化逻辑
    if (pitch_control.time >= ANGLE_CHANGE_INTERVAL)
    {
        pitch_control.time = 0; // 重置时间

        // 记录当前 Pitch 电流值到静态全局变量中
        gimbal_pitch_current = gimbal_control_set->gimbal_pitch_motor.current_set;

        // 根据目标角度和当前角度的大小关系决定步长方向
        if (pitch_control.target_angle > pitch_control.current_angle) {
            pitch_control.current_angle += pitch_control.step;
            if (pitch_control.current_angle > pitch_control.target_angle) {
                pitch_control.current_angle = pitch_control.target_angle; // 到达目标角度后保持
            }
        } else {
            pitch_control.current_angle -= pitch_control.step;
            if (pitch_control.current_angle < pitch_control.target_angle) {
                pitch_control.current_angle = pitch_control.target_angle; // 到达目标角度后保持
            }
        }

        // 直接输出 Pitch 角度（已经是弧度制）
        *pitch = pitch_control.current_angle;
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