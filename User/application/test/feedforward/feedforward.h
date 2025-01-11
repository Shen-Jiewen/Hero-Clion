/**
* @file feedforward.h
 * @brief 云台前馈控制模块头文件
 * @details 该模块实现了云台的 yaw 和 pitch 轴角度控制，并提供了电流值的获取接口。
 * @author YourName
 * @date 2025-01-11
 */

#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

#include "struct_typedef.h"
#include "gimbal.h"

// #define feedforward_test


/**
 * @brief 云台前馈控制函数
 * @param yaw 指向 yaw 角度值的指针，用于输出当前 yaw 角度
 * @param pitch 指向 pitch 角度值的指针，用于输出当前 pitch 角度
 * @param gimbal_control_set 云台控制设定值结构体指针
 */
void gimbal_feedforward_control(fp32 *yaw, fp32 *pitch, const gimbal_control_t *gimbal_control_set);

/**
 * @brief 获取 Yaw 轴电流值
 * @return Yaw 轴当前电流值
 */
fp32 get_gimbal_yaw_current(void);

/**
 * @brief 获取 Pitch 轴电流值
 * @return Pitch 轴当前电流值
 */
fp32 get_gimbal_pitch_current(void);

#endif // FEEDFORWARD_H