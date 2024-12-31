/*******************************************************************************
 * @file: dm_4310.h
 * @author: Javen
 * @date: 2024年12月30日
 * @brief: DM 4310电机驱动头文件
 * @note: 该文件定义了DM 4310电机的相关数据结构、枚举类型和函数声明。
 *        支持CAN通信解析、电机控制模式切换以及PID控制器配置。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#ifndef DM_4310_H_
#define DM_4310_H_

#include "main.h"
#include "struct_typedef.h"
#include "pid.h"
#include "detect.h"

/**
 * @brief 定义电机控制参数的取值范围
 */
#define P_MIN (-12.5f)      // 位置最小值，单位：弧度 (rad)
#define P_MAX 12.5f         // 位置最大值，单位：弧度 (rad)
#define V_MIN (-45.0f)      // 速度最小值，单位：弧度/秒 (rad/s)
#define V_MAX 45.0f         // 速度最大值，单位：弧度/秒 (rad/s)
#define KP_MIN 0.0f         // 位置比例系数最小值
#define KP_MAX 500.0f       // 位置比例系数最大值
#define KD_MIN 0.0f         // 位置微分系数最小值
#define KD_MAX 5.0f         // 位置微分系数最大值
#define T_MIN (-18.0f)      // 转矩最小值，单位：牛·米 (Nm)
#define T_MAX 18.0f         // 转矩最大值，单位：牛·米 (Nm)

/**
 * @brief 定义电机FDCAN ID的枚举类型
 */
typedef enum
{
	FDCAN_DM4310_ALL_ID = 0x1FF,  // 所有电机的FDCAN ID
	FDCAN_DM4310_M1_ID = 0x201,   // 电机1的FDCAN ID
	FDCAN_DM4310_M2_ID = 0x202,   // 电机2的FDCAN ID
	FDCAN_DM4310_M3_ID = 0x203,   // 电机3的FDCAN ID
} dm_4310_id_e;

/**
 * @brief 定义电机控制模式的枚举类型
 */
typedef enum
{
	DM4310_MOTOR_RAW = 0,       // 电机原始值控制模式
	DM4310_MOTOR_POSITION,      // 电机位置控制模式
	DM4310_MOTOR_SPEED          // 电机速度控制模式
} dm_4310_mode_e;

/**
 * @brief 定义电机测量数据结构体
 */
typedef struct
{
	uint16_t ecd;               // 编码器数据
	int16_t speed_rpm;          // 电机转速，单位：转/分钟
	int16_t given_current;      // 给定电流值
	uint8_t temperature;        // 电机温度
	int16_t last_ecd;           // 上一次的编码器数据
} dm_4310_measure_t;

/**
 * @brief 定义电机控制数据结构体
 */
typedef struct
{
	const dm_4310_measure_t* motor_measure;  // 指向电机测量数据的常量指针

	pid_type_def position_pid;               // 位置PID控制器
	pid_type_def speed_pid;                  // 速度PID控制器

	dm_4310_mode_e motor_mode;               // 当前电机控制模式
	dm_4310_mode_e last_motor_mode;          // 上一次电机控制模式

	fp32 position_set;                       // 设定位置，单位：弧度 (rad)
	fp32 speed_set;                          // 设定速度，单位：弧度/秒 (rad/s)
	fp32 current_set;                        // 电流设定值
	int16_t given_current;                   // 实际给定电流值
} dm_4310_t;

/**
 * @brief 获取指定索引的dm_4310_measure_t结构体指针。
 *
 * @param i 数组索引，范围为0到2。
 * @return 指向指定索引的dm_4310_measure_t结构体的指针。
 */
const dm_4310_measure_t* get_dm_4310_measure_point(uint8_t i);

/**
 * @brief FDCAN回调函数，处理接收到的FDCAN消息。
 *
 * @param can_id FDCAN消息的ID。
 * @param rx_data 指向接收到的FDCAN数据的字节数组的指针。
 */
void dm_4310_fdcan_callback(uint32_t can_id, const uint8_t* rx_data);

/**
 * @brief 发送标准ID的数据帧。
 *
 * @param hfdcan FDCAN的句柄。
 * @param ID 数据帧ID。
 * @param pData 数组指针。
 * @param Len 字节数0~8。
 */
uint8_t DM4310_SendStdData(FDCAN_HandleTypeDef* hfdcan, uint16_t ID, uint8_t* pData);

/**
 * @brief 使能电机。
 */
void DM4310_MotorEnable(void);

/**
 * @brief MIT模式控制电机。
 *
 * @param hfdcan FDCAN的句柄。
 * @param id 数据帧的ID。
 * @param pos 位置给定。
 * @param vel 速度给定。
 * @param KP 位置比例系数。
 * @param KD 位置微分系数。
 * @param torq 转矩给定值。
 */
void DM4310_MIT_CtrlMotor(FDCAN_HandleTypeDef* hfdcan,
	uint16_t id,
	float pos,
	float vel,
	float KP,
	float KD,
	float torq);

/**
 * @brief 位置速度模式控制电机。
 *
 * @param hfdcan FDCAN的句柄。
 * @param id 数据帧的ID。
 * @param _pos 位置给定。
 * @param _vel 速度给定。
 */
void DM4310_PosSpeed_CtrlMotor(FDCAN_HandleTypeDef* hfdcan, uint16_t id, float _pos, float _vel);

/**
 * @brief 速度模式控制电机。
 *
 * @param hfdcan FDCAN的句柄。
 * @param ID 数据帧的ID。
 * @param _vel 速度给定。
 */
void DM4310_Speed_CtrlMotor(FDCAN_HandleTypeDef* hfdcan, uint16_t ID, float _vel);

/**
 * @brief 将无符号整数转换为浮点数。
 *
 * @param x_int 要转换的无符号整数。
 * @param x_min 目标浮点数的最小值。
 * @param x_max 目标浮点数的最大值。
 * @param bits 无符号整数的位数。
 * @return 转换后的浮点数。
 */
float DM4310_UintToFloat(int x_int, float x_min, float x_max, int bits);

/**
 * @brief 将浮点数转换为无符号整数。
 *
 * @param x 要转换的浮点数。
 * @param x_min 浮点数的最小值。
 * @param x_max 浮点数的最大值。
 * @param bits 无符号整数的位数。
 * @return 转换后的无符号整数。
 */
int DM4310_FloatToUint(float x, float x_min, float x_max, int bits);

#endif // DM_4310_H_