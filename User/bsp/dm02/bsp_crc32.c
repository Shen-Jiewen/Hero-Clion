//
// Created by Rick on 24-11-25.
//

#include "bsp_crc32.h"

extern CRC_HandleTypeDef hcrc;

/**
 * @brief 计算数据的 CRC32 校验值
 * @author Javen
 * @param data 指向要计算 CRC32 校验值的数据指针
 * @param len 数据的长度，以 32 位字（uint32_t）为单位
 * @return 返回计算得到的 CRC32 校验值
 *
 * @note 此函数使用 HAL_CRC_Calculate 函数调用硬件 CRC 模块计算校验值。
 *       确保已经正确初始化了 hcrc 句柄以及 CRC 硬件模块。
 *       如果数据格式为字节或半字，请在调用此函数前处理为 32 位对齐。
 */
uint32_t get_crc32_check_sum(uint32_t* data, uint32_t len)
{
	return HAL_CRC_Calculate(&hcrc, data, len);
}

/**
 * @brief 验证数据中的 CRC32 校验值是否正确
 * @author Javen
 * @param data 指向要验证的数据指针
 * @param len 数据的长度（包括最后的 CRC32 校验值，以 32 位字为单位）
 * @return 如果 CRC 校验正确返回 true，否则返回 false
 *
 * @note 数据的最后一个元素（data[len - 1]）应为预期的 CRC32 校验值。
 *       函数会计算 data 中前 len-1 个元素的 CRC32 值并与 data[len - 1] 比较。
 *       确保数据已经正确对齐到 32 位字，并初始化了 hcrc 句柄。
 */
bool_t verify_crc32_check_sum(uint32_t* data, uint32_t len)
{
	static uint32_t crc32;
	crc32 = get_crc32_check_sum(data, len - 1); // 计算数据（不包括最后一个）的 CRC32
	return (crc32 == data[len - 1]);           // 与数据末尾的校验值进行比较
}

/**
 * @brief 为数据添加 CRC32 校验值
 * @author Javen
 * @param data 指向要附加 CRC32 校验值的数据指针
 * @param len 数据的总长度（包括要附加 CRC32 校验值的占位，以 32 位字为单位）
 *
 * @note 数据的最后一个元素（data[len - 1]）应为 CRC32 校验值的存放位置。
 *       函数会计算 data 中前 len-1 个元素的 CRC32 校验值并存入 data[len - 1]。
 *       确保数据已经正确对齐到 32 位字，并初始化了 hcrc 句柄。
 */
void append_crc32_check_sum(uint32_t* data, uint32_t len)
{
	uint32_t crc32;
	crc32 = get_crc32_check_sum(data, len - 1); // 计算前 len-1 个元素的 CRC32
	data[len - 1] = crc32;                     // 将 CRC32 值存入最后一个元素
}
