//
// Created by Rick on 2024/11/28.
//


#include "dt7.h"

// 遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

static int16_t dt7_abs(int16_t value);

// 遥控器控制变量
static RC_ctrl_t dt7_ctrl;

/**
  * @brief          获取遥控器控制数据
  * @retval         指向当前遥控器控制数据的指针
  *
  * 返回指向遥控器控制数据结构体 `RC_ctrl_t` 的指针，包含遥控器的当前状态。
  */
RC_ctrl_t *get_dt7_point(void){
	return &dt7_ctrl;
}

/**
  * @brief          检查遥控器数据是否有错误
  * @retval         0: 数据正常, 1: 数据有错误
  *
  * 检查遥控器的通道值和开关状态是否超出允许的误差范围。如果检测到错误，清空控制数据并返回1。
  */
uint8_t dt7_data_is_error(void){
	if (dt7_abs(dt7_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
	{
		goto error;
	}
	if (dt7_abs(dt7_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
	{
		goto error;
	}
	if (dt7_abs(dt7_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
	{
		goto error;
	}
	if (dt7_abs(dt7_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
	{
		goto error;
	}
	if (dt7_ctrl.rc.s[0] == 0)
	{
		goto error;
	}
	if (dt7_ctrl.rc.s[1] == 0)
	{
		goto error;
	}
	return 0;
error:
	dt7_ctrl.rc.ch[0] = 0;
	dt7_ctrl.rc.ch[1] = 0;
	dt7_ctrl.rc.ch[2] = 0;
	dt7_ctrl.rc.ch[3] = 0;
	dt7_ctrl.rc.ch[4] = 0;
	dt7_ctrl.rc.s[0] = RC_SW_DOWN;
	dt7_ctrl.rc.s[1] = RC_SW_DOWN;
	dt7_ctrl.mouse.x = 0;
	dt7_ctrl.mouse.y = 0;
	dt7_ctrl.mouse.z = 0;
	dt7_ctrl.mouse.press_l = 0;
	dt7_ctrl.mouse.press_r = 0;
	dt7_ctrl.key.v = 0;
	return 1;
}

/**
  * @brief          处理遥控器丢失的情况
  * @retval         none
  *
  * 当遥控器数据丢失时，调用该函数进行重新初始化。
  */
void solve_dt7_lost(void){
	RC_restart(SBUS_RX_BUF_NUM);
}

/**
  * @brief          处理数据错误的情况
  * @retval         none
  *
  * 当遥控器数据发生错误时，调用该函数重新初始化遥控器接收。
  */
void solve_data_error(void){
	RC_restart(SBUS_RX_BUF_NUM);
}

/**
  * @brief          取正函数
  * @param[in]      value: 需要取绝对值的整数
  * @retval         正数：返回值的绝对值
  *
  * 计算并返回输入值的绝对值。
  */
static int16_t dt7_abs(int16_t value)
{
	if (value > 0)
	{
		return value;
	}
	else
	{
		return (int16_t )-value;
	}
}

void sbus_to_dt7(RC_ctrl_t *rc_ctrl, uint8_t *sbus_buf)
{
	// 检查sbuff_buf是否为空或长度不足
	if (rc_ctrl == NULL || sbus_buf == NULL) {
		return;
	}


	// 解析遥控器通道值并校准
	rc_ctrl->rc.ch[0] = ((uint16_t)sbus_buf[0] | ((uint16_t)sbus_buf[1] << 8)) & 0x07FF; // 将buff[0]和buff[1]的值组合为ch0通道的值，并将其限制在11位
	rc_ctrl->rc.ch[0] -= 1024; // 中心值为0

	rc_ctrl->rc.ch[1] = (((uint16_t)sbus_buf[1] >> 3) | ((uint16_t)sbus_buf[2] << 5)) & 0x07FF;
	rc_ctrl->rc.ch[1] -= 1024;

	rc_ctrl->rc.ch[2] = (((uint16_t)sbus_buf[2] >> 6) | ((uint16_t)sbus_buf[3] << 2) | ((uint16_t)sbus_buf[4] << 10)) & 0x07FF;
	rc_ctrl->rc.ch[2] -= 1024;

	rc_ctrl->rc.ch[3] = (((uint16_t)sbus_buf[4] >> 1) | ((uint16_t)sbus_buf[5] << 7)) & 0x07FF;
	rc_ctrl->rc.ch[3] -= 1024;

	rc_ctrl->rc.ch[4] = ((uint16_t)sbus_buf[16] | ((uint16_t)sbus_buf[17] << 8)) & 0x07FF; // 左上角滚轮
	rc_ctrl->rc.ch[4] -= 1024;

	// 解析开关状态
	rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0C) >> 2;
	rc_ctrl->rc.s[1] = (sbus_buf[5] >> 4) & 0x03;
}
