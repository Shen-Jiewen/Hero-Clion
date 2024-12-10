//
// Created by Rick on 2024/11/28.
//


#include "dt7.h"

// 遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

static int16_t dt7_abs(int16_t value);

// 遥控器控制变量
static DT7_ctrl_t dt7_ctrl;

/**
  * @brief          初始化DT7模块
  * @retval         none
  *
  * 初始化遥控器的接收缓冲区，设置DMA数据传输相关配置。
  */
void dt7_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num){
	RC_Init(rx1_buf, rx2_buf, dma_buf_num);
}

/**
  * @brief          获取遥控器控制数据
  * @retval         指向当前遥控器控制数据的指针
  *
  * 返回指向遥控器控制数据结构体 `DT7_ctrl_t` 的指针，包含遥控器的当前状态。
  */
DT7_ctrl_t *get_dt7_point(void){
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

/**
  * @brief          将SBUS数据转换为DT7遥控器控制数据
  * @param[in]      memory_index: 选择的DMA内存区域（0或1）
  * @retval         none
  *
  * 解析SBUS协议数据，将其转换为DT7遥控器控制数据（`DT7_ctrl_t`）。包括遥控器通道值、开关状态、鼠标位置和键盘值等。
  */
void sbus_to_dt7(const uint8_t *sbus_buf)
{
	if (sbus_buf == NULL)
	{
		return;
	}

	// 解析各个遥控器通道值
	dt7_ctrl.rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0 NOLINT(*-narrowing-conversions)
	dt7_ctrl.rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1 NOLINT(*-narrowing-conversions)
	dt7_ctrl.rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2 NOLINT(*-narrowing-conversions)
		(sbus_buf[4] << 10)) & 0x07ff;
	dt7_ctrl.rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3 NOLINT(*-narrowing-conversions)
	dt7_ctrl.rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch right !!!!
	dt7_ctrl.rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch left !!!!!
	dt7_ctrl.mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis NOLINT(*-narrowing-conversions)
	dt7_ctrl.mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis NOLINT(*-narrowing-conversions)
	dt7_ctrl.mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis NOLINT(*-narrowing-conversions)
	dt7_ctrl.mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
	dt7_ctrl.mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
	dt7_ctrl.key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
	dt7_ctrl.rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 // NULL NOLINT(*-narrowing-conversions)

	// 校准遥控器通道值
	dt7_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
	dt7_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
	dt7_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
	dt7_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
	dt7_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
