//
// Created by Rick on 2024/11/28.
//

#include "dt7.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

static int16_t dt7_abs(int16_t value);

// 遥控器控制变量
static DT7_ctrl_t dt7_ctrl;

// 接收原始数据,为18个字节,给了36个字节长度,防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
dt7_dma_memory_e dma_memory = DT7_DMA_MEMORY_0;

void dt7_init(void){
	RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

void DT7_change_dma_memory(dt7_dma_memory_e memory_index){
	if(memory_index == DT7_DMA_MEMORY_0){
		RC_change_dma_memory(sbus_rx_buf[0], SBUS_RX_BUF_NUM);
		dma_memory = DT7_DMA_MEMORY_1;
	}
	else if(memory_index == DT7_DMA_MEMORY_1){
		RC_change_dma_memory(sbus_rx_buf[1], SBUS_RX_BUF_NUM);
		dma_memory = DT7_DMA_MEMORY_0;
	}
}
dt7_dma_memory_e get_dt7_dma_memory(void){
	return dma_memory;
}

const DT7_ctrl_t *get_dt7_point(void){
	return &dt7_ctrl;
}

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

void solve_dt7_lost(void){
	RC_restart(SBUS_RX_BUF_NUM);
}

void solve_data_error(void){
	RC_restart(SBUS_RX_BUF_NUM);
}

//取正函数
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
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @retval         none
  */
void sbus_to_dt7(dt7_dma_memory_e memory_index)
{
	uint8_t *sbus_buf = NULL;
	if(memory_index == DT7_DMA_MEMORY_0){
		sbus_buf = sbus_rx_buf[0];
	}else if(memory_index == DT7_DMA_MEMORY_1){
		sbus_buf = sbus_rx_buf[1];
	}else{
		return;
	}

	if (sbus_buf == NULL)
	{
		return;
	}

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
	dt7_ctrl.rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL NOLINT(*-narrowing-conversions)

	dt7_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
	dt7_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
	dt7_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
	dt7_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
	dt7_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;
}


