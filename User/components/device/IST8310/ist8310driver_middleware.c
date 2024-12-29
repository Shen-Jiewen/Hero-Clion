/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310middleware.c/h
  * @brief      IST8310磁力计中间层，完成IST8310的IIC通信,因为设置MPU6500的SPI通信
  *             所以设置的是通过mpu6500的IIC_SLV0完成读取，IIC_SLV4完成写入。
  * @note       IST8310只支持IIC读取
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "ist8310driver_middleware.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_gpio.h"

#define RSTN_IST8310_GPIO_Port GPIOB
#define RSTN_IST8310_Pin GPIO_PIN_0

extern I2C_HandleTypeDef hi2c1;

void ist8310_GPIO_init(void)
{
	// 初始化IST8310复位引脚
	BSP_GPIO_InitTypeDef gpio_config = {
		.port = RSTN_IST8310_GPIO_Port,
		.pin = RSTN_IST8310_Pin,
		.mode = BSP_GPIO_MODE_OUTPUT,
		.pull = GPIO_NOPULL,
		.speed = GPIO_SPEED_FREQ_LOW
	};
	BSP_GPIO_Init(&gpio_config);
}

void ist8310_com_init(void)
{

}

uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
	uint8_t res;
	HAL_I2C_Mem_Read(&hi2c1, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
	return res;
}
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c1, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
	HAL_I2C_Mem_Read(&hi2c1, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
	HAL_I2C_Mem_Write(&hi2c1, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}
void ist8310_delay_ms(uint16_t ms)
{
	osDelay(ms);
}

void ist8310_RST_H(void)
{
	BSP_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
}

extern void ist8310_RST_L(void)
{
	BSP_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
}
