//
// Created by Rick on 24-11-25.
//

#ifndef DM_02_HERO_USER_BSP_DM02_BSP_UART_H_
#define DM_02_HERO_USER_BSP_DM02_BSP_UART_H_

#include "main.h"

extern HAL_StatusTypeDef uart1_send_data(uint8_t* pData, uint16_t Size);
extern HAL_StatusTypeDef uart7_send_data(uint8_t* pData, uint16_t Size);
extern HAL_StatusTypeDef uart10_send_data(uint8_t* pData, uint16_t Size);

#endif //DM_02_HERO_USER_BSP_DM02_BSP_UART_H_
