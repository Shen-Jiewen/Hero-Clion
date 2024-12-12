//
// Created by Rick on 2024/11/29.
//

#ifndef DM_02_HERO_USER_BSP_DM02_BSP_RC_H_
#define DM_02_HERO_USER_BSP_DM02_BSP_RC_H_

#include "main.h"
#include "struct_typedef.h"

extern void RC_Init(uint8_t *rx_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);

#endif //DM_02_HERO_USER_BSP_DM02_BSP_RC_H_
