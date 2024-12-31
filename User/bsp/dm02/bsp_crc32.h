//
// Created by Rick on 24-11-25.
//

#ifndef BSP_CRC32_H_
#define BSP_CRC32_H_

#include "main.h"
#include "struct_typedef.h"

extern uint32_t get_crc32_check_sum(uint32_t* data, uint32_t len);
extern bool_t verify_crc32_check_sum(uint32_t* data, uint32_t len);
extern void append_crc32_check_sum(uint32_t* data, uint32_t len);

#endif //BSP_CRC32_H_
