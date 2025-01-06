//
// Created by Jehn on 2025/1/6.
//

#ifndef DM_02_HERO_SUPERCAP_H
#define DM_02_HERO_SUPERCAP_H
#include "main.h"
#include "bsp_can.h"
#endif //DM_02_HERO_SUPERCAP_H

/* ----------------------- CAP CAN Definition----------------------------- */
#define CAP_TX_ID	0X329
#define CAP_RX_ID	0x330
#define HCAN_CAPOWER hfdcan2

/* ----------------------- Data Struct ------------------------------------- */
typedef struct{
    float Pin;				//输入功率
    float Pout;				//底盘功率
    float Plimit;			//限制功率
    float voltage;		    //电容电压
    uint8_t cap_energy;	    //电容电量-百分比
    uint8_t efficiency;     //总效率
    uint8_t now_mode;		//当前模式
    uint8_t set_mode;		//设置的模式
}CapDataTypedef;