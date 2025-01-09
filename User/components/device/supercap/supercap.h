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
    float Ucap;				//电容电压
    uint8_t cap_energy;	//电容电量百分比
    uint8_t now_state;		//当前模式 1:待机状态 2:输入欠压 3:底盘超压 4:电容端过压 5:充电电流过大 6:放电电流过大 7:dcdc输入端过流
    float	Plim;			//设置的限制功率
    uint8_t capower_enable;
    uint8_t robot_type;
    /********************set_mode位定义解析***********************/
    /* 7bit  6bit  5bit  4bit  3bit  2bit     1bit        0bit */
    /*            7--3bit保留       enable  robot_type   轮腿detect */
    /*
    *set_mode共8位，7--3bit保留
    *2bit为炒鸡电龙启动开关，0为启动，1为关闭，启动时炒鸡电龙工作，关闭时相当于没有炒鸡电龙
    *1bit为机器人类型位，0为轮腿机器人，1为其他机器人
    *0bit为轮腿米机发疯处理，轮腿机器人需在500ms内发1，否则轮腿底盘断电
    */
    uint8_t set_mode;		//设置的模式
}CapDataTypedef;

extern   CapDataTypedef CAP_CANData;
void CAP_CAN_RxCallback(uint8_t *rx_data, uint16_t can_id);
uint8_t CAP_CAN_DataSend(FDCAN_HandleTypeDef* hfdcan,float lim_power, uint8_t capower_enable);