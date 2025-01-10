//
// Created by Jehn on 2025/1/6.
//
#include "chassis_power_control.h"
extern RC_ctrl_t rc_ctrl;
extern FDCAN_HandleTypeDef hfdcan1;
uint8_t cap_state = 0;
FDCAN_TxHeaderTypeDef cap_TX_header;
float input_power = 50;
void chassis_power_control(chassis_control_t *chassis_power_control)
{
    uint16_t max_power_limit = 40; //最大功率限制
    fp32 chassis_max_power = 0;
    // input power from battery (referee system)
    float initial_give_power[4]; // initial power from PID calculation
    float initial_total_power = 0;
    fp32 scaled_give_power[4];

    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;

    fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
    fp32 a = 1.23e-07;						 // k1
    fp32 k2 = 1.453e-07;					 // k2
    fp32 constant = 4.081f;

    //get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer,&max_power_limit);//获取裁判系统数据
    //PID_calc(&chassis_power_control->buffer_pid, chassis_power_buffer, 50);
    //get_chassis_max_power(&max_power_limit);
    //input_power = max_power_limit - chassis_power_control->buffer_pid.out; // Input power floating at maximum power    输入功率在最大功率下浮动
    //input_power = 0.5f*(input_power - chassis_power) + input_power; //修bug

    if(chassis_power < 0.9f*max_power_limit)
    {
        input_power = -0.5f*(max_power_limit - chassis_power) + max_power_limit; //修bug
    }
    else
    {
        input_power = 0.5f*(max_power_limit - chassis_power) + max_power_limit; //修bug
    }


    CAP_CAN_DataSend(&hfdcan1,input_power, ENABLE_CAP);// 设置电容器控制器的输入功率set the input power of capacitor controller

    if (rc_ctrl.key.v & KEY_SPEED_UP) //按下加速键，开启“暴走模式”，英雄机器人将消耗更高的超级电容能量
    {
        cap_state = 1;
    }
    else                              //松开加速键，退出“暴走模式”
    {
        cap_state = 0;
    }
    //如果超级电容剩余能量在(60,100]
    if (CAP_CANData.cap_energy > 60)
    {
        if (cap_state == 0) //默认模式
        {
            chassis_max_power = input_power + 50; // 略大于最大功率，避免电容器一直充满，提高能量利用率
        }
        else                //暴走模式
        {
            chassis_max_power = input_power + 80;	//暴走模式下，向超级电容申请更多能量
        }
    }
    //超级电容剩余能量在(40,60]
    else if (CAP_CANData.cap_energy <= 60 && CAP_CANData.cap_energy > 40)
    {
        if (cap_state == 0) //默认模式
        {
            chassis_max_power = input_power + 30; //限制底盘能量
        }
        else                //暴走模式
        {
            chassis_max_power = input_power + 40; //申请更多能量
        }

    }
    //超级电容剩余能量在(30,40]
    else if (CAP_CANData.cap_energy <= 40 && CAP_CANData.cap_energy > 30)
    {
        chassis_max_power = input_power + 10;   //降低底盘功率消耗
    }
    //超级电容能量在[0,30]|[100，∞]
    else
    {
        chassis_max_power = input_power;    //严格限制底盘功率消耗
    }
    for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
    {
        initial_give_power[i] = chassis_power_control->motor_speed_pid[i].out * toque_coefficient * chassis_power_control->motor_chassis[i].speed +
                                k2 * chassis_power_control->motor_chassis[i].speed * chassis_power_control->motor_chassis[i].speed +
                                a * chassis_power_control->motor_speed_pid[i].out * chassis_power_control->motor_speed_pid[i].out + constant;

        if (initial_give_power < 0) // negative power not included (transitory)
            continue;
        initial_total_power += initial_give_power[i];
    }

    if (initial_total_power > chassis_max_power) // determine if larger than max power
    {
        fp32 power_scale = chassis_max_power / initial_total_power;//计算放缩比例
        for (uint8_t i = 0; i < 4; i++)
        {
            scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
            if (scaled_give_power[i] < 0)
            {
                continue;
            }

            fp32 b = toque_coefficient * chassis_power_control->motor_chassis[i].speed;
            fp32 c = k2 * chassis_power_control->motor_chassis[i].speed * chassis_power_control->motor_chassis[i].speed - scaled_give_power[i] + constant;

            if (chassis_power_control->motor_speed_pid[i].out > 0) // Selection of the calculation formula according to the direction of the original moment
            {
                fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
                if (temp > 16000)
                {
                    chassis_power_control->motor_speed_pid[i].out = 16000;
                }
                else
                    chassis_power_control->motor_speed_pid[i].out = temp;
            }
            else
            {
                fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
                if (temp < -16000)
                {
                    chassis_power_control->motor_speed_pid[i].out = -16000;
                }
                else
                    chassis_power_control->motor_speed_pid[i].out = temp;
            }
        }
    }
}