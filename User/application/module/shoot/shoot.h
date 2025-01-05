//
// Created by Rick on 2024/12/20.
//

#ifndef SHOOT_H_
#define SHOOT_H_

#include "main.h"
#include "struct_typedef.h"
#include "dji_3508.h"
#include "dm_4310.h"
#include "dt7.h"
#include "user_lib.h"
#include "arm_math.h"
#include "pid.h"

//任务初始化延迟一段时间，等待拨盘达妙使能
#define SHOOT_TASK_INIT_TIME    2000

//拨盘电机 角度环 角度由编码器 PID参数以及  PID最大输出，积分输出
#define Down_TRIGGER_ANGLE_PID_KP        	-30000.0f
#define Down_TRIGGER_ANGLE_PID_KI        	0.0f
#define Down_TRIGGER_ANGLE_PID_KD        	-250000.0f
#define Down_TRIGGER_ANGLE_PID_MAX_OUT  	300000.0f
#define Down_TRIGGER_ANGLE_PID_MAX_IOUT 	12000.0f

//双极摩擦轮外电机 速度环 PID参数以及 PID最大输出，积分输出
#define OUTSIDE_SHOOT_SPEED_PID_KP 17000.0f   //15000
#define OUTSIDE_SHOOT_SPEED_PID_KI 10.0f      //10
#define OUTSIDE_SHOOT_SPEED_PID_KD 0.0f
#define OUTSIDE_SHOOT_SPEED_PID_MAX_OUT  16000.0f
#define OUTSIDE_SHOOT_SPEED_PID_MAX_IOUT 2000.0f

//双极摩擦轮内电机 速度环 PID参数以及 PID最大输出，积分输出
#define INSIDE_SHOOT_SPEED_PID_KP 15000.0f   //15000
#define INSIDE_SHOOT_SPEED_PID_KI 10.0f      //10
#define INSIDE_SHOOT_SPEED_PID_KD 0.0f
#define INSIDE_SHOOT_SPEED_PID_MAX_OUT  16000.0f
#define INSIDE_SHOOT_SPEED_PID_MAX_IOUT 2000.0f

//m3508速度转化的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f

#define SHOOT_CONTROL_TIME 1

//射击控制遥控器通道
#define SHOOT_RC_MODE_CHANNEL       1

//射击摩擦轮打开/关闭键盘值
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

#define SHOOT_HEAT_NEED_VALUE     100     //发射一次大弹丸需要的热量

#define OUTSIDE_SPEED_SET        2.0f
#define INSIDE_SPEED_SET         1.8f

#define Down_TRIGGER_POSITION  -1.04719755	//拨弹电机MIT模式下绝对位置
#define Down_TRIGGER_MOTOR_ID 0x103			//拨弹电机MIT模式下电机ID

typedef enum
{
   SHOOT_STOP = 0,
   SHOOT_READY_FRIC,
   SHOOT_READY_BULLET,
   SHOOT_READY,
   SHOOT_BULLET,
   SHOOT_CONTINUE_BULLET,
   SHOOT_DONE,
} shoot_mode_e;

/**
 * @brief 定义电机类型的枚举类型
 */
typedef enum {
   MOTOR_4310,
   MOTOR_3508,
   MOTOR_UNKNOWN
}shoot_motor_type_e;

/**
 * @brief 定义电机控制数据结构体
 */
typedef struct {
   shoot_motor_type_e motor_type;                  //电机类型
   union {
      const motor_3508_measure_t* motor_3508;      //指向3508电机测量数据的常量指针
      const motor_4310_measure_t* motor_4310;      //指向4310电机测量数据的常量指针
   }motor_measure;

   pid_type_def trigger_angle_pid;                 //拨盘电机角度环PID
   int16_t give_current;                            // 实际给定电流值
   fp32 speed_set;                                  // 电机设定速度
   float angle_set;                                 //电机角度设定值

}shoot_motor_t;

typedef struct
{
   const RC_ctrl_t* shoot_rc_ctrl;                 //指向发射机构遥控控制输入的常量指针
   shoot_motor_t trigger_motor;                    //拨盘电机结构体
   motor_3508_t friction_motor[4];                 //四个摩擦轮电机结构体
   shoot_mode_e shoot_mode;                        //发射的状态机枚举
   pid_type_def friction_speed_pid[4];             //四个摩擦轮电机的PID
   bool_t fric_flag;                              //摩擦轮标志位
   bool_t press_l;                                 //鼠标左键状态
   bool_t press_r;                                 //鼠标右键状态
   bool_t last_press_l;                            //上一次鼠标左键状态
   bool_t last_press_r;                            //上一次鼠标右键状态
   uint16_t shoot_speed_limit;                     //射速限制
   uint16_t last_shoot_speed_limit;                //上一次射速限制
   uint16_t heat_limit;                            //热量限制
   uint16_t heat;	                                 //热量
   fp32 outside_fric_speed_set;                    //靠外摩擦轮速度设定值
   fp32 inside_fric_speed_set;                     //靠内摩擦轮速度设定值
   bool_t move_flag;                               //单发标志位
}shoot_control_t;

shoot_control_t* get_shoot_control_point(void);
void shoot_init(shoot_control_t* shoot_init);
void shoot_feedback_update(shoot_control_t* shoot_feedback);
void shoot_set_mode(void);
#endif //SHOOT_H_
