//
// Created by Rick on 2024/11/28.
//

#ifndef DM_02_HERO_USER_COMPONENTS_DEVICE_CONTROL_DT7_DT7_H_
#define DM_02_HERO_USER_COMPONENTS_DEVICE_CONTROL_DT7_DT7_H_

#include "main.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
	struct
	{
		int16_t ch[5];   // 通道数据
		char s[2];       // 状态信息
	} rc;

	struct
	{
		int16_t x;       // 鼠标X轴
		int16_t y;       // 鼠标Y轴
		int16_t z;       // 鼠标Z轴
		uint8_t press_l; // 鼠标左键状态
		uint8_t press_r; // 鼠标右键状态
	} mouse;

	struct
	{
		uint16_t v;      // 键盘值
	} key;

} __attribute__((packed)) RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

extern void sbus_to_dt7(RC_ctrl_t *rc_ctrl, const uint8_t *sbus_buf);
extern RC_ctrl_t *get_dt7_point(void);

extern uint8_t dt7_data_is_error(void);
extern void solve_dt7_lost(void);
extern void solve_data_error(void);

#endif //DM_02_HERO_USER_COMPONENTS_DEVICE_CONTROL_DT7_DT7_H_
