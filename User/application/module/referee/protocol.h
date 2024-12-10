#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

/**
 * @brief 赛况命令ID枚举，定义了比赛过程中的各类状态信息、结果、事件、机器人状态等的命令标识。
 */
typedef enum
{
	GAME_STATE_CMD_ID = 0x0001,                ///< 比赛状态信息
	GAME_RESULT_CMD_ID = 0x0002,               ///< 比赛结果
	GAME_ROBOT_HP_CMD_ID = 0x0003,             ///< 机器人血量
	FIELD_EVENTS_CMD_ID = 0x0101,              ///< 场地事件数据
	SUPPLY_PROJECTILE_ACTION_CMD_ID = 0x0102,  ///< 补给站动作标识
	SUPPLY_PROJECTILE_BOOKING_CMD_ID = 0x0103, ///< 裁判系统版本更新后被取消
	REFEREE_WARNING_CMD_ID = 0x0104,           ///< 裁判系统警告信息
	ROBOT_STATE_CMD_ID = 0x0201,               ///< 比赛机器人状态
	POWER_HEAT_DATA_CMD_ID = 0x0202,           ///< 实时功率热量
	ROBOT_POS_CMD_ID = 0x0203,                 ///< 机器人位置
	BUFF_MUSK_CMD_ID = 0x0204,                 ///< 机器人增益
	AERIAL_ROBOT_ENERGY_CMD_ID = 0x0205,       ///< 空中机器人能量状态
	ROBOT_HURT_CMD_ID = 0x0206,                ///< 伤害状态
	SHOOT_DATA_CMD_ID = 0x0207,                ///< 实时射击数据
	BULLET_REMAINING_CMD_ID = 0x0208,          ///< 子弹剩余发射数
	STUDENT_INTERACTIVE_DATA_CMD_ID = 0x0301,  ///< 交互数据接收信息
	IDCustomData,                              ///< 用户自定义数据标识
} __attribute__((packed)) referee_cmd_id_t;    ///< 赛况命令ID枚举


/**
 * @brief 帧头结构体，定义了每个数据包的头部信息，包括起始符、数据长度、序列号和CRC校验码。
 */
typedef struct __attribute__((packed))
{
	uint8_t SOF;          ///< 帧头标识符，Start of Frame，指示数据帧的开始
	uint16_t data_length; ///< 数据部分的长度，指示后续数据的字节数
	uint8_t seq;          ///< 序列号，标识数据包的顺序
	uint8_t CRC8;         ///< 8位CRC校验码，用于检测数据包头是否被篡改
} frame_header_struct_t;  ///< 帧头结构体


/**
 * @brief 解包步骤枚举，定义了数据解析过程中的各个步骤。
 */
typedef enum __attribute__((packed))
{
	STEP_HEADER_SOF = 0,   ///< 步骤1：解析帧头的SOF（起始符）
	STEP_LENGTH_LOW = 1,   ///< 步骤2：解析数据长度的低字节
	STEP_LENGTH_HIGH = 2,  ///< 步骤3：解析数据长度的高字节
	STEP_FRAME_SEQ = 3,    ///< 步骤4：解析序列号
	STEP_HEADER_CRC8 = 4,  ///< 步骤5：解析帧头的CRC8校验
	STEP_DATA_CRC16 = 5,   ///< 步骤6：解析数据部分的CRC16校验
} __attribute__((packed)) unpack_step_e;  ///< 解包步骤枚举


/**
 * @brief 解包数据结构体，定义了整个数据包的解包过程，包含帧头、数据缓冲区和解包步骤。
 */
typedef struct __attribute__((packed))
{
	frame_header_struct_t* p_header;           ///< 指向帧头结构体的指针，用于存储和访问数据包头部信息
	uint16_t data_len;                         ///< 数据部分的长度，用于标记数据的有效长度
	uint8_t protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE]; ///< 协议数据包，存储整个数据包（包括头部和数据部分）
	unpack_step_e unpack_step;                 ///< 当前解包的步骤，用于标识当前解包进度
	uint16_t index;                            ///< 当前处理的字节索引，指示解析的进度
} unpack_data_t;  ///< 解包数据结构体

#endif //ROBOMASTER_PROTOCOL_H
