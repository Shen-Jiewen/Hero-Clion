#include "referee.h"


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;										//比赛状态
ext_game_result_t game_result;										//比赛结果
ext_game_robot_HP_t game_robot_HP_t;								//机器人血量

ext_event_data_t field_event;										//场地事件数据
ext_supply_projectile_action_t supply_projectile_action_t;			//补给站动作标识
ext_supply_projectile_booking_t supply_projectile_booking_t;
ext_referee_warning_t referee_warning_t;							//裁判系统警告信息


ext_game_robot_state_t robot_state;									//比赛机器人状态
ext_power_heat_data_t power_heat_data_t;  							//实时功率热量
ext_game_robot_pos_t game_robot_pos_t;								//机器人位置
ext_buff_musk_t buff_musk_t;										//机器人增益
aerial_robot_energy_t robot_energy_t;								//空中机器人能量状态
ext_robot_hurt_t robot_hurt_t;										//伤害状态
ext_shoot_data_t shoot_data_t;          							//实时射击数据
ext_bullet_remaining_t bullet_remaining_t;							//子弹剩余发射数量
ext_student_interactive_data_t student_interactive_data_t;			//交互数据接收信息

ext_game_robot_state_t game_robot_state_t; 							//速率上限-用于超级电容

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));

    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));

    memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));
    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));

    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));
}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;


        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(field_event));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
        }
        break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
        {
            memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(robot_state));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
        }
        break;
        default:
        {
            break;
        }
    }
}

/**
  * @brief          从FIFO缓冲区中解包单字节数据，并根据裁判系统协议解析数据包。
  * @param[in]      referee_unpack_obj: 指向解包数据对象的指针，用于存储解包状态和数据。
  * @param[in]      referee_fifo: 指向FIFO缓冲区对象的指针，用于获取接收到的字节流。
  * @retval         none
  *
  * 该函数通过解析FIFO缓冲区中的字节流数据，根据裁判系统协议完成数据包的解包。
  * 解包过程分为以下步骤：
  * 1. 匹配帧头（SOF），确保数据包的起始。
  * 2. 获取数据长度低位和高位，确定数据包的总长度。
  * 3. 校验帧头部分的 CRC8 校验码，确保帧头完整性。
  * 4. 获取完整的数据部分并进行 CRC16 校验，确保数据完整性。
  * 5. 如果数据包完整且校验通过，则调用 `referee_data_solve` 处理解析后的数据。
  *
  * ### 注意事项：
  * - 如果帧头校验或数据校验失败，函数会重置解包状态并重新尝试解包。
  * - 该函数使用状态机的方式逐字节解析数据包，解包状态存储在 `unpack_data_t` 中。
  */
void referee_unpack_fifo_data(unpack_data_t *referee_unpack_obj, fifo_s_t *referee_fifo)
{
	uint8_t byte = 0;                      // 临时存储从FIFO读取的字节
	uint8_t sof = HEADER_SOF;              // 定义帧头的起始符
	unpack_data_t *p_obj = referee_unpack_obj; // 指向解包对象的指针

	// 当FIFO中还有数据时，循环解析字节
	while (fifo_s_used(referee_fifo))
	{
		byte = fifo_s_get(referee_fifo); // 从FIFO中获取一个字节

		// 根据解包步骤解析数据
		switch (p_obj->unpack_step)
		{
		case STEP_HEADER_SOF:
		{
			// 如果匹配到帧头SOF，进入下一步骤；否则重置索引
			if (byte == sof)
			{
				p_obj->unpack_step = STEP_LENGTH_LOW;
				p_obj->protocol_packet[p_obj->index++] = byte;
			}
			else
			{
				p_obj->index = 0;
			}
		} break;

		case STEP_LENGTH_LOW:
		{
			// 保存数据长度的低字节，并进入下一步骤
			p_obj->data_len = byte;
			p_obj->protocol_packet[p_obj->index++] = byte;
			p_obj->unpack_step = STEP_LENGTH_HIGH;
		} break;

		case STEP_LENGTH_HIGH:
		{
			// 保存数据长度的高字节，计算完整长度并校验
			p_obj->data_len |= (byte << 8);
			p_obj->protocol_packet[p_obj->index++] = byte;

			if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
			{
				p_obj->unpack_step = STEP_FRAME_SEQ;
			}
			else
			{
				// 如果数据长度超出范围，重置解包状态
				p_obj->unpack_step = STEP_HEADER_SOF;
				p_obj->index = 0;
			}
		} break;

		case STEP_FRAME_SEQ:
		{
			// 保存帧序列号并进入CRC8校验步骤
			p_obj->protocol_packet[p_obj->index++] = byte;
			p_obj->unpack_step = STEP_HEADER_CRC8;
		} break;

		case STEP_HEADER_CRC8:
		{
			// 保存CRC8校验字节
			p_obj->protocol_packet[p_obj->index++] = byte;

			// 如果帧头长度已达到，进行CRC8校验
			if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
			{
				if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
				{
					// 校验通过，进入数据部分校验步骤
					p_obj->unpack_step = STEP_DATA_CRC16;
				}
				else
				{
					// 校验失败，重置解包状态
					p_obj->unpack_step = STEP_HEADER_SOF;
					p_obj->index = 0;
				}
			}
		} break;

		case STEP_DATA_CRC16:
		{
			// 保存数据部分字节
			if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
			{
				p_obj->protocol_packet[p_obj->index++] = byte;
			}

			// 如果数据部分接收完成，进行CRC16校验
			if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
			{
				p_obj->unpack_step = STEP_HEADER_SOF;
				p_obj->index = 0;

				if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
				{
					// 数据校验通过，处理数据包
					referee_data_solve(p_obj->protocol_packet);
				}
			}
		} break;

		default:
		{
			// 如果发生意外情况，重置解包状态
			p_obj->unpack_step = STEP_HEADER_SOF;
			p_obj->index = 0;
		} break;
		}
	}
}

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer,fp32 *current_power_limit)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;
	*current_power_limit = game_robot_state_t.chassis_power_limit;
}

//该机器人设置为42mm 1号
uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

uint8_t get_robot_level(void){
	return robot_state.robot_level;
}

void get_robot_power_limit(uint16_t *supercap_power_limit)
{
    *supercap_power_limit = game_robot_state_t.chassis_power_limit;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = robot_state.shooter_id1_42mm_cooling_limit;
    *heat0 = power_heat_data_t.shooter_id1_42mm_cooling_heat;
}

void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_state.shooter_id2_17mm_cooling_limit;
    *heat1 = power_heat_data_t.shooter_id2_17mm_cooling_heat;
}

void get_shoot_speed_limit (uint16_t *shoot_limit )
{
	*shoot_limit = robot_state.shooter_id1_42mm_speed_limit;
}
