//
// Created by 16844 on 2025/1/6.
//
#include "supercap.h"
CapDataTypedef CAP_CANData;
FDCAN_TxHeaderTypeDef  CAP_TxHeader;
uint8_t CAP_CAN_txbuf[8];

void supercap_measure_parse(CapDataTypedef * ptr, const uint8_t *rx_data)
{
    ptr->Pin = (float)((int16_t)(rx_data[0]<<8|rx_data[1]))/100.0f;
    ptr->Pout = (float)((int16_t)(rx_data[2]<<8|rx_data[3]))/100.0f;
    ptr->Ucap = (float)((int16_t)(rx_data[4]<<8|rx_data[4]))/100.0f;
    ptr->Plim = rx_data[6];
    ptr->now_state = rx_data[7];
}


/**
 * @brief CAN接收超级电容数据回调，解析数据
 * @param pCAP_Data
 * @param rx_data
 * @return none
 */
//CAN接收数据回调，解析数据
void CAP_CAN_RxCallback(uint8_t *rx_data, uint16_t can_id)
{
    if (can_id == CAP_TX_ID)
    supercap_measure_parse(&CAP_CANData, rx_data);

}


uint8_t CAP_CAN_DataSend(FDCAN_HandleTypeDef* hfdcan,float lim_power, uint8_t capower_enable)
{
    FDCAN_TxHeaderTypeDef Tx_Header = {
            .Identifier = CAP_RX_ID,
            .IdType = FDCAN_STANDARD_ID,
            .TxFrameType = FDCAN_DATA_FRAME,
            .DataLength = FDCAN_DLC_BYTES_8,
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch = FDCAN_BRS_OFF,
            .FDFormat = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            .MessageMarker = 0
    };

    CAP_CANData.Plim = lim_power;
    CAP_CANData.set_mode = (uint8_t)((capower_enable<<2)|((uint8_t)1<<1)|((uint8_t)0<<0));
    uint32_t send_mail_box;
    int16_t temp_data;
    temp_data = CAP_CANData.Plim*100;
    CAP_CAN_txbuf[0] = temp_data>>8;
    CAP_CAN_txbuf[1] = temp_data&0xff;
    CAP_CAN_txbuf[2] = CAP_CANData.set_mode;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &Tx_Header, CAP_CAN_txbuf) != HAL_OK)
    {
        return 1;   //发送失败
    }
        return 0;   //发送成功
}



//返回数据指针
CapDataTypedef *get_CAPower_measure_point(void)
{
    return &CAP_CANData;
}

