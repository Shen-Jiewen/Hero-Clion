//
// Created by Rick on 24-11-25.
//

#include "bsp_can.h"


extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

void fdcan1_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter_st;

	// 初始化FDCAN1过滤器
	fdcan_filter_st.IdType = FDCAN_STANDARD_ID;                    // 使用标准标识符(11位)
	fdcan_filter_st.FilterIndex = 0;                               // 过滤器索引0
	fdcan_filter_st.FilterType = FDCAN_FILTER_MASK;                // 标识符屏蔽模式
	fdcan_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;        // 通过过滤器接收的消息放到FIFO0
	fdcan_filter_st.FilterID1 = 0x000;                             // 标识符(起始值)
	fdcan_filter_st.FilterID2 = 0x000;                             // 屏蔽标识符
	HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter_st);    //配置过滤器

	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);

	// 启动FDCAN1
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}

	// 激活FDCAN1的接收中断
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}
}

void fdcan2_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter_st;

	// 初始化FDCAN2过滤器
	fdcan_filter_st.IdType = FDCAN_STANDARD_ID;                        // 使用标准标识符(11位)
	fdcan_filter_st.FilterIndex = 0;                                // 过滤器索引0
	fdcan_filter_st.FilterType = FDCAN_FILTER_MASK;                 // 标识符屏蔽模式
	fdcan_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;         // 通过过滤器接收的消息放到FIFO0
	fdcan_filter_st.FilterID1 = 0x000;                                // 标识符(起始值)
	fdcan_filter_st.FilterID2 = 0x000;                                // 屏蔽标识符
	HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter_st);    //配置过滤器

	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);

	// 启动FDCAN2
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
	{
		Error_Handler();
	}

	// 激活FDCAN2的接收中断
	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}
}

void fdcan3_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter_st;

	// 初始化FDCAN3过滤器
	fdcan_filter_st.IdType = FDCAN_STANDARD_ID;                    // 使用标准标识符(11位)
	fdcan_filter_st.FilterIndex = 0;                            // 过滤器索引0
	fdcan_filter_st.FilterType = FDCAN_FILTER_MASK;                // 标识符屏蔽模式
	fdcan_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;        // 通过过滤器接收的消息放到FIFO0
	fdcan_filter_st.FilterID1 = 0x000;                            // 标识符(起始值)
	fdcan_filter_st.FilterID2 = 0x000;                            // 屏蔽标识符
	HAL_FDCAN_ConfigFilter(&hfdcan3, &fdcan_filter_st);    //配置过滤器

	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);

	// 启动FDCAN3
	if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
	{
		Error_Handler();
	}

	// 激活FDCAN3的接收中断
	if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8]; // 数据缓存

	// 检查触发回调的具体 FDCAN 实例
	if (hfdcan == &hfdcan1)
	{
		// FDCAN1 的接收处理
		if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
		{
			// 处理 FDCAN1 的接收到的消息
			motor_3508_can_callback(RxHeader.Identifier, RxData);
		}
	}
	else if (hfdcan == &hfdcan2)
	{
		// FDCAN2 的接收处理
		if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
		{
			// 处理 FDCAN2 的接收到的消息
			// 摩擦轮3508、6020电机和4310电机的数据处理
			motor_6020_can_callback(RxHeader.Identifier, RxData);
			motor_4310_v41_can_callback(RxHeader.Identifier, RxData);
			shoot_3508_can_callback(RxHeader.Identifier, RxData);
			CAP_CAN_RxCallback(RxHeader.Identifier, RxData);
		}
	}
	else if (hfdcan == &hfdcan3)
	{
		// FDCAN3 的接收处理
		if (HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
		{
			// 处理 FDCAN3 的接收到的消息
		}
	}
}


