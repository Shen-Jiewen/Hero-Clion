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
	fdcan_filter_st.FilterIndex = 0;                            // 过滤器索引0
	fdcan_filter_st.FilterType = FDCAN_FILTER_MASK;                // 标识符屏蔽模式
	fdcan_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;        // 通过过滤器接收的消息放到FIFO0
	fdcan_filter_st.FilterID1 = 0x000;                            // 标识符(起始值)
	fdcan_filter_st.FilterID2 = 0x000;                            // 屏蔽标识符
	HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter_st);    //配置过滤器

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
	fdcan_filter_st.IdType = FDCAN_STANDARD_ID;                    // 使用标准标识符(11位)
	fdcan_filter_st.FilterIndex = 0;                            // 过滤器索引0
	fdcan_filter_st.FilterType = FDCAN_FILTER_MASK;                // 标识符屏蔽模式
	fdcan_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;        // 通过过滤器接收的消息放到FIFO0
	fdcan_filter_st.FilterID1 = 0x000;                            // 标识符(起始值)
	fdcan_filter_st.FilterID2 = 0x000;                            // 屏蔽标识符
	HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter_st);    //配置过滤器

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