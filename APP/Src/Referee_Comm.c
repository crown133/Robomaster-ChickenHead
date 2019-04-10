#include "Referee_Comm.h"
#include "usart.h"
#include "string.h"
#include "DataScope_DP.h"

ext_referee_data_t RefereeData_t;

uint8_t USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];
uint8_t USART6_DMA_TX_BUF[BSP_USART6_DMA_TX_BUF_LEN];

void Referee_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);                                       //开启不定长中断
	HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);
}

void Referee_Data_Receive(void)
{
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);                                           //清除空闲中断的标志
		(void)USART6->SR;                                                             //清空SR寄存器
		(void)USART6->DR;                                                             //清空DR寄存器
		__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx,DMA_FLAG_TCIF1_5);                       //清除 DMA2_Steam1传输完成标志
		HAL_UART_DMAStop(&huart6);                                                    //传输完成以后关闭串口DMA
		HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);  //接受数据
		if(USART6_DMA_RX_BUF[0] == FRAME_HEADER_SOF)                                  //判断数据包帧头
		{
			Referee_Decode(USART6_DMA_RX_BUF);                                         //进入数据解码函数
		}
	}
}

void Referee_Decode(uint8_t *pData)
{
	uint8_t frameLoc = 0;
	uint8_t *frameHeadLoc;					//暂存当前帧的帧头地址
	uint16_t dataLength, cmdID;
	
	while (frameLoc < BSP_USART6_DMA_RX_BUF_LEN)		//缓存区只能保存128个字节，循环检查是不是有数据包在内
	{
		/* 当前帧的帧头首地址为pData + frameLoc */
		if (pData[frameLoc] == FRAME_HEADER_SOF)
		{
			if (Verify_CRC8_Check_Sum(pData + frameLoc, FRAME_HEADER_LEN) == 1)		//帧头CRC8校验成功
			{
				frameHeadLoc = pData + frameLoc;	//单纯暂存，简化后面的代码
				memcpy(&dataLength, frameHeadLoc + 1, 2);	//获取数据包长度后校验整包
				if (Verify_CRC16_Check_Sum(pData + frameLoc, FRAME_HEADER_LEN + CMD_ID_LEN + dataLength + CRC16_LEN) == 1)
				{
					memcpy(&cmdID, frameHeadLoc + FRAME_HEADER_LEN, 2);
					switch (cmdID)
					{
						case GAME_STATE_CMD_ID:		//1Hz
							memcpy(&RefereeData_t.GameState_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_STATE_LEN);
							break;
						case GAME_RESULT_CMD_ID:	//比赛结束后发送
							memcpy(&RefereeData_t.GameResult_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_RESULT_LEN);
							break;
						case GAME_ROBOTSURVIVORS_CMD_ID:	//1Hz发送
							memcpy(&RefereeData_t.GameRobotSurvivors_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_ROBOTSURVIVORS_LEN);
							break;
						case EVENT_DATA_CMD_ID:		//事件改变后发送
							memcpy(&RefereeData_t.EventData_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, EVENT_DATA_LEN);
							break;
//						case SUPPLY_PROJECTILE_ACTION_CMD_ID:	//动作改变后发送
//							memcpy(&RefereeData_t.SupplyProjectileAction_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, SUPPLY_PROJECTILE_ACTION_LEN);
//							break;
						case GAME_ROBOT_STATE_CMD_ID:	//10Hz
							memcpy(&RefereeData_t.GameRobotState_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_ROBOT_STATE_LEN);
							break;
						case POWER_HEAT_DATA_CMD_ID:	//50Hz
							memcpy(&RefereeData_t.PowerHeatData_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, POWER_HEAT_DATA_LEN);
							break;
						case GAME_ROBOT_POS_CMD_ID:		//10Hz
							memcpy(&RefereeData_t.GameRobotPos_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_ROBOT_POS_LEN);
							break;
						case BUFF_MUSK_CMD_ID:			//增益状态改变后发送
							memcpy(&RefereeData_t.BuffMusk_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, BUFF_MUSK_LEN);
							break;
//						case AERIAL_ROBOT_ENERGY_CMD_ID:	//空中机器人能量状态数据，10Hz
//							memcpy(&RefereeData_t.AerialRobotEnergy_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, AERIAL_ROBOT_ENERGY_LEN);
//							break;
						case ROBOT_HURT_CMD_ID:			//伤害发生后发送
							memcpy(&RefereeData_t.RobotHurt_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, ROBOT_HURT_LEN);
							break;
						case SHOOT_DATA_CMD_ID:			//子弹发射后发送
							memcpy(&RefereeData_t.ShootData_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, SHOOT_DATA_LEN);
							break;
						default:
							break;
					}
					frameLoc += FRAME_HEADER_LEN + CMD_ID_LEN + dataLength + CRC16_LEN;		//整包处理完毕，向后一个整包长度
				}
				else
					frameLoc += FRAME_HEADER_LEN;		//帧头正确且包头校正正确，但是整包出错，向后一个包头长度
			}
			else
				frameLoc++;			//帧头正确，但是包头校验出错，向后一个字节
		}
		else
			frameLoc++;			//未识别帧头，向后一个字节
	}
}
