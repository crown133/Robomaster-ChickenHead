#include "Referee_Comm.h"
#include "usart.h"
#include "string.h"
#include "DataScope_DP.h"

ext_referee_data_t RefereeData_t;

uint8_t USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];
uint8_t USART6_DMA_TX_BUF[BSP_USART6_DMA_TX_BUF_LEN];

void Referee_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);                                       //�����������ж�
	HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);
}

void Referee_Data_Receive(void)
{
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);                                           //��������жϵı�־
		(void)USART6->SR;                                                             //���SR�Ĵ���
		(void)USART6->DR;                                                             //���DR�Ĵ���
		__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx,DMA_FLAG_TCIF1_5);                       //��� DMA2_Steam1������ɱ�־
		HAL_UART_DMAStop(&huart6);                                                    //��������Ժ�رմ���DMA
		HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);  //��������
		if(USART6_DMA_RX_BUF[0] == FRAME_HEADER_SOF)                                  //�ж����ݰ�֡ͷ
		{
			Referee_Decode(USART6_DMA_RX_BUF);                                         //�������ݽ��뺯��
		}
	}
}

void Referee_Decode(uint8_t *pData)
{
	uint8_t frameLoc = 0;
	uint8_t *frameHeadLoc;					//�ݴ浱ǰ֡��֡ͷ��ַ
	uint16_t dataLength, cmdID;
	
	while (frameLoc < BSP_USART6_DMA_RX_BUF_LEN)		//������ֻ�ܱ���128���ֽڣ�ѭ������ǲ��������ݰ�����
	{
		/* ��ǰ֡��֡ͷ�׵�ַΪpData + frameLoc */
		if (pData[frameLoc] == FRAME_HEADER_SOF)
		{
			if (Verify_CRC8_Check_Sum(pData + frameLoc, FRAME_HEADER_LEN) == 1)		//֡ͷCRC8У��ɹ�
			{
				frameHeadLoc = pData + frameLoc;	//�����ݴ棬�򻯺���Ĵ���
				memcpy(&dataLength, frameHeadLoc + 1, 2);	//��ȡ���ݰ����Ⱥ�У������
				if (Verify_CRC16_Check_Sum(pData + frameLoc, FRAME_HEADER_LEN + CMD_ID_LEN + dataLength + CRC16_LEN) == 1)
				{
					memcpy(&cmdID, frameHeadLoc + FRAME_HEADER_LEN, 2);
					switch (cmdID)
					{
						case GAME_STATE_CMD_ID:		//1Hz
							memcpy(&RefereeData_t.GameState_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_STATE_LEN);
							break;
						case GAME_RESULT_CMD_ID:	//������������
							memcpy(&RefereeData_t.GameResult_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_RESULT_LEN);
							break;
						case GAME_ROBOTSURVIVORS_CMD_ID:	//1Hz����
							memcpy(&RefereeData_t.GameRobotSurvivors_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_ROBOTSURVIVORS_LEN);
							break;
						case EVENT_DATA_CMD_ID:		//�¼��ı����
							memcpy(&RefereeData_t.EventData_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, EVENT_DATA_LEN);
							break;
//						case SUPPLY_PROJECTILE_ACTION_CMD_ID:	//�����ı����
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
						case BUFF_MUSK_CMD_ID:			//����״̬�ı����
							memcpy(&RefereeData_t.BuffMusk_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, BUFF_MUSK_LEN);
							break;
//						case AERIAL_ROBOT_ENERGY_CMD_ID:	//���л���������״̬���ݣ�10Hz
//							memcpy(&RefereeData_t.AerialRobotEnergy_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, AERIAL_ROBOT_ENERGY_LEN);
//							break;
						case ROBOT_HURT_CMD_ID:			//�˺���������
							memcpy(&RefereeData_t.RobotHurt_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, ROBOT_HURT_LEN);
							break;
						case SHOOT_DATA_CMD_ID:			//�ӵ��������
							memcpy(&RefereeData_t.ShootData_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, SHOOT_DATA_LEN);
							break;
						default:
							break;
					}
					frameLoc += FRAME_HEADER_LEN + CMD_ID_LEN + dataLength + CRC16_LEN;		//����������ϣ����һ����������
				}
				else
					frameLoc += FRAME_HEADER_LEN;		//֡ͷ��ȷ�Ұ�ͷУ����ȷ�����������������һ����ͷ����
			}
			else
				frameLoc++;			//֡ͷ��ȷ�����ǰ�ͷУ��������һ���ֽ�
		}
		else
			frameLoc++;			//δʶ��֡ͷ�����һ���ֽ�
	}
}
