#include "Referee_Comm.h"
#include "CRC.h"
#include "usart.h"
#include "string.h"
#include "Shoot_Ctrl.h"
#include "Remote_Ctrl.h"

uint8_t USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN];  //����һ���������ڴ�Ŵ�DMA���յ��Ĳ���ϵͳ����
uint8_t USART3_DMA_TX_BUF[28];  //19+9=28

uint8_t send_flag;

uint8_t guard_state;  //shaobing

ext_referee_data_t  RefereeData_t;
send_data_t send_data;  	 //�ش������˻��ͻ���
send_data_guard send_guard;  //���ձ���֡

/**
  * @brief :  ��ʼ�������ϵͳ�������ݵ�֡ͷ  
  *  Ĭ�ϳ�ʼ��Ϊ�췽�����ˣ� ͨ�����ذ����л�Ϊ����������
  */
void data_head_init(void)
{
	//����֡ͷ
	send_data.frame_header.sof = FRAME_HEADER_SOF;
	send_data.frame_header.dataLength = 19;
	send_data.frame_header.seq = 0;
	send_data.cmd_id = 0x301;
	//�������ݰ�ͷ
	send_data.client_header.data_cmd_id = 0xD180;     	//�ͻ����Զ�������ID
	send_data.client_header.receiver_ID = 0x106;     	//���в����ֿͻ���
	send_data.client_header.send_ID = 6;              	//�췽���˻�
	
//	send_data.client_header.receiver_ID = 0x0116;     	//���в����ֿͻ���
//	send_data.client_header.send_ID = 16;              	//�������˻�
	
	/***************************************************/
	
	send_guard.frame_header.sof = FRAME_HEADER_SOF;
	send_guard.frame_header.dataLength = 7;
	send_guard.frame_header.seq = 0;
	send_guard.cmd_id = 0x301;
	
	send_guard.client_header.data_cmd_id = 0x0201;     	  //���ձ��Զ�������ID
	send_guard.client_header.receiver_ID = 7;     		  //�췽�ձ�ID
	send_guard.client_header.send_ID = 6;              	  //�췽���˻�
	
}

void data_head_blue_init(void)
{
  send_data.client_header.receiver_ID = 0x0116;     	//���в����ֿͻ���
  send_data.client_header.send_ID = 16;              	//�������˻�
  
  send_guard.client_header.receiver_ID = 17;			//�����ձ�
  send_guard.client_header.send_ID = 16;
}
/**
  * @brief :  �������ϵͳ����
  */
void Referee_Decode(uint8_t *pData)
{
	uint8_t frameLoc = 0;
	uint8_t *frameHeadLoc;						//�ݴ浱ǰ֡��֡ͷ��ַ
	uint16_t dataLength, cmdID;
	
	while (frameLoc < BSP_USART3_DMA_RX_BUF_LEN)//������ֻ�ܱ���128���ֽڣ�ѭ������ǲ��������ݰ�����
	{
		/* ��ǰ֡��֡ͷ�׵�ַΪpData + frameLoc */
		if (pData[frameLoc] == FRAME_HEADER_SOF)
		{
			if (Verify_CRC8_Check_Sum(pData + frameLoc, FRAME_HEADER_LEN) == 1)		//֡ͷCRC8У��ɹ�
			{
				frameHeadLoc = pData + frameLoc;			//�����ݴ棬�򻯺���Ĵ���
				memcpy(&dataLength, frameHeadLoc + 1, 2);	//��ȡ���ݰ����Ⱥ�У������
				if (Verify_CRC16_Check_Sum(pData + frameLoc, FRAME_HEADER_LEN + CMD_ID_LEN + dataLength + CRC16_LEN) == 1)
				{
					memcpy(&cmdID, frameHeadLoc + FRAME_HEADER_LEN, 2);
					switch (cmdID)
					{
//						case GAME_STATE_CMD_ID:		//1Hz
//							memcpy(&RefereeData_t.GameState_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_STATE_LEN);
//							break;
//						case GAME_RESULT_CMD_ID:	//������������
//							memcpy(&RefereeData_t.GameResult_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_RESULT_LEN);
//  						break;
//						case GAME_ROBOTSURVIVORS_CMD_ID:	//1Hz����
//							memcpy(&RefereeData_t.GameRobotSurvivors_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_ROBOTSURVIVORS_LEN);
//							break;
//						case EVENT_DATA_CMD_ID:		//�¼��ı����
//							memcpy(&RefereeData_t.EventData_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, EVENT_DATA_LEN);
//							break;
//						case SUPPLY_PROJECTILE_ACTION_CMD_ID:	//�����ı����
//							memcpy(&RefereeData_t.SupplyProjectileAction_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, SUPPLY_PROJECTILE_ACTION_LEN);
//							break;
//						case GAME_ROBOT_STATE_CMD_ID:	//10Hz
//							memcpy(&RefereeData_t.GameRobotState_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_ROBOT_STATE_LEN);
//							break;
//						case POWER_HEAT_DATA_CMD_ID:	//50Hz
//							memcpy(&RefereeData_t.PowerHeatData_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, POWER_HEAT_DATA_LEN);
//							break;
//						case GAME_ROBOT_POS_CMD_ID:		//10Hz
//							memcpy(&RefereeData_t.GameRobotPos_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_ROBOT_POS_LEN);
//							break;
//						case BUFF_MUSK_CMD_ID:			//����״̬�ı����
//							memcpy(&RefereeData_t.BuffMusk_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, BUFF_MUSK_LEN);
//							break;
//						case AERIAL_ROBOT_ENERGY_CMD_ID:	//���л���������״̬���ݣ�10Hz
//							memcpy(&RefereeData_t.AerialRobotEnergy_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, AERIAL_ROBOT_ENERGY_LEN);
//							break;
//						case ROBOT_HURT_CMD_ID:			//�˺���������
//							memcpy(&RefereeData_t.RobotHurt_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, ROBOT_HURT_LEN);
//							break;
//						case SHOOT_DATA_CMD_ID:			//�ӵ��������
//							memcpy(&RefereeData_t.ShootData_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, SHOOT_DATA_LEN);
//							break;
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

/**
  * @brief :  ����Ҫ���͸�����ϵͳ���Զ�������
  */

void SendDatabuild(void)
{
//	send_flag = !send_flag;  //ÿ���л�����Ŀ�� ���˻��ͻ��˺��ձ�
	send_data.frame_header.seq++;
	send_guard.frame_header.seq = send_data.frame_header.seq;  //������һ��
	
//	//��������
//	if(send_flag)
	{
		send_data.client_data.data1 = mouseR;
	//	send_data.client_data.data2 = ;   
	//	send_data.client_data.data3 = ;
		send_data.client_data.mask = RemoteCtrlData.key.shift;  //��1 ��ɫΪ���鿪������ɫΪ����ر�
		//����֡ͷ
		memcpy(USART3_DMA_TX_BUF,&send_data, 4);
		//����CRC8�ֽ�
		send_data.frame_header.crc8 = Get_CRC8_Check_Sum(USART3_DMA_TX_BUF,4,0xff);	
		memcpy(USART3_DMA_TX_BUF,&send_data, 26);
		//����CRC16�ֽ�
		send_data.CRC16 = Get_CRC16_Check_Sum(USART3_DMA_TX_BUF ,26, 0xffff);
		//�����ݿ�����������
		memcpy(USART3_DMA_TX_BUF,&send_data, 28);
	}

}
