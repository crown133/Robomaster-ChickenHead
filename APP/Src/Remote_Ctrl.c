#include "usart.h"
#include "Pc_Uart.h"
#include "Remote_Ctrl.h"
#include "math.h"

uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];  //����һ���������ڴ�Ŵ�DMA���յ���ң��������


RemoteCtrl_t RemoteCtrlData;       //ң��������
int8_t RemoteCtrlFlag = 0;			//ң�������ձ�־λ
uint8_t mouseRFlag1 = 0, mouseRFlag2 = 0, ReceiveTimes = 0;  //
uint8_t mouseRFlag3 = 0, mouseRFlag4 = 0;
uint8_t mouseRFlag5 = 0, mouseRFlag6 = 0;
uint8_t mouseRFlag7 = 0, mouseRFlag8 = 0;
uint8_t mouseRFlag01 = 0, mouseRFlag02 = 0;
uint8_t mouseRFlag03 = 0, mouseRFlag04 = 0;
uint8_t mouseRFlag05 = 0, mouseRFlag06 = 0;

/**
  * @brief	����ң����Э��Խ��н��յ������ݽ��д���
  * @param	pData:	һ��ָ��8λ���ݵ�ָ��
  * @retval	None
  */
void RC_DataHandle(uint8_t *pData)
{
	if (pData == NULL)
    {
        return;
    }
	
//	ReceiveTimes++;
//	if(ReceiveTimes > 50)
//		ReceiveTimes = 0;
	
	RemoteCtrlFlag = 1;
	/* pData[0]Ϊch0�ĵ�8λ��Data[1]�ĵ�3λch0�ĸ�3λ */
	RemoteCtrlData.remote.ch0 = (uint16_t)(pData[0] | pData[1] << 8) & 0x07FF;
	
	/* pData[1]�ĸ�5λΪch1�ĵ�5λ��pData[2]�ĵ�6λΪch1�ĸ�6λ */
	RemoteCtrlData.remote.ch1 = (uint16_t)(pData[1] >> 3 | pData[2] << 5) & 0x07FF;
	
	/* pData[2]�ĸ�2λΪch2�ĵ�2λ, pData[3]Ϊch2����8λ��pData[4]�ĵ�1λΪch2�ĸ�1λ */
	RemoteCtrlData.remote.ch2 = (uint16_t)(pData[2] >> 6 | pData[3] << 2 | pData[4] << 10) & 0x07FF;
	
	/* pData[4]�ĸ�7λΪch3�ĵ�7λ��pData[5]�ĵ�4λΪch3�ĸ�4λ */
	RemoteCtrlData.remote.ch3 = (uint16_t)(pData[4] >> 1 | pData[5] << 7) & 0x07FF;

	/* pData[5]�ĸ�2λΪs1 */
	RemoteCtrlData.remote.s1  = ((pData[5] >> 6) & 0x03);
	
	/* pData[6]��6��7λΪs2 */
	RemoteCtrlData.remote.s2  = ((pData[5] >> 4) & 0x03);
	
	/* pData[6],pData[7]Ϊx */
	RemoteCtrlData.mouse.x    = (int16_t)(pData[6] | pData[7] << 8);
	
	/* pData[8],pData[9]Ϊy */
	RemoteCtrlData.mouse.y    = (int16_t)(pData[8] | pData[9] << 8);
	
	/* pData[10],pData[11]Ϊz */
	RemoteCtrlData.mouse.z    = (int16_t)(pData[10] | pData[11] << 8);
	
	/* pData[12]Ϊ��� */
	RemoteCtrlData.mouse.press_l = pData[12];
	
	/* pData[13]Ϊ�Ҽ� */		
	if(!mouseRFlag1)
	{
		RemoteCtrlData.mouse.press_r = pData[13];
		
		if(pData[13] == 1)
		mouseRFlag1++;
	}
	if(mouseRFlag1)
	{
		mouseRFlag2++;
	}
	if(mouseRFlag2 > 50)
	{
		mouseRFlag1 = 0;
		mouseRFlag2 = 0;
	}
	
	/* pData[14],pData[15]Ϊ����ֵ */
	{
		if(!mouseRFlag3)
		{
			RemoteCtrlData.key.shift = (int16_t)((pData[14] >> 4) & 0x01);
 			
			if(RemoteCtrlData.key.shift == 1)
			mouseRFlag3++;
		}

		if(mouseRFlag3)
		{
			mouseRFlag4++;
		}
		if(mouseRFlag4 > 50)
		{
			mouseRFlag3 = 0;
			mouseRFlag4 = 0;
		}
	}
	
	{
		if(!mouseRFlag5)
		{
			RemoteCtrlData.key.S = (int16_t)((pData[14] >> 1) & 0x01);	
			if(RemoteCtrlData.key.S == 1)
			mouseRFlag5++;
		}
		if(mouseRFlag5)
		{
			mouseRFlag6++;
		}
		if(mouseRFlag6 > 30)
		{
			mouseRFlag5 = 0;
			mouseRFlag6 = 0;
		}
	}
	
	{
		if(!mouseRFlag7)
		{
			RemoteCtrlData.key.A = (int16_t)((pData[14] >> 2) & 0x01);	
			if(RemoteCtrlData.key.A == 1)
			mouseRFlag7++;
		}
		if(mouseRFlag7)
		{
			mouseRFlag8++;
		}
		if(mouseRFlag8 > 30)
		{
			mouseRFlag7 = 0;
			mouseRFlag8 = 0;
		}
	}

	{
		if(!mouseRFlag01)
		{
			RemoteCtrlData.key.Q = (int16_t)((pData[14] >> 6) & 0x01);	
			if(RemoteCtrlData.key.Q == 1)
			mouseRFlag01++;
		}
		if(mouseRFlag01)
		{
			mouseRFlag02++;
		}
		if(mouseRFlag02 > 30)
		{
			mouseRFlag01 = 0;
			mouseRFlag02 = 0;
		}
	}
	
	{
		if(!mouseRFlag03)
		{
			RemoteCtrlData.key.W = (int16_t)((pData[14]) & 0x01);	
			if(RemoteCtrlData.key.W == 1)
			mouseRFlag03++;
		}
		if(mouseRFlag03)
		{
			mouseRFlag04++;
		}
		if(mouseRFlag04 > 30)
		{
			mouseRFlag03 = 0;
			mouseRFlag04 = 0;
		}
	}
	
	{
		if(!mouseRFlag05)
		{
			RemoteCtrlData.key.E = (int16_t)((pData[14] >> 7) & 0x01);	
			if(RemoteCtrlData.key.E == 1)
			mouseRFlag05++;
		}
		if(mouseRFlag05)
		{
			mouseRFlag06++;
		}
		if(mouseRFlag06 > 30)
		{
			mouseRFlag05 = 0;
			mouseRFlag06 = 0;
		}
	}	
//	RemoteCtrlData.key.shift = (int16_t)((pData[14] >> 4) & 0x01);
//		  if(RemoteCtrlData.key.shift == 1)
//	  {
//		  UART7_TX_BUF[2] = !UART7_TX_BUF[2];
//		  HAL_UART_Transmit(&huart7, UART7_TX_BUF, 3, 10);
//		  RemoteCtrlData.key.shift = 0;
//	  }
	/* ���ֽ��н��� */
	RemoteCtrlData.remote.figWheel = (uint16_t)(pData[16] | pData[17] << 8);
}

/**
  * @brief	��Ӧ��ң�������뺯��
  * @param	None
  * @retval	None
  */
//int16_t RemoteCH_Decode(uint16_t remoteDate)
//{
//	int16_t temValue;
//	temValue = remoteDate - 1024;
//	if(temValue > 0)
//	{
//		return temValue;
//	}
//}

 /*�������ƣ�RemotreCtl_Data_Receive
  *�������ܣ�����ң�������ݣ�λ��USART1���жϺ�����
  *��ڲ�������
  *����ֵ  ����
  */
void RemoteCtl_Data_Receive(void)
{
	uint32_t rx_data_len = 0;														//���ν��ճ���
	
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);									   		  //��������жϵı�־
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF2_6);                       //��� DMA2_Steam2������ɱ�־
		HAL_UART_DMAStop(&huart1);                                                    //��������Ժ�رմ���DMA
		rx_data_len = BSP_USART1_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); //��ȡ��һ����������С���ܳ���-�����ĳ��ȣ�
		HAL_UART_Receive_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);  //��������
		if (rx_data_len == RC_FRAME_LENGTH)                                           //�ж������Ƿ�Ϊ��ȷ�����ݳ���
		{
			RemoteCtrlFlag = 1; //���յ�����
			RC_DataHandle(USART1_DMA_RX_BUF);                                        //�������ݽ��뺯��
		}
		else
		{
			RemoteCtrlFlag = 0;
		}
	}
}

/**
  *�������ƣ�RemotreCtl_Data_Receive
  *�������ܣ�����ң�������ݣ�λ��USART1���жϺ�����
  *��ڲ�������
  *����ֵ  ����
  */
//void RemoteCtl_Data_Receive_Start(void)
//{
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                                   //�����������ж�
//	HAL_UART_Receive_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);
//}
