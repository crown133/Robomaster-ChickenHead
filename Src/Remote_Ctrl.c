#include "usart.h"
#include "can.h"
#include "Remote_Ctrl.h"
#include "Motor_Ctrl.h"
#include "math.h"

uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];  //����һ���������ڴ�Ŵ�DMA���յ���ң��������


RemoteCtrl_t RemoteCtrlData;       //ң��������
int8_t RemoteCtrlFlag = 0;			//ң�������ձ�־λ

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
	RemoteCtrlData.mouse.press_r = pData[13];
	
	/* pData[14],pData[15]Ϊ����ֵ */
	RemoteCtrlData.key.v 		 = (int16_t)(pData[14] | pData[15] << 8);
	
	/* �������ؽ��н��� */

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
	uint32_t rx_data_len = 0;															//���ν��ճ���
	RemoteCtrlFlag = 1; //���յ�����
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);												//��������жϵı�־
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF2_6);                       //��� DMA2_Steam2������ɱ�־
		HAL_UART_DMAStop(&huart1);                                                    //��������Ժ�رմ���DMA
		rx_data_len = BSP_USART1_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); //��ȡ��һ����������С���ܳ���-�����ĳ��ȣ�
		HAL_UART_Receive_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);  //��������
		if (rx_data_len == RC_FRAME_LENGTH)                                           //�ж������Ƿ�Ϊ��ȷ�����ݳ���
		{
			RC_DataHandle(USART1_DMA_RX_BUF);                                        //�������ݽ��뺯��
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
