#include "Pc_Uart.h"
#include "string.h"
#include "usart.h"


uint8_t UART7_DMA_RX_BUF[UART7_DMA_RX_BUF_LEN];

uint32_t rxLength;

float yawInc, pitchInc;

void pcUartDecode(void); //���ڽ��뺯��

void pcUartReceive(void)
{
	uint32_t rx_data_len = 0;															//���ν��ճ���
	
	if((__HAL_UART_GET_FLAG(&huart7,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart7);									   		//��������жϵı�־
		__HAL_DMA_CLEAR_FLAG(&hdma_uart7_rx,DMA_FLAG_TCIF2_6);                       //��� DMA1_Steam3������ɱ�־
		HAL_UART_DMAStop(&huart7);                                                    //��������Ժ�رմ���DMA
		rx_data_len = UART7_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_uart7_rx); //��ȡ��һ����������С���ܳ���-�����ĳ��ȣ�
		HAL_UART_Receive_DMA(&huart7, UART7_DMA_RX_BUF, UART7_DMA_RX_BUF_LEN);  //��������
		if (rx_data_len == pcDataLength)                                           //�ж������Ƿ�Ϊ��ȷ�����ݳ���
		{
			pcUartDecode();                                        //�������ݽ��뺯��
		}
	}
	rxLength = rx_data_len;
}

void pcUartDecode(void)
{
	if((UART7_DMA_RX_BUF[0] == 0xA5) && (UART7_DMA_RX_BUF[1] == 0x5A))
	{
		memcpy(&yawInc, UART7_DMA_RX_BUF+2, 4);
		memcpy(&pitchInc, UART7_DMA_RX_BUF+6, 4);
//		pitchInc = (UART7_DMA_RX_BUF[9]<<24 | UART7_DMA_RX_BUF[8]<<16 | UART7_DMA_RX_BUF[7]<<8 | UART7_DMA_RX_BUF[6]);
	}
	else
	{
		yawInc = 0;
		pitchInc = 0;
	}
}	




