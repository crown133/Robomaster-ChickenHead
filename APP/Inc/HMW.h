#pragma once

#include "sys.h"
#include "Motor_Ctrl.h"
#include "usart.h"

#define USART6_DMA_TX_Len 10     //DMA �����ֽڸ���
#define USART6_DMA_RX_Len 4      //DMA �����ֽڸ���


static uint8_t USART6_DMA_TX_BUF[USART6_DMA_TX_Len];
static uint8_t USART6_DMA_RX_BUF[USART6_DMA_RX_Len];

static inline void moto_angle_send(void)//DMA���ͽǶȿ����ź�
{		
		USART6_DMA_TX_BUF[0]=0xA1;//�Ƕȿ�������ͷ��
//		USART6_DMA_TX_BUF[1]=(int)td2.v1 >> 8;//���1�Ƕȸ�λ
//		USART6_DMA_TX_BUF[2]=(int)td2.v1;	 //���1�Ƕȵ�λ
		USART6_DMA_TX_BUF[1]=0xF;//���1�Ƕȸ�λ
		USART6_DMA_TX_BUF[2]=0xA0;	 //���1�Ƕȵ�λ
//		USART6_DMA_TX_BUF[3]=moto2_angle>>8;//���1�Ƕȸ�λ
//		USART6_DMA_TX_BUF[4]=moto2_angle;	 //���1�Ƕȵ�λ
//		USART6_DMA_TX_BUF[5]=moto3_angle>>8;//���1�Ƕȸ�λ
//		USART6_DMA_TX_BUF[6]=moto3_angle;	 //���1�Ƕȵ�λ
//		USART6_DMA_TX_BUF[7]=moto4_angle>>8;//���1�Ƕȸ�λ
//		USART6DMA_TX_BUF[8]=moto4_angle;	 //���1�Ƕȵ�λ	
//		USART6_DMA_TX_BUF[9]=USART6_DMA_TX_BUF[0]+USART6_DMA_TX_BUF[1]+USART6_DMA_TX_BUF[2]+USART6_DMA_TX_BUF[3]
//								 +USART6_DMA_TX_BUF[4]+USART6_DMA_TX_BUF[5]+USART6_DMA_TX_BUF[6]+USART6_DMA_TX_BUF[7]
//								 +USART6_DMA_TX_BUF[8];//ǰ9λ�ֽ�У���	
		USART6_DMA_TX_BUF[9]=USART6_DMA_TX_BUF[0]+USART6_DMA_TX_BUF[1]+USART6_DMA_TX_BUF[2];//ǰ9λ�ֽ�У���
	
		HAL_UART_DMAStop(&huart6);
		HAL_UART_Transmit_DMA(&huart6, USART6_DMA_TX_BUF, USART6_DMA_TX_Len);
	
//	DMA_Cmd(DMA1_Channel4,DISABLE);//�رմ���1DMA����
//		DMA_SetCurrDataCounter(DMA1_Channel4,DMA_TX_Len);//�����ط���������װ��DMA����
//		DMA_Cmd(DMA1_Channel4, ENABLE);	//�򿪴���1DMAʹ�ܷ���	
}

static inline void moto_moment_send(void)//DMA�������ؿ����ź�
{		
		USART6_DMA_TX_BUF[0]=0xA0;//�Ƕȿ�������ͷ��
		USART6_DMA_TX_BUF[1]=(int)motorPitch.veloCtrl.output>>8;//���1�Ƕȸ�λ
		USART6_DMA_TX_BUF[2]=(int)motorPitch.veloCtrl.output;	  //���1�Ƕȵ�λ
//		USART6_DMA_TX_BUF[3]=moto2_moment>>8;//���1�Ƕȸ�λ
//		USART6_DMA_TX_BUF[4]=moto2_moment;	  //���1�Ƕȵ�λ
//		USART6_DMA_TX_BUF[5]=moto3_moment>>8;//���1�Ƕȸ�λ
//		USART6_DMA_TX_BUF[6]=moto3_moment;	  //���1�Ƕȵ�λ
//		USART6_DMA_TX_BUF[7]=moto4_moment>>8;//���1�Ƕȸ�λ
//		USART6_DMA_TX_BUF[8]=moto4_moment;	  //���1�Ƕȵ�λ	
		USART6_DMA_TX_BUF[9]=USART6_DMA_TX_BUF[0]+USART6_DMA_TX_BUF[1]+USART6_DMA_TX_BUF[2]+USART6_DMA_TX_BUF[3]
								 +USART6_DMA_TX_BUF[4]+USART6_DMA_TX_BUF[5]+USART6_DMA_TX_BUF[6]+USART6_DMA_TX_BUF[7]
								 +USART6_DMA_TX_BUF[8];//ǰ9λ�ֽ�У���	
	
		HAL_UART_DMAStop(&huart6);
		HAL_UART_Transmit_DMA(&huart6, USART6_DMA_TX_BUF, USART6_DMA_TX_Len);
}

static inline void get_moto_Practical_angle(void)
{
	uint32_t rx_data_len = 0;														//���ν��ճ���
	
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);									   		  //��������жϵı�־
		__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx,DMA_FLAG_TCIF1_5);                       //��� DMA2_Steam1������ɱ�־
		HAL_UART_DMAStop(&huart6);                                                    //��������Ժ�رմ���DMA
		rx_data_len = USART6_DMA_RX_Len - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx); //��ȡ��һ����������С���ܳ���-�����ĳ��ȣ�
		HAL_UART_Receive_DMA(&huart1, USART6_DMA_RX_BUF, USART6_DMA_RX_Len);  //��������
		if (rx_data_len == USART6_DMA_RX_Len)                                           //�ж������Ƿ�Ϊ��ȷ�����ݳ���
		{
	
			if(USART6_DMA_RX_BUF[3]==(USART6_DMA_RX_BUF[0]+USART6_DMA_RX_BUF[1]+USART6_DMA_RX_BUF[2]))//У��ͨ��	
			{
				if(USART6_DMA_RX_BUF[0]==0XB1)
					motorPitch.posCtrl.motorPos=(USART6_DMA_RX_BUF[1]<<8)+USART6_DMA_RX_BUF[2];//��ȡ�ӻ�1����������
		//		else if(USART6_DMA_RX_BUF[0]==0XB2)
		//			moto2_Practical_angle=(USART6_DMA_RX_BUF[1]<<8)+USART6_DMA_RX_BUF[2];//��ȡ�ӻ�2����������
		//		else if(USART6_DMA_RX_BUF[0]==0XB3)
		//			moto3_Practical_angle=(USART6_DMA_RX_BUF[1]<<8)+USART6_DMA_RX_BUF[2];//��ȡ�ӻ�3����������
		//		else if(USART6_DMA_RX_BUF[0]==0XB4)
		//			moto4_Practical_angle=(USART6_DMA_RX_BUF[1]<<8)+USART6_DMA_RX_BUF[2];//��ȡ�ӻ�4����������
			}
		}
	}
}

