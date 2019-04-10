#include "Pc_Uart.h"
#include "string.h"
#include "usart.h"


uint8_t UART7_DMA_RX_BUF[UART7_DMA_RX_BUF_LEN];

uint32_t rxLength;

float yawInc, pitchInc;

void pcUartDecode(void); //串口解码函数

void pcUartReceive(void)
{
	uint32_t rx_data_len = 0;															//本次接收长度
	
	if((__HAL_UART_GET_FLAG(&huart7,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart7);									   		//清除空闲中断的标志
		__HAL_DMA_CLEAR_FLAG(&hdma_uart7_rx,DMA_FLAG_TCIF2_6);                       //清除 DMA1_Steam3传输完成标志
		HAL_UART_DMAStop(&huart7);                                                    //传输完成以后关闭串口DMA
		rx_data_len = UART7_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_uart7_rx); //获取这一次数据量大小（总长度-保留的长度）
		HAL_UART_Receive_DMA(&huart7, UART7_DMA_RX_BUF, UART7_DMA_RX_BUF_LEN);  //接受数据
		if (rx_data_len == pcDataLength)                                           //判断数据是否为正确的数据长度
		{
			pcUartDecode();                                        //进入数据解码函数
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




