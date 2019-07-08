#pragma once

#include "sys.h"
#include "Motor_Ctrl.h"
#include "usart.h"

#define USART6_DMA_TX_Len 10     //DMA 发送字节个数
#define USART6_DMA_RX_Len 4      //DMA 接受字节个数


static uint8_t USART6_DMA_TX_BUF[USART6_DMA_TX_Len];
static uint8_t USART6_DMA_RX_BUF[USART6_DMA_RX_Len];

static inline void moto_angle_send(void)//DMA发送角度控制信号
{		
		USART6_DMA_TX_BUF[0]=0xA1;//角度控制命令头码
//		USART6_DMA_TX_BUF[1]=(int)td2.v1 >> 8;//电机1角度高位
//		USART6_DMA_TX_BUF[2]=(int)td2.v1;	 //电机1角度低位
		USART6_DMA_TX_BUF[1]=0xF;//电机1角度高位
		USART6_DMA_TX_BUF[2]=0xA0;	 //电机1角度低位
//		USART6_DMA_TX_BUF[3]=moto2_angle>>8;//电机1角度高位
//		USART6_DMA_TX_BUF[4]=moto2_angle;	 //电机1角度低位
//		USART6_DMA_TX_BUF[5]=moto3_angle>>8;//电机1角度高位
//		USART6_DMA_TX_BUF[6]=moto3_angle;	 //电机1角度低位
//		USART6_DMA_TX_BUF[7]=moto4_angle>>8;//电机1角度高位
//		USART6DMA_TX_BUF[8]=moto4_angle;	 //电机1角度低位	
//		USART6_DMA_TX_BUF[9]=USART6_DMA_TX_BUF[0]+USART6_DMA_TX_BUF[1]+USART6_DMA_TX_BUF[2]+USART6_DMA_TX_BUF[3]
//								 +USART6_DMA_TX_BUF[4]+USART6_DMA_TX_BUF[5]+USART6_DMA_TX_BUF[6]+USART6_DMA_TX_BUF[7]
//								 +USART6_DMA_TX_BUF[8];//前9位字节校验和	
		USART6_DMA_TX_BUF[9]=USART6_DMA_TX_BUF[0]+USART6_DMA_TX_BUF[1]+USART6_DMA_TX_BUF[2];//前9位字节校验和
	
		HAL_UART_DMAStop(&huart6);
		HAL_UART_Transmit_DMA(&huart6, USART6_DMA_TX_BUF, USART6_DMA_TX_Len);
	
//	DMA_Cmd(DMA1_Channel4,DISABLE);//关闭串口1DMA发送
//		DMA_SetCurrDataCounter(DMA1_Channel4,DMA_TX_Len);//设置重发个数重新装载DMA数据
//		DMA_Cmd(DMA1_Channel4, ENABLE);	//打开串口1DMA使能发送	
}

static inline void moto_moment_send(void)//DMA发送力矩控制信号
{		
		USART6_DMA_TX_BUF[0]=0xA0;//角度控制命令头码
		USART6_DMA_TX_BUF[1]=(int)motorPitch.veloCtrl.output>>8;//电机1角度高位
		USART6_DMA_TX_BUF[2]=(int)motorPitch.veloCtrl.output;	  //电机1角度低位
//		USART6_DMA_TX_BUF[3]=moto2_moment>>8;//电机1角度高位
//		USART6_DMA_TX_BUF[4]=moto2_moment;	  //电机1角度低位
//		USART6_DMA_TX_BUF[5]=moto3_moment>>8;//电机1角度高位
//		USART6_DMA_TX_BUF[6]=moto3_moment;	  //电机1角度低位
//		USART6_DMA_TX_BUF[7]=moto4_moment>>8;//电机1角度高位
//		USART6_DMA_TX_BUF[8]=moto4_moment;	  //电机1角度低位	
		USART6_DMA_TX_BUF[9]=USART6_DMA_TX_BUF[0]+USART6_DMA_TX_BUF[1]+USART6_DMA_TX_BUF[2]+USART6_DMA_TX_BUF[3]
								 +USART6_DMA_TX_BUF[4]+USART6_DMA_TX_BUF[5]+USART6_DMA_TX_BUF[6]+USART6_DMA_TX_BUF[7]
								 +USART6_DMA_TX_BUF[8];//前9位字节校验和	
	
		HAL_UART_DMAStop(&huart6);
		HAL_UART_Transmit_DMA(&huart6, USART6_DMA_TX_BUF, USART6_DMA_TX_Len);
}

static inline void get_moto_Practical_angle(void)
{
	uint32_t rx_data_len = 0;														//本次接收长度
	
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);									   		  //清除空闲中断的标志
		__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx,DMA_FLAG_TCIF1_5);                       //清除 DMA2_Steam1传输完成标志
		HAL_UART_DMAStop(&huart6);                                                    //传输完成以后关闭串口DMA
		rx_data_len = USART6_DMA_RX_Len - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx); //获取这一次数据量大小（总长度-保留的长度）
		HAL_UART_Receive_DMA(&huart1, USART6_DMA_RX_BUF, USART6_DMA_RX_Len);  //接受数据
		if (rx_data_len == USART6_DMA_RX_Len)                                           //判断数据是否为正确的数据长度
		{
	
			if(USART6_DMA_RX_BUF[3]==(USART6_DMA_RX_BUF[0]+USART6_DMA_RX_BUF[1]+USART6_DMA_RX_BUF[2]))//校验通过	
			{
				if(USART6_DMA_RX_BUF[0]==0XB1)
					motorPitch.posCtrl.motorPos=(USART6_DMA_RX_BUF[1]<<8)+USART6_DMA_RX_BUF[2];//获取从机1编码器读数
		//		else if(USART6_DMA_RX_BUF[0]==0XB2)
		//			moto2_Practical_angle=(USART6_DMA_RX_BUF[1]<<8)+USART6_DMA_RX_BUF[2];//获取从机2编码器读数
		//		else if(USART6_DMA_RX_BUF[0]==0XB3)
		//			moto3_Practical_angle=(USART6_DMA_RX_BUF[1]<<8)+USART6_DMA_RX_BUF[2];//获取从机3编码器读数
		//		else if(USART6_DMA_RX_BUF[0]==0XB4)
		//			moto4_Practical_angle=(USART6_DMA_RX_BUF[1]<<8)+USART6_DMA_RX_BUF[2];//获取从机4编码器读数
			}
		}
	}
}

