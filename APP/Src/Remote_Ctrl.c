#include "usart.h"
#include "Pc_Uart.h"
#include "Remote_Ctrl.h"
#include "math.h"

uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];  //定义一个数组用于存放从DMA接收到的遥控器数据


RemoteCtrl_t RemoteCtrlData;       //遥控器输入
int8_t RemoteCtrlFlag = 0;			//遥控器接收标志位
uint8_t mouseRFlag1 = 0, mouseRFlag2 = 0, ReceiveTimes = 0;  //
uint8_t mouseRFlag3 = 0, mouseRFlag4 = 0;
uint8_t mouseRFlag5 = 0, mouseRFlag6 = 0;
uint8_t mouseRFlag7 = 0, mouseRFlag8 = 0;
uint8_t mouseRFlag01 = 0, mouseRFlag02 = 0;
uint8_t mouseRFlag03 = 0, mouseRFlag04 = 0;
uint8_t mouseRFlag05 = 0, mouseRFlag06 = 0;

/**
  * @brief	根据遥控器协议对进行接收到的数据进行处理
  * @param	pData:	一个指向8位数据的指针
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
	/* pData[0]为ch0的低8位，Data[1]的低3位ch0的高3位 */
	RemoteCtrlData.remote.ch0 = (uint16_t)(pData[0] | pData[1] << 8) & 0x07FF;
	
	/* pData[1]的高5位为ch1的低5位，pData[2]的低6位为ch1的高6位 */
	RemoteCtrlData.remote.ch1 = (uint16_t)(pData[1] >> 3 | pData[2] << 5) & 0x07FF;
	
	/* pData[2]的高2位为ch2的低2位, pData[3]为ch2的中8位，pData[4]的低1位为ch2的高1位 */
	RemoteCtrlData.remote.ch2 = (uint16_t)(pData[2] >> 6 | pData[3] << 2 | pData[4] << 10) & 0x07FF;
	
	/* pData[4]的高7位为ch3的低7位，pData[5]的低4位为ch3的高4位 */
	RemoteCtrlData.remote.ch3 = (uint16_t)(pData[4] >> 1 | pData[5] << 7) & 0x07FF;

	/* pData[5]的高2位为s1 */
	RemoteCtrlData.remote.s1  = ((pData[5] >> 6) & 0x03);
	
	/* pData[6]的6，7位为s2 */
	RemoteCtrlData.remote.s2  = ((pData[5] >> 4) & 0x03);
	
	/* pData[6],pData[7]为x */
	RemoteCtrlData.mouse.x    = (int16_t)(pData[6] | pData[7] << 8);
	
	/* pData[8],pData[9]为y */
	RemoteCtrlData.mouse.y    = (int16_t)(pData[8] | pData[9] << 8);
	
	/* pData[10],pData[11]为z */
	RemoteCtrlData.mouse.z    = (int16_t)(pData[10] | pData[11] << 8);
	
	/* pData[12]为左键 */
	RemoteCtrlData.mouse.press_l = pData[12];
	
	/* pData[13]为右键 */		
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
	
	/* pData[14],pData[15]为键盘值 */
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
	/* 拨轮进行解码 */
	RemoteCtrlData.remote.figWheel = (uint16_t)(pData[16] | pData[17] << 8);
}

/**
  * @brief	对应的遥控器解码函数
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

 /*函数名称：RemotreCtl_Data_Receive
  *函数功能：接收遥控器数据，位于USART1的中断函数中
  *入口参数：无
  *返回值  ：无
  */
void RemoteCtl_Data_Receive(void)
{
	uint32_t rx_data_len = 0;														//本次接收长度
	
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);									   		  //清除空闲中断的标志
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF2_6);                       //清除 DMA2_Steam2传输完成标志
		HAL_UART_DMAStop(&huart1);                                                    //传输完成以后关闭串口DMA
		rx_data_len = BSP_USART1_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); //获取这一次数据量大小（总长度-保留的长度）
		HAL_UART_Receive_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);  //接受数据
		if (rx_data_len == RC_FRAME_LENGTH)                                           //判断数据是否为正确的数据长度
		{
			RemoteCtrlFlag = 1; //接收到数据
			RC_DataHandle(USART1_DMA_RX_BUF);                                        //进入数据解码函数
		}
		else
		{
			RemoteCtrlFlag = 0;
		}
	}
}

/**
  *函数名称：RemotreCtl_Data_Receive
  *函数功能：接收遥控器数据，位于USART1的中断函数中
  *入口参数：无
  *返回值  ：无
  */
//void RemoteCtl_Data_Receive_Start(void)
//{
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                                   //开启不定长中断
//	HAL_UART_Receive_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);
//}
