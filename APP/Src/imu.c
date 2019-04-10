#include "imu.h"
#include "usart.h"
#include "Remote_Ctrl.h"

uint8_t JY901_Receive_Data[JY901_DataLength];

gyroValue JYgyro;

//static float angVelo[3][6] = {0};
//static float ang[3][6] = {0};

void JY901_Decode(void)
{
	
	if((JY901_Receive_Data[11] == 0x55) && (JY901_Receive_Data[12] == 0x52))
	{
//		angVelo[0][5] = ((int16_t)(JY901_Receive_Data[14]<<8 | JY901_Receive_Data[13]))/32768.0*2000; //单位 °/s
//		angVelo[1][5] = ((int16_t)(JY901_Receive_Data[16]<<8 | JY901_Receive_Data[15]))/32768.0*2000;
//		angVelo[2][5] = ((int16_t)(JY901_Receive_Data[18]<<8 | JY901_Receive_Data[17]))/32768.0*2000;
		JYgyro.gyroVelox = ((int16_t)(JY901_Receive_Data[14]<<8 | JY901_Receive_Data[13]))/32768.0*2000; //单位 °/s
		JYgyro.gyroVeloy = ((int16_t)(JY901_Receive_Data[16]<<8 | JY901_Receive_Data[15]))/32768.0*2000;
		JYgyro.gyroVeloz = ((int16_t)(JY901_Receive_Data[18]<<8 | JY901_Receive_Data[17]))/32768.0*2000;
	}
	if((JY901_Receive_Data[22] == 0x55) && (JY901_Receive_Data[23] == 0x53))
	{
		JYgyro.gyroAnglex = ((int16_t)(JY901_Receive_Data[25]<<8 | JY901_Receive_Data[24]))/32768.0*180 + 180; //单位 °
		JYgyro.gyroAngley = ((int16_t)(JY901_Receive_Data[27]<<8 | JY901_Receive_Data[26]))/32768.0*180 + 180; 
		JYgyro.gyroAnglez = ((int16_t)(JY901_Receive_Data[29]<<8 | JY901_Receive_Data[28]))/32768.0*180 + 180; 
	}
//	for(int i = 0; i < 5; i++)
//	{
//		angVelo[0][i] = angVelo[0][i+1];
//		angVelo[1][i] = angVelo[1][i+1];
//		angVelo[2][i] = angVelo[2][i+1];
//		ang[0][i] = ang[0][i+1];
//		ang[1][i] = ang[1][i+1];
//		ang[2][i] = ang[2][i+1];
//	}
//	JYgyro.gyroVelox = (angVelo[0][0] + angVelo[0][1] + angVelo[0][2] + angVelo[0][3] + angVelo[0][4]) / 5;
//	JYgyro.gyroVeloy = (angVelo[1][0] + angVelo[1][1] + angVelo[1][2] + angVelo[1][3] + angVelo[1][4]) / 5;
//	JYgyro.gyroVeloz = (angVelo[2][0] + angVelo[2][1] + angVelo[2][2] + angVelo[2][3] + angVelo[2][4]) / 5;
//	JYgyro.gyroAnglex = (ang[0][0] + ang[0][1] + ang[0][2] + ang[0][3] + ang[0][4]) / 5 + 180;
//	JYgyro.gyroAngley = (ang[1][0] + ang[1][1] + ang[1][2] + ang[1][3] + ang[1][4]) / 5 + 180;
//	JYgyro.gyroAnglez = (ang[2][0] + ang[2][1] + ang[2][2] + ang[2][3] + ang[2][4]) / 5 + 180;
}
void JY901_Data_Receive(void)
{
	uint32_t rx_data_len = 0;															//本次接收长度
	if((__HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart8);										//清除空闲中断的标志
		__HAL_DMA_CLEAR_FLAG(&hdma_uart8_rx, DMA_FLAG_TCIF2_6);                       //清除 DMA2_Steam2传输完成标志
		HAL_UART_DMAStop(&huart8);                                                    //传输完成以后关闭串口DMA
		rx_data_len = JY901_DataLength - __HAL_DMA_GET_COUNTER(&hdma_uart8_rx); //获取这一次数据量大小（总长度-保留的长度）
		HAL_UART_Receive_DMA(&huart8, JY901_Receive_Data, JY901_DataLength);  //接受数据
		if (rx_data_len == 33u)                                           //判断数据是否为正确的数据长度
		{
			JY901_Decode();                                        //进入数据解码函数
		}
	}
}







