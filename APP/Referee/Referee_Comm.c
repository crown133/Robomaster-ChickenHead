#include "Referee_Comm.h"
#include "CRC.h"
#include "usart.h"
#include "string.h"
#include "Shoot_Ctrl.h"
#include "Remote_Ctrl.h"
#include "Pc_Uart.h"

uint8_t USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];  //定义一个数组用于存放从DMA接收到的裁判系统数据
uint8_t USART6_DMA_TX_BUF[BSP_USART6_DMA_TX_BUF_LEN];  //

uint8_t send_flag;

uint8_t guard_state;  //shaobing

ext_referee_data_t  RefereeData_t;
send_data_t send_data;  	 //回传给无人机客户端
//send_data_guard send_guard;  //给烧饼的帧

send_data_graphic graph_draw;
/**
  * @brief :  初始化向裁判系统发送数据的帧头  
  *  默认初始化为红方机器人， 通过板载按键切换为蓝方机器人
  */
void data_head_init(void)
{
	//设置帧头
	send_data.frame_header.sof = FRAME_HEADER_SOF;
	send_data.frame_header.dataLength = 19;
	send_data.frame_header.seq = 0;
	send_data.cmd_id = 0x301;
	//设置数据包头
	send_data.client_header.data_cmd_id = 0xD180;     	//客户端自定义数据ID
	send_data.client_header.receiver_ID = 0x106;     	//空中操作手客户端
	send_data.client_header.send_ID = 6;              	//红方无人机
	
//	send_data.client_header.receiver_ID = 0x0116;     	//空中操作手客户端
//	send_data.client_header.send_ID = 16;              	//蓝方无人机
	
	/***************************************************/
	
//	send_guard.frame_header.sof = FRAME_HEADER_SOF;
//	send_guard.frame_header.dataLength = 7;
//	send_guard.frame_header.seq = 0;
//	send_guard.cmd_id = 0x301;
//	
//	send_guard.client_header.data_cmd_id = 0x0201;     	  //与烧饼自定义数据ID
//	send_guard.client_header.receiver_ID = 7;     		  //红方烧饼ID
//	send_guard.client_header.send_ID = 6;              	  //红方无人机

	/*****************************************************/
	graph_draw.frame_header.sof = FRAME_HEADER_SOF;
	graph_draw.frame_header.dataLength = 61;
	graph_draw.frame_header.seq = 0;
	graph_draw.cmd_id = 0x301;
	
	graph_draw.client_header.data_cmd_id = 0x100;
	graph_draw.client_header.receiver_ID = 0x106;
	graph_draw.client_header.send_ID = 6;
	
	
}

void data_head_blue_init(void)
{
	send_data.client_header.receiver_ID = 0x116;     	//空中操作手客户端
	send_data.client_header.send_ID = 16;              	//蓝方无人机
	
//  send_guard.client_header.receiver_ID = 17;			//蓝方烧饼
//  send_guard.client_header.send_ID = 16;
	
	graph_draw.client_header.receiver_ID = 0x116;
	graph_draw.client_header.send_ID = 16;
}
/**
  * @brief :  解码裁判系统数据
  */
void Referee_Decode(uint8_t *pData)
{
	uint8_t frameLoc = 0;
	uint8_t *frameHeadLoc;						//暂存当前帧的帧头地址
	uint16_t dataLength, cmdID;
	
	while (frameLoc < BSP_USART6_DMA_RX_BUF_LEN)//缓存区只能保存128个字节，循环检查是不是有数据包在内
	{
		/* 当前帧的帧头首地址为pData + frameLoc */
		if (pData[frameLoc] == FRAME_HEADER_SOF)
		{
			if (Verify_CRC8_Check_Sum(pData + frameLoc, FRAME_HEADER_LEN) == 1)		//帧头CRC8校验成功
			{
				frameHeadLoc = pData + frameLoc;			//单纯暂存，简化后面的代码
				memcpy(&dataLength, frameHeadLoc + 1, 2);	//获取数据包长度后校验整包
				if (Verify_CRC16_Check_Sum(pData + frameLoc, FRAME_HEADER_LEN + CMD_ID_LEN + dataLength + CRC16_LEN) == 1)
				{
					memcpy(&cmdID, frameHeadLoc + FRAME_HEADER_LEN, 2);
					switch (cmdID)
					{
//						case GAME_STATE_CMD_ID:		//1Hz
//							memcpy(&RefereeData_t.GameState_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_STATE_LEN);
//							break;
//						case GAME_RESULT_CMD_ID:	//比赛结束后发送
//							memcpy(&RefereeData_t.GameResult_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_RESULT_LEN);
//  						break;
//						case GAME_ROBOTSURVIVORS_CMD_ID:	//1Hz发送
//							memcpy(&RefereeData_t.GameRobotSurvivors_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, GAME_ROBOTSURVIVORS_LEN);
//							break;
//						case EVENT_DATA_CMD_ID:		//事件改变后发送
//							memcpy(&RefereeData_t.EventData_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, EVENT_DATA_LEN);
//							break;
//						case SUPPLY_PROJECTILE_ACTION_CMD_ID:	//动作改变后发送
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
//						case BUFF_MUSK_CMD_ID:			//增益状态改变后发送
//							memcpy(&RefereeData_t.BuffMusk_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, BUFF_MUSK_LEN);
//							break;
//						case AERIAL_ROBOT_ENERGY_CMD_ID:	//空中机器人能量状态数据，10Hz
//							memcpy(&RefereeData_t.AerialRobotEnergy_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, AERIAL_ROBOT_ENERGY_LEN);
//							break;
//						case ROBOT_HURT_CMD_ID:			//伤害发生后发送
//							memcpy(&RefereeData_t.RobotHurt_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, ROBOT_HURT_LEN);
//							break;
//						case SHOOT_DATA_CMD_ID:			//子弹发射后发送
//							memcpy(&RefereeData_t.ShootData_t, frameHeadLoc + FRAME_HEADER_LEN + CMD_ID_LEN, SHOOT_DATA_LEN);
//							break;
						default:
							break;
					}
					frameLoc += FRAME_HEADER_LEN + CMD_ID_LEN + dataLength + CRC16_LEN;		//整包处理完毕，向后一个整包长度
				}
				else
					frameLoc += FRAME_HEADER_LEN;		//帧头正确且包头校正正确，但是整包出错，向后一个包头长度
			}
			else
				frameLoc++;			//帧头正确，但是包头校验出错，向后一个字节
		}
		else
			frameLoc++;			//未识别帧头，向后一个字节
	}
}

/**
  * @brief :  生成要发送给裁判系统的自定义数据
  */

void SendDatabuild(void)
{
	//设置数据包头
	send_data.client_header.data_cmd_id = 0xD180;     	//客户端自定义数据ID
	graph_draw.frame_header.seq = send_data.frame_header.seq++;  //包序列一致
	
	//设置数据
	{
		send_data.client_data.data1 = mouseR | (RemoteCtrlData.remote.s2 != 1);  //摩擦轮是否开启
		send_data.client_data.data2 = UART7_TX_BUF[2];    //视觉识别为地面目标还是基地
	//	send_data.client_data.data3 = ;
		send_data.client_data.mask = RemoteCtrlData.key.shift;  //灯1 绿色为自瞄开启，红色为自瞄关闭
		//传送帧头
		memcpy(USART6_DMA_TX_BUF,&send_data, 4);
		//生成CRC8字节
		send_data.frame_header.crc8 = Get_CRC8_Check_Sum(USART6_DMA_TX_BUF, 4, 0xff);	
		memcpy(USART6_DMA_TX_BUF,&send_data, 26);
		//生成CRC16字节
		send_data.CRC16 = Get_CRC16_Check_Sum(USART6_DMA_TX_BUF, 26, 0xffff);
		//将数据拷贝到缓冲区
		memcpy(USART6_DMA_TX_BUF,&send_data, 28);
	}

}

void graphic_draw(uint8_t name, uint8_t operate_tpye, uint16_t x, uint16_t y, uint8_t r)
{
	graph_draw.client_header.data_cmd_id = 0x0100;     	//客户端自定义数据ID
	send_data.frame_header.seq = graph_draw.frame_header.seq++;  //包序列一致

//	graph_draw.graphic.operate_tpye = operate_tpye;  //增加图形
	graph_draw.graphic.graphic_tpye = 3;  //绘制正圆
//	graph_draw.graphic.graphic_name[0] = name;  //图形名
	graph_draw.graphic.layer = 0;  //图层 同一图层内，先画的覆盖后画的
	graph_draw.graphic.color = 0;  //红方为红色 蓝方为蓝色
	graph_draw.graphic.width = 10;  //线宽
	
	graph_draw.graphic.start_x = x;  
	graph_draw.graphic.start_y = y;
	
	graph_draw.graphic.radius = r;
	//传送帧头
	memcpy(USART6_DMA_TX_BUF,&graph_draw, 4);
	//生成CRC8字节
	graph_draw.frame_header.crc8 = Get_CRC8_Check_Sum(USART6_DMA_TX_BUF, 4, 0xff);	
	memcpy(USART6_DMA_TX_BUF,&graph_draw, 5+61+2);
	//生成CRC16字节
	graph_draw.CRC16 = Get_CRC16_Check_Sum(USART6_DMA_TX_BUF, 5+61+2, 0xffff);
	//将数据拷贝到缓冲区
	memcpy(USART6_DMA_TX_BUF,&graph_draw, 5+61+2+2);
	
	HAL_UART_Transmit(&huart6, USART6_DMA_TX_BUF, 70, 10);
}
