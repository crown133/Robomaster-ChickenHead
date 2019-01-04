#ifndef _REMOTE_CTRL_H_
#define _REMOTE_CTRL_H_

#include "stm32f4xx_hal.h"

#define BSP_USART1_DMA_RX_BUF_LEN 30u            //ң�������ݽ���DMA�洢����

/**********************ң��������********************/
#define REMOTE_INPUT					1u
#define KEY_MOUSE_INPUT					3u
#define REMOTE_CHASIS_SPEED_TO_REF		1.197f          //1219*0.5/660f
#define REMOTE_PITCH_ANGLE_TO_REF		0.004f
#define REMOTE_YAW_ANGLE_TO_REF			0.004f
#define ROTATE_TO_REF					0.8f

#define RC_CH_VALUE_MIN					((uint16_t)364)
#define RC_CH_VALUE_OFFSET				((uint16_t)1024)
#define RC_CH_VALUE_MAX					((uint16_t)1684)
#define RC_CH_VALUE_RANGE				660.0f

#define RC_SW_UP						((uint16_t)1)
#define RC_SW_MID						((uint16_t)3)
#define RC_SW_DOWN						((uint16_t)2)

#define RC_FRAME_LENGTH					18u

typedef struct
{
	struct
	{
		uint16_t ch0;//ͨ��0
		uint16_t ch1;//ͨ��1
		uint16_t ch2;//ͨ��2
		uint16_t ch3;//ͨ��3
		uint8_t  s1;//����1
		uint8_t  s2;//����2
	} remote;
	
	struct
	{
		int16_t  x;//���x
		int16_t  y;//���y
		int16_t  z;//���z
		uint8_t  press_l;//������
		uint8_t  press_r;//����Ҽ�
	} mouse;
	
	struct
	{
		uint16_t v;//����
	} key;
	
}RemoteCtrl_t;

extern RemoteCtrl_t RemoteCtrlData;
extern int8_t RemoteCtrlFlag;
void RC_DataHandle(uint8_t *pData);

extern uint8_t g_AutoMode;
extern uint8_t g_ShootMode;

void RemoteCtl_Data_Receive(void);

#endif