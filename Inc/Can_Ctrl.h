#ifndef _CAN_CTRL_H_
#define _CAN_CTRL_H_

#include "main.h"

#include "can.h"
#include "Motor_Func.h"

#define C610_POS_RANGE	((uint8_t)8192)

/*********** 无人机云台中需要控制的电机 ************/
extern Motor_t motorYaw, motorPitch, motorBodan;

/*************************************/
extern void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);

extern void CAN_MotorRxMsgConv(CAN_HandleTypeDef *hcan, Motor_t *motor);
extern void CAN_MotorTxMsgConv(CAN_HandleTypeDef* hcan, int16_t ID1Msg, int16_t ID2Msg, 
									   int16_t ID3Msg, int16_t ID4Msg);
#endif
