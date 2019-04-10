#ifndef _CAN_CTRL_H_
#define _CAN_CTRL_H_

#include "main.h"

#include "can.h"
#include "Motor_Ctrl.h"

#define C610_POS_RANGE	((uint8_t)8192)

/*************************************/
extern void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);

extern void CAN_MotorRxMsgConv(CAN_HandleTypeDef *hcan, Motor_t *motor);
extern uint8_t CAN_CMD_GIMBAL(int16_t Yaw, int16_t Pitch, int16_t Bodan, int16_t rev); //·¢ËÍº¯Êý

extern float temperature;
	
#endif
