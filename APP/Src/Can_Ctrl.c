#include "Can_Ctrl.h"
#include "Motor_Ctrl.h"


/**
  *	@brief	receive callback function
  *	@param	hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  *	@retval	None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	//如果是CAN1
	
	{
		if (hcan == &hcan1)
		{
			if(hcan->pRxMsg->IDE == CAN_ID_STD && hcan->pRxMsg->RTR == CAN_RTR_DATA)
			{
				
				switch (hcan->pRxMsg->StdId)
				{
					//CAN1,鸡头Yaw轴电机
					case 0x205:
					{
//						if(initFlag)
						CAN_MotorRxMsgConv(hcan, &motorYaw);
//						motorYaw.posCtrl.rawPos = -(int16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);  //实际位置
					}break;
				
				 //CAN1,鸡头Pitch轴电机
					 case 0x206:
					{
//						motorPitch.posCtrl.rawPosLast = motorPitch.posCtrl.rawPos;
//						motorPitch.posCtrl.rawPos = (int16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);  //实际位置
//						motorPitch.veloCtrl.rawVel = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);//实际速度
						CAN_MotorRxMsgConv(hcan, &motorPitch);
					}break; 
				 //CAN1, 拨蛋轮电机
					 case 0x207:
					{
						motorBodan.posCtrl.rawPos = (int16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);  //实际位置
						motorBodan.veloCtrl.rawVel = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);//实际速度
						CAN_MotorRxMsgConv(hcan, &motorBodan);
					}break;
				default:
					{
						
					}break;
				}
			}
			__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);		//重新打开CAN中断
		}
	}
}

/**
  *	@brief	转换发送给Motor的数据格式 
  *	@param	hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  *	@retval	None
  */
float temperature = 0;  //GM6020电机显示温度

void CAN_MotorRxMsgConv(CAN_HandleTypeDef *hcan, Motor_t *motor)
{
	if(initFlag)
	{
		motor->posCtrl.rawPosLast = motor->posCtrl.rawPos;
		motor->posCtrl.rawPos = (int16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);  //实际位置

		motor->posCtrl.relaPos = motor->posCtrl.rawPos;
	}
	motor->veloCtrl.rawVel = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);//实际速度
	motor->posCtrl.motorPos = (int16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);  //实际位置
	
	motor->torque = (int16_t)(hcan->pRxMsg->Data[4] << 8 | hcan->pRxMsg->Data[5]);  //实际转矩
	
	if(hcan->pRxMsg->StdId == 0x205) 
	{
		temperature = hcan->pRxMsg->Data[6]; //温度
	}
}


/**
	*	@brief	转换发送给Motor的数据格式
	*	@param	偏航电机值 
	*	@param	俯仰电机值
	*	@param	拨蛋电机值 
	*	@param	备用 
	*	@retval None
	*/
uint8_t CAN_CMD_GIMBAL(int16_t Yaw, int16_t Pitch, int16_t Bodan, int16_t rev)
{
	hcan1.pTxMsg->StdId = SECOND_FOUR_ID;
	hcan1.pTxMsg->ExtId = SECOND_FOUR_ID;
	hcan1.pTxMsg->IDE = CAN_ID_STD;		//标准格式
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;	//数据帧
	hcan1.pTxMsg->DLC = DLC_LEN;
	
	
	hcan1.pTxMsg->Data[0] = (uint8_t)(Yaw >> 8);
	hcan1.pTxMsg->Data[1] = (uint8_t)Yaw;
	
	hcan1.pTxMsg->Data[2] = (uint8_t)(Pitch >> 8);
	hcan1.pTxMsg->Data[3] = (uint8_t)Pitch;
	
	hcan1.pTxMsg->Data[4] = (uint8_t)(Bodan >> 8);
	hcan1.pTxMsg->Data[5] = (uint8_t)Bodan;
	
	hcan1.pTxMsg->Data[6] = (uint8_t)(rev >> 8);
	hcan1.pTxMsg->Data[7] = (uint8_t)rev;
	
	if(HAL_CAN_Transmit(&hcan1, 0) != HAL_OK)
	return 0;
	
	return 1;
}
