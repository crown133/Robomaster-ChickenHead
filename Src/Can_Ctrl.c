#include "Can_Ctrl.h"
#include "Motor_Func.h"

/*********** 无人机云台中需要控制的电机 ************/
Motor_t motorYaw, motorPitch, motorBodan;
int bodanCwCcwFlag = 50;

static int8_t revCount = 0;  //串口发送电机数据记次数

/**
  *	@brief	receive callback function
  *	@param	hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  *	@retval	None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	//如果是CAN1
	if (hcan == &hcan1)
	{
		if(hcan->pRxMsg->IDE == CAN_ID_STD && hcan->pRxMsg->RTR == CAN_RTR_DATA)
		{
			
			switch (hcan->pRxMsg->StdId)
			{
				/* CAN中提取位置和速度反馈，随后进行控制并发送 */
				/*case CM_L_ID:			//底盘左轮电机
					CAN_MotorRxMsgConv(hcan, &CM_Left);
					Chassis_MotorCtrl(&CM_Left);
					CAN_MotorTxMsgConv(hcan, 
									   CM_Left.velCtrl.output, CM_Right.velCtrl.output,
									   LM.velCtrl.output, 0);
					CAN_SendMsg(hcan, SECOND_FOUR_ID);
					break;
				case CM_R_ID:			//底盘右轮电机
					CAN_MotorRxMsgConv(hcan, &CM_Right);
					Chassis_MotorCtrl(&CM_Right);
					CAN_MotorTxMsgConv(hcan, 
									   CM_Left.velCtrl.output, CM_Right.velCtrl.output,
									   LM.velCtrl.output, 0);
					CAN_SendMsg(hcan, SECOND_FOUR_ID);
					break;*/
//				case LM_ID:				//供弹电机
//					CAN_MotorRxMsgConv(hcan, &LM);
//					Loader_UpdateState(&LM);		//根据电机运行情况更改状态
//					Loader_MotorCtrl(&LM);
//					CAN_MotorTxMsgConv(hcan, 
//									   CM_Left.velCtrl.output, CM_Right.velCtrl.output,
//									   LM.velCtrl.output, 0);
//					CAN_SendMsg(hcan, SECOND_FOUR_ID);
//					break;
				 //CAN1，鸡头YAW轴电机
				case 0x205:
				{
					CAN_MotorRxMsgConv(hcan, &motorYaw);
					{
						Motor_PosCtrl(&motorYaw.posCtrl);  //位置pid计算
						motorYaw.veloCtrl.refVel = motorYaw.posCtrl.output;
						Motor_VeloCtrl(&motorYaw.veloCtrl); //速度pid计算
						
						CAN_MotorTxMsgConv(hcan, motorYaw.veloCtrl.output, motorPitch.veloCtrl.output, \
						motorBodan.veloCtrl.output, 0);
					}
					CAN_SendMsg(hcan, SECOND_FOUR_ID); //0x1ff
				}break;
			
			 //CAN1， 鸡头Pitch轴电机
				 case 0x206:
				{
					CAN_MotorRxMsgConv(hcan, &motorPitch);
					//if(!initFlag)  //如果在初始化中就初始化
					{
						Motor_PosCtrl(&motorPitch.posCtrl);  //位置pid计算
						motorPitch.veloCtrl.refVel = motorPitch.posCtrl.output;
						Motor_VeloCtrl(&motorPitch.veloCtrl); //速度pid计算
						
						CAN_MotorTxMsgConv(hcan, motorYaw.veloCtrl.output, motorPitch.veloCtrl.output, \
						motorBodan.veloCtrl.output, 0);
					}
					CAN_SendMsg(hcan, SECOND_FOUR_ID); //0x1ff
				}break; 
			 //CAN1, 拨蛋轮电机
				 case 0x207:
			    {
					CAN_MotorRxMsgConv(hcan, &motorBodan);
					Motor_VeloCtrl(&motorBodan.veloCtrl); //速度pid计算
//					spe = (int16_t)BodanPid.output;  //+逆时针  -顺时针		
					if(((motorBodan.veloCtrl.output == 9000) || (motorBodan.veloCtrl.output == -9000)) && (motorBodan.veloCtrl.rawVel == 0)) //卡弹反转
					{
						bodanCwCcwFlag = 0;
					}
					if(bodanCwCcwFlag < 50)
					{
						motorBodan.veloCtrl.output = -motorBodan.veloCtrl.output;
						bodanCwCcwFlag ++;
				    }					
					CAN_MotorTxMsgConv(hcan, motorYaw.veloCtrl.output, motorPitch.veloCtrl.output, \
					motorBodan.veloCtrl.output, 0);
				}break;
   			default:
				{
				}break;
			}

			/* 通过串口发送调试数据 */
			//revCount++;
			if (revCount >= 30)
			{
//				CtrlDebug(CM_Left.velCtrl.rawVel, CM_Left.posCtrl.rawPos, 0,
//						  CM_Right.velCtrl.rawVel, CM_Right.posCtrl.rawPos, 0,
//						  LM.velCtrl.rawVel, LM.posCtrl.relaPos, 0, 
//						  0);
				revCount = 0;
			}
		}
		__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);		//重新打开CAN中断
	}
}

/**
  *	@brief	转换发送给Motor的数据格式 
  *	@param	hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  *	@retval	None
  */
float detaPos = 0;

void CAN_MotorRxMsgConv(CAN_HandleTypeDef *hcan, Motor_t *motor)
{
	detaPos = 0;
	motor->posCtrl.rawPosLast = motor->posCtrl.rawPos;  //
	motor->posCtrl.rawPos = (int16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);  //实际位置
	
	motor->veloCtrl.rawVel = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);//实际速度
	
	motor->torque = (int16_t)(hcan->pRxMsg->Data[4] << 8 | hcan->pRxMsg->Data[5]);  //实际转矩
	
	detaPos = motor->posCtrl.rawPos - motor->posCtrl.rawPosLast;
	
		if (detaPos > 5000)		//反转过一圈
		{
			motor->posCtrl.round --;
		}
		else if (detaPos < -5000)	//正转过一圈
		{
			motor->posCtrl.round ++;
		}
	motor->posCtrl.relaPos = motor->posCtrl.rawPos + motor->posCtrl.round * 8192;

}

/**
	*	@brief	转换发送给Motor的数据格式
	*	@param	ID1Msg message sent to the ID1 C610
	*	@param	ID2Msg message sent to the ID2 C610
	*	@param	ID3Msg message sent to the ID3 C610
	*	@param	ID4Msg message sent to the ID4 C610
	*	@retval None
	*/
void CAN_MotorTxMsgConv(CAN_HandleTypeDef* hcan, int16_t ID1Msg, int16_t ID2Msg, 
									   int16_t ID3Msg, int16_t ID4Msg)
{
	hcan->pTxMsg->Data[0] = (uint8_t)(ID1Msg >> 8);
	hcan->pTxMsg->Data[1] = (uint8_t)ID1Msg;
	
	hcan->pTxMsg->Data[2] = (uint8_t)(ID2Msg >> 8);
	hcan->pTxMsg->Data[3] = (uint8_t)ID2Msg;
	
	hcan->pTxMsg->Data[4] = (uint8_t)(ID3Msg >> 8);
	hcan->pTxMsg->Data[5] = (uint8_t)ID3Msg;
	
	hcan->pTxMsg->Data[6] = (uint8_t)(ID4Msg >> 8);
	hcan->pTxMsg->Data[7] = (uint8_t)ID4Msg;
}
