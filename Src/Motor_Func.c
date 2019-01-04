#include "Motor_Func.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "math.h"


/**
  * @brief	给电机赋期望速度值
  * @param	motor:	Motor_t结构体的指针
  * @param	speed:	预设的速度值
  * @retval	None
  * @note	注意电机速度方向和实际需要的方向是否相同
  */
void Motor_SetVel(VeloPidCtrl_t *vel_t, float velo)
{
	vel_t->refVel = velo;
}

/**
  * @brief	给电机赋期望位置值
  * @param	motor:	Motor_t结构体的指针
  * @param	pos_t:	预设的位置值
  * @note	注意电机转子位置范围
  * @retval	None
  */
void Motor_IncPos(PosPidCtrl_t *pos_t, float pos, float posMax, float posMin)
{
	pos_t->refPos += pos;
	
	if(pos_t->refPos > posMax)
		pos_t->refPos = posMax;
	if(pos_t->refPos < posMin)
		pos_t->refPos = posMin;
	
	pos_t->posReady = POS_CTRL_UNREADY;
}

/**
	Motor PID Value Set
  */
void Motor_ValueSet(Motor_t *motor, float veloKp, float veloKi, float veloKd, \
	float veloAcc,  float veloDec, float veloMax, float veloMin, float posKp, \
		float posKi, float posKd, float posAcc, float posOutMax, float posOutMin)
{
	motor->status = ENABLE;
	
	motor->veloCtrl.acc = veloAcc;
	motor->veloCtrl.dec  = veloDec;
	motor->veloCtrl.kp = veloKp;
	motor->veloCtrl.ki = veloKi;
	motor->veloCtrl.kd = veloKd;
	motor->veloCtrl.outputMax = veloMax;
	motor->veloCtrl.outputMin = veloMin;
	
	motor->posCtrl.acc = posAcc;
	motor->posCtrl.kp = posKp;
	motor->posCtrl.ki = posKi;
	motor->posCtrl.kd = posKd;
	motor->posCtrl.outputMax = posOutMax;
	motor->posCtrl.outputMin = posOutMin;
}
/**
  * @brief	进行电机位置控制
  * @note	在减速段使用了固定加速度逼近
  *	@param	pos: PosCtrl_t结构体的指针，电机位置控制结构体的指针
  *	@retval	None
  */
void Motor_PosCtrl(PosPidCtrl_t *pos_t)
{
	float diff;
	float refVel;
	float sign = 1.0f;
	static uint8_t readyCount = 0;				//设置计数避免误判
	
	switch (pos_t->posReady)
	{
		case POS_CTRL_READY:					//到达预定位置
		{
			/* 重置参数 */
		//	pos_t->refPos = 0;
			pos_t->relaPos = pos_t->refPos;
			pos_t->err = 0;
			pos_t->errLast = 0;
			pos_t->integ = 0;
			pos_t->output = 0;
			pos_t->posReady = POS_CTRL_READY;
			break;
		}
		case POS_CTRL_UNREADY:					//没有到达预定位置
		{
			/* 计算误差值，err保存当前的误差，errLast保存上一次的误差 */
			pos_t->errLast = pos_t->err;
			pos_t->err = pos_t->refPos - pos_t->relaPos;
		
			/* 判断是否已经完成位置闭环 */
			if (pos_t->err > -20 && pos_t->err < 20)		//已经完成
			{
				readyCount++;
				if (readyCount == 10)					//到达预定位置
				{
					readyCount = 0;						//重置计数值
					pos_t->posReady = POS_CTRL_READY;
					return;
				}
			}
			else										//没有完成，继续位置闭环
			{
				readyCount = 0;
			
				/* 保证当前的控制极性 */
				if (pos_t->err < 0.0f)
					sign = -1.0f;
				
				/* 计算积分值，注意末尾积分限幅 */
				pos_t->integ += pos_t->err;
				if(pos_t->integ >= 10000)
					pos_t->integ = 10000;
				if(pos_t->integ <= -10000)
					pos_t->integ = -10000;
				
				diff = pos_t->err - pos_t->errLast;	//计算误差变化率
				
				/* 绝对式方法计算PID输出 */
				pos_t->output = pos_t->kp * pos_t->err * pos_t->err / 100 + pos_t->ki * pos_t->integ + pos_t->kd * diff;
				
				/* 用固定加速度逼近终值， 0.8为积分裕量，用来削减积分值和实际值之间的偏差 */
				refVel = sign * sqrtf(2.0f * 0.8f * pos_t->acc * sign * pos_t->err);
				
				/* 如果接近终值则切换成PID控制 */
				if (fabsf(refVel) < fabsf(pos_t->output))
					pos_t->output = refVel;
				
				/* 输出限幅 */
				if(pos_t->output >= pos_t->outputMax)
					pos_t->output = pos_t->outputMax;
				if(pos_t->output <= pos_t->outputMin)
					pos_t->output = pos_t->outputMin;
			}
			break;
		}
		default:
			break;
	}
}

/**
  * @brief	进行电机速度控制
  * @note	加速度用固定加速度控制，减速段前期使用固定加速度减速，后期使用PID逼近终值
  *	@param	pos_t:	PosCtrl_t结构体的指针，电机位置控制结构体的指针
  *	@retval	None
  */
void Motor_VeloCtrl(VeloPidCtrl_t *vel_t)
{
	float diff;
	
	/* 加减速斜坡 */
	if (vel_t->refVel_Soft < (vel_t->refVel - vel_t->acc))		//需要加速，使用加速加速度
		vel_t->refVel_Soft += vel_t->acc;
	else if (vel_t->refVel_Soft > (vel_t->refVel + vel_t->dec))	//需要减速，使用减速加速度
		vel_t->refVel_Soft -= vel_t->dec;
	else													//在线性加速度范围内使用PID调节
		vel_t->refVel_Soft = vel_t->refVel;
	
	/* 速度PID */
	vel_t->errLast = vel_t->err;
	vel_t->err = vel_t->refVel_Soft - vel_t->rawVel;		//使用vel_t->refVel_Soft作为速度期望
	diff = vel_t->err - vel_t->errLast;
	vel_t->integ += vel_t->err;
	 
	/* 积分限幅 */
	if (vel_t->integ >= 10000)
		vel_t->integ = 10000;
	if (vel_t->integ <= -10000)
		vel_t->integ = -10000;
		
	vel_t->output = vel_t->kp * vel_t->err + vel_t->ki * vel_t->integ + vel_t->kd * diff;
	
	/* 输出限幅 */
	if(vel_t->output >= vel_t->outputMax)
		vel_t->output = vel_t->outputMax;
	if(vel_t->output <= vel_t->outputMin)
		vel_t->output = vel_t->outputMin;
}

/**
  * @brief	串口调试电机控制参数
  *	@param	data1~10:	共十个通道的数据
  * @note	通过串口7发送
  *	@retval	None
  */
void CtrlDebug(float data1, float data2, float data3, float data4, 
				float data5, float data6, float data7, float data8, 
				float data9, float data10)
{
	unsigned char Send_Count;
	int i;

	DataScope_Get_Channel_Data(data1, 1); //将数据 1.0  写入通道 1
    DataScope_Get_Channel_Data(data2, 2); //将数据 2.0  写入通道 2
    DataScope_Get_Channel_Data(data3, 3); //将数据 3.0  写入通道 3
    DataScope_Get_Channel_Data(data4, 4); //将数据 4.0  写入通道 4
	DataScope_Get_Channel_Data(data5, 5); //将数据 5.0  写入通道 5
    DataScope_Get_Channel_Data(data6, 6); //将数据 6.0  写入通道 6
	DataScope_Get_Channel_Data(data7, 7); //将数据 7.0  写入通道 7
    DataScope_Get_Channel_Data(data8, 8); //将数据 8.0  写入通道 8
	DataScope_Get_Channel_Data(data9, 9); //将数据 9.0  写入通道 9
    DataScope_Get_Channel_Data(data10, 10); //将数据 10.0 写入通道 10

	Send_Count = DataScope_Data_Generate(10); //生成10个通道的格式化帧数据，返回帧数据长度
	
	//HAL_UART_Transmit(&huart8, DataScope_OutPut_Buffer, Send_Count, 50);
	for( i = 0 ; i < Send_Count; i++)  //循环发送,直到发送完毕  
	{
		HAL_UART_Transmit(&huart7, DataScope_OutPut_Buffer + i, 1, 50);
	}
}




