#include "Motor_Func.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "math.h"


#define limit_output(x, min, max)	( (x) <= (min) ? (min) : (x) >= (max) ? (max) : (x) )//限幅函数

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

void Motor_AbsPos(PosPidCtrl_t *pos_t, float pos, float posMax, float posMin)
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
			float veloMax, float veloMin, float posKp, float posKi, float posKd, \
				float posOutMax, float posOutMin, int state)
{	
	motor->status = state;
	motor->veloCtrl.kp = veloKp;
	motor->veloCtrl.ki = veloKi;
	motor->veloCtrl.kd = veloKd;
	motor->veloCtrl.outputMax = veloMax;
	motor->veloCtrl.outputMin = veloMin;
	
	motor->posCtrl.kp = posKp;
	motor->posCtrl.ki = posKi;
	motor->posCtrl.kd = posKd;
	motor->posCtrl.outputMax = posOutMax;
	motor->posCtrl.outputMin = posOutMin;
}
/**
  * @brief	进行电机位置控制
  *	@retval	None
  */
void Motor_PosCtrl(PosPidCtrl_t *pos_t)
{
	float diff;
	
	/* 计算误差值，err保存当前的误差，errLast保存上一次的误差 */
	pos_t->errLast = pos_t->err;

	pos_t->err = pos_t->refPos + pos_t->motorBias - pos_t->rawPos;
	
	{	
		/* 计算积分值，注意末尾积分限幅 */
		pos_t->integ += pos_t->err;
		if(pos_t->integ >= 5000)
		pos_t->integ = 5000;
		if(pos_t->integ <= -5000)
		pos_t->integ = -5000;
					
		diff = pos_t->err - pos_t->errLast;	//计算误差变化率
				
		/* 绝对式方法计算PID输出 */
		pos_t->output = pos_t->kp * pos_t->err + pos_t->ki * pos_t->integ + pos_t->kd * diff;
		/* 输出限幅 */
		if(pos_t->output >= pos_t->outputMax)
		{
			pos_t->output = pos_t->outputMax;
		}
		if(pos_t->output <= pos_t->outputMin)
		{
			pos_t->output = pos_t->outputMin;
		}
	}
}

/**
  * @brief	进行电机速度控制
  *	@retval	None
  */
void Motor_VeloCtrl(VeloPidCtrl_t *vel_t)
{
	float diff;
	
	/* 速度PID */
	vel_t->errLast = vel_t->err;
	vel_t->err = vel_t->refVel - vel_t->rawVel;		//使用vel_t->refVel作为速度期望
	diff = vel_t->err - vel_t->errLast;
	vel_t->integ += vel_t->err;
	 
	/* 积分限幅 */
	vel_t->integ = limit_output(vel_t->integ, -10000, 10000);
		
	vel_t->output = vel_t->kp * vel_t->err + vel_t->ki * vel_t->integ + vel_t->kd * diff;
	
	/* 输出限幅 */
	vel_t->output = limit_output(vel_t->output, vel_t->outputMin, vel_t->outputMax);
}


/**
  * @brief	串口调试电机控制参数
  *	@param	data1~10:	共十个通道的数据
  * @note	通过串口7发送
  *	@retval	None
  */
void CtrlDebug(float data1, float data2, float data3, float data4, float data5,
				float data6, float data7, float data8, float data9, float data10)
{
	int Send_Count;

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

	Send_Count = DataScope_Data_Generate(4); //生成10个通道的格式化帧数据，返回帧数据长度
	
	HAL_UART_Transmit(&huart7, DataScope_OutPut_Buffer, Send_Count, 1000);
}




