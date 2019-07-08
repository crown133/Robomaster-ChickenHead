#include "Motor_Ctrl.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "Can_Ctrl.h"
#include "HMW.h"

#include "math.h"

#define limit_output(x, min, max)	( (x) <= (min) ? (min) : (x) >= (max) ? (max) : (x) )//限幅函数

/**********************************************/
TD td1, td2; 		  //速度环参考值使用跟踪微分器安排过渡过程
TD tdYawPc, tdPitchPc;//速度反馈值使用跟踪微分器进行微分
ADRC_Data ADRC_Yaw;   //Yaw 电机 adrc控制体

/*********** 无人机云台中需要控制的电机 ************/
Motor_t motorYaw, motorPitch, motorBodan;

static uint8_t bodanCwCcwFlag = 50;  //卡弹反转标志

/************************************************/
static void motorYawCalcuPos(void);		//Yaw 电机位置环pid计算
static void motorPitchCalcuPos(void);   //Pitch 电机位置环pid计算
static void Motor_VeloCtrl(VeloPidCtrl_t *vel_t);  //速度环pid计算
static inline void MotorFliter_VeloCtrl(VeloPidCtrl_t *vel_t);

int posRaw, posOut, velRef, velOut;


//云台电机计算控制函数
void Gimbal_Control(void)
{
	/*	0x205  yaw */
	motorYawCalcuPos(); 				//位置pid计算
//	TD_Calculate(&td1, motorYaw.posCtrl.output);
//	motorYaw.veloCtrl.refVel = td1.v1;  //速度参考值由跟踪微分器给出，来达到安排过渡过程的目的
	motorYaw.veloCtrl.refVel = motorYaw.posCtrl.output;
	MotorFliter_VeloCtrl(&motorYaw.veloCtrl); //速度pid计算
	
//	ADRC_Control(&ADRC_Yaw, kp, motorYaw.veloCtrl.rawVel);
//	ADRC_Yaw.u = limit_float(ADRC_Yaw.u, -16000, 16000);
	
	/*  pitch	*/
	motorPitchCalcuPos();  				  //位置pid计算
	TD_Calculate(&td2, motorPitch.posCtrl.output);
	motorPitch.veloCtrl.refVel = td2.v1;  //速度参考值由跟踪微分器给出，来达到安排过渡过程的目的
// 	motorPitch.veloCtrl.refVel = motorPitch.posCtrl.output;
	Motor_VeloCtrl(&motorPitch.veloCtrl); //速度pid计算
	
	/*	0x207  bodan	*/
	Motor_VeloCtrl(&motorBodan.veloCtrl); //速度pid计算
	//卡弹反转  +逆时针  -顺时针
	if(((motorBodan.veloCtrl.output >= 8000) || (motorBodan.veloCtrl.output <= -8000)) && (motorBodan.veloCtrl.rawVel == 0)) //卡弹反转
		{
			bodanCwCcwFlag = 0;
		}
	if(bodanCwCcwFlag < 50)
		{
			motorBodan.veloCtrl.output = -motorBodan.veloCtrl.output;
			bodanCwCcwFlag ++;
		}

	/*	电机值输出	*/
	CAN_CMD_GIMBAL(motorYaw.veloCtrl.output, motorPitch.veloCtrl.output, motorBodan.veloCtrl.output, 0);
}


/*	Motor PID Value Set  */
void Motor_ValueSet(Motor_t *motor, float veloKp, float veloKi, float veloKd, float veloMax, float veloMin, \
					float posKp, float posKi, float posKd, float posOutMax, float posOutMin, int state)
{	
//	motor->status = state;
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


/*	电机编码器模式下的位置值设置	*/
void Motor_IncPos(PosPidCtrl_t *pos_t, float pos, float posMax, float posMin)  
{
	pos_t->refPos += pos;
	
	if(pos_t->refPos > posMax)
		pos_t->refPos = posMax;
	if(pos_t->refPos < posMin)
		pos_t->refPos = posMin;
	
}

/*	电机IMU反馈闭环模式下的位置值设置 */
void Motor_AbsPos(PosPidCtrl_t *pos_t, float pos, float posMax, float posMin)  
{
	pos_t->refPos += pos;
	
	if(pos_t->refPos > posMax)
		pos_t->refPos = posMax;
	if(pos_t->refPos < posMin)
		pos_t->refPos = posMin;
	
}

/* Yaw 电机位置环pid计算 */
void motorYawCalcuPos(void)
{
	float diff;
	
	// 计算误差值，err保存当前的误差，errLast保存上一次的误差 
	motorYaw.posCtrl.errLast = motorYaw.posCtrl.err;

	motorYaw.posCtrl.err = motorYaw.posCtrl.refPos - motorYaw.posCtrl.relaPos;
	{	
		// 计算积分值，注意末尾积分限幅 */
		motorYaw.posCtrl.integ += motorYaw.posCtrl.err;
		
		//积分限幅
		motorYaw.posCtrl.integ = limit_output(motorYaw.posCtrl.integ, -5000, 5000);
		
//		TD_Calculate(&td2, motorYaw.posCtrl.err);	
//		diff = td2.v2;  //使用跟踪微分器输出微分值
		diff = motorYaw.posCtrl.err - motorYaw.posCtrl.errLast;	//计算误差变化率
				
		// 绝对式方法计算PID输出 */ 	             //* abs(motorYaw.posCtrl.err) / 2
		motorYaw.posCtrl.output = motorYaw.posCtrl.kp * motorYaw.posCtrl.err + motorYaw.posCtrl.ki * motorYaw.posCtrl.integ + motorYaw.posCtrl.kd * diff;
	
		// 输出限幅 */
		motorYaw.posCtrl.output = limit_output(motorYaw.posCtrl.output, motorYaw.posCtrl.outputMin, motorYaw.posCtrl.outputMax);
	}
}

/* Pitch 电机位置环pid计算 */
void motorPitchCalcuPos(void)
{
	float diff;
//	float refVel;
//	float sign = 1.0f;
	
	// 计算误差值，err保存当前的误差，errLast保存上一次的误差 */
	motorPitch.posCtrl.errLast = motorPitch.posCtrl.err;
	motorPitch.posCtrl.err = motorPitch.posCtrl.refPos - motorPitch.posCtrl.rawPos;
	{	
		// 计算积分值，注意末尾积分限幅 
		motorPitch.posCtrl.integ += motorPitch.posCtrl.err;
		
		//积分限幅
		motorPitch.posCtrl.integ = limit_output(motorPitch.posCtrl.integ, -5000, 5000);
					
		diff = motorPitch.posCtrl.err - motorPitch.posCtrl.errLast;	//计算误差变化率
				
		// 绝对式方法计算PID输出                         // * abs(motorPitch.posCtrl.err)
		motorPitch.posCtrl.output = motorPitch.posCtrl.kp * motorPitch.posCtrl.err + motorPitch.posCtrl.ki * motorPitch.posCtrl.integ + motorPitch.posCtrl.kd * diff;
		
		if(motorPitch.posCtrl.output > 100)
		{
			motorPitch.posCtrl.output += 200;
		}
//		/* 用固定加速度逼近终值， 0.8为积分裕量，用来削减积分值和实际值之间的偏差 */
//		if (motorPitch.posCtrl.err < 0.0f)
//			sign = -1.0f;
//		
//		refVel = sign * __sqrtf(2.0f * 0.8f * motorPitch.posCtrl.acc * sign * motorPitch.posCtrl.err);
//				
//		/* 如果接近终值则切换成PID控制 */
//		if (fabsf(refVel) < fabsf(motorPitch.posCtrl.output))
//		motorPitch.posCtrl.output = refVel;
		
		// 输出限幅 
		motorPitch.posCtrl.output = limit_output(motorPitch.posCtrl.output, motorPitch.posCtrl.outputMin, motorPitch.posCtrl.outputMax);
	}
}

/*	进行电机速度控制  */
void Motor_VeloCtrl(VeloPidCtrl_t *vel_t)
{
	float diff;
	
	// 速度PID 
	vel_t->errLast = vel_t->err;
	vel_t->err = vel_t->refVel - vel_t->rawVel;		//使用vel_t->refVel作为速度期望
	diff = vel_t->err - vel_t->errLast;
	vel_t->integ += vel_t->err;
	 
	//积分限幅 
	vel_t->integ = limit_output(vel_t->integ, -4000, 4000);
		
	vel_t->output = vel_t->kp * vel_t->err + vel_t->ki * vel_t->integ + vel_t->kd * diff;
	
	//输出限幅 
	vel_t->output = limit_output(vel_t->output, vel_t->outputMin, vel_t->outputMax);
}
static inline void MotorFliter_VeloCtrl(VeloPidCtrl_t *vel_t)
{
	float diff;
	
	// 速度PID 
	vel_t->errLast = vel_t->err;
	vel_t->err = vel_t->refVel - vel_t->filrawVel;		//使用vel_t->refVel作为速度期望
	diff = vel_t->err - vel_t->errLast;
	vel_t->integ += vel_t->err;
	 
	//积分限幅 
	vel_t->integ = limit_output(vel_t->integ, -4000, 4000);
		
	vel_t->output = vel_t->kp * vel_t->err + vel_t->ki * vel_t->integ + vel_t->kd * diff;
	
	//输出限幅 
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

