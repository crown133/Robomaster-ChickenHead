#include "Motor_Func.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "math.h"


#define limit_output(x, min, max)	( (x) <= (min) ? (min) : (x) >= (max) ? (max) : (x) )//�޷�����

/**
  * @brief	������������ٶ�ֵ
  * @param	motor:	Motor_t�ṹ���ָ��
  * @param	speed:	Ԥ����ٶ�ֵ
  * @retval	None
  * @note	ע�����ٶȷ����ʵ����Ҫ�ķ����Ƿ���ͬ
  */
void Motor_SetVel(VeloPidCtrl_t *vel_t, float velo)
{
	vel_t->refVel = velo;
}

/**
  * @brief	�����������λ��ֵ
  * @param	motor:	Motor_t�ṹ���ָ��
  * @param	pos_t:	Ԥ���λ��ֵ
  * @note	ע����ת��λ�÷�Χ
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
  * @brief	���е��λ�ÿ���
  *	@retval	None
  */
void Motor_PosCtrl(PosPidCtrl_t *pos_t)
{
	float diff;
	
	/* �������ֵ��err���浱ǰ����errLast������һ�ε���� */
	pos_t->errLast = pos_t->err;

	pos_t->err = pos_t->refPos + pos_t->motorBias - pos_t->rawPos;
	
	{	
		/* �������ֵ��ע��ĩβ�����޷� */
		pos_t->integ += pos_t->err;
		if(pos_t->integ >= 5000)
		pos_t->integ = 5000;
		if(pos_t->integ <= -5000)
		pos_t->integ = -5000;
					
		diff = pos_t->err - pos_t->errLast;	//�������仯��
				
		/* ����ʽ��������PID��� */
		pos_t->output = pos_t->kp * pos_t->err + pos_t->ki * pos_t->integ + pos_t->kd * diff;
		/* ����޷� */
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
  * @brief	���е���ٶȿ���
  *	@retval	None
  */
void Motor_VeloCtrl(VeloPidCtrl_t *vel_t)
{
	float diff;
	
	/* �ٶ�PID */
	vel_t->errLast = vel_t->err;
	vel_t->err = vel_t->refVel - vel_t->rawVel;		//ʹ��vel_t->refVel��Ϊ�ٶ�����
	diff = vel_t->err - vel_t->errLast;
	vel_t->integ += vel_t->err;
	 
	/* �����޷� */
	vel_t->integ = limit_output(vel_t->integ, -10000, 10000);
		
	vel_t->output = vel_t->kp * vel_t->err + vel_t->ki * vel_t->integ + vel_t->kd * diff;
	
	/* ����޷� */
	vel_t->output = limit_output(vel_t->output, vel_t->outputMin, vel_t->outputMax);
}


/**
  * @brief	���ڵ��Ե�����Ʋ���
  *	@param	data1~10:	��ʮ��ͨ��������
  * @note	ͨ������7����
  *	@retval	None
  */
void CtrlDebug(float data1, float data2, float data3, float data4, float data5,
				float data6, float data7, float data8, float data9, float data10)
{
	int Send_Count;

	DataScope_Get_Channel_Data(data1, 1); //������ 1.0  д��ͨ�� 1
    DataScope_Get_Channel_Data(data2, 2); //������ 2.0  д��ͨ�� 2
    DataScope_Get_Channel_Data(data3, 3); //������ 3.0  д��ͨ�� 3
    DataScope_Get_Channel_Data(data4, 4); //������ 4.0  д��ͨ�� 4
	DataScope_Get_Channel_Data(data5, 5); //������ 5.0  д��ͨ�� 5
    DataScope_Get_Channel_Data(data6, 6); //������ 6.0  д��ͨ�� 6
	DataScope_Get_Channel_Data(data7, 7); //������ 7.0  д��ͨ�� 7
    DataScope_Get_Channel_Data(data8, 8); //������ 8.0  д��ͨ�� 8
	DataScope_Get_Channel_Data(data9, 9); //������ 9.0  д��ͨ�� 9
    DataScope_Get_Channel_Data(data10, 10); //������ 10.0 д��ͨ�� 10

	Send_Count = DataScope_Data_Generate(4); //����10��ͨ���ĸ�ʽ��֡���ݣ�����֡���ݳ���
	
	HAL_UART_Transmit(&huart7, DataScope_OutPut_Buffer, Send_Count, 1000);
}




