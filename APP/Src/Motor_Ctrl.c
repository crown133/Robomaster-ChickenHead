#include "Motor_Ctrl.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "Can_Ctrl.h"
#include "HMW.h"

#include "math.h"

#define limit_output(x, min, max)	( (x) <= (min) ? (min) : (x) >= (max) ? (max) : (x) )//�޷�����

/**********************************************/
TD td1, td2; 		  //�ٶȻ��ο�ֵʹ�ø���΢�������Ź��ɹ���
TD tdYawPc, tdPitchPc;//�ٶȷ���ֵʹ�ø���΢��������΢��
ADRC_Data ADRC_Yaw;   //Yaw ��� adrc������

/*********** ���˻���̨����Ҫ���Ƶĵ�� ************/
Motor_t motorYaw, motorPitch, motorBodan;

static uint8_t bodanCwCcwFlag = 50;  //������ת��־

/************************************************/
static void motorYawCalcuPos(void);		//Yaw ���λ�û�pid����
static void motorPitchCalcuPos(void);   //Pitch ���λ�û�pid����
static void Motor_VeloCtrl(VeloPidCtrl_t *vel_t);  //�ٶȻ�pid����
static inline void MotorFliter_VeloCtrl(VeloPidCtrl_t *vel_t);

int posRaw, posOut, velRef, velOut;


//��̨���������ƺ���
void Gimbal_Control(void)
{
	/*	0x205  yaw */
	motorYawCalcuPos(); 				//λ��pid����
//	TD_Calculate(&td1, motorYaw.posCtrl.output);
//	motorYaw.veloCtrl.refVel = td1.v1;  //�ٶȲο�ֵ�ɸ���΢�������������ﵽ���Ź��ɹ��̵�Ŀ��
	motorYaw.veloCtrl.refVel = motorYaw.posCtrl.output;
	MotorFliter_VeloCtrl(&motorYaw.veloCtrl); //�ٶ�pid����
	
//	ADRC_Control(&ADRC_Yaw, kp, motorYaw.veloCtrl.rawVel);
//	ADRC_Yaw.u = limit_float(ADRC_Yaw.u, -16000, 16000);
	
	/*  pitch	*/
	motorPitchCalcuPos();  				  //λ��pid����
	TD_Calculate(&td2, motorPitch.posCtrl.output);
	motorPitch.veloCtrl.refVel = td2.v1;  //�ٶȲο�ֵ�ɸ���΢�������������ﵽ���Ź��ɹ��̵�Ŀ��
// 	motorPitch.veloCtrl.refVel = motorPitch.posCtrl.output;
	Motor_VeloCtrl(&motorPitch.veloCtrl); //�ٶ�pid����
	
	/*	0x207  bodan	*/
	Motor_VeloCtrl(&motorBodan.veloCtrl); //�ٶ�pid����
	//������ת  +��ʱ��  -˳ʱ��
	if(((motorBodan.veloCtrl.output >= 8000) || (motorBodan.veloCtrl.output <= -8000)) && (motorBodan.veloCtrl.rawVel == 0)) //������ת
		{
			bodanCwCcwFlag = 0;
		}
	if(bodanCwCcwFlag < 50)
		{
			motorBodan.veloCtrl.output = -motorBodan.veloCtrl.output;
			bodanCwCcwFlag ++;
		}

	/*	���ֵ���	*/
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


/*	���������ģʽ�µ�λ��ֵ����	*/
void Motor_IncPos(PosPidCtrl_t *pos_t, float pos, float posMax, float posMin)  
{
	pos_t->refPos += pos;
	
	if(pos_t->refPos > posMax)
		pos_t->refPos = posMax;
	if(pos_t->refPos < posMin)
		pos_t->refPos = posMin;
	
}

/*	���IMU�����ջ�ģʽ�µ�λ��ֵ���� */
void Motor_AbsPos(PosPidCtrl_t *pos_t, float pos, float posMax, float posMin)  
{
	pos_t->refPos += pos;
	
	if(pos_t->refPos > posMax)
		pos_t->refPos = posMax;
	if(pos_t->refPos < posMin)
		pos_t->refPos = posMin;
	
}

/* Yaw ���λ�û�pid���� */
void motorYawCalcuPos(void)
{
	float diff;
	
	// �������ֵ��err���浱ǰ����errLast������һ�ε���� 
	motorYaw.posCtrl.errLast = motorYaw.posCtrl.err;

	motorYaw.posCtrl.err = motorYaw.posCtrl.refPos - motorYaw.posCtrl.relaPos;
	{	
		// �������ֵ��ע��ĩβ�����޷� */
		motorYaw.posCtrl.integ += motorYaw.posCtrl.err;
		
		//�����޷�
		motorYaw.posCtrl.integ = limit_output(motorYaw.posCtrl.integ, -5000, 5000);
		
//		TD_Calculate(&td2, motorYaw.posCtrl.err);	
//		diff = td2.v2;  //ʹ�ø���΢�������΢��ֵ
		diff = motorYaw.posCtrl.err - motorYaw.posCtrl.errLast;	//�������仯��
				
		// ����ʽ��������PID��� */ 	             //* abs(motorYaw.posCtrl.err) / 2
		motorYaw.posCtrl.output = motorYaw.posCtrl.kp * motorYaw.posCtrl.err + motorYaw.posCtrl.ki * motorYaw.posCtrl.integ + motorYaw.posCtrl.kd * diff;
	
		// ����޷� */
		motorYaw.posCtrl.output = limit_output(motorYaw.posCtrl.output, motorYaw.posCtrl.outputMin, motorYaw.posCtrl.outputMax);
	}
}

/* Pitch ���λ�û�pid���� */
void motorPitchCalcuPos(void)
{
	float diff;
//	float refVel;
//	float sign = 1.0f;
	
	// �������ֵ��err���浱ǰ����errLast������һ�ε���� */
	motorPitch.posCtrl.errLast = motorPitch.posCtrl.err;
	motorPitch.posCtrl.err = motorPitch.posCtrl.refPos - motorPitch.posCtrl.rawPos;
	{	
		// �������ֵ��ע��ĩβ�����޷� 
		motorPitch.posCtrl.integ += motorPitch.posCtrl.err;
		
		//�����޷�
		motorPitch.posCtrl.integ = limit_output(motorPitch.posCtrl.integ, -5000, 5000);
					
		diff = motorPitch.posCtrl.err - motorPitch.posCtrl.errLast;	//�������仯��
				
		// ����ʽ��������PID���                         // * abs(motorPitch.posCtrl.err)
		motorPitch.posCtrl.output = motorPitch.posCtrl.kp * motorPitch.posCtrl.err + motorPitch.posCtrl.ki * motorPitch.posCtrl.integ + motorPitch.posCtrl.kd * diff;
		
		if(motorPitch.posCtrl.output > 100)
		{
			motorPitch.posCtrl.output += 200;
		}
//		/* �ù̶����ٶȱƽ���ֵ�� 0.8Ϊ����ԣ����������������ֵ��ʵ��ֵ֮���ƫ�� */
//		if (motorPitch.posCtrl.err < 0.0f)
//			sign = -1.0f;
//		
//		refVel = sign * __sqrtf(2.0f * 0.8f * motorPitch.posCtrl.acc * sign * motorPitch.posCtrl.err);
//				
//		/* ����ӽ���ֵ���л���PID���� */
//		if (fabsf(refVel) < fabsf(motorPitch.posCtrl.output))
//		motorPitch.posCtrl.output = refVel;
		
		// ����޷� 
		motorPitch.posCtrl.output = limit_output(motorPitch.posCtrl.output, motorPitch.posCtrl.outputMin, motorPitch.posCtrl.outputMax);
	}
}

/*	���е���ٶȿ���  */
void Motor_VeloCtrl(VeloPidCtrl_t *vel_t)
{
	float diff;
	
	// �ٶ�PID 
	vel_t->errLast = vel_t->err;
	vel_t->err = vel_t->refVel - vel_t->rawVel;		//ʹ��vel_t->refVel��Ϊ�ٶ�����
	diff = vel_t->err - vel_t->errLast;
	vel_t->integ += vel_t->err;
	 
	//�����޷� 
	vel_t->integ = limit_output(vel_t->integ, -4000, 4000);
		
	vel_t->output = vel_t->kp * vel_t->err + vel_t->ki * vel_t->integ + vel_t->kd * diff;
	
	//����޷� 
	vel_t->output = limit_output(vel_t->output, vel_t->outputMin, vel_t->outputMax);
}
static inline void MotorFliter_VeloCtrl(VeloPidCtrl_t *vel_t)
{
	float diff;
	
	// �ٶ�PID 
	vel_t->errLast = vel_t->err;
	vel_t->err = vel_t->refVel - vel_t->filrawVel;		//ʹ��vel_t->refVel��Ϊ�ٶ�����
	diff = vel_t->err - vel_t->errLast;
	vel_t->integ += vel_t->err;
	 
	//�����޷� 
	vel_t->integ = limit_output(vel_t->integ, -4000, 4000);
		
	vel_t->output = vel_t->kp * vel_t->err + vel_t->ki * vel_t->integ + vel_t->kd * diff;
	
	//����޷� 
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

