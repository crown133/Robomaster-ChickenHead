#include "Motor_Ctrl.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "Can_Ctrl.h"

#include "math.h"

#define limit_output(x, min, max)	( (x) <= (min) ? (min) : (x) >= (max) ? (max) : (x) )//�޷�����

/**********************************************/
TD td1, td2, td1_velo, td2_velo;; 		  //�ٶȻ��ο�ֵʹ�ø���΢�������Ź��ɹ���
TD tdYawPc, tdPitchPc;//�ٶȷ���ֵʹ�ø���΢��������΢��
ESO eso1;
ESO_AngularRate eso2;

ADRC_Data ADRC_Yaw;   //Yaw ��� adrc������
TD4 trackerYawInc, trackerPitchInc, trackerYaw, trackerPitch;

/*********** ���˻���̨����Ҫ���Ƶĵ�� ************/
Motor_t motorYaw, motorPitch, motorBodan;

static uint8_t bodanCwCcwFlag = 50;  //������ת��־

/************************************************/
static void motorYawCalcuPos(void);		//Yaw ���λ�û�pid����
static void motorPitchCalcuPos(void);   //Pitch ���λ�û�pid����
static void Motor_VeloCtrl(VeloPidCtrl_t *vel_t);  //�ٶȻ�pid����
static inline void MotorFliter_VeloCtrl(VeloPidCtrl_t *vel_t);
static inline void motor_pitch_veloCtrl(void);

int posRaw, posOut, velRef, velOut;
float kp=0.01, kd = 50;

//��̨���������ƺ���
void Gimbal_Control(void)
{
	/*	0x205  yaw */
	motorYawCalcuPos(); 				//λ��pid����
	TD_Calculate(&td1, motorYaw.posCtrl.output);
//	ADRC_LESO(&eso1, motorYaw.veloCtrl.rawVel);
//	eso1.u = kp*(td1.v1 - eso1.z1) + kd*(td1.v2 - eso1.z2) - eso1.z3/eso1.b0;
	
//	motorYaw.veloCtrl.output = limit_float(eso1.u, -10000, 10000);
	
//	TD4_track4(&trackerYaw, motorYaw.posCtrl.output, 1.0f/200);
//	motorYaw.veloCtrl.refVel = trackerYaw.x1;
	motorYaw.veloCtrl.refVel = td1.v1;
	MotorFliter_VeloCtrl(&motorYaw.veloCtrl); //�ٶ�pid����
	
//	ADRC_Control(&ADRC_Yaw, kp, motorYaw.veloCtrl.rawVel);
//	ADRC_Yaw.u = limit_float(ADRC_Yaw.u, -16000, 16000);
	
	/*  pitch	*/
	motorPitchCalcuPos();  				  //λ��pid����
	TD_Calculate(&td2, motorPitch.posCtrl.output);
//	TD_Calculate(&td2_velo, motorPitch.veloCtrl.rawVel);
	
	ADRC_LESO(&eso1, motorPitch.veloCtrl.rawVel);
	eso1.u = kp*(td2.v1 - eso1.z1) + kd*(td2.v2 - eso1.z2) - eso1.z3/eso1.b0;
	motorPitch.veloCtrl.output = limit_float(eso1.u, -20000, 20000);
	
	
	motorPitch.veloCtrl.refVel = td2.v1 + td2.v2*kd;  //�ٶȲο�ֵ�ɸ���΢�������������ﵽ���Ź��ɹ��̵�Ŀ��
// 	motorPitch.veloCtrl.refVel = motorPitch.posCtrl.output;
//	Motor_VeloCtrl(&motorPitch.veloCtrl); //�ٶ�pid����
//	motor_pitch_veloCtrl();
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
	CAN_CMD_GIMBAL(motorYaw.veloCtrl.output, motorPitch.veloCtrl.output + 1000, motorBodan.veloCtrl.output, 0);
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
		ESO_AngularRate_run(&eso2, diff, 1.0f/200);
		
		// ����ʽ��������PID���                         // * abs(motorPitch.posCtrl.err)
		motorPitch.posCtrl.output = motorPitch.posCtrl.kp * motorPitch.posCtrl.err + motorPitch.posCtrl.ki * motorPitch.posCtrl.integ + motorPitch.posCtrl.kd * diff;
		
//		if(motorPitch.posCtrl.output > 100)
//		{
//			motorPitch.posCtrl.output += 200;
//		}
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

static inline void MotorFliter_VeloCtrl(VeloPidCtrl_t *vel_t)  //�ٶȷ���ʹ��td�˲����ֵ
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

static inline void motor_pitch_veloCtrl(void)
{
	float diff;
	
	// �ٶ�PID 
	motorPitch.veloCtrl.errLast = motorPitch.veloCtrl.err;
	
//	TD_Calculate(&td2_velo, motorPitch.veloCtrl.err);
	TD_Calculate(&td2_velo, motorPitch.veloCtrl.rawVel);
	
	motorPitch.veloCtrl.err = motorPitch.veloCtrl.refVel - td2_velo.v1;//motorPitch.veloCtrl.rawVel;		//ʹ��vel_t->refVel��Ϊ�ٶ�����
	diff = motorPitch.veloCtrl.err - motorPitch.veloCtrl.errLast;
	motorPitch.veloCtrl.integ += motorPitch.veloCtrl.err;
	
	//�����޷� 
	motorPitch.veloCtrl.integ = limit_output(motorPitch.veloCtrl.integ, -4000, 4000);
		
	motorPitch.veloCtrl.output = motorPitch.veloCtrl.kp * motorPitch.veloCtrl.err + motorPitch.veloCtrl.ki * motorPitch.veloCtrl.integ + motorPitch.veloCtrl.kd * (td2.v2 - td2_velo.v2);
	
	//����޷� 
	motorPitch.veloCtrl.output = limit_output(motorPitch.veloCtrl.output, motorPitch.veloCtrl.outputMin, motorPitch.veloCtrl.outputMax);
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

