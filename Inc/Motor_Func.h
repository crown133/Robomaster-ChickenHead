#ifndef _MOTOR_FUNC_H_
#define _MOTOR_FUNC_H_

#include "main.h"

/* CANͨѶID */
#define FIRST_FOUR_ID		0x200
#define SECOND_FOUR_ID		0x1FF


/* ����ٶȿ��ƽṹ�� */
typedef struct
{
	float refVel_Soft;
	float refVel;
	float rawVel;	//ʵ���ٶ�
	
	float acc;		//�ٶȿ����м��ٵļ��ٶ�
	float dec;		//�ٶȿ����м��ٵļ��ٶ�
	
	float kp;
	float ki;
	float kd;
	float integ;
	float err, errLast;

	float output;
	float outputMin;
	float outputMax;
}VeloPidCtrl_t;


/* ���λ�ÿ��ƽṹ�� */
typedef struct
{
	float refPos;		//λ�����������λ��
	float relaPos;		//���λ��
	
	float round;		//Ȧ��
	
	float rawPos;		//��ǰλ��
	float rawPosLast;	//��һ�ε�λ��
	
	float acc;			//λ�ÿ���ֻ�м��ٶ���Ҫ�м��ٶȣ����ٶȿ�����decһ��
	
	float kp;
	float ki;
	float kd;
	float integ;
	float err, errLast;
	
	float output;
	float outputMin;
	float outputMax;
	
	uint8_t posReady;
}PosPidCtrl_t;

#define POS_CTRL_READY		1
#define POS_CTRL_UNREADY	0

#define veloCtrlMode	1   //�ٶȻ�����ģʽ
#define posCtrlMode		0	//λ�û�����ģʽ

/* ����ṹ�� */
typedef struct
{	
	FunctionalState status; //�Ƿ�ʹ��
	
	VeloPidCtrl_t veloCtrl;
	
	PosPidCtrl_t posCtrl;
	
	int16_t torque;
}Motor_t;


/************* �����ʼ������ *************/
void Motor_SetVel(VeloPidCtrl_t *vel_t, float velo);
void Motor_IncPos(PosPidCtrl_t *pos_t, float pos, float posMax, float posMin);

void Motor_PosCtrl(PosPidCtrl_t *pos_t);
void Motor_VeloCtrl(VeloPidCtrl_t *vel_t);

void Motor_ValueSet(Motor_t *motor, float veloKp, float veloKi, float veloKd, \
	float veloAcc,  float veloDec, float veloMax, float veloMin, float posKp, \
		float posKi, float posKd, float posAcc, float posOutMax, float posOutMin);

void CtrlDebug(float data1, float data2, float data3, float data4, 
				float data5, float data6, float data7, float data8, 
				float data9, float data10);
				

#endif
