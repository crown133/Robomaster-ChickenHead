#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "adrc.h"
#include "TD4.h"
#include "ESO.h"

/* CANͨѶID */
#define FIRST_FOUR_ID		0x200
#define SECOND_FOUR_ID		0x1FF


/* ����ٶȿ��ƽṹ�� */
typedef struct
{
	float refVel_Soft;
	float refVel;
	float rawVel;	//ʵ���ٶ�
	float filrawVel;
	
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
	float relaPos;		//����λ��
	float motorPos;		//���������λ��
	float posBias;
	float motorBias;	//�����������imu�Ĳ�ֵ
	
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
	
//	float times;
	
	uint8_t posReady;
}PosPidCtrl_t;


/* ����ṹ�� */
typedef struct
{	
	FunctionalState status; //�Ƿ�ʹ��
	
	VeloPidCtrl_t veloCtrl;
	
	PosPidCtrl_t posCtrl;
	
	int16_t torque;
}Motor_t;

/*********** ���˻���̨����Ҫ���Ƶĵ�� ************/
extern Motor_t motorYaw, motorPitch, motorBodan;

/************* �����ʼ������ *************/
extern void Motor_SetVel(VeloPidCtrl_t *vel_t, float velo);	//����ٶ�ֵ����
extern void Motor_IncPos(PosPidCtrl_t *pos_t, float pos, float posMax, float posMin);  //���������ģʽ�µ�λ��ֵ����
extern void Motor_AbsPos(PosPidCtrl_t *pos_t, float pos, float posMax, float posMin);  //���IMU�����ջ�ģʽ�µ�λ��ֵ����

extern void Motor_ValueSet(Motor_t *motor, float veloKp, float veloKi, float veloKd, \
	float veloMax, float veloMin, float posKp, float posKi, float posKd, float posOutMax, float posOutMin, int state);

/******************************************/
//debug 
extern void CtrlDebug(float data1, float data2, float data3, float data4, 
				float data5, float data6, float data7, float data8, float data9, float data10);

/************************************/
extern ADRC_Data ADRC_Yaw;  //Yaw ���ADRC������
extern TD td1, td2, td1_velo, td2_velo;
extern TD tdYawPc, tdPitchPc;

extern float kp, kd;

extern ESO eso1;
extern ESO_AngularRate eso2;

extern TD4 trackerYawInc, trackerPitchInc, trackerYaw, trackerPitch;

extern void Gimbal_Control(void);

#endif
