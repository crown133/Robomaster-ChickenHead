#ifndef _MOTOR_FUNC_H_
#define _MOTOR_FUNC_H_

#include "main.h"

/* CAN通讯ID */
#define FIRST_FOUR_ID		0x200
#define SECOND_FOUR_ID		0x1FF


/* 电机速度控制结构体 */
typedef struct
{
	float refVel_Soft;
	float refVel;
	float rawVel;	//实际速度
	
	float acc;		//速度控制中加速的加速度
	float dec;		//速度控制中减速的加速度
	
	float kp;
	float ki;
	float kd;
	float integ;
	float err, errLast;

	float output;
	float outputMin;
	float outputMax;
}VeloPidCtrl_t;


/* 电机位置控制结构体 */
typedef struct
{
	float refPos;		//位置期望是相对位置
	float relaPos;		//相对位移
	
	float round;		//圈数
	
	float rawPos;		//当前位置
	float rawPosLast;	//上一次的位置
	
	float acc;			//位置控制只有减速段需要有加速度，和速度控制中dec一致
	
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

#define veloCtrlMode	1   //速度环控制模式
#define posCtrlMode		0	//位置环控制模式

/* 电机结构体 */
typedef struct
{	
	FunctionalState status; //是否使能
	
	VeloPidCtrl_t veloCtrl;
	
	PosPidCtrl_t posCtrl;
	
	int16_t torque;
}Motor_t;


/************* 电机初始化设置 *************/
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
