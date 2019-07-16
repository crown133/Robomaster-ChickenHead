#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "adrc.h"
#include "TD4.h"
#include "ESO.h"

/* CAN通讯ID */
#define FIRST_FOUR_ID		0x200
#define SECOND_FOUR_ID		0x1FF


/* 电机速度控制结构体 */
typedef struct
{
	float refVel_Soft;
	float refVel;
	float rawVel;	//实际速度
	float filrawVel;
	
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
	float relaPos;		//绝对位置
	float motorPos;		//电机编码器位置
	float posBias;
	float motorBias;	//电机编码器与imu的差值
	
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
	
//	float times;
	
	uint8_t posReady;
}PosPidCtrl_t;


/* 电机结构体 */
typedef struct
{	
	FunctionalState status; //是否使能
	
	VeloPidCtrl_t veloCtrl;
	
	PosPidCtrl_t posCtrl;
	
	int16_t torque;
}Motor_t;

/*********** 无人机云台中需要控制的电机 ************/
extern Motor_t motorYaw, motorPitch, motorBodan;

/************* 电机初始化设置 *************/
extern void Motor_SetVel(VeloPidCtrl_t *vel_t, float velo);	//电机速度值设置
extern void Motor_IncPos(PosPidCtrl_t *pos_t, float pos, float posMax, float posMin);  //电机编码器模式下的位置值设置
extern void Motor_AbsPos(PosPidCtrl_t *pos_t, float pos, float posMax, float posMin);  //电机IMU反馈闭环模式下的位置值设置

extern void Motor_ValueSet(Motor_t *motor, float veloKp, float veloKi, float veloKd, \
	float veloMax, float veloMin, float posKp, float posKi, float posKd, float posOutMax, float posOutMin, int state);

/******************************************/
//debug 
extern void CtrlDebug(float data1, float data2, float data3, float data4, 
				float data5, float data6, float data7, float data8, float data9, float data10);

/************************************/
extern ADRC_Data ADRC_Yaw;  //Yaw 电机ADRC控制体
extern TD td1, td2, td1_velo, td2_velo;
extern TD tdYawPc, tdPitchPc;

extern float kp, kd;

extern ESO eso1;
extern ESO_AngularRate eso2;

extern TD4 trackerYawInc, trackerPitchInc, trackerYaw, trackerPitch;

extern void Gimbal_Control(void);

#endif
