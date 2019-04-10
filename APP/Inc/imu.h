#ifndef _IMU_H_
#define _IMU_H_

#include "sys.h"

#define JY901_DataLength 40u

typedef struct
{
	float gyroVelox;  //x轴角速度
	float gyroVeloy;  //y轴角速度
	float gyroVeloz;  //z轴角速度
	
	float gyroAnglex;  //x轴角度
	float gyroAngley;  //y轴角度
	float gyroAnglez;  //z轴角度
	
}gyroValue;

void JY901_Data_Receive(void);
extern uint8_t JY901_Receive_Data[JY901_DataLength];
extern gyroValue JYgyro;

#endif
