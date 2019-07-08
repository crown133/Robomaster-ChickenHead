#ifndef _IMU_H_
#define _IMU_H_

#include "sys.h"

#define JY901_DataLength 40u

typedef struct
{
	int16_t gyroVelox;  //x����ٶ�
	int16_t gyroVeloy;  //y����ٶ�
	int16_t gyroVeloz;  //z����ٶ�
	
	float gyroAnglex;  //x��Ƕ�
	float gyroAngley;  //y��Ƕ�
	float gyroAnglez;  //z��Ƕ�
	
}gyroValue;

void JY901_Data_Receive(void);
extern uint8_t JY901_Receive_Data[JY901_DataLength];
extern gyroValue JYgyro;

#endif
