#ifndef _SHOOT_CTRL_H_
#define _SHOOT_CTRL_H_

#include "stm32f4xx_hal.h"
#include "Motor_Ctrl.h" 

#define MocaWheelA TIM4->CCR1
#define MocaWheelB TIM4->CCR2

#define FireStop	((uint16_t)1)
#define FireWeak	((uint16_t)3)
#define FireFire	((uint16_t)2)

void ShootWheel_Control(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void MocaWheelInit(VeloPidCtrl_t *motor, float p, float i, float d, float outmax, float outmin);

extern int setVelo;
extern int mouseR, mouseFlag;
extern VeloPidCtrl_t motorA, motorB;
extern float shootVeloA[6], shootVeloB[6];  //最后一位是平均值

#endif
