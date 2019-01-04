#ifndef _SHOOT_CTRL_H_
#define _SHOOT_CTRL_H_

#include "stm32f4xx_hal.h"

#define MocaWheelA TIM4->CCR1
#define MocaWheelB TIM4->CCR2

#define FireStop	((uint16_t)1)
#define FireWeak	((uint16_t)3)
#define FireFire	((uint16_t)2)

void ShootWheel_Control(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif
