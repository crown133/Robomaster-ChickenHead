/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __tim_H
#define __tim_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"


extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

extern void _Error_Handler(char *, int);

void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

#endif /*__ tim_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
