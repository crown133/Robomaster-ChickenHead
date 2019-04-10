/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/******* INCLUDE ********/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */


extern unsigned int initFlag;

#define Latitude_At_Local 39.5427f   //±±æ©£∫39.5427f  …Ó€⁄£∫22.57025f

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */


#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
