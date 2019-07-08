#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;

/* USER CODE BEGIN Private defines */
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);
void MX_UART7_Init(void);
void MX_UART8_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */


