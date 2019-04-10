/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */

#include "usart.h"
#include "tim.h"
#include "spi.h"

#include "Remote_Ctrl.h"
#include "Control_Loop.h"
#include "Shoot_Ctrl.h"
#include "imu.h"
#include "ins.h"
#include "Pc_Uart.h"
#include "mpu6500driver_middleware.h"
/* USER CODE END 0 */
/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;  //Motor 
extern DMA_HandleTypeDef hdma_tim5_ch1;
extern DMA_HandleTypeDef hdma_tim5_ch2;
extern DMA_HandleTypeDef hdma_usart1_rx; //DBus
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi5;  //MPU6500
/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
}
/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  while (1)
  {}
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  while (1)
  {

  }
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  while (1)
  {

  }
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{

}
/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{

}
/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{

}
/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/******* 定时器中断 *******/

void TIM3_IRQHandler(void)
{
  sysControl();  //Control_Loop
  HAL_TIM_IRQHandler(&htim3);
}

void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim5);
}
/***** CAN1  TX interrupts ******/

//void CAN1_TX_IRQHandler(void)
//{
//  HAL_CAN_IRQHandler(&hcan1);
//}

/******* CAN1 RX interrupts *******/
void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

/***** USART1 DBus interrupt *****/
void USART1_IRQHandler(void)
{
  RemoteCtl_Data_Receive();
  //HAL_UART_IRQHandler(&huart1);
}

/******* UART7 ********/
void UART7_IRQHandler(void)
{
  pcUartReceive();
  //HAL_UART_IRQHandler(&huart7);
	
}

/***** UART8 JY901 RECEIVE ******/
void UART8_IRQHandler(void)
{
  JY901_Data_Receive();
 // HAL_UART_IRQHandler(&huart8);
}

/**** EXTI line[9:5] interrupts ****/
void EXTI9_5_IRQHandler(void)  //MPU6500外部中断读取
{
//  mpu6500_get_data(mpu6500_spi_rxbuf); //开启DMA传输	
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
}

void EXTI2_IRQHandler(void)  //按键中断
{
  
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/********* DMA Interrupt *********/

/**** DMA2 stream2 interrupt ****/
void DMA2_Stream2_IRQHandler(void)  //DBUS
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx);  
}	

void DMA2_Stream5_IRQHandler(void)  //MPU6500
{
//	mpu6500_end_data();
  __HAL_DMA_CLEAR_FLAG(&hdma_spi5_rx, DMA_FLAG_TCIF1_5);
//  HAL_DMA_IRQHandler(&hdma_spi5_rx);
}
/**** DMA1 stream1 interrupt ****/
//void DMA1_Stream1_IRQHandler(void)  //串口7发送
//{
//  HAL_DMA_IRQHandler(&hdma_uart7_tx);
//}

/**** DMA1 stream6 interrupt ****/

/*DMA1 stream2 霍尔1 interrupt*/
void DMA1_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tim5_ch1);
}
/*DMA1 stream3  interrupt*/
void DMA1_Stream3_IRQHandler(void)  //串口7发送
{
  HAL_DMA_IRQHandler(&hdma_uart7_rx);
}

/*DMA1 stream4 霍尔2 interrupt*/
void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tim5_ch2);
}

void DMA1_Stream6_IRQHandler(void)  //JY901
{
  HAL_DMA_IRQHandler(&hdma_uart8_rx);
}

/**** DMA2 stream5 interrupt ****/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
