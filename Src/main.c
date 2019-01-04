/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "tim.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "sys.h"

#include "Can_Ctrl.h"
#include "Shoot_Ctrl.h"
#include "Remote_Ctrl.h"
/*---------------------------------------------------------*/

void SystemClock_Config(void);

int initFlag = 1;  //云台位置初始化

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();  //DBus
  MX_UART7_Init();        //串口通信
  MX_TIM4_Init();		  //PWM输出，驱动Snail电机 周期2ms

  MX_TIM3_Init();  //定时器3中断，总控制中断

  MX_TIM5_Init();  //摩擦轮霍尔捕获输入
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
	
  CANFilter_Init(&hcan1);
	
  SysNVIC_SetPriority();  //统一在此设置优先级
  HAL_Delay(1000);
  
  /*********** Motor init position ***********/
  Motor_IncPos(&(motorPitch.posCtrl), motorPitch.posCtrl.rawPos, 8000, -8000); //初始位置等于上电位置
  Motor_IncPos(&(motorYaw.posCtrl), 2650, 8000, -8000); //初始位置等于上电位置
  /********** motor control value set **********/
  Motor_ValueSet(&motorPitch, 8, 0, 0, 	15, 30, 9000, -9000, 100, \
		5, 5, 20, 8000, -8000);  //Pitch轴电机
  Motor_ValueSet(&motorYaw, 25, 0.2, 0, 20, 25, 25000, -25000, 10, \
		0, 0, 60, 300, -300);  //Yaw轴电机
  Motor_ValueSet(&motorBodan, 6, 0, 0, 	10,  30, 9000, -9000, 0, \
		0, 0, 0, 0, 0);  //拨蛋电机

  HAL_TIM_Base_Start_IT(&htim3);  //开启中断
  while (1)
  {
	  ShootWheel_Control();
	  
	 /******************* LASER *******************/
	if(RemoteCtrlData.remote.s2 == 1)
	{
		PGout(13) = 1;
	}
	else 
	{
		PGout(13) = 0;
	}
//	  if(1 == initFlag)
//	  {
		  
//	  }
  }

}

/********** 系统时钟配置 **********/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
