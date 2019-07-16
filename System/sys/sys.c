#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F429������
//ϵͳʱ�ӳ�ʼ��	
//����ʱ������/�жϹ���/GPIO���õ�
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/1/5
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//********************************************************************************
//�޸�˵��
//��
////////////////////////////////////////////////////////////////////////////////// 

//ʱ��ϵͳ���ú���
//Fvco=Fs*(plln/pllm);
//SYSCLK=Fvco/pllp=Fs*(plln/(pllm*pllp));
//Fusb=Fvco/pllq=Fs*(plln/(pllm*pllq));

//Fvco:VCOƵ��
//SYSCLK:ϵͳʱ��Ƶ��
//Fusb:USB,SDIO,RNG�ȵ�ʱ��Ƶ��
//Fs:PLL����ʱ��Ƶ��,������HSI,HSE��. 
//plln:��PLL��Ƶϵ��(PLL��Ƶ),ȡֵ��Χ:64~432.
//pllm:��PLL����ƵPLL��Ƶϵ��(PLL֮ǰ�ķ�Ƶ),ȡֵ��Χ:2~63.
//pllp:ϵͳʱ�ӵ���PLL��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:2,4,6,8.(������4��ֵ!)
//pllq:USB/SDIO/������������ȵ���PLL��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:2~15.

//�ⲿ����Ϊ25M��ʱ��,�Ƽ�ֵ:plln=360,pllm=25,pllp=2,pllq=8.
//�õ�:Fvco=25*(360/25)=360Mhz
//     SYSCLK=360/2=180Mhz
//     Fusb=360/8=45Mhz
//����ֵ:0,�ɹ�;1,ʧ��
//void Stm32_Clock_Init(u32 plln,u32 pllm,u32 pllp,u32 pllq)
//{
//    HAL_StatusTypeDef ret = HAL_OK;
//    RCC_OscInitTypeDef RCC_OscInitStructure; 
//    RCC_ClkInitTypeDef RCC_ClkInitStructure;
//    
//    __HAL_RCC_PWR_CLK_ENABLE(); //ʹ��PWRʱ��
//    
//    //������������������õ�ѹ�������ѹ�����Ա�������δ�����Ƶ�ʹ���
//    //ʱʹ�����빦��ʵ��ƽ�⣬�˹���ֻ��STM32F42xx��STM32F43xx�����У�
//    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);//���õ�ѹ�������ѹ����1
//    
//    RCC_OscInitStructure.OscillatorType=RCC_OSCILLATORTYPE_HSE;    //ʱ��ԴΪHSE
//    RCC_OscInitStructure.HSEState=RCC_HSE_ON;                      //��HSE
//    RCC_OscInitStructure.PLL.PLLState=RCC_PLL_ON;//��PLL
//    RCC_OscInitStructure.PLL.PLLSource=RCC_PLLSOURCE_HSE;//PLLʱ��Դѡ��HSE
//    RCC_OscInitStructure.PLL.PLLM=pllm; //��PLL����ƵPLL��Ƶϵ��(PLL֮ǰ�ķ�Ƶ),ȡֵ��Χ:2~63.
//    RCC_OscInitStructure.PLL.PLLN=plln; //��PLL��Ƶϵ��(PLL��Ƶ),ȡֵ��Χ:64~432.  
//    RCC_OscInitStructure.PLL.PLLP=pllp; //ϵͳʱ�ӵ���PLL��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:2,4,6,8.(������4��ֵ!)
//    RCC_OscInitStructure.PLL.PLLQ=pllq; //USB/SDIO/������������ȵ���PLL��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:2~15.
//    ret=HAL_RCC_OscConfig(&RCC_OscInitStructure);//��ʼ��
//	
//    if(ret!=HAL_OK) while(1);
//    
//    ret=HAL_PWREx_EnableOverDrive(); //����Over-Driver����
//    if(ret!=HAL_OK) while(1);
//    
//    //ѡ��PLL��Ϊϵͳʱ��Դ��������HCLK,PCLK1��PCLK2
//    RCC_ClkInitStructure.ClockType=(RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2);
//    RCC_ClkInitStructure.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;//����ϵͳʱ��ʱ��ԴΪPLL
//    RCC_ClkInitStructure.AHBCLKDivider=RCC_SYSCLK_DIV1;//AHB��Ƶϵ��Ϊ1
//    RCC_ClkInitStructure.APB1CLKDivider=RCC_HCLK_DIV4; //APB1��Ƶϵ��Ϊ4
//    RCC_ClkInitStructure.APB2CLKDivider=RCC_HCLK_DIV2; //APB2��Ƶϵ��Ϊ2
//    ret=HAL_RCC_ClockConfig(&RCC_ClkInitStructure,FLASH_LATENCY_5);//ͬʱ����FLASH��ʱ����Ϊ5WS��Ҳ����6��CPU���ڡ�
//		
//    if(ret!=HAL_OK) while(1);
//}
/************************���ȼ�ͳһ�ڴ�����*************************/

void SysNVIC_SetPriority(void)
{
	/******** CAN1 interrupt Init **********/
//	HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 2);
//    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 2);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	/********* CAN2 interrupt Init *********/
//    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 1, 1);
//    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
//    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 1, 1);
//    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
	
	/******** USART1 interrupt Init ********/
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
	
	/******** USART3 interrupt Init ********/  //����ϵͳ
    HAL_NVIC_SetPriority(USART3_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
	
	/******** USART6 interrupt Init ********/  //HMW���
    HAL_NVIC_SetPriority(USART6_IRQn, 1, 3);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
	
    /* UART7 interrupt Init */
    HAL_NVIC_SetPriority(UART7_IRQn, 1, 4);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
	
	/******** UART8 interrupt Init IMU*********/
    HAL_NVIC_SetPriority(UART8_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(UART8_IRQn);
	
	/***************** tim3  ****************/
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);  //control loop
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
	
	/***************** tim5 ****************/
//	HAL_NVIC_SetPriority(TIM5_IRQn, 1, 4);  //������������
//  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* DMA interrupt init */
 
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 6); //����7���Խ���
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */  
//  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 1, 3); //����1
//  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 2); //����2
//  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 1); //����8IMUģ��JY901
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 2, 1); //����6 HMW�������
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 8); //����6 HMW�������
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0); //DBus
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	//MPU6000 �ⲿ�ж϶�ȡdma
  /* DMA2_Stream4_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);  //SPI5  TX
//  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

//  /* DMA2_Stream5_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 1, 5);  //SPI5  RX
//  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 5);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 4);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  //
}

#ifdef  USE_FULL_ASSERT
//��������ʾ�����ʱ��˺����������������ļ���������
//file��ָ��Դ�ļ�
//line��ָ�����ļ��е�����
void assert_failed(uint8_t* file, uint32_t line)
{ 
	while (1)
	{
	}
}
#endif

//THUMBָ�֧�ֻ������
//�������·���ʵ��ִ�л��ָ��WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//�ر������ж�(���ǲ�����fault��NMI�ж�)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}
//���������ж�
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}
//����ջ����ַ
//addr:ջ����ַ
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}


