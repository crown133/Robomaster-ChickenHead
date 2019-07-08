/* Includes ------------------------------------------------------------------*/
#include "dma.h"


/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  __HAL_RCC_DMA2_CLK_ENABLE();  //
  __HAL_RCC_DMA1_CLK_ENABLE();  //

}


