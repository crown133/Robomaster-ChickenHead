#include "spi.h"

#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef hdma_spi5_rx;
DMA_HandleTypeDef hdma_spi5_tx;

/* SPI5 init function */
void SPI5_Init(void)
{
	//设置SPI为8位，8分频
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;  //全双工 主spi
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;  //时钟空闲相位高
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;    //第二个上升沿采集数据
  hspi5.Init.NSS = SPI_NSS_SOFT;  
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  //分频
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
  hspi5.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  __HAL_RCC_GPIOF_CLK_ENABLE();
	
  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI5)
  {
    /* SPI5 clock enable */
    __HAL_RCC_SPI5_CLK_ENABLE();
  
    /**SPI5 GPIO Configuration    
    PF7     ------> SPI5_SCK
    PF9     ------> SPI5_MOSI
    PF8     ------> SPI5_MISO 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* SPI5 DMA Init */
	__HAL_RCC_DMA2_CLK_ENABLE();  //
    /* SPI5_RX Init */
    hdma_spi5_rx.Instance = DMA2_Stream5;
    hdma_spi5_rx.Init.Channel = DMA_CHANNEL_7;
    hdma_spi5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi5_rx.Init.Mode = DMA_NORMAL;
    hdma_spi5_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi5_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi5_rx);
	__HAL_DMA_ENABLE_IT(&hdma_spi5_rx, DMA_IT_TC);

//    /* SPI5_TX Init */
//    hdma_spi5_tx.Instance = DMA2_Stream4;
//    hdma_spi5_tx.Init.Channel = DMA_CHANNEL_2;
//    hdma_spi5_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
//    hdma_spi5_tx.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_spi5_tx.Init.MemInc = DMA_MINC_ENABLE;
//    hdma_spi5_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//    hdma_spi5_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//    hdma_spi5_tx.Init.Mode = DMA_NORMAL;
//    hdma_spi5_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
//    hdma_spi5_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//    if (HAL_DMA_Init(&hdma_spi5_tx) != HAL_OK)
//    {
//      _Error_Handler(__FILE__, __LINE__);
//    }

//    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi5_tx);

  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI5)
  {

    /* Peripheral clock disable */
    __HAL_RCC_SPI5_CLK_DISABLE();
  
    /**SPI5 GPIO Configuration    
    PF7     ------> SPI5_SCK
    PF9     ------> SPI5_MOSI
    PF8     ------> SPI5_MISO 
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_8);

    /* SPI5 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);

  }
} 

void SPI5SetSpeedAndDataSize(uint16_t Speed, uint16_t DataSize)
{
    SPI5->CR1 &= 0xF7C7;
    SPI5->CR1 |= Speed;
    SPI5->CR1 |= DataSize;
    SPI5->CR1 |= 1 << 6;
}

