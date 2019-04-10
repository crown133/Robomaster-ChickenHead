#ifndef _SPI_H_
#define _SPI_H_

#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi5;
extern DMA_HandleTypeDef hdma_spi5_rx;
extern DMA_HandleTypeDef hdma_spi5_tx;

extern void SPI5_Init(void);

#endif
