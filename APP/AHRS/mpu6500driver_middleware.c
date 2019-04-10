/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       mpu6500middleware.c/h
  * @brief      mpu6500磁力计中间层，完成mpu6500的通信函数,延时函数。
  *             
  * @note       IST8310只支持IIC读取
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "mpu6500driver_middleware.h"
#include "stm32f4xx.h"
#include "delay.h"

#include "mpu6500reg.h"
#include "ins.h"

#if defined(MPU6500_USE_SPI)

#include "spi.h"

#elif defined(MPU6500_USE_IIC)

#endif

void mpu6500_GPIO_init(void)
{

#if defined(MPU6500_USE_SPI)

    GPIO_InitTypeDef GPIO_InitStructure;

    //MPU NS	
    GPIO_InitStructure.Pin = GPIO_PIN_6;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);

#elif defined(MPU6500_USE_IIC)

#endif
}


void mpu6500_middleware_delay_ms(uint16_t ms)
{
    delay_ms(ms);
}
void mpu6500_middleware_delay_us(uint32_t us)
{
    delay_us(us);
}

#if defined(MPU6500_USE_SPI)

void mpu6500_SPI_NS_H(void)
{
	PFout(6) = 1;
}
void mpu6500_SPI_NS_L(void)
{
    PFout(6) = 0;
}

static uint8_t mpu6500_SPI_read_write_byte(uint8_t TxData)
{
	uint8_t rxData = 0;

	HAL_SPI_Transmit(&hspi5, &TxData, 1, 50);

	TxData = 0xff;
	
	HAL_SPI_TransmitReceive(&hspi5, &TxData, &rxData, 1, 50);
	
    return rxData;
}

void mpu6500_write_single_reg(uint8_t reg, uint8_t data)
{
    mpu6500_SPI_NS_L();
	HAL_SPI_Transmit(&hspi5, &reg, 1, 50);
	HAL_SPI_Transmit(&hspi5, &data, 1, 50);
    mpu6500_SPI_NS_H();
}

uint8_t mpu6500_read_single_reg(uint8_t reg)
{
    uint8_t res;
	uint8_t mpuTx = 0xff;
	reg |= MPU_SPI_READ_MSB;
    mpu6500_SPI_NS_L();
	HAL_SPI_Transmit(&hspi5, &reg, 1, 50);
	HAL_SPI_TransmitReceive(&hspi5, &mpuTx, &res, 1, 50);
    mpu6500_SPI_NS_H();
    return res;
}

void mpu6500_get_data(uint8_t *rxBuf)
{
	uint8_t reg = MPU_ACCEL_XOUT_H | MPU_SPI_READ_MSB;
	mpu6500_SPI_NS_L();
	
	HAL_SPI_Transmit(&hspi5, &reg, 1, 50);
	
	hspi5.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;  //只接收，减少发送浪费时间
	HAL_SPI_Init(&hspi5);
	
	HAL_SPI_Receive_DMA(&hspi5, rxBuf, IMU_DMA_RX_NUM); //
    //剩下的在DMA传输完成中断中完成 mpu6500_SPI_NS_H();
}

void mpu6500_end_data(void)
{
	HAL_SPI_DMAStop(&hspi5);
	hspi5.Init.Direction = SPI_DIRECTION_2LINES;  //只接收，减少发送浪费时间
	HAL_SPI_Init(&hspi5);
	mpu6500_read_single_reg(MPU_INT_STATUS); //清空中断标志寄存器
	
	PFout(6) = 1;   //set spi H 
}

void mpu6500_write_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    mpu6500_SPI_NS_L();
    HAL_SPI_Transmit(&hspi5, &reg, 1, 50);
    if (len != 0)
    {
        uint8_t i;
        for (i = 0; i < len; i++)
        {
            HAL_SPI_Transmit(&hspi5, buf, 1, 50);
            buf++;
        }
    }
    mpu6500_SPI_NS_H();
}

void mpu6500_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    mpu6500_SPI_NS_L();
	uint8_t mpuTx = 0xff;
    reg |= MPU_SPI_READ_MSB;
	HAL_SPI_Transmit(&hspi5, &reg, 1, 50);
    if (len != 0)
    {
        uint8_t i;
        for (i = 0; i < len; i++)
        {
            HAL_SPI_TransmitReceive(&hspi5, &mpuTx, buf, 1, 50);
            buf++;
        }
    }
    mpu6500_SPI_NS_H();
}

#elif defined(MPU6500_USE_IIC)

#endif
