#ifndef _CRC_H_
#define _CRC_H_

#include "stm32f4xx_hal.h"

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
unsigned int Verify_CRC16_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC16_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

#endif
