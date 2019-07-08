#ifndef _PC_UART_H_
#define _PC_UART_H_
#include "sys.h"

#define UART7_DMA_RX_BUF_LEN 20
#define pcDataLength  12

extern uint32_t rxLength;
extern float yawInc, pitchInc;

extern uint8_t UART7_DMA_RX_BUF[UART7_DMA_RX_BUF_LEN];
extern uint8_t UART7_TX_BUF[3];

extern void pcUartReceive(void);

#endif

