#ifndef _USART4_H
#define _USART4_H

#include "system.h"



/* ���ջ��� */
#define    VISION_BUFFER_LEN           100   //��΢�����

extern uint8_t  Com4_Vision_Buffer[ VISION_BUFFER_LEN ];


void UART4_Init(void);
void UART4_DMA_Init(void);

void UART4_SendChar(uint8_t cData);

#endif
