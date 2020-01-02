#include "stm32f4xx.h"
#include "USART_comms.h"

// Sends one string over USART
// example usage: Serial_sendString("Nando eats Nando's at Nando's");
void Serial_sendString(volatile char *str)
{
	while (*str) {
		// Once previous byte is finished being transmitted, transmit next byte
		while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
		USART_SendData(USART6, *str);
		str++;
	}
}

void Serial_sendInt(int num)
{
	USART_SendData(USART6, num);
}

