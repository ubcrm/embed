#include "stm32f4xx.h"
#include "USART_comms.h"
#include <stdio.h>

int num_digits (int n);

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

void Serial_sendStringPart(volatile char *str, int length)
{
	int i = 0;
	while (*str && i < length) {
		// Once previous byte is finished being transmitted, transmit next byte
		while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
		USART_SendData(USART6, *str);
		str++;
		i++;
	}
}

void Serial_sendInt(int num)
{
	const int digits = num_digits(num) + 3;
	char str[digits];
	sprintf(str, "%d", num);
	Serial_sendString(str);
}

// assumes n is not the min or max int possible
int num_digits (int n) {
    if (n < 0) n = -n;
    if (n < 10) return 1;
    if (n < 100) return 2;
    if (n < 1000) return 3;
    if (n < 10000) return 4;
    if (n < 100000) return 5;
    if (n < 1000000) return 6;
    if (n < 10000000) return 7;
    if (n < 100000000) return 8;
    if (n < 1000000000) return 9;
    /*      2147483647 is 2^31-1 - add more ifs as needed
       and adjust this final return as well. */
    return 10;
}

