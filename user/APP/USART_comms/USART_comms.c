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

//This function is bad don't use
void Serial_sendInt(int num)
{
	/*
	int *temp;
  int n = 0;
	while(num > 10){
		temp[n] = num%10;
		num/=10;
		n++;
	}
	temp[n] = num; //Breaks num into each bit
	
	char *str;
	for (int i = 0; i <= n; i++) {
		str[i] = '0' + temp[n-i];
	}
	
	Serial_sendString(str);
	*/
	Serial_sendString((char*) num);
}

