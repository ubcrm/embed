#include "stm32f4xx.h"
#include "USART_comms.h"
#include <stdio.h>

// Sends one string over USART
// Example usage: serial_send_string("Nando eats Nando's at Nando's");
void serial_send_string(volatile char *str)
{
	while (*str) {
		// Once previous byte is finished being transmitted, transmit next byte
		while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
		USART_SendData(USART6, *str);
		str++;
	}
}

// Send (a part of) an array
// ** Assumes length <= length of array
void serial_send_int_array(volatile int *arr, int length)
{
	for (int i = 0; i < length; i++) {
		// Once previous byte is finished being transmitted, transmit next byte
		while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
		USART_SendData(USART6, *arr);
		arr++;
	}
}

// Send an integer over serial as its ASCII value
void serial_send_int(int num)
{
  // This is hardcoded because dynamic memory allocation
	// is harder than solving the Collatz conjecture.
	char str[32] = {0};
	sprintf(str, "%d\n\r", num);
	serial_send_string(str);

}

// Return the number of digits an int takes in decimal
// ** assumes n is not the min or max int possible
int num_digits(int n) {	
	int count = 1;
	for (int i = 1; n / (i * 10) != 0; i*= 10) {
		count++;
	}
	
	return count;	
}
