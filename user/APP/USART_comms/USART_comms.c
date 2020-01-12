#include "stm32f4xx.h"
#include "USART_comms.h"
#include <stdio.h>

// Sends one string over USART
// Example usage: Serial_sendString("Nando eats Nando's at Nando's");
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

// THIS DOES NOT WORK - to send ints do the following
/*
char str[number of digits your number is + 1];
sprintf(str, "%d", number);
serial_send_string(str);
*/
// Send an integer over serial as its ASCII value
void serial_send_int(int num)
{
	serial_send_string("in send int");
	// Make a string with room for num and EOL characters
	const int digits = num_digits(num) + 3; //4;
	char str[digits];
	serial_send_string("down here");
	
	// Turn num into a string with the format "%d\n\r"
	// and store the result in str
	sprintf(str, "%d\n\r", num);
	
	// Make volatile to avoid potential issues if not volatile
	// volatile char* num_as_string = str;
	// serial_send_string(num_as_string);
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
