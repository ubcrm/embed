#include "test_task.h"
#include "main.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


#include "led.h"
#include "CAN_Receive.h"
#include "delay.h"
#include "USART_comms.h"
#include "stm32f4xx.h"

/*
// uart testing
uint16_t buffer_rx[100] = {0};
int count_rx = 0;
*/


void testTask(void *pvParameters)
{
    while(1) {
			led_green_toggle();
			// Serial_sendStringPart("here\n\r", 2);
			// Serial_sendInt(1);
			delay_ms(100);
    }
    
}

/*
void USART6_IRQHandler(void)
{
	// make sure USART6 was intended to be called for this interrupt
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
		uint16_t USART_Data = USART_ReceiveData(USART6);
		
		// Serial_sendInt(USART_Data + 1);
		
		if (count_rx >= 100 || USART_Data == 13)
		{
			Serial_sendStringPart((volatile char*) buffer_rx, count_rx);
			count_rx = 0;
		} else {
			buffer_rx[count_rx++] = USART_Data;
		}
	}
}
*/
