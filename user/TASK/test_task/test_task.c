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


void testTask(void *pvParameters)
{
    while(1) {
			led_green_toggle();
			int test = num_digits(2);
			serial_send_int(1);
			delay_ms(100);
    }
    
}
