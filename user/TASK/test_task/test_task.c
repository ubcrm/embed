#include "test_task.h"
#include "main.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


#include "led.h"
#include "CAN_Receive.h"
#include "delay.h"
#include "USART_comms.h"


void testTask(void *pvParameters)
{
    while(1) {
			  CAN_CMD_GIMBAL(2000, 0, 0, 0);
			
				Serial_sendInt(5);			  
			  //Serial_sendInt(yaw_raw_data->speed_rpm);
			  delay_ms(100);
    }
    
}
