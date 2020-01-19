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

#include <stdio.h>

Gimbal_Motor_t gimbal_yaw_motor;


void testTask(void *pvParameters)
{
	  gimbal_yaw_motor.gimbal_motor_raw = get_Yaw_Gimbal_Motor_Measure_Point();
    char str[20];//uart data buffer
	
    while(1) {
      led_green_toggle();
			serial_send_int(1);			
			//delay_ms(100);

			  //Make the motor turn
			  CAN_CMD_GIMBAL(2000, 2000, 0, 0);
			
			  //Get CAN received data
			  gimbal_yaw_motor.gimbal_pos_raw = gimbal_yaw_motor.gimbal_motor_raw->ecd;
			  gimbal_yaw_motor.gimbal_speed_raw = gimbal_yaw_motor.gimbal_motor_raw->speed_rpm;
			  gimbal_yaw_motor.gimbal_tq_current_raw = gimbal_yaw_motor.gimbal_motor_raw->given_current;
			
			
			  //Sending data via UART
			  sprintf(str, "position: %d\n\r", gimbal_yaw_motor.gimbal_pos_raw);
				serial_send_string(str);		 
				sprintf(str, "speed: %d\n\r", gimbal_yaw_motor.gimbal_speed_raw);
				serial_send_string(str);				
			  sprintf(str, "current: %d\n\r", gimbal_yaw_motor.gimbal_tq_current_raw);
				serial_send_string(str);	
			  delay_ms(100);
    }
    
}
