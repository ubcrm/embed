#include "test_task.h"
#include "main.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


#include "led.h"
#include "CAN_Receive.h"
#include "delay.h"
#include "USART_comms.h"
#include <stdio.h>

#include "pid.h"

PidTypeDef pid;

Gimbal_Motor_t gimbal_yaw_motor;


void testTask(void *pvParameters)
{
	  gimbal_yaw_motor.gimbal_motor_raw = get_Yaw_Gimbal_Motor_Measure_Point();
    char str[20];//uart data buffer
	
	  const fp32 PID[3] = {10.0, 0.0, 0.0};
		PID_Init(&pid, 0, PID, 100, 100);
	
    while(1) {
			  led_green_toggle();
			  //Make the motor turn
				int angle = PID_Calc(&pid, 0.0, 90.0);

			  CAN_CMD_GIMBAL(0, angle, 0, 0);
			
			  //Get CAN received data
			  gimbal_yaw_motor.gimbal_pos_raw = gimbal_yaw_motor.gimbal_motor_raw->ecd;
			  gimbal_yaw_motor.gimbal_speed_raw = gimbal_yaw_motor.gimbal_motor_raw->speed_rpm;
			  gimbal_yaw_motor.gimbal_tq_current_raw = gimbal_yaw_motor.gimbal_motor_raw->given_current;
			
			
			  //Sending data via UART
			  sprintf(str, "position: %d\n\r", gimbal_yaw_motor.gimbal_pos_raw);
				Serial_sendString(str);		 
				sprintf(str, "speed: %d\n\r", gimbal_yaw_motor.gimbal_speed_raw);
				Serial_sendString(str);				
			  sprintf(str, "current: %d\n\r", gimbal_yaw_motor.gimbal_tq_current_raw);
				Serial_sendString(str);	
			  delay_ms(100);
    }
    
}
