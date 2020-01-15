#include "test_task.h"
#include "main.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "CAN_Receive.h"
#include "delay.h"
#include "INS_task.h"
#include "USART_comms.h"
<<<<<<< HEAD
#include "stdio.h"
#include "mpu6500driver.h"
=======

#include "stm32f4xx.h"

#include <stdio.h>
>>>>>>> dbb93161ede86a5450a792483bb01801c67ecd08

Gimbal_t gimbal;


void testTask(void *pvParameters)
{
	  //Link pointers
	  gimbal.gimbal_yaw_motor->gimbal_motor_raw = get_Yaw_Gimbal_Motor_Measure_Point();
	  gimbal.gyro_reading_raw = get_MPU6500_Gyro_Data_Point();
		gimbal.acce_reading_raw = get_MPU6500_Accel_Data_Point();
    char str[20];//uart data buffer
	
    while(1) {
			
			  //Sending data via UART
			  
			  sprintf(str, "gyro x: %f\n\r", gimbal.gyro_reading_raw[INS_GYRO_X_ADDRESS_OFFSET]);
				Serial_sendString(str);		 
				sprintf(str, "gyro y: %f\n\r", gimbal.gyro_reading_raw[INS_GYRO_Y_ADDRESS_OFFSET]);
				Serial_sendString(str);	
			  sprintf(str, "gyro z: %f\n\r", gimbal.gyro_reading_raw[INS_GYRO_Z_ADDRESS_OFFSET]);
				Serial_sendString(str);	
			
			
				sprintf(str, "accel x : %f\n\r", gimbal.acce_reading_raw[INS_ACCEL_X_ADDRESS_OFFSET]);
				Serial_sendString(str);		 
				sprintf(str, "accel y: %f\n\r", gimbal.acce_reading_raw[INS_ACCEL_Y_ADDRESS_OFFSET]);
				Serial_sendString(str);	
			  sprintf(str, "accel z: %f\n\r", gimbal.acce_reading_raw[INS_ACCEL_Z_ADDRESS_OFFSET]);
				Serial_sendString(str);	
			  delay_ms(200);
				
			  
			
			  //Make the motor turn
			  //CAN_CMD_GIMBAL(2000, 0, 0, 0);
			
			  //Get CAN received data
			  /*
			  gimbal_yaw_motor.gimbal_pos_raw = gimbal_yaw_motor.gimbal_motor_raw->ecd;
			  gimbal_yaw_motor.gimbal_speed_raw = gimbal_yaw_motor.gimbal_motor_raw->speed_rpm;
			  gimbal_yaw_motor.gimbal_tq_current_raw = gimbal_yaw_motor.gimbal_motor_raw->given_current;
			  */
			
			
			  //Sending data via UART
			  /*
			  sprintf(str, "position: %d\n\r", gimbal_yaw_motor.gimbal_pos_raw);
				serial_send_string(str);		 
				sprintf(str, "speed: %d\n\r", gimbal_yaw_motor.gimbal_speed_raw);
				serial_send_string(str);				
			  sprintf(str, "current: %d\n\r", gimbal_yaw_motor.gimbal_tq_current_raw);
				serial_send_string(str);	
			  delay_ms(100);
				*/
    }
    
}
