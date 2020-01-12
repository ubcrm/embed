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
#include "stdio.h"

Gimbal_t gimbal;
Gimbal_Motor_t gimbal_yaw_motor;


void testTask(void *pvParameters)
{
	  gimbal_yaw_motor.gimbal_motor_raw = get_Yaw_Gimbal_Motor_Measure_Point();
	  gimbal.gyro_reading_raw = get_MPU6500_Gyro_Data_Point();
		gimbal.acce_reading_raw = get_MPU6500_Accel_Data_Point();
    char str[20];//uart data buffer
	
    while(1) {
			
			  //Sending data via UART
			  
			  sprintf(str, "gyro[0]: %f\n\r", gimbal.gyro_reading_raw[0]);
				Serial_sendString(str);		 
				sprintf(str, "gyro[1]: %f\n\r", gimbal.gyro_reading_raw[1]);
				Serial_sendString(str);	
			  sprintf(str, "gyro[2]: %f\n\r", gimbal.gyro_reading_raw[2]);
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
				Serial_sendString(str);		 
				sprintf(str, "speed: %d\n\r", gimbal_yaw_motor.gimbal_speed_raw);
				Serial_sendString(str);				
			  sprintf(str, "current: %d\n\r", gimbal_yaw_motor.gimbal_tq_current_raw);
				Serial_sendString(str);	
			  delay_ms(100);
				*/
    }
    
}
