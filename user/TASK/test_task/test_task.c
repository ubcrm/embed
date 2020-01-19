#include "test_task.h"
#include "main.h"
#include "stm32f4xx.h"
#include <stdio.h>


#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "CAN_Receive.h"
#include "delay.h"
#include "INS_task.h"
#include "USART_comms.h"

#include "stdio.h"
#include "mpu6500driver.h"


Gimbal_t gimbal;
char str[32] = {0};


void testTask(void *pvParameters)
{
	  //Link pointers
	  gimbal.yaw_motor->gimbal_motor_raw = get_Yaw_Gimbal_Motor_Measure_Point();
	  gimbal.gyro_reading_raw = get_MPU6500_Gyro_Data_Point();
		gimbal.acce_reading_raw = get_MPU6500_Accel_Data_Point();
	
    while(1) {
			
			  //Sending data via UART

				sprintf(str, "Gyro X: %f\n\r", gimbal.gyro_reading_raw[INS_GYRO_X_ADDRESS_OFFSET]);
				serial_send_string(str);
				sprintf(str, "Gyro Y: %f\n\r", gimbal.gyro_reading_raw[INS_GYRO_Y_ADDRESS_OFFSET]);
				serial_send_string(str);
				sprintf(str, "Gyro Z: %f\n\r", gimbal.gyro_reading_raw[INS_GYRO_Z_ADDRESS_OFFSET]);
				serial_send_string(str);
			
				sprintf(str, "Acce X: %f\n\r", gimbal.acce_reading_raw[INS_ACCEL_X_ADDRESS_OFFSET]);
				serial_send_string(str);
				sprintf(str, "Acce Y: %f\n\r", gimbal.acce_reading_raw[INS_ACCEL_Y_ADDRESS_OFFSET]);
				serial_send_string(str);
				sprintf(str, "Acce Z: %f\n\r", gimbal.acce_reading_raw[INS_ACCEL_Z_ADDRESS_OFFSET]);
				serial_send_string(str);

			  delay_ms(200);
			  
			  //Make the motor turn
			  CAN_CMD_GIMBAL(2000, 0, 0, 0);
			
			  //Get CAN received data
			  
			  gimbal.yaw_motor->pos_raw = gimbal.yaw_motor->gimbal_motor_raw->ecd;
			  gimbal.yaw_motor->speed_raw = gimbal.yaw_motor->gimbal_motor_raw->speed_rpm;
			  gimbal.yaw_motor->current_raw = gimbal.yaw_motor->gimbal_motor_raw->given_current;
			
			
			  //Sending data via UART
				serial_send_string("position: %d\n\r");
				serial_send_int(gimbal.yaw_motor->pos_raw);
				serial_send_string("speed: %d\n\r");
				serial_send_int(gimbal.yaw_motor->speed_raw);
				serial_send_string("current: %d\n\r");
				serial_send_int(gimbal.yaw_motor->current_raw);	
			  vTaskDelay(200);
				
				
    }
    
}
