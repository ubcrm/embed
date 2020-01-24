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
#include "mpu6500driver.h"

Gimbal_t gimbal;
char str[32] = {0};

void test_task(void *pvParameters)
{
    while(1) {
			  
        vTaskDelay(200);

    }  
}

void test_imu_readings(){
    //Link pointers
    gimbal.gyro_reading_raw = get_MPU6500_Gyro_Data_Point();
		gimbal.acce_reading_raw = get_MPU6500_Accel_Data_Point();
    
    //Sending gyro and accelerometer data via UART
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
}

void test_CAN_gimbal_motors(){
    //Link pointer
    gimbal.yaw_motor->gimbal_motor_raw = get_Yaw_Gimbal_Motor_Measure_Point();
    
    //Make the motor turn
    CAN_CMD_GIMBAL(2000, 0, 0, 0);

    //Get CAN received data
    gimbal.yaw_motor->pos_raw = gimbal.yaw_motor->gimbal_motor_raw->ecd;
    gimbal.yaw_motor->speed_raw = gimbal.yaw_motor->gimbal_motor_raw->speed_rpm;
    gimbal.yaw_motor->current_raw = gimbal.yaw_motor->gimbal_motor_raw->given_current;


    //Sending data via UART
    sprintf(str, "position: %d\n\r", gimbal.yaw_motor->pos_raw);
    serial_send_int(str);
    sprintf(str, "speed: %d\n\r", gimbal.yaw_motor->speed_raw);
    serial_send_int(str);
    sprintf(str, "current: %d\n\r", gimbal.yaw_motor->current_raw);
    serial_send_int(str);	
}
