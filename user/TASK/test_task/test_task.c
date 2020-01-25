/**
  ******************************************************************************
    * @file    TASK/test_task
    * @date    24-January/2020
    * @brief   This file contains tasks and functions used for hardware testing
    * @attention Leave the main loop blank
  ******************************************************************************
**/
#include "test_task.h"
#include "main.h"
#include "stm32f4xx.h"
#include <stdio.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


/******************** User Includes ********************/
#include "led.h"
#include "CAN_Receive.h"
#include "delay.h"
#include "INS_task.h"
#include "USART_comms.h"
#include "mpu6500driver.h"


/******************** Private User Declarations ********************/
Gimbal_t gimbal;
char str[32] = {0};

//Reading angle, gyro, and accelerometer data and printing to serial
static void test_imu_readings(uint8_t angle, uint8_t gyro, uint8_t acce);
//Turns a gimbal motor (GM6020) and outputs its position, rpm, and current
static void test_GM6020(void);


/******************** Task/Functions Called Outside ********************/

/**
 * @brief: Task used for testing. Usually left blank.
 * @param: None
 * @retval: None
 */
static void test_task(void *pvParameters)
{
    while(1) {
			  
        vTaskDelay(200);

    }  
}



/******************** Private Functions ********************/

/**
 * @brief  Reading angle, gyro, and accelerometer data and printing to serial
 * @param  angle: displays angle offset from start (zero) if TRUE
 * @param  gyro: displays gyroscope data if TRUE
 * @param  acce: displays accelerometer data if TRUE
 * @retval None
 */
static void test_imu_readings(uint8_t angle, uint8_t gyro, uint8_t acce){
    //Link pointers
    gimbal.angle_reading_raw = get_INS_angle_point();
    gimbal.gyro_reading_raw = get_MPU6500_Gyro_Data_Point();
	gimbal.acce_reading_raw = get_MPU6500_Accel_Data_Point();
    
    if (angle == TRUE) {        
        //Sending angle data via UART
        sprintf(str, "Angle yaw: %f\n\r", gimbal.angle_reading_raw[INS_YAW_ADDRESS_OFFSET]);
        serial_send_string(str);
        sprintf(str, "Angle pitch: %f\n\r", gimbal.angle_reading_raw[INS_PITCH_ADDRESS_OFFSET]);
        serial_send_string(str);
        sprintf(str, "Angle roll: %f\n\r", gimbal.angle_reading_raw[INS_ROLL_ADDRESS_OFFSET]);
        serial_send_string(str);
        serial_send_string("\n");
    }
    
    if (gyro == TRUE) {
        //Sending gyro data via UART
        sprintf(str, "Gyro X: %f\n\r", gimbal.gyro_reading_raw[INS_GYRO_X_ADDRESS_OFFSET]);
        serial_send_string(str);
        sprintf(str, "Gyro Y: %f\n\r", gimbal.gyro_reading_raw[INS_GYRO_Y_ADDRESS_OFFSET]);
        serial_send_string(str);
        sprintf(str, "Gyro Z: %f\n\r", gimbal.gyro_reading_raw[INS_GYRO_Z_ADDRESS_OFFSET]);
        serial_send_string(str);
        serial_send_string("\n");
    }
 
    if (acce == TRUE) {        
        //Sending accelerometer data via UART
        sprintf(str, "Acce X: %f\n\r", gimbal.acce_reading_raw[INS_ACCEL_X_ADDRESS_OFFSET]);
        serial_send_string(str);
        sprintf(str, "Acce Y: %f\n\r", gimbal.acce_reading_raw[INS_ACCEL_Y_ADDRESS_OFFSET]);
        serial_send_string(str);
        sprintf(str, "Acce Z: %f\n\r", gimbal.acce_reading_raw[INS_ACCEL_Z_ADDRESS_OFFSET]);
        serial_send_string(str);
        serial_send_string("\n");
    }
}


/** 
 * @brief  Turns a gimbal motor (GM6020) and outputs its position, rpm, and current
 * @param  None
 * @retval None
 * @attention The motor needs to be set to ID 1
 */
static void test_GM6020(void){
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
    serial_send_string(str);
    sprintf(str, "speed: %d\n\r", gimbal.yaw_motor->speed_raw);
    serial_send_string(str);
    sprintf(str, "current: %d\n\r", gimbal.yaw_motor->current_raw);
    serial_send_string(str);	
}
