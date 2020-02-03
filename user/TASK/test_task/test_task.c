/**
  ******************************************************************************
    * @file    TASK/test_task
    * @date    24-January/2020
    * @brief   This file contains tasks and functions used for hardware testing
    * @attention Leave the task loop blank
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
#include "remote_control.h"
#include "string.h"
#include "gimbal_task.h"

Gimbal_t gimbal_test;
Gimbal_Motor_t gimbal_pitch_motor_test;

//UART mailbox
char str_0[32] = {0};

/******************** Task/Functions Called Outside ********************/
/**
 * @brief: Task used for testing. Usually left blank.
 * @param: None
 * @retval: None
 */
void test_task(void *pvParameters)
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
    gimbal_test.angle_reading_raw = get_INS_angle_point();
    gimbal_test.gyro_reading_raw = get_MPU6500_Gyro_Data_Point();
	gimbal_test.acce_reading_raw = get_MPU6500_Accel_Data_Point();
    
    if (angle == TRUE) {        
        //Sending angle data via UART
        sprintf(str_0, "Angle yaw: %f\n\r", gimbal_test.angle_reading_raw[INS_YAW_ADDRESS_OFFSET]);
        serial_send_string(str_0);
        sprintf(str_0, "Angle pitch: %f\n\r", gimbal_test.angle_reading_raw[INS_PITCH_ADDRESS_OFFSET]);
        serial_send_string(str_0);
        sprintf(str_0, "Angle roll: %f\n\r", gimbal_test.angle_reading_raw[INS_ROLL_ADDRESS_OFFSET]);
        serial_send_string(str_0);
        serial_send_string("\n");
    }
    
    if (gyro == TRUE) {
        //Sending gyro data via UART
        sprintf(str_0, "Gyro X: %f\n\r", gimbal_test.gyro_reading_raw[INS_GYRO_X_ADDRESS_OFFSET]);
        serial_send_string(str_0);
        sprintf(str_0, "Gyro Y: %f\n\r", gimbal_test.gyro_reading_raw[INS_GYRO_Y_ADDRESS_OFFSET]);
        serial_send_string(str_0);
        sprintf(str_0, "Gyro Z: %f\n\r", gimbal_test.gyro_reading_raw[INS_GYRO_Z_ADDRESS_OFFSET]);
        serial_send_string(str_0);
        serial_send_string("\n");
    }
 
    if (acce == TRUE) {        
        //Sending accelerometer data via UART
        sprintf(str_0, "Acce X: %f\n\r", gimbal_test.acce_reading_raw[INS_ACCEL_X_ADDRESS_OFFSET]);
        serial_send_string(str_0);
        sprintf(str_0, "Acce Y: %f\n\r", gimbal_test.acce_reading_raw[INS_ACCEL_Y_ADDRESS_OFFSET]);
        serial_send_string(str_0);
        sprintf(str_0, "Acce Z: %f\n\r", gimbal_test.acce_reading_raw[INS_ACCEL_Z_ADDRESS_OFFSET]);
        serial_send_string(str_0);
        serial_send_string("\n");
    }
}


/** 
 * @brief  Turns P19 (Chassis Motor) and returns encoder position
 * @param  int id
 * @retval None
 * @attention The motor needs to be set to ID 1
 */
static void test_P19(int id){
    //
}

/** 
 * @brief  Turns a gimbal motor (GM6020) and outputs its position, rpm, and current
 * @param  None
 * @retval None
 * @attention The motor needs to be set to ID 1
 */
static void test_GM6020(void){
    //Link pointer (only needs to be done once)
    gimbal_test.yaw_motor->gimbal_motor_raw = get_Yaw_Gimbal_Motor_Measure_Point();
    
    //Make the motor turn
    CAN_CMD_GIMBAL(2000, 0, 0, 0);

    //Get CAN received data
    gimbal_test.yaw_motor->pos_raw = gimbal_test.yaw_motor->gimbal_motor_raw->ecd;
    gimbal_test.yaw_motor->speed_raw = gimbal_test.yaw_motor->gimbal_motor_raw->speed_rpm;
    gimbal_test.yaw_motor->current_raw = gimbal_test.yaw_motor->gimbal_motor_raw->given_current;

    //Sending data via UART
    sprintf(str_0, "position: %d\n\r", gimbal_test.yaw_motor->pos_raw);
    serial_send_string(str_0);
    sprintf(str_0, "speed: %d\n\r", gimbal_test.yaw_motor->speed_raw);
    serial_send_string(str_0);
    sprintf(str_0, "current: %d\n\r", gimbal_test.yaw_motor->current_raw);
    serial_send_string(str_0);	
}
