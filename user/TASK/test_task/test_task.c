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
#include "pid.h"
#include "remote_control.h"
#include "chassis_task.h"

/******************** Private User Declarations ********************/
Gimbal_t gimbal;
Gimbal_Motor_t gimbal_pitch_motor;

//UART mailbox
char str[32] = {0};

//Turns a gimbal motor (GM6020) and outputs its position, rpm, and current
static void test_GM6020(void);
//Enables Debug of P19 (chassis) motor
static void test_P19(int id);

/******************** Task/Functions Called Outside ********************/

/**
 * @brief: Task used for testing. Usually left blank.
 * @param: None
 * @retval: None
 */
void test_task(void *pvParameters)
{
    while(1) {
        vTaskDelay(2000);
    }  
}



/******************** Private Functions ********************/



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

