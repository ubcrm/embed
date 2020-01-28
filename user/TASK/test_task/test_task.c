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
#define pid_kp 4
#define pid_ki 0.01
#define pid_kd 0.5
#define max_out 15000
#define max_iout 0

Gimbal_t gimbal;
Gimbal_Motor_t gimbal_pitch_motor;

//UART mailbox
char str[32] = {0};

//Turns a gimbal motor (GM6020) and outputs its position, rpm, and current
static void test_GM6020(void);

/******************** Private User Declarations (VISION TO BE MIGRATED) ********************/

static void testVision(void);
int get_vision_signal(void); 
void send_to_uart(Gimbal_Motor_t gimbal_yaw_motor, PidTypeDef pid, fp32 pitch_signal);

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
 * @brief  Reads vision instruction from UART and cap to certin values
 * @param  None
 * @retval Vision signal in range of 0 and 8191
 */
static void testVision(void) {

    gimbal_pitch_motor.gimbal_motor_raw = get_Pitch_Gimbal_Motor_Measure_Point();
    
    fp32 pitch_signal;
    int vision_signal;
    
    PidTypeDef pid;
    fp32 pid_constants[3] = {pid_kp, pid_ki, pid_kd};
    PID_Init(&pid, PID_POSITION, pid_constants, max_out, max_iout);

    while (1) {     
        // Get CAN received data
        gimbal_pitch_motor.pos_raw = gimbal_pitch_motor.gimbal_motor_raw->ecd;
        gimbal_pitch_motor.speed_raw = gimbal_pitch_motor.gimbal_motor_raw->speed_rpm;
        gimbal_pitch_motor.current_raw = gimbal_pitch_motor.gimbal_motor_raw->given_current;

        vision_signal = get_vision_signal();
        
        pitch_signal = PID_Calc(&pid, gimbal_pitch_motor.pos_raw, vision_signal);

        // Turn gimbal motor
        CAN_CMD_GIMBAL(0, pitch_signal, 0, 0);

        //Sending data via UART
        send_to_uart(gimbal_pitch_motor, pid, pitch_signal);
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

/** 
 * @brief  Reads vision instruction from UART and cap to certin values
 * @param  None
 * @retval Vision signal in range of 0 and 8191
 */
int get_vision_signal(void) {
    int vision_signal = -1000;  // TODO: Get real values from vision
        
    while (vision_signal > 8191) {
        vision_signal -= 8191;
    }
    while (vision_signal < 0) {
        vision_signal += 8192;
    }
    
    return vision_signal;
}


/** 
 * @brief  
 * @param  
 * @retval None
 */
void send_to_uart(Gimbal_Motor_t gimbal_yaw_motor, PidTypeDef pid, fp32 pitch_signal) 	
{
    char str[20]; //uart data buffer

    sprintf(str, "position: %d\n\r", gimbal_yaw_motor.pos_raw);
    serial_send_string(str);

    sprintf(str, "speed: %d\n\r", gimbal_yaw_motor.speed_raw);
    serial_send_string(str);       

    sprintf(str, "current: %d\n\r", gimbal_yaw_motor.current_raw);
    serial_send_string(str);

    sprintf(str, "motor kp: %f\n\r", pid.Kp);
    serial_send_string(str);

    sprintf(str, "motor kd: %f\n\r", pid.Kd);
    serial_send_string(str);    

    sprintf(str, "motor ki: %f\n\r", pid.Ki);
    serial_send_string(str);
    
    sprintf(str, "pitch signal: %f\n\r", pitch_signal);
    serial_send_string(str);
}
