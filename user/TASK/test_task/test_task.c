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
#include "pid.h"
#include "remote_control.h"


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

//Reading angle, gyro, and accelerometer data and printing to serial
static void test_imu_readings(uint8_t angle, uint8_t gyro, uint8_t acce);
//Turns a gimbal motor (GM6020) and outputs its position, rpm, and current
static void test_GM6020(void);
//Enables Debug of P19 (chassis) motor
static void test_P19(int id);



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
 * @brief  Reads vision instruction from UART and cap to certin values
 * @param  None
 * @retval Vision signal in range of 0 and 8191
 */
int get_vision_signal(void) {
    int vision_signal = -1000;  // TODO: Get real values from vision
        
    while (vision_signal > 8191) {
        vision_signal -= 8191;
    }
    while (signal < 0) {
        signal+= 8192;
    }
    return signal;
}

int get_vision_signal() {
    int vision_signal = 3000;  // TODO: Get real values from vision
    int output_signal = truncate_pid_signal(vision_signal);
    return output_signal;
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
