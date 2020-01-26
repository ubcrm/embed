#include "chassis_task.h"
#include "main.h"
#include "stm32f4xx.h"
#include <stdio.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "CAN_Receive.h"
#include "delay.h"
#include "USART_comms.h"
#include "INS_task.h"
#include "remote_control.h"


static uint8_t chassis_init(Chassis_t *chassis_move_init);
static void chassis_set_mode(void);
static void chassis_remote_calc(chassis_user_mode_e mode);
static void chassis_set_mode(void);
static void chassis_motor_calc(void);
static void chassis_PID(void);

static Chassis_t chassis;
    


void chassis_task(void *pvParameters){
    //Initializes chassis
    while(!chassis_init(&chassis)){
    }
    
	while(1) {
        //set mode
        chassis_set_mode();
        //process RC data into xyz speed
        chassis_remote_calc(CHASSIS_VECTOR_RAW);
        //process xyz speed into vector decomposition 
        chassis_motor_calc();
        //PID calculations, process 
        chassis_PID();
        //output, change to actual output
        CAN_CMD_CHASSIS(0, 0, 0, 0);
        vTaskDelay(50);
    }        
}


/**
 * @brief Starts chassis motors and peripherals, initializes a front vector
 * @param
 * @retval TRUE if init is completed
 */

uint8_t chassis_init(Chassis_t *chassis_init){
    //Forces motors to reset ID
    CAN_CMD_CHASSIS_RESET_ID();
    
    //Link pointers with CAN motors
    for (int i = 0; i < 4; i++) {
        chassis_init->motor[i].motor_raw = get_Chassis_Motor_Measure_Point(i);
        chassis_init->motor[i].speed_raw = chassis_init->motor[i].motor_raw->speed_rpm;
        chassis_init->motor[i].pos_raw = chassis_init->motor[i].motor_raw->ecd;
        chassis_init->motor[i].current_raw = chassis_init->motor[i].motor_raw->given_current;
    }
    
    //Init yaw and front vector
    chassis_init->vec_raw = get_INS_angle_point();
    chassis_init->yaw_pos_raw = chassis_init->vec_raw + INS_YAW_ADDRESS_OFFSET;
    
    //Init PID constants
    
    
    //Pointer remote
    chassis_init->rc_raw = get_remote_control_point();
    
    return TRUE;
}


/**
 * @brief Based on the mode of operation, remote control data is processed
 * @param 
 * @retval 
 */

void chassis_set_mode(void){
	//Don't do anything
}


/**
 * @brief Takes the remote control data and converts as specified by user
 * @param  A value from the enum chassis_user_mode_e
 * @retval 
 */

void chassis_remote_calc(chassis_user_mode_e mode){
    //Get remote control data and put into x_speed_raw etc
    //process based on mode (which is currently none) and put into x_speed_set
    //Debug print out current 
    
    
}


/**
 * @brief Handles the data from remote controller
 * @param 
 * @retval 
 */

void chassis_motor_calc(void){
	//Take x_speed_set etc
    //Handle mechanum wheels
    //Put results into Chassis_Motor_t speed_set (and/or pos_set)
}

/**
 * @brief PID calculations for motors, both in translation and rotation 
 * @param 
 * @retval 
 */

void chassis_PID(void){
    //Don't worry about this for now
	//translation
    //rotation
	  while(1) {
        // todo
      }
}
