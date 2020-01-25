#include "chassis_task.h"
#include "main.h"
#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include "led.h"
#include "CAN_Receive.h"
#include "delay.h"
#include "USART_comms.h"
#include "INS_task.h"


static uint8_t chassis_init(chassis_t *chassis_move_init);
static void chassis_set_mode(void);
static void chassis_remote_calc(chassis_user_mode_e mode);
static void chassis_set_mode(void);
static void chassis_motor_calc(void);
static void chassis_PID(void);

static chassis_t chassis;
    


void chassis_task(void *pvParameters){

    while(!chassis_init(&chassis)){
    }
    
	while(1) {
        //set mode
        chassis_set_mode();
        //process data/motor vector decomposition 
        //PID
        //output
        CAN_CMD_CHASSIS(0, 0, 0, 0);
        vTaskDelay(50);
    }        
}


/**
 * @brief Starts chassis motors and peripherals, initializes a front vector
 * @param
 * @retval TRUE if init is completed
 */

uint8_t chassis_init(chassis_t *chassis_init){
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
    return TRUE;
}


/**
 * @brief Based on the mode of operation, remote control data is processed
 * @param A value from the enum chassis_user_mode_e
 * @retval 
 */

void chassis_set_mode(void){
	
}


/**
 * @brief Takes the remote control data and converts as specified by user
 * @param
 * @retval 
 */

void chassis_remote_calc(chassis_user_mode_e mode){
    //Get remote control data
    //Debug print out current motor data
    
    
}


/**
 * @brief Handles the data from remote controller
 * @param 
 * @retval 
 */

void chassis_motor_calc(void){
	
}

/**
 * @brief PID calculations for motors, both in translation and rotation 
 * @param 
 * @retval 
 */

void chassis_PID(void){
	//translation
    //rotation
}
