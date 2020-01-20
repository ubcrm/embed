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


static uint8_t chassis_init(void);
static void chassis_set_mode(void);
static void chassis_get_raw_data(void);
static void chassis_set_mode(void);
static void chassis_data_process(void);


void chassis_task(void *pvParameters){
    while(!chassis_init()){
    }
    
	while(1) {
        //get data
        //set mode
        //process data/motor vector decomposition 
        //PID
        //output
    }        
}


/**
 * @brief Starts chassis motors and peripherals, initializes a front vector
 * @param
 * @retval TRUE if init is completed
 */

uint8_t chassis_init(void){
	
    return TRUE;
}

/**
 * @brief Starts chassis motors and peripherals, initializes a front vector
 * @param
 * @retval 
 */

void chassis_get_raw_data(void){
    
    
}

/**
 * @brief Based on the mode of operation, remote control data is processed
 * @param A value from the enum chassis_user_mode_e
 * @retval 
 */

void chassis_set_mode(void){
	
}

/**
 * @brief Handles the data from remote controller
 * @param 
 * @retval 
 */

void chassis_data_process(void){
	
}

/**
 * @brief PID calculations for motors
 * @param 
 * @retval 
 */

void chassis_PID(void){
	
}
