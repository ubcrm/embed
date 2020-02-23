/**
  ******************************************************************************
    * @file    TASK/chassis_task
    * @date    03-February/2020
    * @brief   This file contains tasks and functions to control the chassis.
    *          Raw control mode maps the motor output to wheel speed, no interaction with gimbal.
    * @attention PID tuning required.
  ******************************************************************************
**/

#include "chassis_task.h"
#include "main.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


/******************** User Includes ********************/
#include "CAN_Receive.h"
#include "USART_comms.h"
#include "remote_control.h"
#include "INS_task.h"
#include "pid.h"


/******************** Private User Declarations ********************/
static void chassis_init(Chassis_t *chassis_init);
static void get_new_data(Chassis_t *chassis_update);
static void set_control_mode(Chassis_t *chassis);
static void calculate_chassis_motion_setpoints(chassis_user_mode_e mode);
static void calculate_motor_setpoints(Chassis_t *chassis);
static void increment_PID(uint8_t debug);

static Chassis_t chassis;
    


/******************** Main Task/Functions Called from Outside ********************/

/**
 * @brief Starts chassis motors and peripherals, initializes a front vector
 * @param FreeRTOS parameters
 * @retval None
 */
void chassis_task(void *pvParameters){
    
    // Delay to make sure critical communications/timers have been initialised
    vTaskDelay(20);
    //Initializes chassis with pointers to RC commands and CAN feedback messages
    
    chassis_init(&chassis);
    
	while(1) {
        
        get_new_data(&chassis); //updates RC commands and CAN motor feedback
        set_control_mode(&chassis); //Note: currently not implemented
        calculate_chassis_motion_setpoints(CHASSIS_VECTOR_RAW);
        calculate_motor_setpoints(&chassis);
        increment_PID(FALSE);
        //output
        CAN_CMD_CHASSIS(chassis.motor[FRONT_RIGHT].current_out, 
                        chassis.motor[FRONT_LEFT].current_out, 
                        chassis.motor[BACK_LEFT].current_out, 
                        chassis.motor[BACK_RIGHT].current_out);
        
        vTaskDelay(1);
    
    }
}


/**
 * @brief Util, returns a pointer to the main chassis struct
 * @param None
 * @retval A pointer to the main chassis struct
 */
Chassis_t* get_chassis_point(void) {
    return &chassis;
}




/******************** Private User Functions ********************/

/**
 * @brief Starts chassis motors and peripherals, initializes a front vector, a PID struct, and links motor data pointers.
 * @param chassis_init pointer to chassis struct
 * @retval None
 */
static void chassis_init(Chassis_t *chassis_init){    
    //Init PID constants
    fp32 def_pid_constants[3]  = {M3508_KP, M3508_KI, M3508_KD};
    
    //Link pointers with CAN motors
    for (int i = 0; i < 4; i++) {
        chassis_init->motor[i].motor_feedback = get_Chassis_Motor_Measure_Point(i);
        chassis_init->motor[i].speed_read = chassis_init->motor[i].motor_feedback->speed_rpm;
        chassis_init->motor[i].pos_read = chassis_init->motor[i].motor_feedback->ecd;
        chassis_init->motor[i].current_read = chassis_init->motor[i].motor_feedback->given_current;
			
		PID_Init(&chassis_init->motor[i].pid_controller, PID_DELTA, def_pid_constants, M3508_MAX_OUT, M3508_MIN_OUT);
    }
    
    //Init yaw and front vector
    chassis_init->vec_raw = get_INS_angle_point();
    chassis_init->yaw_pos_raw = chassis_init->vec_raw + INS_YAW_ADDRESS_OFFSET;
		
    //Pointer remote
    chassis_init->rc_update = get_remote_control_point();
}


/**
 * @brief Updates speed, position, and current data from all motors.
 * @param None
 * @retval None
 */
static void get_new_data(Chassis_t *chassis_update){
		for (int i = 0; i < 4; i++) {
        chassis_update->motor[i].speed_read = chassis_update->motor[i].motor_feedback->speed_rpm;
        chassis_update->motor[i].pos_read = chassis_update->motor[i].motor_feedback->ecd;
        chassis_update->motor[i].current_read = chassis_update->motor[i].motor_feedback->given_current;
		}
}


/**
 * @brief Based on the mode of operation, remote control data is processed. 
 *      Currently blank as raw control is implemented
 * @param None
 * @retval None
 */

static void set_control_mode(Chassis_t *chassis){
	//Don't do anything
    //Implement chassis_follow_gimbal_yaw mode in the future
}



/**
 * @brief Takes the remote control data and converts to the desired robot velocities
 * @param  mode the chassis driving mode
 * @retval None
 */

static void calculate_chassis_motion_setpoints(chassis_user_mode_e mode){
    //Get remote control data and put into x_speed_read etc
    //process based on mode (which is currently none) and put into x_speed_set
    //Debug print out current 
    
    // get rc data and put into chassis struct
	
	//Switch Chassis Only
	if(switch_is_down(chassis.rc_update->rc.s[0])){
        chassis.x_speed_read = chassis.rc_update->rc.ch[RC_X];
        chassis.y_speed_read = chassis.rc_update->rc.ch[RC_Y];
        chassis.z_speed_read = chassis.rc_update->rc.ch[RC_Z];
    }else if(switch_is_mid(chassis.rc_update->rc.s[0])){
        chassis.x_speed_read = 0;
        chassis.y_speed_read = chassis.rc_update->rc.ch[RC_Y];
        chassis.z_speed_read = chassis.rc_update->rc.ch[RC_Z];
    }else if(switch_is_up(chassis.rc_update->rc.s[0])){
        chassis.x_speed_read = 0;
        chassis.y_speed_read = 0;
        chassis.z_speed_read = 0;
    }
    
    // process raw sppeds based on modes (for now they are just the same)
    chassis.x_speed_set = chassis.x_speed_read;
    chassis.y_speed_set = chassis.y_speed_read;
    chassis.z_speed_set = chassis.z_speed_read;
}


/**
 * @brief Handles the data from remote controller. Converts xyz axis into mecanum wheel current values.
 * @param None
 * @retval None
 */
static void calculate_motor_setpoints(Chassis_t *chassis){
	//Take x_speed_set etc and handle mechanum wheels
    //Put results into Chassis_Motor_t speed_set (and/or pos_set)
    chassis->motor[FRONT_LEFT].speed_set = MULTIPLIER * (chassis->y_speed_set + chassis->z_speed_set + chassis->x_speed_set);
    chassis->motor[BACK_LEFT].speed_set = MULTIPLIER * (chassis->y_speed_set + chassis->z_speed_set - chassis->x_speed_set);
    chassis->motor[FRONT_RIGHT].speed_set = MULTIPLIER * (-chassis->y_speed_set + chassis->z_speed_set + chassis->x_speed_set);
    chassis->motor[BACK_RIGHT].speed_set = MULTIPLIER * (-chassis->y_speed_set + chassis->z_speed_set - chassis->x_speed_set);
}


//Buffer used to store data for PID debugging
char pid_out[64];


/**
 * @brief PID calculations for motors, ensures that the motors run at a given speed
 * @param None
 * @retval None
 */
static void increment_PID(uint8_t debug){
	//translation
    //rotation
	fp32 front_right;
    fp32 back_right;
    fp32 front_left;
    fp32 back_left;

	front_right = PID_Calc(&chassis.motor[FRONT_RIGHT].pid_controller, 
	chassis.motor[FRONT_RIGHT].speed_read,
	chassis.motor[FRONT_RIGHT].speed_set);
	chassis.motor[FRONT_RIGHT].current_out += front_right;
	
	back_right = PID_Calc(&chassis.motor[BACK_RIGHT].pid_controller, 
	chassis.motor[BACK_RIGHT].speed_read, 
	chassis.motor[BACK_RIGHT].speed_set);
	chassis.motor[BACK_RIGHT].current_out += back_right;
	
	front_left = PID_Calc(&chassis.motor[FRONT_LEFT].pid_controller, 
	chassis.motor[FRONT_LEFT].speed_read, 
	chassis.motor[FRONT_LEFT].speed_set);
	chassis.motor[FRONT_LEFT].current_out += front_left;
	
	back_left = PID_Calc(&chassis.motor[BACK_LEFT].pid_controller, 
	chassis.motor[BACK_LEFT].speed_read, 
	chassis.motor[BACK_LEFT].speed_set);
	chassis.motor[BACK_LEFT].current_out += back_left;
	
    if (debug == 1) {
        sprintf(pid_out, "Front Right - target: %d, sensor: %d, output: %d \n\r", 
        chassis.motor[FRONT_RIGHT].speed_set, 
        chassis.motor[FRONT_RIGHT].speed_read, 
        chassis.motor[FRONT_RIGHT].current_out);
        serial_send_string(pid_out);
        
        sprintf(pid_out, "Back Right - target: %d, sensor: %d, output: %d \n\r", 
        chassis.motor[BACK_RIGHT].speed_set, 
        chassis.motor[BACK_RIGHT].speed_read, 
        chassis.motor[BACK_RIGHT].current_out);
        serial_send_string(pid_out);
        
        sprintf(pid_out, "Front Left - target: %d, sensor: %d, output: %d \n\r", 
        chassis.motor[FRONT_LEFT].speed_set, 
        chassis.motor[FRONT_LEFT].speed_read, 
        chassis.motor[FRONT_LEFT].current_out);
        serial_send_string(pid_out);
        
        sprintf(pid_out, "Back Left - target: %d, sensor: %d, output: %d \n\r", 
        chassis.motor[BACK_LEFT].speed_set, 
        chassis.motor[BACK_LEFT].speed_read, 
        chassis.motor[BACK_LEFT].current_out);
        serial_send_string(pid_out);
    }
	
}
