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
static void calculate_chassis_motion_setpoints(Chassis_t *chassis_set);
static void calculate_motor_setpoints(Chassis_t *chassis_motors);
static void increment_PID(Chassis_t *chassis_pid);
static void send_feedback_to_uart(Chassis_t *chassis);

static Chassis_t chassis;
    
#define DEBUG 0

/******************** Main Task/Functions Called from Outside ********************/

/**
 * @brief Starts chassis motors and peripherals, initializes a front vector
 * @param FreeRTOS parameters
 * @retval None
 */
void chassis_task(void *pvParameters){
    
    // Delay to make sure critical communications/timers have been initialised
    vTaskDelay(CHASSIS_INIT_DELAY);
    //Initializes chassis with pointers to RC commands and CAN feedback messages
    
    chassis_init(&chassis);
    
	while(1) {
        
        get_new_data(&chassis); //updates RC commands and CAN motor feedback
        set_control_mode(&chassis); //Note: currently not implemented
        calculate_chassis_motion_setpoints(&chassis);
        calculate_motor_setpoints(&chassis);
        increment_PID(&chassis);
        //output
        CAN_CMD_CHASSIS(chassis.motor[FRONT_RIGHT].current_out, 
                        chassis.motor[FRONT_LEFT].current_out, 
                        chassis.motor[BACK_LEFT].current_out, 
                        chassis.motor[BACK_RIGHT].current_out);
        
        vTaskDelay(CHASSIS_TASK_DELAY);
    
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
        chassis_init->motor[i].current_read = chassis_init->motor[i].motor_feedback->current_read;
			
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
        chassis_update->motor[i].current_read = chassis_update->motor[i].motor_feedback->current_read;
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

static void calculate_chassis_motion_setpoints(Chassis_t *chassis_set){
    //Get remote control data and put into x_speed_read etc
    //process based on mode (which is currently none) and put into x_speed_set
    //Debug print out current 
    
    // get rc data and put into chassis struct
	
	//Switch Chassis Only
	if(switch_is_down(chassis_set->rc_update->rc.s[0])){
        chassis_set->x_speed_read = chassis_set->rc_update->rc.ch[RC_X];
        chassis_set->y_speed_read = chassis_set->rc_update->rc.ch[RC_Y];
        chassis_set->z_speed_read = chassis_set->rc_update->rc.ch[RC_Z];
    }else if(switch_is_mid(chassis_set->rc_update->rc.s[0])){
        chassis_set->x_speed_read = 0;
        chassis_set->y_speed_read = chassis_set->rc_update->rc.ch[RC_Y];
        chassis_set->z_speed_read = chassis_set->rc_update->rc.ch[RC_Z];
    }else if(switch_is_up(chassis_set->rc_update->rc.s[0])){
        chassis_set->x_speed_read = 0;
        chassis_set->y_speed_read = 0;
        chassis_set->z_speed_read = 0;
    }
    
    // process raw sppeds based on modes (for now they are just the same)
    chassis_set->x_speed_set = chassis_set->x_speed_read;
    chassis_set->y_speed_set = chassis_set->y_speed_read;
    chassis_set->z_speed_set = chassis_set->z_speed_read;
}


/**
 * @brief Handles the data from remote controller. Converts xyz axis into mecanum wheel current values.
 * @param None
 * @retval None
 */
static void calculate_motor_setpoints(Chassis_t *chassis_motors){
	//Take x_speed_set etc and handle mechanum wheels
    //Put results into Chassis_Motor_t speed_set (and/or pos_set)
    chassis_motors->motor[FRONT_LEFT].speed_set = MULTIPLIER * (chassis_motors->y_speed_set + chassis_motors->z_speed_set + chassis_motors->x_speed_set);
    chassis_motors->motor[BACK_LEFT].speed_set = MULTIPLIER * (chassis_motors->y_speed_set + chassis_motors->z_speed_set - chassis_motors->x_speed_set);
    chassis_motors->motor[FRONT_RIGHT].speed_set = MULTIPLIER * (-chassis_motors->y_speed_set + chassis_motors->z_speed_set + chassis_motors->x_speed_set);
    chassis_motors->motor[BACK_RIGHT].speed_set = MULTIPLIER * (-chassis_motors->y_speed_set + chassis_motors->z_speed_set - chassis_motors->x_speed_set);
}


//Buffer used to store data for PID debugging
char pid_out[64];


/**
 * @brief PID calculations for motors, ensures that the motors run at a given speed
 * @param None
 * @retval None
 */
static void increment_PID(Chassis_t *chassis_pid){
	//translation
    //rotation
	fp32 front_right;
    fp32 back_right;
    fp32 front_left;
    fp32 back_left;

	front_right = PID_Calc(&chassis_pid->motor[FRONT_RIGHT].pid_controller, 
	chassis_pid->motor[FRONT_RIGHT].speed_read,
	chassis_pid->motor[FRONT_RIGHT].speed_set);
	chassis_pid->motor[FRONT_RIGHT].current_out += front_right;
	
	back_right = PID_Calc(&chassis_pid->motor[BACK_RIGHT].pid_controller, 
	chassis_pid->motor[BACK_RIGHT].speed_read, 
	chassis_pid->motor[BACK_RIGHT].speed_set);
	chassis_pid->motor[BACK_RIGHT].current_out += back_right;
	
	front_left = PID_Calc(&chassis_pid->motor[FRONT_LEFT].pid_controller, 
	chassis_pid->motor[FRONT_LEFT].speed_read, 
	chassis_pid->motor[FRONT_LEFT].speed_set);
	chassis_pid->motor[FRONT_LEFT].current_out += front_left;
	
	back_left = PID_Calc(&chassis_pid->motor[BACK_LEFT].pid_controller, 
	chassis_pid->motor[BACK_LEFT].speed_read, 
	chassis_pid->motor[BACK_LEFT].speed_set);
	chassis_pid->motor[BACK_LEFT].current_out += back_left;
	
    if (DEBUG == 1) {
        sprintf(pid_out, "Front Right - target: %d, sensor: %d, output: %d \n\r", 
        chassis_pid->motor[FRONT_RIGHT].speed_set, 
        chassis_pid->motor[FRONT_RIGHT].speed_read, 
        chassis_pid->motor[FRONT_RIGHT].current_out);
        serial_send_string(pid_out);
        
        sprintf(pid_out, "Back Right - target: %d, sensor: %d, output: %d \n\r", 
        chassis_pid->motor[BACK_RIGHT].speed_set, 
        chassis_pid->motor[BACK_RIGHT].speed_read, 
        chassis_pid->motor[BACK_RIGHT].current_out);
        serial_send_string(pid_out);
        
        sprintf(pid_out, "Front Left - target: %d, sensor: %d, output: %d \n\r", 
        chassis_pid->motor[FRONT_LEFT].speed_set, 
        chassis_pid->motor[FRONT_LEFT].speed_read, 
        chassis_pid->motor[FRONT_LEFT].current_out);
        serial_send_string(pid_out);
        
        sprintf(pid_out, "Back Left - target: %d, sensor: %d, output: %d \n\r", 
        chassis_pid->motor[BACK_LEFT].speed_set, 
        chassis_pid->motor[BACK_LEFT].speed_read, 
        chassis_pid->motor[BACK_LEFT].current_out);
        serial_send_string(pid_out);
    }
	
}


static char message[64] = {0};
static int counter = 0;

static void send_feedback_to_uart(Chassis_t *chassis){
    if(counter == 0){
        sprintf(message, "(2,3) rpm is: %i \n\r", chassis->motor[0].motor_feedback->speed_rpm);
        serial_send_string(message);
        sprintf(message, "(4,5) torque feedback is: %i \n\r", chassis->motor[0].motor_feedback->current_read);
        serial_send_string(message);
    }
    
    counter = (counter + 1) % 1000;
    
}
