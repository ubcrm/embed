/**
* Gimbal task 
* Converts user input to Gimbal motor setpoints
*
* Gimbal position is defined by angles phi and theta
* Theta increases counterclockwise if viewed from the top and is 0 when facing fwd.
* Phi increases counterclockwise if viewed from the right side and is 0 when horizontal
* (when Theta is 0)
*
* Gimbal control strategy and either be gyro based or encoder based 
* RM motors give encoders wrap around at a half rotation negative and positive
* Encoders results are between 0 and 8191
*/


#include "gimbal_task.h"
#include "stm32f4xx.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


/******************** User Includes ********************/
#include "CAN_Receive.h"
#include "user_lib.h"
#include "remote_control.h"
#include "USART_comms.h"
#include <stdio.h>
#include "pid.h"

// This is accessbile globally and some data is loaded from INS_task
Gimbal_t gimbal;

//UART mailbox
char str[32] = {0};

/******************** Functions ********************/
static void initialization(Gimbal_t *gimbal);
static void get_new_data(Gimbal_t *gimbal);
static void update_setpoints(Gimbal_t *gimbal);
static void increment_PID(Gimbal_t *gimbal);


/**
 * @brief Initializes PID and fetches Gimbal motor data to ensure 
 *  gimbal points in direction specified. 
 * @param FreeRTOS parameters
 * @retval None
 */
void gimbal_task(void* parameters){

    vTaskDelay(200);
	initialization(&gimbal);
    
    while(1){	
        //send_to_uart(&gimbal);
        
        /* For now using strictly encoder feedback for position */
        
        get_new_data(&gimbal);
        //send_to_uart(&gimbal);
        update_setpoints(&gimbal);
        //send_to_uart(&gimbal);
        increment_PID(&gimbal);
        //send_to_uart(&gimbal);
        // Turn gimbal motor
        
        
        CAN_CMD_GIMBAL( (int16_t) gimbal.yaw_motor.current_out, 
                        (int16_t) gimbal.pitch_motor.current_out,
                        0,
                        0);
        
          //Sending data via UART
        vTaskDelay(5);
	}
}

/** 
 * @brief  Initializes gimbal struct and loads pointers for RC and motor feedback
 * @param  None
 * @retval None
 */
static void initialization(Gimbal_t *gimbal_ptr){
    gimbal_ptr->pitch_motor.motor_feedback = get_Pitch_Gimbal_Motor_Measure_Point();
    gimbal_ptr->yaw_motor.motor_feedback = get_Yaw_Gimbal_Motor_Measure_Point(); 

    fp32 pid_constants_yaw[3] = {pid_kp_yaw, pid_ki_yaw, pid_kd_yaw};
    fp32 pid_constants_pitch[3] = {pid_kp_pitch, pid_ki_pitch, pid_kd_pitch};

    PID_Init(&(gimbal_ptr->pitch_motor.pid_controller), PID_POSITION, pid_constants_pitch, max_out_pitch, max_i_term_out_pitch);
    PID_Init(&(gimbal_ptr->yaw_motor.pid_controller), PID_POSITION, pid_constants_yaw, max_out_yaw, max_i_term_out_yaw);
    
    gimbal_ptr->rc_update = get_remote_control_point();
    
    gimbal_ptr->pitch_motor.pos_set = 6000;
    gimbal_ptr->yaw_motor.pos_set = 6000;
}

/** 
 * @brief  Update encoder positions
 * @param  None
 * @retval None
 */
static void get_new_data(Gimbal_t *gimbal){
        // Get CAN received data 
        gimbal->pitch_motor.pos_read = gimbal->pitch_motor.motor_feedback->ecd;
        gimbal->pitch_motor.speed_read = gimbal->pitch_motor.motor_feedback->speed_rpm;

        gimbal->yaw_motor.pos_read = gimbal->yaw_motor.motor_feedback->ecd;
        gimbal->yaw_motor.speed_read = gimbal->yaw_motor.motor_feedback->speed_rpm;
}

/** 
 * @brief  Updates desired setpoints from RC signal
 * @param  Gimbal struct containing info on Gimbal
 * @retval None
 */
static void update_setpoints(Gimbal_t *gimbal){
    
    gimbal->yaw_motor.pos_set += gimbal->rc_update->rc.ch[2] / 10;
    gimbal->pitch_motor.pos_set += gimbal->rc_update->rc.ch[3] / 10;
    
    // Calculate setpoints based on RC signal. //TODO:Place in function
//    if(switch_is_mid(gimbal->rc_update->rc.s[0]) || switch_is_up(gimbal->rc_update->rc.s[0])){
        
        //linear_map_int_to_int(gimbal->rc_update->rc.ch[2], RC_MIN, RC_MAX, YAW_MIN, YAW_MAX);
        //linear_map_int_to_int(gimbal->rc_update->rc.ch[3], RC_MIN, RC_MAX, PITCH_MAX, PITCH_MIN);
    
    /*}else{
        gimbal->yaw_motor.pos_set = linear_map_int_to_int(average(YAW_MIN, YAW_MAX), RC_MIN, RC_MAX, YAW_MAX, YAW_MIN);
        gimbal->pitch_motor.pos_set = linear_map_int_to_int(average(PITCH_MIN, PITCH_MAX), RC_MIN, RC_MAX, PITCH_MAX, PITCH_MIN);
    }*/
}

/** 
 * @brief  Increments PID loop based on latest setpoints and latest positions
 * @param  None
 * @retval None
 */
static void increment_PID(Gimbal_t *gimbal){
    gimbal->pitch_motor.current_out = PID_Calc(&gimbal->pitch_motor.pid_controller, gimbal->pitch_motor.pos_read, gimbal->pitch_motor.pos_set);
    gimbal->yaw_motor.current_out = PID_Calc(&gimbal->yaw_motor.pid_controller, gimbal->yaw_motor.pos_read, gimbal->yaw_motor.pos_set);
}

/** 
 * @brief  Reads vision instruction from UART and cap to certin values
 * @param  None
 * @retval Vision signal in range of 0 and 8191
 */
int get_vision_signal(void) {
    int vision_signal = 5700;  // TODO: Get real values from vision
        
    while (vision_signal > 8191) {
        vision_signal -= 8191;
    }
    while (vision_signal < 0) {
        vision_signal+= 8192;
    }
    return vision_signal;
}

/** 
 * @brief Updates Uart with position information on the yaw motor and the PID settings
 * This will block for several milliseconds
 * @param gimbal_yaw_motor struct containing information about the gimbal yaw motor
 * @param pid struct containing pid coefficients
 * @param pitch_signal signal to pitch motor
 * @retval None
 */
void send_to_uart(Gimbal_t *gimbal) 	
{
    char str[20]; //uart data buffer

    if(gimbal->yaw_motor.pos_read == NULL){
        sprintf(str, "null ... :( %d\n\r", 123);
    serial_send_string(str);
    } else{
        sprintf(str, "yaw position read: %d\n\r", 321);
    serial_send_string(str);
    }
        

/*
    sprintf(str, "yaw speed read: %d\n\r", gimbal->yaw_motor->speed_read);
    serial_send_string(str);       

    sprintf(str, "yaw current read: %d\n\r", gimbal->yaw_motor->current_read);
    serial_send_string(str);
*/  
    //sprintf(str, "yaw setpoint: %d\n\r", gimbal->yaw_motor->pos_set);
    //serial_send_string(str);
    
    //sprintf(str, "yaw current out: %d\n\r", gimbal->yaw_motor->current_out);
    //serial_send_string(str);
}
