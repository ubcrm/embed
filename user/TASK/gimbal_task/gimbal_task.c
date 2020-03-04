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
#include "start_task.h"


/******************** User Includes ********************/
#include "CAN_Receive.h"
#include "user_lib.h"
#include "remote_control.h"
#include "USART_comms.h"
#include <stdio.h>
#include "pid.h"

#define DEADBAND 10

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

    vTaskDelay(GIMBAL_INIT_DELAY);
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
        vTaskDelay(GIMBAL_TASK_DELAY);
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
static void get_new_data(Gimbal_t *gimbal_data){
        // Get CAN received data 
        gimbal_data->pitch_motor.pos_read = gimbal_data->pitch_motor.motor_feedback->ecd;
        gimbal_data->pitch_motor.speed_read = gimbal_data->pitch_motor.motor_feedback->speed_rpm;

        gimbal_data->yaw_motor.pos_read = gimbal_data->yaw_motor.motor_feedback->ecd;
        gimbal_data->yaw_motor.speed_read = gimbal_data->yaw_motor.motor_feedback->speed_rpm;
}

/** 
 * @brief  Updates desired setpoints from RC signal
 * @param  Gimbal struct containing info on Gimbal
 * @retval None
 */
static void update_setpoints(Gimbal_t *gimbal_set){
    
    gimbal_set->yaw_motor.pos_set += (-1) * int16_deadzone(gimbal_set->rc_update->rc.ch[2], -DEADBAND, DEADBAND) / 10;
    gimbal_set->pitch_motor.pos_set += int16_deadzone(gimbal_set->rc_update->rc.ch[3], -DEADBAND, DEADBAND) / 10;
    
    int16_constrain(gimbal_set->yaw_motor.pos_set, YAW_MIN, YAW_MAX);
    int16_constrain(gimbal_set->pitch_motor.pos_set, PITCH_MIN, PITCH_MAX);

    //TODO: worry about the case where pos_set is unsigned and rc channel manages to push it    
    // negative for a moment, causing the position to wrap from below 0 to a maxvalue. 
}

/** 
 * @brief  Increments PID loop based on latest setpoints and latest positions
 * @param  None
 * @retval None
 */
static void increment_PID(Gimbal_t *gimbal_pid){
    gimbal_pid->pitch_motor.current_out = PID_Calc(&gimbal_pid->pitch_motor.pid_controller, gimbal_pid->pitch_motor.pos_read, gimbal_pid->pitch_motor.pos_set);
    gimbal_pid->yaw_motor.current_out = PID_Calc(&gimbal_pid->yaw_motor.pid_controller, gimbal_pid->yaw_motor.pos_read, gimbal_pid->yaw_motor.pos_set);
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
void send_to_uart(Gimbal_t *gimbal_msg) 	
{
    char str[20]; //uart data buffer

    //TODO - fix below / fill as needed

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
