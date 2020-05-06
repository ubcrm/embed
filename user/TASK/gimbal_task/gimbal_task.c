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
#include "arm_math.h"

/******************** User Includes ********************/
#include "CAN_Receive.h"
#include "user_lib.h"
#include "remote_control.h"
#include "USART_comms.h"
#include <stdio.h>
#include "pid.h"
#include "shoot_task.h"
#include <math.h>

#define DEADBAND 1

// This is accessbile globally and some data is loaded from INS_task
Gimbal_t gimbal;

static int loop_counter = 0;

/******************** Functions ********************/
static void initialization(Gimbal_t *gimbal);
static void get_new_data(Gimbal_t *gimbal);
static void update_setpoints(Gimbal_t *gimbal);
static void increment_PID(Gimbal_t *gimbal);
static void fill_complex_equivalent(fp32 position[2], uint16_t ecd_value);

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
        CAN_CMD_GIMBAL( (int16_t) gimbal.yaw_motor.voltage_out, 
                        (int16_t) gimbal.pitch_motor.voltage_out,
                        (int16_t) gimbal.launcher->trigger_motor.speed_set, 
                        (int16_t) gimbal.launcher->hopper_motor.speed_set);
        
          //Sending data via UART
        vTaskDelay(GIMBAL_TASK_DELAY);
        send_to_uart(&gimbal);
	}
}

/** 
 * @brief  Initializes gimbal struct and loads pointers for RC and motor feedback
 * @param  None
 * @retval None
 */
static void initialization(Gimbal_t *gimbal_ptr){
    gimbal_ptr->pitch_motor.motor_feedback = get_pitch_motor_feedback_pointer();
    gimbal_ptr->yaw_motor.motor_feedback = get_yaw_gimbal_motor_feedback_pointer(); 
    gimbal_ptr->launcher = get_launcher_pointer();
    fp32 pid_constants_yaw[3] = {pid_kp_yaw, pid_ki_yaw, pid_kd_yaw};
    fp32 pid_constants_pitch[3] = {pid_kp_pitch, pid_ki_pitch, pid_kd_pitch};
    
    gimbal_ptr->yaw_setpoint[0] = 0.0;
    gimbal_ptr->yaw_setpoint[1] = -1.0;
    gimbal_ptr->yaw_position[0] = 0.0;
    gimbal_ptr->yaw_position[1] = -1.0;
    gimbal_ptr->yaw_error = 0.0;
    
    PID_Init(&(gimbal_ptr->pitch_motor.pid_controller), PID_POSITION, pid_constants_pitch, max_out_pitch, max_i_term_out_pitch);
    PID_Init(&(gimbal_ptr->yaw_motor.pid_controller), PID_POSITION, pid_constants_yaw, max_out_yaw, max_i_term_out_yaw);
    
    gimbal_ptr->rc_update = get_remote_control_point();
    
    gimbal_ptr->pitch_motor.pos_set = GIMBAL_PITCH_INITIAL_POSITION;
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
    
    fill_complex_equivalent(gimbal_data->yaw_position, gimbal_data->yaw_motor.pos_read);
}


static void fill_complex_equivalent(fp32 position[2], uint16_t ecd_value){
    fp32 theta = ecd_value * MOTOR_ECD_TO_RAD;
    position[0] = cos(theta);
    position[1] = sin(theta);
}

static void multiply_complex_a_by_b(fp32 a[2], fp32 b[2]){
    fp32 real = a[0] * b[0] - a[1] * b[1];
    fp32 imaj = a[0] * b[1] + a[1] * b[0];
    a[0] = real;
    a[1] = imaj;
}

static void make_unit_length(fp32 n[2]){
    fp32 length = sqrt(n[0] * n[0] + n[1] * n[1]);
    n[0] = n[0] / length;
    n[1] = n[1] / length;
}

/** 
 * @brief  Updates desired setpoints from RC signal
 * @param  Gimbal struct containing info on Gimbal
 * @retval None
 */
static void update_setpoints(Gimbal_t *gimbal_set){
    
    if(gimbal_set->rc_update->rc.s[0] == RC_SW_MID || gimbal_set->rc_update->rc.s[0] == RC_SW_UP){
        fp32 theta = -1 * int16_deadzone(gimbal_set->rc_update->rc.ch[2], -DEADBAND, DEADBAND)
                * MOTOR_ECD_TO_RAD / 80.0f;
        fp32 rotation[2] = {arm_cos_f32(theta), arm_sin_f32(theta)};
        multiply_complex_a_by_b(gimbal_set->yaw_setpoint, rotation);
        make_unit_length(gimbal_set->yaw_setpoint);

        gimbal_set->pitch_motor.pos_set += int16_deadzone(gimbal_set->rc_update->rc.ch[3], -DEADBAND, DEADBAND) / 80.0f;
    }
    
    gimbal_set->pitch_motor.pos_set = int16_constrain(gimbal_set->pitch_motor.pos_set, PITCH_MIN, PITCH_MAX);
}

static fp32 a_dot_b(fp32 a[2], fp32 b[2]){
    return a[0] * b[0] + a[1] * b[1];
}

static fp32 length_of_a_cross_b(fp32 a[2], fp32 b[2]){
    return a[0] * b[1] - a[1] * b[0];
}

static int16_t get_error_sign(fp32 actual_position[2], fp32 set_position[2]){
    return length_of_a_cross_b(actual_position, set_position) >= 0 ? 1 : -1;
}    


/** 
 * @brief  Increments PID loop based on latest setpoints and latest positions
 * @param  None
 * @retval None
 */
static void increment_PID(Gimbal_t *gimbal_pid){
    // error magnitude 1 * ERROR_MULTIPLIER corresponds to gimbal being 90 degrees out of place --> jump action observed by pedram
    // TODO: consider revising PID to eliminate the ERROR_MULTIPLIER constant
    
    fp32 error = get_error_sign(gimbal_pid->yaw_position, gimbal_pid->yaw_setpoint) 
                    * (1 - a_dot_b(gimbal_pid->yaw_position,gimbal_pid->yaw_setpoint))
                    * ERROR_MULTIPLIER;
    
    // TODO: Consider how this isn't quite a linear error (dot product)
    gimbal_pid->yaw_motor.voltage_out = PID_Calc(&gimbal_pid->yaw_motor.pid_controller, 0.0f, error);
    
    gimbal_pid->pitch_motor.voltage_out = PID_Calc(&gimbal_pid->pitch_motor.pid_controller, gimbal_pid->pitch_motor.pos_read, gimbal_pid->pitch_motor.pos_set);
    
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
    if(loop_counter == 0){
    }
    loop_counter = (loop_counter + 1) % 1000;
}
