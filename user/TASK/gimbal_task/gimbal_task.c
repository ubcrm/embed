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
#include "shoot_task.h"
#include <math.h>

#define DEADBAND 10

// This is accessbile globally and some data is loaded from INS_task
Gimbal_t gimbal;

//UART mailbox
char str[32] = {0}; // TODO: figure out why ins_task is using this... delete and compile to reproduce
static char message[64] = {0};
static int loop_counter = 0;

/******************** Functions ********************/
static void initialization(Gimbal_t *gimbal);
static void get_new_data(Gimbal_t *gimbal);
static void update_setpoints(Gimbal_t *gimbal);
static void increment_PID(Gimbal_t *gimbal);
static void fill_complex_equivalent(fp32 position[2], uint16_t ecd_value);

/**
* Slew Control
* While the current is over a limiting constant:
* Reduce control signal by 0.5 every 5 ms
* When current returns the acceptable range, double the 
* allowed current every 5 ms until it returns to full output. 
* 
* Disable/reset I-term at this time whenever current limiting is active.\
* 
* Do we want reductions to follow : 1/2, 1/4, 1/8, ... 
*                                    or 1/2, 1/4 , OFF
* Or mutliply out put constant
* Tunable Parameters
* redutions factor: e.g. reduce current by 0.5
* reduction time: e.g. reduce current every 5 ms
* increase factor
* increase time
* I-term off/reset
* hysteresis time: how long should the current limiter 
*       wait before starting to increase/decrease current.
*/

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
    gimbal_ptr->pitch_motor.motor_feedback = get_Pitch_Gimbal_Motor_Measure_Point();
    gimbal_ptr->yaw_motor.motor_feedback = get_Yaw_Gimbal_Motor_Measure_Point(); 
    gimbal_ptr->launcher = get_launcher_pointer();
    fp32 pid_constants_yaw[3] = {pid_kp_yaw, pid_ki_yaw, pid_kd_yaw};
    fp32 pid_constants_pitch[3] = {pid_kp_pitch, pid_ki_pitch, pid_kd_pitch};
    
    gimbal_ptr->yaw_setpoint[0] = 1.0;
    gimbal_ptr->yaw_setpoint[1] = 0.0;
    gimbal_ptr->yaw_position[0] = 1.0;
    gimbal_ptr->yaw_position[1] = 0.0;
    gimbal_ptr->yaw_error = 0.0;
    
    PID_Init(&(gimbal_ptr->pitch_motor.pid_controller), PID_POSITION, pid_constants_pitch, max_out_pitch, max_i_term_out_pitch);
    PID_Init(&(gimbal_ptr->yaw_motor.pid_controller), PID_POSITION, pid_constants_yaw, max_out_yaw, max_i_term_out_yaw);
    
    gimbal_ptr->rc_update = get_remote_control_point();
    
    gimbal_ptr->pitch_motor.pos_set = 3000;
    gimbal_ptr->yaw_motor.pos_set = 6000;
    // TODO: find better bounds on Pitch
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
    fp32 theta = ecd_value * Motor_Ecd_to_Rad;
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
    
    //yaw:
    //rc -> theta -> complex rotation -> new setpoint

    // TODO: FIX
    if(gimbal_set->rc_update->rc.s[0] == RC_SW_MID || gimbal_set->rc_update->rc.s[0] == RC_SW_UP){
        fp32 theta = -1 * int16_deadzone(gimbal_set->rc_update->rc.ch[2], -DEADBAND, DEADBAND)
                * Motor_Ecd_to_Rad / 40.0f;
        fp32 rotation[2] = {cos(theta), sin(theta)}; // TODO: consider alternative {cos{theta}, sin{theta}}
        multiply_complex_a_by_b(gimbal_set->yaw_setpoint, rotation);
        make_unit_length(gimbal_set->yaw_setpoint);

        gimbal_set->pitch_motor.pos_set += int16_deadzone(gimbal_set->rc_update->rc.ch[3], -DEADBAND, DEADBAND) / 40.0f;
    }
    
    char print[30];
    sprintf(print, "Constrain pitch %d between %d, %d \n\r", gimbal_set->pitch_motor.pos_set, PITCH_MIN, PITCH_MAX);
    serial_send_string(print);
    //int16_constrain(gimbal_set->yaw_motor.pos_set, YAW_MIN, YAW_MAX);
    gimbal_set->pitch_motor.pos_set = int16_constrain(gimbal_set->pitch_motor.pos_set, PITCH_MIN, PITCH_MAX);
    sprintf(print, "Constrained to %d \n\r", gimbal_set->pitch_motor.pos_set);
    serial_send_string(print);
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
    /* Get error sign
        get error val
        insert into pid*/
    // error magnitude 1 * ERROR_MULTIPLIER corresponds to gimbal being 90 degrees out of place --> jump action observed by pedram
    // TODO: consider revising PID to eliminate the ERROR_MULTIPLIER constant
    // TODO: retune PID currently very slow
    fp32 error = get_error_sign(gimbal_pid->yaw_position, gimbal_pid->yaw_setpoint) 
                    * (1 - a_dot_b(gimbal_pid->yaw_position,gimbal_pid->yaw_setpoint))
                    * ERROR_MULTIPLIER;
    // TODO: Modify pid to reject massive d term spike this could create. 
    // TODO: Consider how this isn't quite a linear error (dot product)
    gimbal_pid->yaw_motor.current_out = PID_Calc(&gimbal_pid->yaw_motor.pid_controller, 0.0f, error);
    
    gimbal_pid->pitch_motor.current_out = PID_Calc(&gimbal_pid->pitch_motor.pid_controller, gimbal_pid->pitch_motor.pos_read, gimbal_pid->pitch_motor.pos_set);
    
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

    //TODO - fix below / fill as needed
    
    if(loop_counter == 0){
        /*sprintf(str, "yaw position re: %.2f im: %.2f \n\r", gimbal_msg->yaw_position[0], gimbal_msg->yaw_position[1]);
        serial_send_string(str);
        sprintf(message, "yaw setpoint re: %.2f im: %.2f \n\r", gimbal_msg->yaw_setpoint[0], gimbal_msg->yaw_setpoint[1]);
        serial_send_string(message);
        sprintf(message, "--- \n\r");
        serial_send_string(message);
        sprintf(message, "pitch: %i", gimbal_msg->pitch_motor.pos_read);
        serial_send_string(message);*/
    }
    loop_counter = (loop_counter + 1) % 1000;
    
    
    /*
    sprintf(str, "we are not infected");
    serial_send_string(str);*/
/*
    sprintf(str, "yaw speed read: %d\n\r", gimbal->yaw_motor->speed_read);
    serial_send_string(str);       

static void send_feedback_to_uart(Gimbal_t *gimbal){
    if(counter == 0){
        sprintf(message, "(2,3) yaw rpm is: %i \n\r", gimbal->yaw_motor.motor_feedback->speed_rpm);
        serial_send_string(message);
        sprintf(message, "(4,5) yaw torque current is: %i \n\r", gimbal->yaw_motor.motor_feedback->current_read);
        serial_send_string(message);
    }
    
    counter = (counter + 1) % 1000; */
    
}
