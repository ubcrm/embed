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

#include "math.h"
#include <stdio.h>
#include "gimbal_task.h"
#include "stm32f4xx.h"
#include "USART_comms.h"
#include "string.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

float theta_setpoint;
float phi_setpoint;
float rc_channel_1;
float rc_channel_2;
Gimbal_t gimbal;
Gimbal_Motor_t gimbal_pitch_motor;
Gimbal_Motor_t gimbal_yaw_motor;

//UART mailbox
char str[32] = {0};

/**
  * @brief      Handles cases when RC joystick is not quite centered
  * @author     RM
  * @param[in]  input, RC joystick value
  * @param[in]  output, control value after deadline is applied
  * @param[in]  deadline, specifies range in which joystick is considered centered
  */
#define rc_deadline_limit(input, output, deadline) \
	{                                              \
		if (input > deadline || input < -deadline) \
            output = input;                        \
        else                                       \
            output = 0;                            \
    }


void gimbal_task(void* parameters){

    vTaskDelay(GIMBAL_TASK_INIT_TIME);
	
    gimbal_pitch_motor.gimbal_motor_raw = get_Pitch_Gimbal_Motor_Measure_Point();
    gimbal_yaw_motor.gimbal_motor_raw = get_Yaw_Gimbal_Motor_Measure_Point();
    
    fp32 pitch_signal;
    fp32 yaw_signal;
    int vision_signal;
    
    PidTypeDef pid_pitch;
    fp32 pid_constants[3] = {pid_kp, pid_ki, pid_kd};
    PID_Init(&pid_pitch, PID_POSITION, pid_constants, max_out, max_iout);

    PidTypeDef pid_yaw;
    PID_Init(&pid_yaw, PID_POSITION, pid_constants, max_out, max_iout);
    
	while(1){	
        /* For now we assume channel 1 is left-right stick and channel 2 is dn-up stick*/
        /* For now using strictly encoder feedback for position */
       
		// Gets update on position from encoders and gyro
		// Comparison to setpoint
		// run PID
        //vTaskDelay(CONTROL_TIME);

        // Update RC pointer
        
        
        // Get CAN received data
        gimbal_pitch_motor.pos_raw = gimbal_pitch_motor.gimbal_motor_raw->ecd;
        gimbal_pitch_motor.speed_raw = gimbal_pitch_motor.gimbal_motor_raw->speed_rpm;
        gimbal_pitch_motor.current_raw = gimbal_pitch_motor.gimbal_motor_raw->given_current;

        gimbal_yaw_motor.pos_raw = gimbal_yaw_motor.gimbal_motor_raw->ecd;
        gimbal_yaw_motor.speed_raw = gimbal_yaw_motor.gimbal_motor_raw->speed_rpm;
        gimbal_yaw_motor.current_raw = gimbal_yaw_motor.gimbal_motor_raw->given_current;

        // Calculate setpoints based on RC signal
        phi_setpoint = linear_map_float_to_int(rc_channel_2, RC_MIN, RC_MAX, PITCH_MIN, PITCH_MAX);
        theta_setpoint = linear_map_float_to_int(rc_channel_1, RC_MIN, RC_MAX, YAW_MIN, YAW_MAX);
        
        pitch_signal = PID_Calc(&pid_pitch, gimbal_pitch_motor.pos_raw, phi_setpoint);
        yaw_signal = PID_Calc(&pid_yaw, gimbal_yaw_motor.pos_raw, theta_setpoint);

        // Turn gimbal motor
        CAN_CMD_GIMBAL(0, pitch_signal, 0, 0);
        
        vTaskDelay(1);
        
        send_to_uart(gimbal_pitch_motor, pid_pitch, pitch_signal);  //Sending data via UART
	}
}


int linear_map_float_to_int(float val, float val_max, float val_min, int out_min, int out_max){
    float percentage = val / (val_max - val_min);
    
    return (int) (percentage * (out_max - out_min)); 
}


float get_domain_angle(float alpha){
	int done = 0;
	while(!done){
		if(alpha > PI){
			alpha -= PI;
		}
		else if(alpha < -PI){
			alpha += PI;\
		}
		else{
			done = TRUE;
		}
	}
    return 0.0;
}


float get_relative_angle(float alpha, float beta){
	return get_domain_angle(beta - alpha);
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
 * @brief  
 * @param  
 * @retval None
 */
void send_to_uart(Gimbal_Motor_t gimbal_yaw_motor, PidTypeDef pid_pitch, fp32 pitch_signal) 	
{
    char str[20]; //uart data buffer

    sprintf(str, "position: %d\n\r", gimbal_yaw_motor.pos_raw);
    serial_send_string(str);

    sprintf(str, "speed: %d\n\r", gimbal_yaw_motor.speed_raw);
    serial_send_string(str);       

    sprintf(str, "current: %d\n\r", gimbal_yaw_motor.current_raw);
    serial_send_string(str);

    sprintf(str, "motor kp: %f\n\r", pid_pitch.Kp);
    serial_send_string(str);

    sprintf(str, "motor kd: %f\n\r", pid_pitch.Kd);
    serial_send_string(str);    

    sprintf(str, "motor ki: %f\n\r", pid_pitch.Ki);
    serial_send_string(str);
    
    sprintf(str, "pitch signal: %f\n\r", pitch_signal);
    serial_send_string(str);
}
