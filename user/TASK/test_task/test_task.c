#include "test_task.h"
#include "main.h"
#include "pid.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "CAN_Receive.h"
#include "delay.h"
#include "USART_comms.h"
#include <stdio.h>

// TODO: determine pid constants
#define pid_kp 2000
#define pid_ki 0
#define pid_kd 0
#define max_out 6000
#define max_iout 0

// #define CHASSIS_CONTROL_TIME_MS 1000

Gimbal_Motor_t gimbal_yaw_motor;

void testTask(void *pvParameters)
{
    gimbal_yaw_motor.gimbal_motor_raw = get_Pitch_Gimbal_Motor_Measure_Point(); // get_Yaw_Gimbal_Motor_Measure_Point();

    int16_t yaw_current;  // electric current?

    PidTypeDef pid;
    fp32 pid_constants[3] = {pid_kp, pid_ki, pid_kd};
    PID_Init(&pid, PID_POSITION, pid_constants, max_out, max_iout);
        
    CAN_CMD_GIMBAL(0, 0, 0, 0);
    
    int move = 0;
    int throttle = 1;
    
    while(1) 
    {
        led_green_toggle();

        // Get CAN received data
        gimbal_yaw_motor.gimbal_pos_raw = gimbal_yaw_motor.gimbal_motor_raw->ecd;
        gimbal_yaw_motor.gimbal_speed_raw = gimbal_yaw_motor.gimbal_motor_raw->speed_rpm;
        gimbal_yaw_motor.gimbal_tq_current_raw = gimbal_yaw_motor.gimbal_motor_raw->given_current;

        yaw_current = PID_Calc(&pid, 0, gimbal_yaw_motor.gimbal_pos_raw); //TODO: confirm ref and set values

        //Make the motor turn
        CAN_CMD_GIMBAL(0, yaw_current, 0, 0);

        /*
        if (throttle)  
        {
            move += 4000;
            CAN_CMD_GIMBAL(0, move, 0, 0);
            move -= 4000;
        }
        else
        {
            move -= 4000;
            CAN_CMD_GIMBAL(0, move, 0, 0);
            move += 4000;
        }
        
        throttle = !(throttle);
        */
        //Sending data via UART
        send_to_uart(gimbal_yaw_motor, pid);			

        // delay_ms(10000);
        // vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}

void send_to_uart(Gimbal_Motor_t gimbal_yaw_motor, PidTypeDef pid) 	
{
    char str[20];//uart data buffer

    sprintf(str, "position: %d\n\r", gimbal_yaw_motor.gimbal_pos_raw);
    Serial_sendString(str);

    sprintf(str, "speed: %d\n\r", gimbal_yaw_motor.gimbal_speed_raw);
    Serial_sendString(str);        

    sprintf(str, "current: %d\n\r", gimbal_yaw_motor.gimbal_tq_current_raw);
    Serial_sendString(str); 

    sprintf(str, "motor kp: %f\n\r", pid.Kp);
    Serial_sendString(str);

    sprintf(str, "motor kd: %f\n\r", pid.Kd);
    Serial_sendString(str);    

    sprintf(str, "motor ki: %f\n\r", pid.Ki);
    Serial_sendString(str);
}
