/**
  ******************************************************************************
    * @file    TASK/chassis_task
    * @date    03-February/2020
    * @brief   This file contains tasks and functions to control the chassis.
    *          Raw control mode maps the motor output to wheel speed, no interaction with gimbal.
    * @attention PID tuning required.
  ******************************************************************************
**/

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "CAN_receive.h"
#include "main.h"
#include "remote_control.h"
#include "pid.h"

/******************************* Task Delays *********************************/
#define CHASSIS_TASK_DELAY 5
#define CHASSIS_INIT_DELAY 20

/******************** User Definitions ********************/

//Chassis motor CAN ID offset 
#define FRONT_RIGHT 0
#define FRONT_LEFT 1
#define BACK_LEFT 2
#define BACK_RIGHT 3

#define SETPOINT_SENSITIVITY 5

// RC channels -- this indicated strafe drive
// left stick: rotation
// right stick: fwd/rev, left/right
#define RC_X 0 
#define RC_Y 1
#define RC_Z 2
// For 

//M3508 motors max and min CAN output
#define M3508_MAX_OUT 10000
#define M3508_MIN_OUT 50.0
#define M3508_MAX_IOUT 40
//M3508 speed PID constants
#define M3508_KP 0.002f
#define M3508_KI 0.00f
#define M3508_KD 0.0f
// Current Limiting Constants
#define HYSTERESIS_PERIOD 5
#define CURRENT_LIMIT 25000

typedef enum{
    FULL_CURRENT,  
    HALF_CURRENT,
    QUARTER_CURRENT,
    NO_CURRENT,
} current_limiter_state_e;

typedef enum{
    CHASSIS_VECTOR_RAW,
    CHASSIS_FOLLOW_GIMBAL_YAW,
    CHASSIS_INDIVIDUAL_CONTROL,
} chassis_user_mode_e;


typedef struct 
{
    const motor_feedback_t *motor_feedback;
    
    //Current speed read from motors
    uint16_t pos_read;
    int16_t speed_read;
    int16_t current_read;
    
    //Target speed set by user/remote control
    int16_t speed_set;
    // TODO: check if these should be unsigned /exist at all
    
    //Final output speed
    int16_t current_out;
    
    // Current limiting parameters
    int8_t limiter_counter;
    current_limiter_state_e limiter;
	
    //Control
    PidTypeDef pid_controller;
} Chassis_Motor_t;


typedef struct 
{
    Chassis_Motor_t motor[4];
    chassis_user_mode_e mode;
    
    //Raw remote control data
    const RC_ctrl_t *rc_update;
    
    //Current front vector
    const fp32 *vec_raw;
    const fp32 *yaw_pos_raw;
    
    //Current speed, vector combination of the speed read from motors
    int16_t x_speed_read;
    int16_t y_speed_read;
    int16_t z_speed_read;
    
    //Speed set by user/remote control
    int16_t x_speed_set;
    int16_t y_speed_set;
    int16_t z_speed_set;
} Chassis_t;






/******************** Main Task/Functions Called from Outside ********************/

extern void chassis_task(void *pvParameters);
extern Chassis_t* get_chassis_point(void);

#endif
