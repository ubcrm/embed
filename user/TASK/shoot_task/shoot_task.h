/**
  ******************************************************************************
    * @file    TASK/shoot_task
    * @date    24-January/2020
    * @brief   Shooting controls
    * @attention Shoot motors are plugged into pins A8 and E14, order unknown (update this if you find out)
  ******************************************************************************
**/

#include "main.h"
#include "stm32f4xx.h"
#include "remote_control.h"
#include "fric.h"
#include "user_lib.h"
#include "CAN_receive.h"

#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H	

//Time between ramp steps, in seconds
#define RAMP_PRD 0.001
//Used for pwm ramp initialization
#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

// User defines
#define POWER_SWITCH 1
#define SHOOT_SWITCH 0

#define ON 1
#define OFF 0

#define HOPPER_SPEED 1500
#define HOPPER_OFF 0
#define TRIGGER_90_DEGS 2048

typedef struct
{
	const motor_measure_t *shoot_motor_raw;
	int16_t pos_raw;
	int16_t speed_raw;
    int16_t pos_set;
	int16_t speed_set;
}Shoot_Motor_t;

typedef struct 
{
    ramp_function_source_t ramp1;
    ramp_function_source_t ramp2;
    uint16_t fric1_pwm;
    uint16_t fric2_pwm;
    const RC_ctrl_t *rc;
    Shoot_Motor_t hopper_motor;
    Shoot_Motor_t trigger_motor;
}Shoot_t;


extern void shoot_task(void *pvParameters);
#endif


