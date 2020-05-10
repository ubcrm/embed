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

#define SHOOT_TASK_DELAY 1
#define SHOOT_INIT_DELAY 2000

//Time between ramp steps, in seconds
#define RAMP_PRD 0.001
//Used for pwm ramp initialization
#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

// User defines
#define POWER_SWITCH 0
#define SHOOT_SWITCH 1

#define DIR_REVERSED -1

//Hopper motor
#define HOPPER_SPEED 800
#define HOPPER_OFF 0

//Trigger motor PID
#define TRIGGER_MAX_OUT 8000.0f
#define TRIGGER_MAX_IOUT 2500.0f
#define TRIGGER_ANGLE_PID_KP 800.0f
#define TRIGGER_ANGLE_PID_KI 0.5f
#define TRIGGER_ANGLE_PID_KD 0.0f

//Trigger motor
#define TRIGGER_90_DEGS 2048
#define HALF_ECD_RANGE 4096
#define FULL_ECD_RANGE 8192
#define HALF_TRIGGER_RATIO 18
#define FULL_TRIGGER_RATIO 36
#define TRIGGER_REACHED_POS_RANGE 500
#define TRIGGER_SPEED -400
#define TRIGGER_OFF 0


typedef struct
{
    const motor_feedback_t *shoot_motor_raw;
	uint16_t pos_raw;
    uint16_t last_pos_raw;
	int16_t speed_raw;
    uint16_t pos_set;
	int16_t speed_set;
    int16_t speed_out;
    
    //Variables used by the trigger motor only
    int8_t ecd_count;
    uint16_t geared_down_pos_raw;
    uint16_t geared_down_pos_set;
    uint8_t move_flag;
    uint16_t cmd_time;
}Shoot_Motor_t;


typedef enum
{
    SHOOT_OFF,
    SHOOT_READY,
    SHOOT_REVERSED,
    SHOOT_RAPID,
    SHOOT_DONE,
} shoot_mode_e;

typedef struct 
{
    ramp_function_source_t ramp1;
    ramp_function_source_t ramp2;
    uint16_t fric1_pwm;
    uint16_t fric2_pwm;
    const RC_ctrl_t *rc;
    Shoot_Motor_t hopper_motor;
    Shoot_Motor_t trigger_motor;
    shoot_mode_e mode;

}Shoot_t;
extern void shoot_task(void *pvParameters);
extern Shoot_t* get_launcher_pointer(void);
#endif


