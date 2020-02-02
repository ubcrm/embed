/**
  ******************************************************************************
    * @file    TASK/test_task
    * @date    24-January/2020
    * @brief   This file contains tasks and functions used for hardware testing
    * @attention Leave the task loop blank
  ******************************************************************************
**/

#ifndef USER_TASK_H
#define USER_TASK_H

#include "CAN_receive.h"
#include "main.h"
#include "pid.h"


/******************** PID Constants (May need to rename...) ********************/

#define pid_kp 20.0f
#define pid_ki 0.0f
#define pid_kd 0.0f
#define max_out 5000.0f
#define max_iout 0


/******************** Public Definitions & Structs ********************/

typedef struct 
{
	const motor_measure_t *gimbal_motor_raw;
	uint16_t pos_raw;
	uint16_t speed_raw;
	uint16_t current_raw;
} Gimbal_Motor_t;

typedef struct 
{
	Gimbal_Motor_t *yaw_motor;
	Gimbal_Motor_t *pitch_motor;
    const fp32 *angle_reading_raw;
	const fp32 *gyro_reading_raw;
	const fp32 *acce_reading_raw;
} Gimbal_t;


/******************** Task/Functions Called Outside ********************/
//Task used for testing. Usually left blank.
void test_task(void *pvParameters);
	
#endif
