/*
* Header file for gimbal_task.c
* Holds tuning constants and unit conversions
*/

#ifndef GIMBAL
#define GIMBAL
#include "main.h"
#include "CAN_receive.h"
#include "pid.h"


/*************** Converts between motor position and degrees *****************/
#define Motor_Ecd_to_Rad 0.000766990394f
#define FALSE 0
#define TRUE 1
#define average(x,y) ((x + y) / 2.0)
#define range(x,y) (y - x)


/****************************** PID Constants ********************************/
#define pid_kp 40.0f
#define pid_ki 0.0f
#define pid_kd 0.0f
#define max_out 5000.0f
#define max_iout 0

/***************************** Gimbal Constants *****************************/
#define GIMBAL_TASK_INIT_TIME 201
#define CONTROL_TIME 1
#define RC_MIN -1.0
#define RC_MAX 1.0
#define ENCODER_MIN 0
#define ENCODER_MAX 8191
#define YAW_MIN 0
#define YAW_MAX 8191
#define PITCH_MIN 0
#define PITCH_MAX 8191


/************************** Gimbal Data Structures ***************************/
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


/******************************* Function Declarations ***********************/
int get_signal(int signal);
extern void gimbal_task(void *pvParameters);
void send_to_uart(Gimbal_Motor_t gimbal_yaw_motor, PidTypeDef pid, fp32 pitch_signal); 

/*
* Returns an angle between -pi and pi
* Requires: an angle in radians
* Returns: an equivalent angle in radians between -pi and pi
*/
float get_domain_angle(float alpha);

/*
* Gets smallest angle required to travel from alpha to beta
* requires: two angles in radians
* returns: angle in radians with magnitude < 2*pi
*/
float get_relative_angle(float alpha, float beta);

/**
 * Maps float in a specified range to an int in a new range as a linear mapping function
 */
int linear_map_int_to_int(int val, int val_min, int val_max, int out_min, int out_max);


/******************************* Variable Declarations ***********************/
// These will later be produced from RC
/* angle in radians*/


#endif
