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


/****************************** PID Constants ********************************/
#define pid_kp 40.0f
#define pid_ki 0.0f
#define pid_kd 0.0f
#define max_out 5000.0f
#define max_iout 0

/***************************** Gimbal Constants *****************************/
#define GIMBAL_TASK_INIT_TIME 201
#define CONTROL_TIME 1
#define RC_MIN -660
#define RC_MAX 660
#define ENCODER_MIN 0
#define ENCODER_MAX 8191
#define YAW_MIN 2359
#define YAW_MAX 6576
#define PITCH_MIN 5500
#define PITCH_MAX 7000


/************************** Gimbal Data Structures ***************************/
typedef struct 
{
	const motor_measure_t *gimbal_motor_feedback;
	uint16_t pos_read;
	uint16_t speed_read;
	uint16_t current_read;
} Gimbal_Motor_t;

typedef struct 
{
	Gimbal_Motor_t *yaw_motor;
	Gimbal_Motor_t *pitch_motor;
    const fp32 *angle_update;
	const fp32 *gyro_update;
	const fp32 *accel_update;

    uint16_t pitch_pos_read;
    uint16_t yaw_pos_read;
    uint16_t pitch_pos_set;
    uint16_t yaw_pos_set;
    
    int16_t pitch_speed_read;
    int16_t yaw_speed_read;
    int16_t pitch_speed_set;
    int16_t yaw_speed_set;
} Gimbal_t;


/******************************* Function Declarations ***********************/
int get_vision_signal(void);
extern void gimbal_task(void *pvParameters);
extern void send_to_uart(Gimbal_Motor_t gimbal_yaw_motor, PidTypeDef pid, fp32 pitch_signal); 



/******************************* Variable Declarations ***********************/
// These will later be produced from RC
/* angle in radians*/


#endif
