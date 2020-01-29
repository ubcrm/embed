#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "CAN_receive.h"
#include "main.h"
#include "remote_control.h"
#include "pid.h"


//Chassis motor CAN ID offset 
#define FRONT_RIGHT 0
#define FRONT_LEFT 1
#define BACK_LEFT 2
#define BACK_RIGHT 3

// RC channels
#define RC_X 2
#define RC_Y 1
#define RC_Z 0


typedef struct 
{
    const motor_measure_t *motor_raw;
    
    //Current speed read from motors
    int16_t pos_raw;
    int16_t speed_raw;
    int16_t current_raw;
    
    //Target speed set by user/remote control
    int16_t pos_set;
    int16_t speed_set;
    
    //Final output speed
    int16_t speed_out;
	
		//Control
		PidTypeDef pid_control;
} Chassis_Motor_t;


typedef struct 
{
    Chassis_Motor_t motor[4];
    
    //Raw remote control data
    const RC_ctrl_t *rc_raw;
    
    //Current front vector
    const fp32 *vec_raw;
    const fp32 *yaw_pos_raw;
    
    //Current speed, vector combination of the speed read from motors
    int16_t x_speed_raw;
    int16_t y_speed_raw;
    int16_t z_speed_raw;
    
    //Speed set by user/remote control
    int16_t x_speed_set;
    int16_t y_speed_set;
    int16_t z_speed_set;
} Chassis_t;


typedef enum{
    CHASSIS_VECTOR_RAW,
    CHASSIS_FOLLOW_GIMBAL_YAW,
    CHASSIS_INDIVIDUAL_CONTROL,
} chassis_user_mode_e;



extern void chassis_task(void *pvParameters);
extern uint8_t chassis_init(Chassis_t *chassis_init);
extern Chassis_t* get_chassis_point(void);

#endif
