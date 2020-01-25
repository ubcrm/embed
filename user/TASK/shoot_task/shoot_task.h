#include "main.h"
#include "stm32f4xx.h"
#include "remote_control.h"
#include "fric.h"
#include "user_lib.h"

#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H	

//Time between ramp steps, in seconds
#define RAMP_PRD 0.001
//Used for pwm ramp initialization
#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

typedef struct 
{
    ramp_function_source_t ramp1;
    ramp_function_source_t ramp2;
    uint16_t fric1_pwm;
    uint16_t fric2_pwm;
    const RC_ctrl_t *rc;
}shoot_t;


extern void shoot_task(void *pvParameters);
#endif


