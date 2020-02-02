/**
  ******************************************************************************
    * @file    TASK/shoot_task
    * @date    24-January/2020
    * @brief   Shooting controls
    * @attention Shoot motors are plugged into pins A8 and E14, order unknown (update this if you find out)
  ******************************************************************************
**/

#include "shoot_task.h"
#include "main.h"
#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


/******************** User Includes ********************/

#include "remote_control.h"
#include "CAN_Receive.h"
#include "user_lib.h"
#include "fric.h"

shoot_t shoot;

static uint16_t shoot_init(void);
static void shoot_control_loop(void);

// user defines
static fp32 pwm_target = Fric_OFF;

/******************** Task/Functions Called from Outside ********************/

void shoot_task(void *pvParameters){
    
    while(!shoot_init()) {
    }
    vTaskDelay(2000);
    while(1) {
		// fric_off();
        shoot_control_loop();
		vTaskDelay(1);
    }
    
}

static uint16_t shoot_init(void) {
    //Get RC pointers
    shoot.rc = get_remote_control_point();

    //Initialize ramp
    ramp_init(&shoot.ramp1, RAMP_PRD, Fric_UP, Fric_OFF);
    ramp_init(&shoot.ramp2, RAMP_PRD, Fric_UP, Fric_OFF);

    //Set pwm motor to zero
    shoot.fric1_pwm = /*Fric_OFF*/ 50;
    shoot.fric2_pwm = /*Fric_OFF*/ 50;

    return TRUE;
}


static void shoot_control_loop(void) {
    if (shoot.rc->rc.s[POWER_SWITCH] == ON) {
        pwm_target = Fric_DOWN;
        
        if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_MID) {
            // no shoot
            pwm_target = Fric_OFF;
        } else if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_DOWN) {
            // single shot
        } else if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_UP) {
            // rapid fire
        } else {
            // throw some kind of error?
        }
        
    } else {
        pwm_target = Fric_OFF;
    }
    
    // ramp
    ramp_calc(&shoot.ramp1, pwm_target);
    ramp_calc(&shoot.ramp2, pwm_target);
    
    //Sets pwm output to be the ramp value
    shoot.fric1_pwm = (uint16_t)(shoot.ramp1.out);
    shoot.fric2_pwm = (uint16_t)(shoot.ramp2.out);
    
    //Set flywheels
    fric1_on(shoot.fric1_pwm);
    fric2_on(shoot.fric2_pwm);
    
}
