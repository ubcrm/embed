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
#include "USART_comms.h"
#include "test_task.h"

shoot_t shoot;

static uint16_t shoot_init(void);
static void shoot_control_loop(void);

// user defines
static uint16_t pwm_target = Fric_OFF;
static uint16_t pwm_output = Fric_OFF;

/******************** Task/Functions Called from Outside ********************/

void shoot_task(void *pvParameters) {
    while(!shoot_init()) {
    }
    vTaskDelay(2000);
    while(1) {
        shoot_control_loop();
	vTaskDelay(5);
    }
}

static uint16_t shoot_init(void) {
    // Get RC pointers
    shoot.rc = get_remote_control_point();

    // Deal with weird starting of the motors
    shoot.fric1_pwm = Fric_INIT;
    shoot.fric2_pwm = Fric_INIT;
    
    fric1_on(shoot.fric1_pwm);

    return TRUE;
}


static void shoot_control_loop(void) {
    if (shoot.rc->rc.s[POWER_SWITCH] == ON) {
        pwm_target = Fric_DOWN;
        
        // Hopper:spin constantly
        
        if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_MID) {
            // no shoot
        } else if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_DOWN) {
            // single shot
            pwm_target = Fric_UP;
        } else if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_UP) {
            // rapid fire
            // Trigger: turn 90 degrees
        } else {
            // throw some kind of error?
            pwm_target = Fric_OFF;
        }
        
    } else {
        pwm_target = Fric_OFF;
        // Hopper: off
    }

    if (pwm_output < pwm_target) {
        pwm_output += 1;
    } else if (pwm_output > pwm_target) {
        pwm_output -= 1;
    }
    
    shoot.fric1_pwm = pwm_output;
    shoot.fric2_pwm = pwm_output;
    
    //Set flywheels
    fric1_on(shoot.fric1_pwm);
    fric2_on(shoot.fric2_pwm);    
}
