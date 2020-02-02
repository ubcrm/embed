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
#include "test_task.h"

shoot_t shoot;

static uint16_t shoot_init(void);
static void shoot_control_loop(void);

/******************** Task/Functions Called from Outside ********************/

void shoot_task(void *pvParameters){
		//fric_off();
		fric1_on(0);
		fric2_on(0);
    while(!shoot_init()) {
        
    }
		fric1_on(0);
		fric2_on(0);
    while(1) {
				fric_off();
			
				vTaskDelay(SHOOT_TASK_DELAY);
        //shoot_control_loop();
    }
    
}

static uint16_t shoot_init(void) {
    //Get RC pointers
    shoot.rc = get_remote_control_point();

    //Initialize ramp
    ramp_init(&shoot.ramp1, RAMP_PRD, Fric_DOWN, Fric_OFF);
    ramp_init(&shoot.ramp2, RAMP_PRD, Fric_DOWN, Fric_OFF);

    //Set pwm motor to zero
    shoot.fric1_pwm = Fric_OFF;
    shoot.fric2_pwm = Fric_OFF;
	
	fric_off();

    return TRUE;
}


static void shoot_control_loop(void) {
    //Add remote control stuff later
    
    if (shoot.rc->rc.s[POWER_SWITCH] == ON) {
        // ramp up wheels to speed
        
        // Hopper:spin constantly
        
        if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_MID) {

            //Currently running SHOOT_ON fully
            shoot.fric1_pwm = Fric_OFF;
            shoot.fric2_pwm = Fric_OFF;

            //Turning the motors on one by one
            ramp_calc(&shoot.ramp1, SHOOT_FRIC_PWM_ADD_VALUE);
            if(shoot.ramp1.out == SHOOT_FRIC_PWM_ADD_VALUE) {
                ramp_calc(&shoot.ramp2, SHOOT_FRIC_PWM_ADD_VALUE);
                
                if(shoot.ramp2.out != SHOOT_FRIC_PWM_ADD_VALUE) {
                    // error, shoot.ramp1 value not set
                }
            } else {
                // error, shoot.ramp1 value not set
            }

            //Sets the ramp maximum, ramp will update to reach this value
            shoot.ramp1.max_value = Fric_DOWN;
            shoot.ramp2.max_value = Fric_DOWN;

            //Sets pwm output to be the ramp value
            shoot.fric1_pwm = (uint16_t)(shoot.ramp1.out);
            shoot.fric2_pwm = (uint16_t)(shoot.ramp2.out);

            //Turn flywheel on
            fric1_on(shoot.fric1_pwm);
            fric2_on(shoot.fric2_pwm);


            // Trigger: stop
            
        } else if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_DOWN) {
            // single shot
            
            // Trigger: turn 90 degrees
        } else if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_UP) {
            // rapid fire
            
            // Trigger: turn 90 degrees
            
        } else {
            // throw some kind of error?
        }
        
    } else if (shoot.rc->rc.s[POWER_SWITCH] == OFF/* or other switch setting*/) {
        // turn off wheels down
        
    }
    
}
