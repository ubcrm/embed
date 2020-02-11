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
#include "start_task.h"

/******************** User Includes ********************/

#include "remote_control.h"
#include "CAN_Receive.h"
#include "user_lib.h"
#include "fric.h"
#include "USART_comms.h"
#include "PID.h"

Shoot_t shoot;

static void shoot_init(Shoot_t *shoot_init);
static void shoot_control_loop(void);
static void shoot_ready_control(Shoot_Motor_t *trigger_motor);
static void shoot_single_control(Shoot_Motor_t *trigger_motor);
static void shoot_rapid_control(Shoot_Motor_t *trigger_motor);
static void shoot_off_control(Shoot_Motor_t *trigger_motor);
static void shoot_set_mode(void);
static void shoot_feedback_update(void); 

// user defines
static uint16_t pwm_target = Fric_OFF;
static uint16_t pwm_output = Fric_OFF;

static PidTypeDef trigger_motor_pid;   

/******************** Task/Functions Called from Outside ********************/

/**
 * @brief Main shoot control task
 * @param None
 * @retval None
 */
void shoot_task(void *pvParameters) {
    shoot_init(&shoot);
    vTaskDelay(SHOOT_INIT_DELAY);
    while(1) {
        shoot_control_loop();
        vTaskDelay(SHOOT_TASK_DELAY);
    }
}


/******************** Private Implementations ********************/

/**
 * @brief Initializes launcher, links RC ointer and ramps up flywheels
 * @param Shoot_t struct
 * @retval None
 */
static void shoot_init(Shoot_t *shoot_init) {
    // Get RC pointers
    shoot_init->rc = get_remote_control_point();

    // Deal with weird starting of the motors
    shoot_init->fric1_pwm = Fric_INIT;
    shoot_init->fric2_pwm = Fric_INIT;
    
    fric1_on(shoot.fric1_pwm);
    
    //Init PID for hopper and trigger motors
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    PID_Init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
}


/**
 * @brief Main launcher control loop. Power switch: up -> hopper spinning, down -> hopper stop
 *       Shoot switch: up -> shoot one, mid -> starts flywheel but hold trigger (ready to shoot), down -> shoot rapidly
 * @param None
 * @retval None
 */
static void shoot_control_loop(void) {   
    shoot_feedback_update();
    

    shoot_set_mode();

    //Ramping...
    if (pwm_output < pwm_target) {
        pwm_output += 1;
    } else if (pwm_output > pwm_target) {
        pwm_output -= 1;
    }
    
    //Set pwm field to the ramp results
    shoot.fric1_pwm = pwm_output;
    shoot.fric2_pwm = pwm_output;
    
    //Set flywheels
    fric1_on(shoot.fric1_pwm);
    fric2_on(shoot.fric2_pwm);    
}


static void shoot_set_mode(void) {
    //Gets outcome of rc
    if (shoot.rc->rc.s[POWER_SWITCH] == ON) {
        if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_MID) {
            pwm_target = Fric_DOWN;
            // no shoot
            shoot.mode = SHOOT_READY;
        } else if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_DOWN) {
            pwm_target = Fric_UP;
            // single shot
            shoot.mode = SHOOT_SINGLE;
        } else if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_UP) {
            // rapid fire
            shoot.mode = SHOOT_RAPID;
        }
    } else {
        pwm_target = Fric_OFF;
        shoot.mode = SHOOT_OFF;
    }
    
    //Switches control function
    if (shoot.mode == SHOOT_READY) {
        shoot_ready_control(&shoot.hopper_motor);
    } else if (shoot.mode == SHOOT_SINGLE) {
        shoot_single_control(&shoot.hopper_motor);
    } else if (shoot.mode == SHOOT_RAPID) {
        shoot_rapid_control(&shoot.hopper_motor);
    } else {
        shoot_off_control(&shoot.hopper_motor);
    }
}


static void shoot_feedback_update(void) {
    
    //Accounts for gear box inside P19 
    if (shoot.trigger_motor.pos_raw - shoot.trigger_motor.last_pos_raw > HALF_ECD_RANGE)
    {
        //Encoder reading went from 0 to 8192, finished one circle counterclockwise
        shoot.trigger_motor.ecd_count--;
    }
    else if (shoot.trigger_motor.pos_raw - shoot.trigger_motor.last_pos_raw < -HALF_ECD_RANGE)
    {
        //Encoder reading went from 8192 to 0, finished one circle clockwise
        shoot.trigger_motor.ecd_count++;
    }
    //Gear ratio of 36:1
    if (shoot.trigger_motor.ecd_count == TRIGGER_RATIO)
    {
        shoot.trigger_motor.ecd_count = -(TRIGGER_RATIO - 1);
    }
    else if (shoot.trigger_motor.ecd_count == -TRIGGER_RATIO)
    {
        shoot.trigger_motor.ecd_count = TRIGGER_RATIO - 1;
    }
    
}

/**
 * @brief Trigger motor locks in place while motors ramp up to speed
 * @param trigger motor struct with its current and target position and speed 
 * @retval None
 */
static void shoot_ready_control(Shoot_Motor_t *trigger_motor)
{
    //Position control PID here for the trigger motor to force it in place
    //Takes pos_raw, result goes in pos_set and speed_set
    shoot.hopper_motor.speed_set = HOPPER_SPEED;
}


/**
 * @brief Trigger motor moves 90 degrees to allow one ball into the launcher
 * @param trigger motor struct with its current and target position and speed 
 * @retval None
 */
static void shoot_single_control(Shoot_Motor_t *trigger_motor)
{
    //use the #define TRIGGER_90_DEGS 2048 which is untested (change after)
    //Position control PID here for the trigger motor to move exactly 90 degrees
    //Takes pos_raw, result goes in pos_set and speed_set
}


/**
 * @brief Trigger motor continues spinning to allow for rapid firing
 * @attention Integrate with referee system in the future 
 * @param trigger motor struct with its current and target position and speed 
 * @retval None
 */
static void shoot_rapid_control(Shoot_Motor_t *trigger_motor)
{
    //Speed control PID here for the trigger motor to force it in place
    //Takes speed_raw, result goes in speed_set
}


static void shoot_off_control(Shoot_Motor_t *trigger_motor)
{
    shoot.hopper_motor.speed_set = HOPPER_OFF;
}
