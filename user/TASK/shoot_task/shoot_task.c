/**
  ******************************************************************************
    * @file    TASK/shoot_task
    * @date    24-January/2020
    * @brief   Shooting controls
    * @attention Shoot motors are plugged into pins A8 and E14, 
    *           Controls: 
    *           Right switch (power switch): top is gimbal, bottom is full drive
    *                                       launcher only enabled in those cases
    *           Left switch (shoot switch): mid is shoot_ready with flywheels and trigger motors only
    *                                       up is shoot_rapid with all motors running and firing balls          
    *                                       down is shoot_reverse with trigger and hopper only, rotating backwards
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
static void shoot_ready_control(Shoot_Motor_t *trigger_motor, Shoot_Motor_t *hopper_motor);
static void shoot_reversed_control(Shoot_Motor_t *trigger_motor, Shoot_Motor_t *hopper_motor);
static void shoot_rapid_control(Shoot_Motor_t *trigger_motor, Shoot_Motor_t *hopper_motor);
static void shoot_off_control(Shoot_Motor_t *trigger_motor, Shoot_Motor_t *hopper_motor);
static void set_control_mode(void);
static void get_new_data(void); 

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
        get_new_data();
        set_control_mode();
        //Handle trigger motor
        shoot.hopper_motor.speed_out = shoot.hopper_motor.speed_set;
        shoot.trigger_motor.speed_out = PID_Calc(&trigger_motor_pid, shoot.trigger_motor.speed_raw, shoot.trigger_motor.speed_set);

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
        vTaskDelay(SHOOT_TASK_DELAY);
    }
}


Shoot_t* get_launcher_pointer(void) {
    return &shoot;
}


/******************** Private Implementations ********************/

/**
 * @brief Initializes launcher, links RC pointer and ramps up flywheels
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
    fric2_on(shoot.fric2_pwm);
    
    //Init PID for hopper and trigger motors
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    PID_Init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_MAX_OUT, TRIGGER_MAX_IOUT);
}




/**
 * @brief Determine behaviour by RC. Power switch: up -> hopper spinning, down -> hopper stop
 *       Shoot switch: up -> shoot one, mid -> starts flywheel but hold trigger (ready to shoot), down -> shoot rapidly
 * @param None
 * @retval None
 */
static void set_control_mode(void) {
    //Gets outcome of rc
    if (shoot.rc->rc.s[POWER_SWITCH] == RC_SW_UP) {
        if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_MID) {
            pwm_target = Fric_DOWN;
            // no shoot
            shoot.mode = SHOOT_READY;
        } else if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_DOWN) {
            pwm_target = Fric_OFF;
            // single shot
            shoot.mode = SHOOT_REVERSED;
        } else if (shoot.rc->rc.s[SHOOT_SWITCH] == RC_SW_UP) {
            // rapid fire
            pwm_target = Fric_UP;
            shoot.mode = SHOOT_RAPID;
        }
    } else {
        pwm_target = Fric_OFF;
        shoot.mode = SHOOT_OFF;
    }
    
    //Switches control function
    if (shoot.mode == SHOOT_READY) {
        shoot_ready_control(&shoot.trigger_motor, &shoot.hopper_motor);
    } else if (shoot.mode == SHOOT_REVERSED) {
        shoot_reversed_control(&shoot.trigger_motor, &shoot.hopper_motor);
    } else if (shoot.mode == SHOOT_RAPID) {
        shoot_rapid_control(&shoot.trigger_motor, &shoot.hopper_motor);
    } else {
        shoot_off_control(&shoot.trigger_motor, &shoot.hopper_motor);
    }
}


/**
 * @brief Update data in the shoot struct. Handles the 36:1 gear ratio in p36. 
 *   This function is commented out temporarily because it's not tested and not high priority before MT video
 * @param None
 * @retval None
 */
static void get_new_data(void) {
    /*
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
    if (shoot.trigger_motor.ecd_count == HALF_TRIGGER_RATIO)
    {
        shoot.trigger_motor.ecd_count = -(HALF_TRIGGER_RATIO - 1);
    }
    else if (shoot.trigger_motor.ecd_count == -HALF_TRIGGER_RATIO)
    {
        shoot.trigger_motor.ecd_count = HALF_TRIGGER_RATIO - 1;
    }
    shoot.trigger_motor.geared_down_pos_raw = (shoot.trigger_motor.ecd_count * FULL_ECD_RANGE + shoot.trigger_motor.pos_raw) / FULL_TRIGGER_RATIO;
    */
}

/**
 * @brief Trigger motor spins; Hopper motor stopped
 * @param Trigger motor and hopper motor structs 
 * @retval None
 */
static void shoot_ready_control(Shoot_Motor_t *trigger_motor, Shoot_Motor_t *hopper_motor)
{
    trigger_motor->speed_set = TRIGGER_SPEED;
    hopper_motor->speed_set = HOPPER_OFF;
}


/**
 * @brief Reverses hopper and trigger in an attemp to unjam balls
 * @param Trigger motor and hopper motor structs
 * @retval None
 */
static void shoot_reversed_control(Shoot_Motor_t *trigger_motor, Shoot_Motor_t *hopper_motor)
{
    hopper_motor->speed_set = HOPPER_SPEED * DIR_REVERSED;
    trigger_motor->speed_set = TRIGGER_SPEED * DIR_REVERSED;
}


/**
 * @brief Trigger motor and hopper motor continuously spinning to allow for rapid firing
 * @param Trigger motor and hopper motor structs
 * @retval None
 */
static void shoot_rapid_control(Shoot_Motor_t *trigger_motor, Shoot_Motor_t *hopper_motor)
{
    //Hopper and trigger motor both spinning, used for clearing bullets
    hopper_motor->speed_set = HOPPER_SPEED;
    trigger_motor->speed_set = TRIGGER_SPEED;
}


/**
 * @brief Trigger and hopper motors both set to stop
 * @param Trigger motor and hopper motor structs
 * @retval None
 */
static void shoot_off_control(Shoot_Motor_t *trigger_motor, Shoot_Motor_t *hopper_motor)
{
    trigger_motor->speed_set = TRIGGER_OFF;
    hopper_motor->speed_set = HOPPER_OFF;
}
