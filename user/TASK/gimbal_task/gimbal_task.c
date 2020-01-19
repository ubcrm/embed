/**
* Gimbal task 
* Converts user input to Gimbal motor setpoints
*
* Gimbal position is defined by angles phi and theta
* Theta increases counterclockwise if viewed from the top and is 0 when facing fwd.
* Phi increases counterclockwise if viewed from the right side and is 0 when horizontal
* (when Theta is 0)
*
* Gimbal control strategy and either be gyro based or encoder based 
* RM motors give encoders wrap around at a half rotation negative and positive
* Encoders results are between 0 and 8191
*/

#include "math.h"
#include "gimbal_task.h"
#include "stm32f4xx.h"

// These will later be produced from RC
/* angle in radians*/
float theta;
float phi;

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
  * @brief      Handles cases when RC joystick is not quite centered
  * @author     RM
  * @param[in]  input, RC joystick value
  * @param[in]  output, control value after deadline is applied
  * @param[in]  deadline, specifies range in which joystick is considered centered
  */
#define rc_deadline_limit(input, output, deadline) \
	{                                              \
		if (input > deadline || input < -deadline) \
            output = input;                        \
        else                                       \
            output = 0;                            \
    }


void gimbalTask(void* parameters){

	
	while(1){	
		// Gets update on position from encoders and gyro
		// Comparison to setpoint
		// run PID
	}
}

float get_domain_angle(float alpha){
	int done = 0;
	while(!done){
		if(alpha > PI){
			alpha -= PI;
		}
		else if(alpha < -PI){
			alpha += PI;\
		}
		else{
			done = TRUE;
		}
	}
}


float get_relative_angle(float alpha, float beta){
	return get_domain_angle(beta - alpha);
}
	