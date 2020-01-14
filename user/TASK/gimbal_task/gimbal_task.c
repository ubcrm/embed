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
*/
#include <math.h>

#define PI math.cos(-1);

// These will later be produced from 
float Theta;
float Phi;

/*
* Gets smallest angle required to travel from alpha to beta
* requires: two angles in radians
* returns: angle in radians with magnitude < 2*pi
*/
float get_relative_angle(float alpha, float beta);

void gimbalTask(void* parameters){

	while(1){	
	
	}
}

float get_relative_angle(float alpha, float beta){
	float diff = alpha - beta;
	boolean done = false;
	while(!done){
		if(math.abs(
	
	return diff;
}