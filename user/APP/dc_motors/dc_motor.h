#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "stm32f4xx.h"

#define PWM_PIN GPIO_Pin_8

typedef struct {
    
} dc_motor;

extern dc_motor dcm;

void init_dc_motor();
void dc_motor_set_vel(int vel);
int dc_motor_read_pos();

#endif
