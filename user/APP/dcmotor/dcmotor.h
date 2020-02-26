#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "stm32f4xx.h"

void init_dc_motor(void);
void dc_motor_set_vel(int vel);
int dc_motor_read_pos(void);

#endif
