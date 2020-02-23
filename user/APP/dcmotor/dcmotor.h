#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "stm32f4xx.h"

#define PWM_PIN GPIO_Pin_0
#define PWM_PIN_SOURCE GPIO_PinSource0
#define GPIO_CLOCK RCC_AHB1Periph_GPIOI

void init_dc_motor(void);
void dc_motor_set_vel(int vel);
int dc_motor_read_pos(void);

#endif
