#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "stm32f4xx.h"

//All of the pin/port specific values for PWM going to the dc motor
#define DC_MOTOR_TIM TIM4
#define DC_MOTOR_PERIPH RCC_APB1Periph_TIM4
#define RCC_PERIF RCC_AHB1Periph_GPIOD
#define DC_MOTOR_PIN GPIO_Pin_12
#define DC_GPIO_PORT GPIOD
#define DC_PIN_SOURCE GPIO_PinSource12
#define DC_ALT_FUNC GPIO_AF_TIM4

//All of the pin/port specific values for the direction signal for the motor controller
#define RCC_PORT_1 RCC_AHB1Periph_GPIOF
#define RCC_PORT_2 RCC_AHB1Periph_GPIOI
#define DIR_GPIO_PIN_1 GPIO_Pin_10
#define DIR_GPIO_PORT_1 GPIOF
#define DIR_GPIO_PIN_2 GPIO_Pin_9
#define DIR_GPIO_PORT_2 GPIOI

void init_dc_motor(void);
void dc_motor_set_vel(int vel);
int dc_motor_read_pos(void);

#endif
