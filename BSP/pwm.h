#ifndef _PWM_H
#define _PWM_H

#include "system.h"

#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
	#define PWM1  TIM4->CCR3     //Ħ����,PB8
	#define PWM2  TIM4->CCR4     //Ħ����,PB9
#else
	#define PWM1  TIM3->CCR1     //Ħ����,PA6
	#define PWM2  TIM3->CCR2     //Ħ����,PA7
#endif

void TIM4_Init(void);
void TIM1_Init(void);

void TIM4_FrictionPwmOutp(int16_t pwm1,int16_t pwm2);

#endif
