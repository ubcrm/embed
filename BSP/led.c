#include "led.h"

void Led_Init(void)
{
	GPIO_InitTypeDef gpiof;	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);

	gpiof.GPIO_PuPd = GPIO_PuPd_DOWN;
	gpiof.GPIO_Speed = GPIO_Speed_50MHz;	
	gpiof.GPIO_Mode = GPIO_Mode_OUT;										
	gpiof.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;  		
	GPIO_Init(GPIOF,&gpiof);	

	Green_Off;
	Red_Off;
	Blue_Off;
	Orange_Off;
}
