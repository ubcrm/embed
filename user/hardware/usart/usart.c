#include "usart.h"
#include "stm32f4xx.h"

void USART_6_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Enable the GPIOG clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	// Enable peripheral clock for USART6
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	// Force USART6 clock to reset
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, DISABLE);

	// Setup GPIO pins for UART

	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);	// Tx
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);		// Rx

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;				// rc USART uses GPIO_PuPd_NOPULL
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	USART_DeInit(USART6);

	// Configure UART - this all needs to match the config on the Pi
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;			// These are the default settings
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					// All set by USART_StructInit()
	USART_InitStructure.USART_Parity = USART_Parity_No;					// These details are here for clarity
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;


	// USART_StructInit(&USART_InitStructure); 	// all the settings above are the default settings, this sets them all
	USART_Init(USART6, &USART_InitStructure);

	// enable interrupts
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

	// enable USART6 peripheral
	USART_Cmd(USART6, ENABLE);

	// configure NVIC for interrupts - only on Rx
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		// set priority group to highest priority achieveable by sw (hw can go negative)
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					// set priority highest in subgroup
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
