#include "dcmotor.h"
#include "stm32f4xx.h"

void setup_pwm(void);
void setup_gpio(void);
void direction(int speed);

void init_dc_motor(void){
    setup_pwm();
    setup_gpio(); //not ready yet
}

void dc_motor_set_vel(int vel){
    direction(vel);
    TIM_SetCompare1(TIM4, vel); //TODO not TIM4 necessarly
}

int dc_motor_read_pos(void){
    return 0;
}

/*
Wiring Connections:
    Servo 1:
        Vcc     -> 5V
        Gnd     -> Gnd
        Signal  -> PB6 (TIM4 Ch1)
    Servo 2:
        Vcc     -> 5V
        Gnd     -> Gnd
        Signal  -> PB7 (TIM4 Ch2)
 
 
 NOTES:
    TIM4 runs on APB1 which clocks at 42MHz. If Prescaler is set to 1, then clock remains 42MHz
        Otherwise, clock is multiplied by 2 (i.e. 84MHz operation) due to PLL, See reference manual for description. This is why we prescale by 84.
*/
//Private functions for hardware
void setup_pwm(void){ //TODO change port to PD 12 port H
    // Structures for configuration
    GPIO_InitTypeDef            GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    TIM_OCInitTypeDef           TIM_OCInitStructure;
    
    // TIM4 Clock Enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    // GPIOD Clock Enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
    // Initalize PB6 (TIM4 Ch1) and PB7 (TIM4 Ch2)
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_12; // | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;    // GPIO_High_Speed
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;         // Weak Pull-up for safety during startup
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // Assign Alternate Functions to pins
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    //GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
    
    /* Setup TIM / PWM values
     Servo Requirements:  (May be different for your servo)
        - 50Hz (== 20ms) PWM signal
        - 0.6 - 2.1 ms Duty Cycle
     
     1. Determine Required Timer_Freq.
            TIM_Period = (Timer_Freq. / PWM_Freq) - 1
     
            - We need a period of 20ms (or 20000탎) and our PWM_Freq = 50Hz (i.e. 1/20ms)
            - See NOTES, for why we use 탎
            TIM_Period = 20000 - 1 = 19999  (since its 0 offset)
     
            Timer_Freq = (TIM_Period + 1) * PWM_Freq.
            Timer_Freq = (19999 + 1) * 50
            Timer_Freq = 1000000 = 1MHz
     
     2. Determine Pre-Scaler
        APB1 clock frequency:
            - SYS_CLK/4 when prescaler == 1 (i.e. 168MHz / 4 = 42MHz)
            - SYS_CLK/2 when prescaler != 1 (i.e. 168MHz / 2 = 84MHz)
     
        Prescaler = APB1_Freq / Timer_Freq
        Prescaler = 84 MHz / 1 MHz
        Prescaler = 84
     
        For our example, we can prescale the TIM clock by 84, which gives us a Timer_Freq of 1MHz
            Timer_Freq = 84 MHz / 84 = 1 MHz
        So the TIMx_CNT register will increase by 1 000 000 ticks every second. When TIMx_CNT is increased by 1 that is 1 탎. So if we want a duty cycle of 1.5ms (1500 탎) then we can set our CCRx register to 1500.
     
     NOTES:
        - TIMx_CNT Register is 16 bits, i.e. we can count from 0 to (2^16)-1 = 65535
        - If the period, TIMx_ARR, is greater than the max TIMx_CNT value (65535), then we need to choose a larger prescaler value in order to slow down the count.
        - We use the 탎 for a more precise adjustment of the duty cycle
     
     */
    uint16_t PrescalerValue = (uint16_t) 84;

    // Time Base Configuration
    TIM_TimeBaseStructure.TIM_Period        = 19999;
    TIM_TimeBaseStructure.TIM_Prescaler     = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    // Common TIM Settings
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;                        // Initial duty cycle
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    
    // Channel 1
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
    // Channel 2
    //TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    //TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    
    // Start timer
    TIM_Cmd(TIM4, ENABLE);
}

void setup_gpio(void){ //TODO change port to PF10 and PI9 (Q1 and Q2)
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOI, ENABLE); //

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOI, &GPIO_InitStructure);
}

void direction(int speed){
    if(speed >= 0){
        GPIO_SetBits(GPIOF, GPIO_Pin_10);
        GPIO_ResetBits(GPIOI, GPIO_Pin_9);
    }else{
        GPIO_SetBits(GPIOI, GPIO_Pin_9);
        GPIO_ResetBits(GPIOF, GPIO_Pin_10);
    }
}

//GPIO_SetBits(GPIOF, GPIO_Pin_14); //LED off
//GPIO_ResetBits(GPIOF, GPIO_Pin_14); //LED on
