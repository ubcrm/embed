/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32初始化以及开始任务freeRTOS。h文件定义相关全局宏定义以及
  *             typedef 一些常用数据类型
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "main.h"

#include "stm32f4xx.h"

#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "delay.h"
#include "flash.h"
#include "fric.h"
#include "laser.h"

#include "led.h"
#include "power_ctrl.h"
#include "rc.h"
#include "rng.h"
#include "sys.h"
#include "timer.h"
#include "usart.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mpu6500driver.h"

#include "start_task.h"

void BSP_init(void);

int main(void)
{
    BSP_init();
    delay_ms(100);
    startTask();
    vTaskStartScheduler();
    while (1)
    {
        ;
    }
}

//24V power output delay time (microseconds)
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
    //Initialize interrupt group 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //Clock init
    delay_init(configTICK_RATE_HZ);
    //LEDs
    led_configuration();
    //stm32 onboard temperature sensor
    temperature_ADC_init();
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    //stm32 random number generator
    RNG_init();
#endif
    //24V power output
    power_ctrl_configuration();
    //flywheel pwm init
    fric_PWM_configuration();
    //buzzer (Alice hates this so it's commented out)
    //buzzer_init(30000, 90);
    //laser init
    laser_configuration();
    //timer 6 init
    TIM6_Init(60000, 90);
    //CAN peripherals init
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
		//IMU init
		mpu6500_init();
		
		USART_6_INIT();

    //24v power output on
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
		
}

