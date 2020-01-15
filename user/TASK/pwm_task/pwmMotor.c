/**
 * This task runs the pwm motors in relation to their desired setting
 */
 
 #include "FreeRTOS.h"
 #include "pwmMotor.h"
 #include "fric.h"
 #include "task.h"
 #include <math.h>
 #include "stm32f4xx.h"
 
 /*
  * The delay time for the control loop in ms. 
	* Can be modified to 1 ms to better refect DJI
	*/
 #define PWM_CONTROL_TIME 10
 #define BUFFER_LENGTH 10
 
 uint16_t pwm_setting = 0;
 uint16_t buffer[BUFFER_LENGTH];
 uint16_t buffer_position = 0;
 
 /**
  * Controls the operation of the PWM motors.
	* Takes commands from a uart in channel
	* Spins the friction motors to the desired speed.
	*/
 void PWMMotorTask(void *pvparameters){
	 fric_off();
	 vTaskDelay(PWM_CONTROL_TIME);
	 /*
	 From DJI Code: The friction wheels need to be turned on one by one, 
	 and cannot be turned on at the same time, 
	 otherwise the motor may not turn
	 */
	 while(1){
		 fric1_on(pwm_setting);
		 vTaskDelay(PWM_CONTROL_TIME);
	 }
 }
 
 /* From Jaden */ /*
extern void USART_puts(USART_TypeDef *USARTx, volatile char *str);
extern volatile uint8_t USART_Data;
extern volatile int Data_received;

// interrupt request handler for all USART6 interrupts
// is called every time 1 byte is received
void USART6_IRQHandler(void)
{
	// make sure USART6 was intended to be called for this interrupt
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
		USART_Data = USART_ReceiveData(USART6);
		Data_received = 1;

	}
}*/
 

 void USART6_IRQHandler(void){
	 if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET){		 
			buffer[buffer_position++] = USART_ReceiveData(USART6);	
		  if(buffer_position > 0){
				
				/* 13 is the enter key on Windows*/ 
				if(buffer[buffer_position-1] == 13){
					pwm_setting = 0;
					int power = buffer_position - 2;
					for(int index = 0; index < BUFFER_LENGTH; index++){
						pwm_setting += pow(buffer[index], power - index);
					}
				}
			}
	}
}