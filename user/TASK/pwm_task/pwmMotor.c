/**
 * This task runs the pwm motors in relation to their desired setting
 */
 
 #include "FreeRTOS.h"
 #include "pwmMotor.h"
 #include "fric.h"
 #include "task.h"
 #include <math.h>
 #include "stm32f4xx.h"
 #include "USART_comms.h"
 
 /*
  * The delay time for the control loop in ms. 
	* Can be modified to 1 ms to better refect DJI
	*/
 #define PWM_CONTROL_TIME 1
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
		 vTaskDelay(PWM_CONTROL_TIME*10);
         
	 }
 }

/*
 * A debug handler used can be used to get a PWM signal directly.
 * Configured to interface with Putty on a windows system. 
 * Enter a number, x (0-1999), and press enter. 
 * This will output a PWM signal with duty cycle x/2000  
 */
void USART6_IRQHandler(void){
	if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET){		 
        buffer[buffer_position++] = USART_ReceiveData(USART6);	
            if(buffer_position > 0){
                /* 13 is the enter key on Windows*/ 
                if(buffer[buffer_position-1] == 13){
                    pwm_setting = 0;
                    int power = buffer_position - 2;
                    for(int index = 0; index < buffer_position - 1; index++){
						pwm_setting += (buffer[index] - ASCII_ZERO) * pow(10 , power - index);
                        buffer[index] = 0;
                        fric1_on(pwm_setting);
                        fric2_on(pwm_setting);
					}
                    serial_send_string("\n ------ \n");
                    serial_send_string("pwm setting: ");
                    serial_send_int(pwm_setting);
                    serial_send_string("\n ------ \n");
                    buffer_position = 0;
				}
			}
	}
}