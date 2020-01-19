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
 #define DELAY_TIME 1
 uint16_t pwm_setting = 150;
 uint16_t pwm_output = 1000;
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
	 	if(pwm_output < pwm_setting){
	 		pwm_output = pwm_output + 1;
	 	}
	 	if(pwm_output > pwm_setting){
	 		pwm_output = pwm_output - 1;
	 	}
	 	serial_send_string("\n ------ \n");
        serial_send_string("pwm setting: ");
	 	  fric1_on(pwm_output);
          fric2_on(pwm_output);
	 	vTaskDelay(DELAY_TIME); 
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
					for(int index = 0; index < buffer_position - 1; index++){
                        serial_send_string("index: ");
                        serial_send_int(index);
                        serial_send_string("power (ptot - i): ");
                        serial_send_int(power - index);
                        serial_send_string("buffer value:");
                        serial_send_int(buffer[index] - ASCII_ZERO);
                        serial_send_string("pwm setting (interm): ");
                        
						pwm_setting += (buffer[index] - ASCII_ZERO) * pow(10 , power - index);
                        serial_send_int(pwm_setting);
                        buffer[index] = 0;
                      
					}

                    serial_send_int(pwm_setting);
                    serial_send_string("\n ------ \n");
                    buffer_position = 0;
				}
			}
	}
}