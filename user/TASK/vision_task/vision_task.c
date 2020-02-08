#include "vision_task.h"
#include "main.h"
#include "stm32f4xx.h"
#include <stdio.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


/******************** User Includes ********************/
#include "CAN_Receive.h"
#include "delay.h"
#include "USART_comms.h"
#include "pid.h"


//The following three things are declared as extern variables because:
//If you declare the same varaible name, the program will not compile.
//I am too lazy to rename everything. 
//But of course you want a new mailbox and a new gimbal "object", at least to test on
//TLDR: rename these variables

/******************** Task/Functions Called Outside ********************/

void vision_task(void *pvParameters){
	  while(1) {
          vTaskDelay(200);
        // todo
      }
}
