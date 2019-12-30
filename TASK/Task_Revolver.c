#include "Task_Revolver.h"


#include "can1.h"
#include "led.h"


float revolverFinalOutput = 2000;


void Task_Revolver(void *pvParameters)
{
	portTickType currentTime;	
	Green_Off;
	
	for(;;)
	{	
		currentTime = xTaskGetTickCount();//Current system time

    CAN1_Revolver_Send(revolverFinalOutput);
		Green_On;
		
		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);//
	}
}
