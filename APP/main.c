#include "system.h"
#include "Timer_Send_Task.h"
#include "Task_Start.h"
#include "can1.h"
#include "led.h"


int main(void)
{
	System_Init();

  //Initializes timers for CAN
	Timer_Send_Create();
	
    //Start Tasks
	App_Task_Create();
	
	//Task scheduler
	vTaskStartScheduler();

	while(1);
	
}
