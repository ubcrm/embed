
#include "system.h"
#include "can1.h"
#include "led.h"


int main(void)
{
	System_Init();

	float* PID_chassis;
	PID_chassis[0] = 10 * 256;
	PID_chassis[1] = 0;
	PID_chassis[2] = 0;
	PID_chassis[3] = 0;
	

	while(1){
		CAN1_Chassis_Send(PID_chassis);
		delay_ms(20);
	}
}
