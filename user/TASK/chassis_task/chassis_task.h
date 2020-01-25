#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "CAN_receive.h"
#include "main.h"

typedef struct 
{
		const motor_measure_t *gimbal_motor_raw;
		uint16_t pos_raw;
		uint16_t speed_raw;
		uint16_t current_raw;
} Chassis_Motor_t;

extern void chassis_task(void *pvParameters);
#endif
	
