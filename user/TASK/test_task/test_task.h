#ifndef USER_TASK_H
#define USER_TASK_H

#include "CAN_receive.h"
#include "main.h"


typedef struct 
{
	const motor_measure_t *gimbal_motor_raw;
	uint16_t gimbal_pos_raw;
	uint16_t gimbal_speed_raw;
	uint16_t gimbal_tq_current_raw;
} Gimbal_Motor_t;

void testTask(void *pvParameters);
	
#endif
