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


typedef struct 
{
	Gimbal_Motor_t *gimbal_motor;
	const fp32 *gyro_reading_raw;
	const fp32 *acce_reading_raw;
} Gimbal_t;

void testTask(void *pvParameters);
	
#endif
