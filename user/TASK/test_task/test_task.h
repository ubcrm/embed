#ifndef USER_TASK_H
#define USER_TASK_H

#include "CAN_receive.h"
#include "main.h"


typedef struct 
{
	const motor_measure_t *gimbal_motor_raw;
	uint16_t pos_raw;
	uint16_t speed_raw;
	uint16_t current_raw;
} Gimbal_Motor_t;


typedef struct 
{
	Gimbal_Motor_t *yaw_motor;
	Gimbal_Motor_t *pitch_motor;
	const fp32 *gyro_reading_raw;
	const fp32 *acce_reading_raw;
} Gimbal_t;

void testTask(void *pvParameters);
	
#endif
