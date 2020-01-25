#ifndef USER_TASK_H
#define USER_TASK_H

#include "CAN_receive.h"
#include "main.h"
#include "pid.h"

typedef struct 
{
    const motor_measure_t *gimbal_motor_raw;
    int16_t gimbal_pos_raw;
    int16_t gimbal_speed_raw;
    int16_t gimbal_tq_current_raw;
	
} Gimbal_Motor_t;

void testTask(void *pvParameters);
void send_to_uart(Gimbal_Motor_t gimbal_yaw_motor, PidTypeDef pi, fp32 pitch_signal);
int get_vision_signal(void);

typedef struct 
{
	Gimbal_Motor_t *yaw_motor;
	Gimbal_Motor_t *pitch_motor;
	const fp32 *gyro_reading_raw;
	const fp32 *acce_reading_raw;
} Gimbal_t;

void test_task(void *pvParameters);

	
#endif
