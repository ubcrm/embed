#include "start_task.h"
#include "main.h"
#include "stm32f4xx.h"
#include <stdio.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "revolver_task.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "test_task.h"
#include "vision_task.h"


#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t start_task_handler;

#define INS_TASK_PRIO 2
#define INS_STK_SIZE 512
static TaskHandle_t chassis_task_handler;

#define CHASSIS_TASK_PRIO 3
#define CHASSIS_STK_SIZE 512
static TaskHandle_t INS_task_handler;

#define GIMBAL_TASK_PRIO 4
#define GIMBAL_STK_SIZE 512
static TaskHandle_t gimbal_task_handler;

#define TEST_TASK_PRIO 5
#define TEST_STK_SIZE 512
static TaskHandle_t test_task_handler;

#define VISION_TASK_PRIO 6
#define VISION_STK_SIZE 512
static TaskHandle_t vision_task_handler;

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

    xTaskCreate((TaskFunction_t) INS_task,
            (const char *)"INS_task",
            (uint16_t) INS_STK_SIZE,
            (void *)NULL,
            (UBaseType_t)INS_TASK_PRIO,
            (TaskHandle_t *)&INS_task_handler);
			
    xTaskCreate((TaskFunction_t) chassis_task,
            (const char *)"chassis_task",
            (uint16_t) CHASSIS_STK_SIZE,
            (void *)NULL,
            (UBaseType_t)CHASSIS_TASK_PRIO,
            (TaskHandle_t *)&chassis_task_handler);
						
    xTaskCreate((TaskFunction_t) gimbal_task,
            (const char *)"gimbal_task",
            (uint16_t) GIMBAL_STK_SIZE,
            (void *)NULL,
            (UBaseType_t)GIMBAL_TASK_PRIO,
            (TaskHandle_t *)&gimbal_task_handler);

    xTaskCreate((TaskFunction_t) test_task,
            (const char *)"test_task",
            (uint16_t) TEST_STK_SIZE,
            (void *)NULL,
            (UBaseType_t)TEST_TASK_PRIO,
            (TaskHandle_t *)&test_task_handler);
            
    xTaskCreate((TaskFunction_t) vision_task,
            (const char *)"vision_task",
            (uint16_t) VISION_STK_SIZE,
            (void *)NULL,
            (UBaseType_t)VISION_TASK_PRIO,
            (TaskHandle_t *)&vision_task_handler);

    vTaskDelete(start_task_handler); //Delete start task
    taskEXIT_CRITICAL();            //Exit critical
}

void startTask(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //Task function
                (const char *)"start_task",          //Task name
                (uint16_t)START_STK_SIZE,            //Task stack size
                (void *)NULL,                        //Inputs to task functions
                (UBaseType_t)START_TASK_PRIO,        //Task priority
                (TaskHandle_t *)&start_task_handler); //Task handler
}
