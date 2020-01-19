#include "Start_Task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "test_task.h"
#include "revolver_task.h"
#include "INS_task.h"

#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;

#define TEST_TASK_PRIO 4
#define TEST_STK_SIZE 256
static TaskHandle_t TestTask_Handler;

#define INS_TASK_PRIO 2
#define INS_STK_SIZE 512
static TaskHandle_t INSTask_Handler;

#define REV_TASK_PRIO 10
#define REV_STK_SIZE 256
static TaskHandle_t RevolverTask_Handler;

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

		xTaskCreate((TaskFunction_t) testTask,
								(const char *)"test_task",
								(uint16_t) TEST_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)TEST_TASK_PRIO,
                (TaskHandle_t *)&TestTask_Handler);
	
								
		xTaskCreate((TaskFunction_t) INSTask,
								(const char *)"INS_task",
								(uint16_t) INS_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)INS_TASK_PRIO,
                (TaskHandle_t *)&INSTask_Handler);
				
/*								
		xTaskCreate((TaskFunction_t) revolverTask,
								(const char *)"revolver_task",
								(uint16_t) REV_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)REV_TASK_PRIO,
                (TaskHandle_t *)&RevolverTask_Handler);
								*/

    vTaskDelete(StartTask_Handler); //Delete start task
    taskEXIT_CRITICAL();            //Exit critical
}

void startTask(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //Task function
                (const char *)"start_task",          //Task name
                (uint16_t)START_STK_SIZE,            //Task stack size
                (void *)NULL,                        //Inputs to task functions
                (UBaseType_t)START_TASK_PRIO,        //Task priority
                (TaskHandle_t *)&StartTask_Handler); //Task handler
}
