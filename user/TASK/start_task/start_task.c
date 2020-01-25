#include "Start_Task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "test_task.h"
#include "revolver_task.h"
#include "INS_task.h"
#include "chassis_task.h"


#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t start_task_handler;

#define TEST_TASK_PRIO 4
#define TEST_STK_SIZE 256
static TaskHandle_t test_task_handler;

#define INS_TASK_PRIO 2
#define INS_STK_SIZE 512
static TaskHandle_t chassis_task_handler;

#define CHASSIS_TASK_PRIO 2
#define CHASSIS_STK_SIZE 512
static TaskHandle_t INS_task_handler;

#define REV_TASK_PRIO 10
#define REV_STK_SIZE 256
static TaskHandle_t revolver_task_handler;

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

    xTaskCreate((TaskFunction_t) test_task,
            (const char *)"test_task",
            (uint16_t) TEST_STK_SIZE,
            (void *)NULL,
            (UBaseType_t)TEST_TASK_PRIO,
            (TaskHandle_t *)&test_task_handler);

                            
    xTaskCreate((TaskFunction_t) INS_task,
            (const char *)"INS_task",
            (uint16_t) INS_STK_SIZE,
            (void *)NULL,
            (UBaseType_t)INS_TASK_PRIO,
            (TaskHandle_t *)&INS_task_handler);
                            
    xTaskCreate((TaskFunction_t) chassis_task,
            (const char *)"INS_task",
            (uint16_t) CHASSIS_STK_SIZE,
            (void *)NULL,
            (UBaseType_t)CHASSIS_TASK_PRIO,
            (TaskHandle_t *)&chassis_task_handler);
            
/*								
    xTaskCreate((TaskFunction_t) revolverTask,
            (const char *)"revolver_task",
            (uint16_t) REV_STK_SIZE,
            (void *)NULL,
            (UBaseType_t)REV_TASK_PRIO,
            (TaskHandle_t *)&revolver_task_handler);
*/

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
