#include "Task_Start.h"

#include "main.h"
#include "Timer_Send_Task.h"
#include "Task_Revolver.h"

//Creates tasks
#define START_TASK_PRIO  11  				//任务优先级
TaskHandle_t StartTask_Handler; 			//任务句柄
void Start_Task(void *pvParameters); 		//任务函数

//Revolver
#define TASK_REVOLVER_PRIO 5  				//任务优先级
TaskHandle_t  Task_Revolver_Handler; 		//任务句柄

/***********************************************************/
void App_Task_Create(void)
{
	xTaskCreate((TaskFunction_t )Start_Task, 			//任务函数
				(const char* )"Start_Task", 			//任务名称
				(uint16_t )STK_SIZE_128, 				//任务堆栈大小
				(void* )NULL, 							//传递给任务函数的参数
				(UBaseType_t )START_TASK_PRIO, 			//任务优先级
				(TaskHandle_t* )&StartTask_Handler); 	//任务句柄

}


/**
  * @brief  创建开始任务任务函数
  * @param  void
  * @retval void
  * @attention 控制任务在此创建
  */
void Start_Task(void *pvParameters)
{
		
	taskENTER_CRITICAL(); 	//进入临界区
								 
	/*------------------------------------------------*/
	//Revolver task
	xTaskCreate((TaskFunction_t )Task_Revolver,
				(const char* )"Task_Revolver",
				(uint16_t )STK_SIZE_128,
				(void* )NULL,
				(UBaseType_t )TASK_REVOLVER_PRIO,
				(TaskHandle_t* )&Task_Revolver_Handler);
			

	vTaskDelay(500);
	vTaskSuspend(StartTask_Handler); 					//删除开始任务			

	taskEXIT_CRITICAL(); 								//退出临界区		
}

