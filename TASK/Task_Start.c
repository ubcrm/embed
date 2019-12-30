#include "Task_Start.h"

#include "main.h"
#include "Timer_Send_Task.h"
#include "Task_Revolver.h"

//Creates tasks
#define START_TASK_PRIO  11  				//�������ȼ�
TaskHandle_t StartTask_Handler; 			//������
void Start_Task(void *pvParameters); 		//������

//Revolver
#define TASK_REVOLVER_PRIO 5  				//�������ȼ�
TaskHandle_t  Task_Revolver_Handler; 		//������

/***********************************************************/
void App_Task_Create(void)
{
	xTaskCreate((TaskFunction_t )Start_Task, 			//������
				(const char* )"Start_Task", 			//��������
				(uint16_t )STK_SIZE_128, 				//�����ջ��С
				(void* )NULL, 							//���ݸ��������Ĳ���
				(UBaseType_t )START_TASK_PRIO, 			//�������ȼ�
				(TaskHandle_t* )&StartTask_Handler); 	//������

}


/**
  * @brief  ������ʼ����������
  * @param  void
  * @retval void
  * @attention ���������ڴ˴���
  */
void Start_Task(void *pvParameters)
{
		
	taskENTER_CRITICAL(); 	//�����ٽ���
								 
	/*------------------------------------------------*/
	//Revolver task
	xTaskCreate((TaskFunction_t )Task_Revolver,
				(const char* )"Task_Revolver",
				(uint16_t )STK_SIZE_128,
				(void* )NULL,
				(UBaseType_t )TASK_REVOLVER_PRIO,
				(TaskHandle_t* )&Task_Revolver_Handler);
			

	vTaskDelay(500);
	vTaskSuspend(StartTask_Handler); 					//ɾ����ʼ����			

	taskEXIT_CRITICAL(); 								//�˳��ٽ���		
}

