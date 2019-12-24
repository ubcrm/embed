#include "Info_Transmitter.h"

#include "control.h"
#include "judge.h"
#include "usart5.h"
#include "vision.h"

void Info_Transmitter_Task(void *pvParameters)
{
	for(;;)
	{
		vTaskDelay(TIME_STAMP_100MS);				//100ms
		
		JUDGE_Show_Data();//�û������ϴ�,10Hz�ٷ������ٶ�
		Vision_Ctrl();//�Ӿ�,ָ�����
	}
}

void Info_JudgeRead_Task(void *pvParameters)
{
	for(;;)
	{
		vTaskDelay(TIME_STAMP_2MS);			//2ms
		
		Judge_Read_Data(Judge_Buffer);		//��ȡ����ϵͳ����	
	}
}
