#include "Timer_Send_Task.h"


/* ���� */
TimerHandle_t	CAN1_Timer_Handle; 			//���ڶ�ʱ�����					
TimerHandle_t	CAN2_Timer_Handle; 			//���ڶ�ʱ�����

/**
  * @brief  CAN1 callback function
  * @param  void
  * @retval void
  * @attention can���н��õȴ�������delay
  */
void CAN1_Timer_Callback( TimerHandle_t xTimer )
{
	CanTxMsg    SendCanTxMsg;

    CAN_Transmit(CAN1, &SendCanTxMsg);//����Ŀ��ֵ

}

/**
  * @brief  CAN2 callback function
  * @param  void
  * @retval void
  * @attention can���н��õȴ�������delay
  */
void CAN2_Timer_Callback( TimerHandle_t xTimer )
{
	CanTxMsg    SendCanTxMsg;

	CAN_Transmit(CAN2, &SendCanTxMsg);//����Ŀ��ֵ
}

void Timer_Send_Create(void)
{
	taskENTER_CRITICAL(); 	//�����ٽ���
	
	//Create timer for CAN1
	 CAN1_Timer_Handle=xTimerCreate((const char*	)"CAN1_Timer",
									 (TickType_t 	)TIME_STAMP_2MS,//2ms
									 (UBaseType_t	)pdTRUE, 		//����ִ��
									 (void *		)0,				//���һ���0
									 (TimerCallbackFunction_t)CAN1_Timer_Callback);//�ص�����
	
	//Start timer for CAN1
	if( CAN1_Timer_Handle != NULL )
	{
		xTimerStart(CAN1_Timer_Handle,0);
	}
									 
	//Create timer for CAN1
	 CAN2_Timer_Handle=xTimerCreate((const char*	)"CAN2_Timer",
									 (TickType_t 	)TIME_STAMP_1MS,//1ms
									 (UBaseType_t	)pdTRUE, 		//����ִ��
									 (void *		)1,				//���һ���0
									 (TimerCallbackFunction_t)CAN2_Timer_Callback);//�ص�����
	
			
									 
	//Start timer for CAN1
	if( CAN2_Timer_Handle != NULL )
	{
		xTimerStart(CAN2_Timer_Handle,0);
	}	
	
	taskEXIT_CRITICAL();
}
