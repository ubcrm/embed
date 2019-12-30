#include "Timer_Send_Task.h"


/* 队列 */
TimerHandle_t	CAN1_Timer_Handle; 			//周期定时器句柄					
TimerHandle_t	CAN2_Timer_Handle; 			//周期定时器句柄

/**
  * @brief  CAN1 callback function
  * @param  void
  * @retval void
  * @attention can队列禁用等待，禁用delay
  */
void CAN1_Timer_Callback( TimerHandle_t xTimer )
{
	CanTxMsg    SendCanTxMsg;

    CAN_Transmit(CAN1, &SendCanTxMsg);//发送目标值

}

/**
  * @brief  CAN2 callback function
  * @param  void
  * @retval void
  * @attention can队列禁用等待，禁用delay
  */
void CAN2_Timer_Callback( TimerHandle_t xTimer )
{
	CanTxMsg    SendCanTxMsg;

	CAN_Transmit(CAN2, &SendCanTxMsg);//发送目标值
}

void Timer_Send_Create(void)
{
	taskENTER_CRITICAL(); 	//进入临界区
	
	//Create timer for CAN1
	 CAN1_Timer_Handle=xTimerCreate((const char*	)"CAN1_Timer",
									 (TickType_t 	)TIME_STAMP_2MS,//2ms
									 (UBaseType_t	)pdTRUE, 		//周期执行
									 (void *		)0,				//编号一般给0
									 (TimerCallbackFunction_t)CAN1_Timer_Callback);//回调函数
	
	//Start timer for CAN1
	if( CAN1_Timer_Handle != NULL )
	{
		xTimerStart(CAN1_Timer_Handle,0);
	}
									 
	//Create timer for CAN1
	 CAN2_Timer_Handle=xTimerCreate((const char*	)"CAN2_Timer",
									 (TickType_t 	)TIME_STAMP_1MS,//1ms
									 (UBaseType_t	)pdTRUE, 		//周期执行
									 (void *		)1,				//编号一般给0
									 (TimerCallbackFunction_t)CAN2_Timer_Callback);//回调函数
	
			
									 
	//Start timer for CAN1
	if( CAN2_Timer_Handle != NULL )
	{
		xTimerStart(CAN2_Timer_Handle,0);
	}	
	
	taskEXIT_CRITICAL();
}
