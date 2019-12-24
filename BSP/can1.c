#include "can1.h"

#include "led.h"

extern QueueHandle_t CAN1_Queue;					//CAN1��Ϣ���о��

/**
  * @brief  ���̵������̨��ʼ��
  * @param  void
  * @retval void
  * @attention 201~204��Ӧ����,205,206��Ӧ��̨
  */
void CAN1_Init(void)
{	
	GPIO_InitTypeDef gpio_str;
	CAN_InitTypeDef can_str;
	CAN_FilterInitTypeDef can_fil_str;
	NVIC_InitTypeDef  NVIC_InitStructure;//�����ж�

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	/**********************************************************/
	gpio_str.GPIO_Mode=GPIO_Mode_AF;
	gpio_str.GPIO_OType=GPIO_OType_PP;
	gpio_str.GPIO_Pin=GPIO_Pin_11| GPIO_Pin_12;
	gpio_str.GPIO_PuPd=GPIO_PuPd_UP;
	gpio_str.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio_str);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
	/**********************************************************/
	
	can_str.CAN_ABOM=DISABLE;
	can_str.CAN_AWUM=DISABLE;
	can_str.CAN_BS1=CAN_BS1_9tq;//Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	can_str.CAN_BS2=CAN_BS2_4tq;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
	can_str.CAN_Mode=CAN_Mode_Normal;
	can_str.CAN_NART=ENABLE;//�������Զ����� 
	can_str.CAN_Prescaler=3;//��Ƶϵ��3
	can_str.CAN_RFLM=DISABLE;//���Ĳ�����,�µĸ��Ǿɵ� 
	can_str.CAN_SJW=CAN_SJW_1tq;//����ͬ����Ծ���
	can_str.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ 
	can_str.CAN_TXFP=DISABLE;
	CAN_Init(CAN1,&can_str);
	
	can_fil_str.CAN_FilterActivation=ENABLE;//���������0
	can_fil_str.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	can_fil_str.CAN_FilterIdHigh=0x0000;//ID���
	can_fil_str.CAN_FilterIdLow=0x0000;
	can_fil_str.CAN_FilterMaskIdHigh    =   0x0000;
    can_fil_str.CAN_FilterMaskIdLow     =   0x0000;
    can_fil_str.CAN_FilterMode          =   CAN_FilterMode_IdMask;
    can_fil_str.CAN_FilterNumber        =   0;//������0
    can_fil_str.CAN_FilterScale         =   CAN_FilterScale_32bit;//32λ 
    CAN_FilterInit(&can_fil_str);
	/******************************************************************/

	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����,ÿ����һ�ν�һ���ж�	    
  
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;//
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/**
  * @brief  CAN1�����ж�
  * @param  void
  * @retval void
  */
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	int16_t speed_measure,rota_measure,current_measure;
	Blue_Off;
  CAN_Receive(CAN1, 0, &RxMessage);
}


/**
  * @brief  ����
  * @param  
  * @retval void
  */
void CAN1_Chassis_Send(float *PID_chassis)
{
	CanTxMsg can_msg;

	can_msg.StdId=0x200;	 // ��׼��ʶ��
    can_msg.IDE=CAN_ID_STD;		  
    can_msg.RTR=CAN_RTR_DATA;		  // ��Ϣ����Ϊ����֡��һ֡8λ
    can_msg.DLC=8;							 // ����8֡��Ϣ
	
	can_msg.Data[0]=(u8)((int16_t)PID_chassis[0]>>8);
	can_msg.Data[1]=(u8)((int16_t)PID_chassis[0]);
	can_msg.Data[2]=(u8)((int16_t)PID_chassis[1]>>8);
	can_msg.Data[3]=(u8)((int16_t)PID_chassis[1]);
	can_msg.Data[4]=(u8)((int16_t)PID_chassis[2]>>8);
	can_msg.Data[5]=(u8)((int16_t)PID_chassis[2]);
	can_msg.Data[6]=(u8)((int16_t)PID_chassis[3]>>8);
	can_msg.Data[7]=(u8)((int16_t)PID_chassis[3]);  
	
	CAN_Transmit(CAN1, &can_msg);
}


/**
  * @brief  ��̨
  * @param  
  * @retval void
  */
void CAN1_Cloud_Send(float *PID_6623)
{
	CanTxMsg can_msg;
	
	can_msg.StdId=0x1FF;	 // ��׼��ʶ��Ϊ0x1FF
    can_msg.IDE=CAN_ID_STD;		  
    can_msg.RTR=CAN_RTR_DATA;		  // ��Ϣ����Ϊ����֡��һ֡8λ
    can_msg.DLC=8;							 // ����8֡��Ϣ
	
	can_msg.Data[0]=(u8)((int16_t)PID_6623[0]>>8);//yaw
	can_msg.Data[1]=(u8)((int16_t)PID_6623[0]);
	can_msg.Data[2]=(u8)((int16_t)PID_6623[1]>>8);//pitch
	can_msg.Data[3]=(u8)((int16_t)PID_6623[1]);
	can_msg.Data[4]=0;//roll
	can_msg.Data[5]=0;
	can_msg.Data[6]=0;
	can_msg.Data[7]=0;        
	
	CAN_Transmit(CAN1, &can_msg);
}

