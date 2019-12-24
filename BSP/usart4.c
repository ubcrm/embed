#include "usart4.h"

#include "led.h"


//�Ӿ�����4����,�Ͳ���ϵͳһ��

/* TX */
#define    GPIO_TX                   GPIOA
#define    GPIO_PIN_TX               GPIO_Pin_0
#define    GPIO_PINSOURCE_TX         GPIO_PinSource0
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOA

/* RX */
#define    GPIO_RX                   GPIOA
#define    GPIO_PIN_RX               GPIO_Pin_1
#define    GPIO_PINSOURCE_RX         GPIO_PinSource1
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOA

/* DMA */
#define    DMA1_Stream_RX            DMA1_Stream2

//���յ����Ӿ������ݴ�������
uint8_t  Com4_Vision_Buffer[ VISION_BUFFER_LEN ] = {0};

int Usart4_Clean_IDLE_Flag = 0;

DMA_InitTypeDef xCom4DMAInit;

//�Ӿ�ͨ�Ŵ��ڳ�ʼ��
void UART4_Init(void)
{
	USART_InitTypeDef  xUsartInit;
	GPIO_InitTypeDef   xGpioInit;
	NVIC_InitTypeDef   xNvicInit;

	RCC_AHB1PeriphClockCmd( RCC_AHB1PERIPH_GPIO_TX | RCC_AHB1PERIPH_GPIO_RX, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART4, ENABLE );

	GPIO_PinAFConfig( GPIO_TX, GPIO_PINSOURCE_TX, GPIO_AF_UART4 );//���Ÿ���
	GPIO_PinAFConfig( GPIO_RX, GPIO_PINSOURCE_RX, GPIO_AF_UART4 ); 

	xGpioInit.GPIO_Pin   = GPIO_PIN_TX;//0
	xGpioInit.GPIO_Mode  = GPIO_Mode_AF;
	xGpioInit.GPIO_OType = GPIO_OType_PP;
	xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
	xGpioInit.GPIO_PuPd  = GPIO_PuPd_UP;

	GPIO_Init( GPIO_TX, &xGpioInit );

	xGpioInit.GPIO_Pin = GPIO_PIN_RX;//1
	GPIO_Init( GPIO_RX, &xGpioInit );

	xUsartInit.USART_BaudRate            = 115200;   //������
	xUsartInit.USART_WordLength          = USART_WordLength_8b;//�ֳ�8����
	xUsartInit.USART_StopBits            = USART_StopBits_1;//һ��ֹͣλ
	xUsartInit.USART_Parity              = USART_Parity_No;//����żУ��
	xUsartInit.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
	xUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Init( UART4, &xUsartInit );
	USART_Cmd( UART4, ENABLE );

	//ʹ�ܴ��ڿ����ж�
	USART_ITConfig( UART4, USART_IT_IDLE, ENABLE );  

	//DMA���������ж�
	USART_DMACmd( UART4, USART_DMAReq_Rx, ENABLE );
	USART_DMACmd( UART4, USART_DMAReq_Tx, ENABLE );

	UART4_DMA_Init();

	//�ж�����
	xNvicInit.NVIC_IRQChannel                    = UART4_IRQn;
	xNvicInit.NVIC_IRQChannelPreemptionPriority  = 0;//���ȼ�
	xNvicInit.NVIC_IRQChannelSubPriority         = 0;
	xNvicInit.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init( &xNvicInit );		
}

//DMA����
void UART4_DMA_Init(void)
{
	DMA_DeInit( DMA1_Stream_RX );
	xCom4DMAInit.DMA_Channel = DMA_Channel_4;

	xCom4DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;//�������赽�洢��

	xCom4DMAInit.DMA_PeripheralBaseAddr  = (uint32_t)&(UART4->DR);
	xCom4DMAInit.DMA_Memory0BaseAddr     = (uint32_t)Com4_Vision_Buffer;
	xCom4DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
	xCom4DMAInit.DMA_BufferSize = 100;
	xCom4DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	xCom4DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
	xCom4DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	xCom4DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	xCom4DMAInit.DMA_Mode = DMA_Mode_Circular;
	xCom4DMAInit.DMA_Priority = DMA_Priority_VeryHigh;
	xCom4DMAInit.DMA_FIFOMode = DMA_FIFOMode_Enable;
	xCom4DMAInit.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	xCom4DMAInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	xCom4DMAInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init( DMA1_Stream_RX, &xCom4DMAInit );	
	DMA_Cmd( DMA1_Stream_RX, ENABLE);  //stream0
}

//����4�жϷ�����
void UART4_IRQHandler(void)
{	
	if(USART_GetITStatus(UART4,USART_IT_IDLE)!=RESET)//��⵽������·
	{		
		//����ST�ٷ��ֲ�,��һ��SR��DR�Ĵ���,IDLE�����ٴ�ʹ��,�����һֱ�����ж�,�ͻ�����ڽ����ж�û����
		Usart4_Clean_IDLE_Flag = UART4->SR ;
		Usart4_Clean_IDLE_Flag = UART4->DR ;
			
		DMA_Cmd(DMA1_Stream2,DISABLE );
		
		Usart4_Clean_IDLE_Flag = VISION_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream2);

		//Vision_Read_Data(Com4_Vision_Buffer);//��ȡ�Ӿ�����	
		
		memset(Com4_Vision_Buffer, 0, 100);
		
		Green_On;
		DMA_Cmd(DMA1_Stream2,ENABLE);//D1S2
	}
}

/**
  * @brief  ����һ�η���һ���ֽ�����
  * @param  ����
  * @retval void
  * @attention  8λ
  */
void UART4_SendChar(uint8_t cData)
{
	while (USART_GetFlagStatus( UART4, USART_FLAG_TC ) == RESET);
	
	USART_SendData( UART4, cData );   
}
