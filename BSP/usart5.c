#include "usart5.h"



/* UART5_RX  ----> DMA1 Ch4 Stream0 */
/* UART4_RX  ----> DMA1 Ch4 Stream2 */
/* USART3_RX ----> DMA1 Ch4 Stream1 */
/* USART2_RX ----> DMA1 Ch4 Stream5 */

/* TX */
#define    GPIO_TX                   GPIOC
#define    GPIO_PIN_TX               GPIO_Pin_12
#define    GPIO_PINSOURCE_TX         GPIO_PinSource12
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOC

/* RX */
#define    GPIO_RX                   GPIOD
#define    GPIO_PIN_RX               GPIO_Pin_2
#define    GPIO_PINSOURCE_RX         GPIO_PinSource2
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOD

/* if use DMA */
#define    DMA1_Stream_RX            DMA1_Stream0

#define    COM5_PACKAGE_HEADER       JUDGE_FRAME_HEADER


//����ϵͳ�������������ݴ�������
uint8_t  Judge_Buffer[ JUDGE_BUFFER_LEN ] = {0};

int Usart5_Clean_IDLE_Flag = 0;

DMA_InitTypeDef xCom5DMAInit;


/***************************����ϵͳ���ڳ�ʼ��***********************************/
void UART5_Init( void )
{
	USART_InitTypeDef  xUsartInit;
	GPIO_InitTypeDef   xGpioInit;
	NVIC_InitTypeDef   xNvicInit;

	RCC_AHB1PeriphClockCmd( RCC_AHB1PERIPH_GPIO_TX | RCC_AHB1PERIPH_GPIO_RX, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART5, ENABLE );

	GPIO_PinAFConfig( GPIO_TX, GPIO_PINSOURCE_TX, GPIO_AF_UART5 );
	GPIO_PinAFConfig( GPIO_RX, GPIO_PINSOURCE_RX, GPIO_AF_UART5 ); 

	xGpioInit.GPIO_Pin   = GPIO_PIN_TX;
	xGpioInit.GPIO_Mode  = GPIO_Mode_AF;
	xGpioInit.GPIO_OType = GPIO_OType_PP;
	xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
	xGpioInit.GPIO_PuPd  = GPIO_PuPd_UP;

	GPIO_Init( GPIO_TX, &xGpioInit );

	xGpioInit.GPIO_Pin = GPIO_PIN_RX;
	GPIO_Init( GPIO_RX, &xGpioInit );

	xUsartInit.USART_BaudRate            = 115200;   
	xUsartInit.USART_WordLength          = USART_WordLength_8b;
	xUsartInit.USART_StopBits            = USART_StopBits_1;
	xUsartInit.USART_Parity              = USART_Parity_No;
	xUsartInit.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
	xUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_Init( UART5, &xUsartInit );
	USART_Cmd( UART5, ENABLE );
	
	USART_ITConfig( UART5, USART_IT_IDLE, ENABLE  ); //ע��Ҫ���óɴ��ڿ����ж� 

	USART_DMACmd( UART5, USART_DMAReq_Rx, ENABLE );
	USART_DMACmd( UART5, USART_DMAReq_Tx, ENABLE );
	
	UART5_DMA_Init( );//��ʼ��usart5��DMA
	
	xNvicInit.NVIC_IRQChannel                    = UART5_IRQn;
	xNvicInit.NVIC_IRQChannelPreemptionPriority  = 0;
	xNvicInit.NVIC_IRQChannelSubPriority         = 0;
	xNvicInit.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init( &xNvicInit );
}

//DMA��ʼ��
void UART5_DMA_Init( void )
{		
	DMA_DeInit( DMA1_Stream_RX );
	xCom5DMAInit.DMA_Channel = DMA_Channel_4;

	xCom5DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;//�������赽�洢��

	xCom5DMAInit.DMA_PeripheralBaseAddr  = (uint32_t)&(UART5->DR);
	xCom5DMAInit.DMA_Memory0BaseAddr     = (uint32_t)Judge_Buffer;
	xCom5DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
	xCom5DMAInit.DMA_BufferSize = 100;
	xCom5DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	xCom5DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
	xCom5DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	xCom5DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	xCom5DMAInit.DMA_Mode = DMA_Mode_Circular;
	xCom5DMAInit.DMA_Priority = DMA_Priority_VeryHigh;
	xCom5DMAInit.DMA_FIFOMode = DMA_FIFOMode_Enable;
	xCom5DMAInit.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	xCom5DMAInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	xCom5DMAInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init( DMA1_Stream_RX, &xCom5DMAInit );	
	DMA_Cmd( DMA1_Stream_RX, ENABLE);  //stream0
}
/********************************************************************************/


void UART5_IRQHandler( void )
{
	if(USART_GetITStatus(UART5,USART_IT_IDLE)!=RESET)//��⵽������·
	{		
		//����ST�ٷ��ֲ�,��һ��SR��DR�Ĵ���,IDLE�����ٴ�ʹ��,�����һֱ�����ж�,�ͻ�����ڽ����ж�û����
		Usart5_Clean_IDLE_Flag = UART5->SR ;
		Usart5_Clean_IDLE_Flag = UART5->DR ;
		
		DMA_Cmd(DMA1_Stream0,DISABLE);
		
		//Usart5_Clean_IDLE_Flag = JUDGE_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

		//Judge_Read_Data(Judge_Buffer);		
		//memset(Judge_Buffer, 0, 200);
		DMA_Cmd(DMA1_Stream0,ENABLE);
	}
}


/**
  * @brief  ����һ�η���һ���ֽ�����
  * @param  �Լ�����õ�Ҫ�������е�����
  * @retval void
  * @attention  ������λ����
  */
void UART5_SendChar(uint8_t cData)
{
	while (USART_GetFlagStatus( UART5, USART_FLAG_TC ) == RESET);
	
	USART_SendData( UART5, cData );   
}


