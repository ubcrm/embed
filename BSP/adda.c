#include "adda.h"
#include "stm32f4xx_dac.h"  

/**
  * @brief  ����������ģת������ʼ��
  * @param  void
  * @retval void
  */
void SuperCap_DAC_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef   DAC_InitType;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//ʹ��DACʱ��

		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ��
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//��ʹ�ô������� TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//��ʹ�ò��η���
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//���Ρ���ֵ����
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1�������ر� BOFF1=1
	DAC_Init(DAC_Channel_1,&DAC_InitType);	 //��ʼ��DACͨ��1

	DAC_Cmd(DAC_Channel_1, ENABLE);  //ʹ��DACͨ��1
  
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12λ�Ҷ������ݸ�ʽ����DACֵ
}



/**
  * @brief  ��������ģ��ת������ʼ��
  * @param  void
  * @retval void
  */
void SuperCap_ADC_Init(void)
{
	GPIO_InitTypeDef  GPIO_ADCInitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��

	//�ȳ�ʼ��ADC1ͨ��10 IO��
	GPIO_ADCInitStructure.GPIO_Pin = GPIO_Pin_0;//PC0 ͨ��10
	GPIO_ADCInitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
	GPIO_ADCInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
	GPIO_Init(GPIOC, &GPIO_ADCInitStructure);//��ʼ��  

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����	 

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��

	ADC_Cmd(ADC1, ENABLE);//����ADת����
}


/**
  * @brief  ��������io�ڳ�ʼ��
  * @param  void
  * @retval void
  * @attention PA5->CAP_IN   PA->CAP_OUT
  */
void SuperCap_IO_Init(void)
{
	GPIO_InitTypeDef gpioa;	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	gpioa.GPIO_PuPd = GPIO_PuPd_DOWN;
	gpioa.GPIO_Speed = GPIO_Speed_50MHz;	
	gpioa.GPIO_Mode = GPIO_Mode_OUT;		
	gpioa.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_2;		
	GPIO_Init(GPIOA,&gpioa);	
	
	CAP_Charge_Off;
	CAP_OUT_Off;
}



/**
  * @brief  ���������ѹ
  * @param  vol:0~3300,����0~3.3V
  * @retval void
  */
void Dac1_Set_Vol(u16 vol)
{
	double temp=vol;
	temp/=1000;
	temp=temp*4096/3.3;
	DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12λ�Ҷ������ݸ�ʽ����DACֵ
}



/**
  * @brief  ���ADCֵ
  * @param  void
  * @retval ת�����
  */
u16 Get_Adc(void)   
{
	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, 10, 1, ADC_SampleTime_480Cycles );	//ADC1,ADCͨ��10,480������,��߲���ʱ�������߾�ȷ��			    
  
	ADC_SoftwareStartConv(ADC1);		//ʹ��ָ����ADC1�����ת����������	
	 	
	return ADC_GetConversionValue(ADC1);

}

/**
  * @brief  ��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
  * @param  times:��ȡ����
  * @retval ͨ��ch��times��ת�����ƽ��ֵ
  */
u16 Get_Adc_Average(u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc();
	}
	return temp_val/times;
} 


/**
  * @brief  ��ȡ��ѹֵ
  * @param  void
  * @retval void
  */
float value = 0;
float Get_Realvoltage(void)
{
	value = Get_Adc_Average(10);		//ȡ10��ƽ��ֵ
	value = (float)value*(3.3/4096);
	return (value*11);
}

/**
  * @brief  ��ѹֵ�ٷְ�
  * @param  void
  * @retval void
  * @attention 22~13��ʾ100%~0%
  */
int Capvoltage_Percent(void)
{
	int percent = 0;
	percent = (Get_Realvoltage() - 13.0f) / (23.5f - 13.0f) * 100.0f;	
	
	if(percent<0)
	{ percent = 0; }
	
	return percent;
}
