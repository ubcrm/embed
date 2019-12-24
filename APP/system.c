#include "system.h"

#include "remote.h"


#include "can1.h"
#include "usart2.h"
#include "pwm.h"
#include "laser.h"
#include "usart5.h"
#include "can2.h"
#include "usart4.h"
#include "led.h"
#include "adda.h"

#include "sys.h"
#include "usart.h"
#include "mpu6050.h"
#include "myiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "delay.h"
#include "MPU_Temperature.h"
#include "iwdg.h"


//�������Լ���
bool pass_flag=1;

//�޷�
int constrain(int amt, int low, int high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

float constrain_float(float amt, float low, float high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int32_t constrain_int32_t(int32_t amt, int32_t low, int32_t high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

/**
  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
  * @param  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval ��ǰ���
  * @attention  
  */
float RAMP_float( float final, float now, float ramp )
{
	  float buffer = 0;
	
	
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}

/**
  * @brief  б�º���,ʹĿ�����ֵ��������ָ������ֵ
  * @param  Ҫ�ڵ�ǰ��������ۼӵ�ֵ,Ŀ�������,��������
  * @retval Ŀ�������
  * @attention  
  *             
*/
float RampInc_float( float *buffer, float now, float ramp )
{

		if (*buffer > 0)
		{
				if (*buffer > ramp)
				{  
						now     += ramp;
					  *buffer -= ramp;
				}   
				else
				{
						now     += *buffer;
					  *buffer  = 0;
				}
		}
		else
		{
				if (*buffer < -ramp)
				{
						now     += -ramp;
					  *buffer -= -ramp;
				}
				else
				{
						now     += *buffer;
					  *buffer  = 0;
				}
		}
		
		return now;
}


/**********************************************/
//������ʼ��
void Parameter_Init(void)
{	

}

//�����ʼ��
void SYSTEM_InitPeripheral(void)
{
	static portTickType ulCurrentTime = 0;
	static portTickType ulLoopTime    = 0;
	static int16_t  	sTimeCnt      = 0;
	
	Delay_init(168);//����ʱ��
	CAN1_Init();//���̺���̨�����ʼ��
	USART2_Init();//���ջ���ʼ��
	UART5_Init();//����ϵͳ��ʼ��
	TIM4_Init();//Ħ���ֳ�ʼ��
	TIM1_Init();//���ֶ����ʼ��
	LASER_Init();//������׼��ʼ��
	CAN2_Init();//���̵����ʼ��
	UART4_Init();//�Ӿ����ڳ�ʼ��
//	MPU_TempPID_Init_IO();//MPU�¶�Ư�Ƴ�ʼ��,����������ǽǶȶ�ȡ���ֵ�λ���壬Ӱ����̨�ȶ���
	Led_Init();//ָʾ�Ƴ�ʼ��
	SuperCap_ADC_Init();//���ݵ�����ȡ��ʼ��
	SuperCap_DAC_Init();//���ݳ����Ƴ�ʼ��
	SuperCap_IO_Init();//����IO�ڳ�ʼ��
	IWDG_Init(4,30000);
	
	//�洫MPU��ʼ��
	MPU_Init();
	while (mpu_dmp_init( )) 
	{
			ulCurrentTime = xTaskGetTickCount();

			if (ulCurrentTime >= ulLoopTime)  
			{
				  /* 100MS��ʱ */
					ulLoopTime = ulCurrentTime + TIME_STAMP_100MS;
				
				  /* 300ms�����Լ� */
					if (sTimeCnt >= 2) 
					{
							pass_flag = 0;
							sTimeCnt  = 0;//10;
					}
					else
					{
							sTimeCnt++;
					}
			}
	}
}
/*********************************************/


//ϵͳ��ʼ��
void System_Init(void)
{
	Parameter_Init();//������ʼ��	
	REMOTE_vResetData();//ң�س�ʼ��
	SYSTEM_InitPeripheral();//�����ʼ��
}
