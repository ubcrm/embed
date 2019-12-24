#ifndef __SYSTEM_H
#define	__SYSTEM_H


/* ����IDԤ����,�������ڵ���,�����������������Ӧ */
#define    DEBUG_ID_ZERO    0       //��ͷ��
#define    DEBUG_ID_ONE     1		//���޳�
#define    DEBUG_ID_TWO     2		//4�Ž�Ȫ��һ��,���޳�
#define    DEBUG_ID_THREE   3		//3�Ž�Ȫ�ڶ���,��ͷ��
#define    DEBUG_ID_FOUR    4		//5�Ž�Ȫ��������������


#define    INFANTRY_DEBUG_ID    DEBUG_ID_FOUR
/*-----------------------------------------*/

/* yaw����Ԥ���� */
#define    YAW_UP		(1)//6623
#define    YAW_DOWN    (-1)//6020

#define    YAW_POSITION    YAW_DOWN
/*-----------------------------------------*/

/* �������ͷԤ���� */
#define    BUFF_CAM_CHAS	0
#define    BUFF_CAM_GIMB	1

#define    BUFF_CAM_TYPE	BUFF_CAM_GIMB
/*-----------------------------------------*/


#include <stdio.h>
#include "stdbool.h"
#include "string.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx.h"
#include "stdint.h"
#include "sys.h" 
#include "usart.h"
#include "delay.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"



#define abs(x) ((x)>0? (x):(-(x)))


/* for FreeRTOS*/
#define    TIME_STAMP_1MS        1
#define    TIME_STAMP_2MS        2
#define    TIME_STAMP_4MS        4
#define    TIME_STAMP_10MS      10
#define    TIME_STAMP_20MS      20
#define    TIME_STAMP_30MS      30
#define    TIME_STAMP_40MS      40
#define    TIME_STAMP_50MS      50
#define    TIME_STAMP_60MS      60
#define    TIME_STAMP_80MS      80
#define    TIME_STAMP_100MS    100
#define    TIME_STAMP_150MS    150
#define    TIME_STAMP_200MS    200
#define    TIME_STAMP_250MS    250
#define    TIME_STAMP_300MS    300
#define    TIME_STAMP_400MS    400
#define    TIME_STAMP_500MS    500
#define    TIME_STAMP_1000MS  1000
#define    TIME_STAMP_2000MS  2000
#define    TIME_STAMP_10S     10000

#define    FALSE    0
#define    TRUE     1

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

/*
	�����ջ��С
	1�� = 4�ֽ�
*/
#define STK_SIZE_64 	64 				//�����ջ64��
#define STK_SIZE_128 	128 			//�����ջ128��
#define STK_SIZE_256 	256 			//�����ջ256��
#define STK_SIZE_512 	512 			//�����ջ512��


#define KP 0
#define KI 1
#define KD 2

#define OUTER 0
#define INNER 1

int constrain(int amt, int low, int high);
float constrain_float(float amt, float low, float high);
int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high);
int32_t constrain_int32_t(int32_t amt, int32_t low, int32_t high);


//б�º���
float RAMP_float( float final, float now, float ramp );
float RampInc_float( float *buffer, float now, float ramp );

void Parameter_Init(void);
void System_Init(void);


#endif 



