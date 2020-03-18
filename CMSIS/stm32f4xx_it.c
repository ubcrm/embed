/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"
#include "gimbal_task.h"
#include "USART_comms.h"
#include <string.h>
#include <stdio.h>

volatile char buffer_rx[BUFFER_SIZE]; //holds 100 bytes
int count_rx = 0;

volatile char UART8_buffer_rx[100];
int UART8_count_rx = 0;

char rando[] = "hello\n\r";
char new_line[] = "\n\r";
int atoi(const char *str);
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
/*
void SVC_Handler(void)
{
}
*/
/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
/*
void PendSV_Handler(void)
{
}
*/
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/*
void SysTick_Handler(void)
{
  
}
*/
/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 
  

// Interrupt handler for USART 6. This is called on the reception of every
// byte on USART6
// Note: USART_Data only holds 1 byte of data despite being a uint16_t variable
void USART6_IRQHandler(void)
{
	// make sure USART6 was intended to be called for this interrupt
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
		uint16_t USART_Data = USART_ReceiveData(USART6);
		
		// This just echos everything back once a CR (carriage return)
		// character (13 in ascii) is received. Windows (or Putty, idk)
		// appends a CR to enter/return so it should work 
		if (USART_Data == 13) {
			buffer_rx[count_rx] = 0;
			serial_send_string(buffer_rx);
			
			count_rx = 0;
			for (int i = 0; i < 100; i++) {
				buffer_rx[i] = 0;
			}
			
		} else if (count_rx < 100 - 1) {
			buffer_rx[count_rx++] = USART_Data;
		} else {
			count_rx = 0;
			for (int i = 0; i < 100; i++) {
				buffer_rx[i] = 0;
			}
		}
		
	}
}


void clear_gimbal_buffer(void) {
    char str[20];
    Gimbal_Angles *gimbal_angles = get_gimbal_angle_struct();
    
    uint16_t angle = atoi(gimbal_angles->buffer_rx);
    sprintf(str, "delta angle: %d\n\r", angle);
    vision_send_string(str);
    
    gimbal_angles->yaw_angle = angle; 
    gimbal_angles->new_angle_flag = 1;
    gimbal_angles->count_rx = 0;
    
    for (int i = 0; i < BUFFER_SIZE; i++) {
        gimbal_angles->buffer_rx[i] = 0;
    }
}

// Interrupt handler for UART 8. This is called on the reception of every
// byte on UART8
// Note: USART_Data only holds 1 byte of data despite being a uint16_t variable
void UART8_IRQHandler(void)
{
	// make sure UART8 was intended to be called for this interrupt
	if(USART_GetITStatus(UART8, USART_IT_RXNE) != RESET) {
		uint16_t UART_Data = USART_ReceiveData(UART8);
        
        Gimbal_Angles *gimbal_angles = get_gimbal_angle_struct();
 
		// Once a carriage return is read or the buffer full, move 
        // data into struct, and clear the buffer
		if (UART_Data == 13 || gimbal_angles->count_rx >= BUFFER_SIZE-1) {
            //vision_send_string(gimbal_angles->buffer_rx); //echo
            //vision_send_string(new_line);
            clear_gimbal_buffer();
            //vision_send_string(gimbal_angles->buffer_rx); //echo
            //vision_send_string(new_line);
		} 
        else {
			gimbal_angles->buffer_rx[gimbal_angles->count_rx++] = UART_Data;
		} 
        
        // This just echos everything back once a CR (carriage return)
		// character (13 in ascii) is received. Windows (or Putty, idk)
		// appends a CR to enter/return so it should work 
        /*
		if (UART_Data == 13) {
			UART8_buffer_rx[UART8_count_rx] = 0;
			vision_send_string(UART8_buffer_rx);
			
			UART8_count_rx = 0;
			for (int i = 0; i < 100; i++) {
				UART8_buffer_rx[i] = 0;
			}
			
		} else if (UART8_count_rx < 100 - 1) {
			UART8_buffer_rx[UART8_count_rx++] = UART_Data;
		} else {
			UART8_count_rx = 0;
			for (int i = 0; i < 100; i++) {
				UART8_buffer_rx[i] = 0;
			}
		}
        */
	}
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
