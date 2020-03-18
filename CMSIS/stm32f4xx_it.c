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

volatile char buffer_rx[BUFFER_SIZE]; //holds 100 bytes
int count_rx = 0;

Gimbal_Angles* gimbal_angles;

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
		// character (13 in ascii) is received. Useful if debugging 
        // from Putty which appends CR if enter key is pressed.
		if (USART_Data == 13) {
			buffer_rx[count_rx] = 0;
			serial_send_string(buffer_rx);
			
            // clear the buffer and counter
			count_rx = 0;
			for (int i = 0; i < BUFFER_SIZE; i++) {
				buffer_rx[i] = 0;
			}
			
		} else if (count_rx < BUFFER_SIZE - 1) {
			buffer_rx[count_rx++] = USART_Data;
		} else {
			count_rx = 0;
			for (int i = 0; i < BUFFER_SIZE; i++) {
				buffer_rx[i] = 0;
			}
		}
		
	}
}

int hex_char_to_int(char c){
    switch(c) {
        case '0': return 0b0000;
        case '1': return 0b0001;
        case '2': return 0b0010;
        case '3': return 0b0011;
        case '4': return 0b0100;
        case '5': return 0b0101;
        case '6': return 0b0110;
        case '7': return 0b0111;
        case '8': return 0b1000;
        case '9': return 0b1001;
        case 'a': return 0b1010;
        case 'b': return 0b1011;
        case 'c': return 0b1100;
        case 'd': return 0b1101;
        case 'e': return 0b1110;
        case 'f': return 0b1111;
        default: return 0;
    }
    return 0;
}

void clear_gimbal_buffer(void) {
    Gimbal_Angles *gimbal_angles = get_gimbal_angle_struct();
    uint16_t angle = 0;
    
    for (int i=0; i < gimbal_angles->count_rx; i++) {
        angle = angle << 4;
        angle += hex_char_to_int(gimbal_angles->buffer_rx[i]);
    }
    
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
void USART8_IRQHandler(void)
{
	// make sure USART8 was intended to be called for this interrupt
	if(USART_GetITStatus(UART8, USART_IT_RXNE) != RESET) {
		uint16_t USART_Data = USART_ReceiveData(UART8);
        Gimbal_Angles *gimbal_angles = get_gimbal_angle_struct();
 
		// Once a carriage return is read, move data into struct, and
        // clear the buffer
		if (USART_Data == 13 || gimbal_angles->count_rx >= BUFFER_SIZE-1) {
            clear_gimbal_buffer();
		} 
        else {
			buffer_rx[count_rx++] = USART_Data;
		} 
	}
}

/*
// Interrupt handler for USART 8. This is called on the reception of every
// byte on USART8
void USART8_IRQHandler(void)
{
	// make sure USART8 was intended to be called for this interrupt
	if(USART_GetITStatus(UART8, USART_IT_RXNE) != RESET) {
		uint16_t USART_Data = USART_ReceiveData(UART8);
        serial_send_string(NOT_READY);
 
        uint8_t mask = 0x01;
        Gimbal_buffer *gimbal_buff = get_gimbal_angle_buffer();
        // first 10 bits are decimal, next 9 integer, next 5 checksum
        switch(gimbal_buff->num_filled) {
            case 0: {
                gimbal_buff->packets[0] = (uint8_t)USART_Data;
                gimbal_buff->decimal = (uint8_t)USART_Data;
                serial_send_string((char)READY);
            };
            case 1: {
                gimbal_buff->packets[1] = (uint8_t)USART_Data;
                uint8_t last_decimal_bits;
                uint8_t first_int_bits;
                
                for (int i=0; i < 8; i++) {
                    if (i < 2) {
                         last_decimal_bits += (uint8_t)USART_Data & mask;
                         last_decimal_bits = last_decimal_bits << 1;
                    }
                    else {
                        first_int_bits += (uint8_t)USART_Data & mask;
                        first_int_bits = first_int_bits << 1;
                    }
                    mask = mask << 1;
                }
                gimbal_buff->integer = first_int_bits;
                gimbal_buff->decimal += last_decimal_bits;
                serial_send_string((char)READY);
            };
            case 2: {
                gimbal_buff->packets[2] = (uint8_t)USART_Data;
                uint8_t last_int_bits;
                uint8_t checksum_bits;

                for (int i=0; i < 8; i++) {
                    if (i < 3) {
                         last_int_bits += (uint8_t)USART_Data & mask;
                         last_int_bits = last_int_bits << 1;
                    }
                    else {
                        checksum_bits += (uint8_t)USART_Data & mask;
                        checksum_bits = checksum_bits << 1;
                    }
                    mask = mask << 1;
                }
                gimbal_buff.checksum = checksum_bits;
                serial_send_string(&READY);
            }
            default:
                serial_send_string(&READY);
        }
 
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
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
