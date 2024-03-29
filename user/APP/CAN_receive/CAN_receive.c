/**
  ******************************************************************************
    * @file    APP/CAN_receive
    * @date    15-January-2020
    * @brief   This file contains functions to write to chassis and gimbal motors 
    *          and read their information using CAN interrupt. 
    * @attention The initialization of CAN channels must be done by calling funtions
    *          in hardware/CAN.c. Additionally, the motor layout are as following:
    *          CAN 1: chassis  M3508  
    *                 motor ID     1     2     3     4
    *                 write ID 0x200 0x200 0x200 0x200
    *                 read ID  0x201 0x202 0x203 0x204 
    *          CAN 2: gimbal M6020_yaw M6020_pitch P36_trigger P36_hopper
    *                 motor ID     1     2     3     4
    *                 motor bits 001   010   N/A   N/A
    *                 write ID 0x1FF 0x1FF 0x1FF 0x1FF
    *                 read ID  0x205 0x206 0x207 0x208
  ******************************************************************************
**/


/******************** User Includes ********************/
#include "CAN_Receive.h"

#include "stm32f4xx.h"
#include "rng.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "delay.h"



/******************** Private Defines for Receiving ********************/

#define fill_motor_readings(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->current_read = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

/******************** Private User Declarations ********************/
		
//CAN received data handler
static void CAN_hook(CanRxMsg *rx_message);
//motor measure pointers declaration
static motor_feedback_t motor_yaw, motor_pit, motor_trigger, motor_hopper, motor_chassis[4];
//CAN transmit message struct declaration
static CanTxMsg GIMBAL_TxMessage;
		

		
/******************** Public CAN Write Functions ********************/

/**
* @brief  Writes a CAN message to 4 gimbal motors
* @param  yaw, pitch: speed for the yaw and pitch channelM6020 motors, in range of 0 to 4095
* @param  trigger, rev: speed for the revolver and trigger triggering motors, in range of 0 to 4095
* @retval None
*/

void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t trigger, int16_t hopper)
{
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (yaw >> 8);
    GIMBAL_TxMessage.Data[1] = yaw;
    GIMBAL_TxMessage.Data[2] = (pitch >> 8);
    GIMBAL_TxMessage.Data[3] = pitch;
    GIMBAL_TxMessage.Data[4] = (trigger >> 8);
    GIMBAL_TxMessage.Data[5] = trigger;
    GIMBAL_TxMessage.Data[6] = (hopper >> 8);
    GIMBAL_TxMessage.Data[7] = hopper;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE

    TIM6->CNT = 0;
    TIM6->ARR = delay_time ;

    TIM_Cmd(TIM6,ENABLE);
#else
	CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif

}


/**
* @brief  Timer 6 interrput handler, functionality unknown...
* @param  None
* @retval None
*/
void TIM6_DAC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM6, TIM_IT_Update )!= RESET )
    {

        TIM_ClearFlag( TIM6, TIM_IT_Update );
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
        CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif
        TIM_Cmd(TIM6,DISABLE);
    }
}


/**
* @brief  Resets chassis motor CAN ID by sending message with 0x700
* @param  None
* @retval None
*/
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CAN2, &TxMessage);
}


/**
 * @brief  Writes a CAN message to chassis motors
 * @param  Input speed values for the four motors (M3508) range untested
 * @retval None
 */
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}



/******************* Public CAN Read Functions *******************/

//Return a pointer to yaw motor data
const motor_feedback_t *get_yaw_gimbal_motor_feedback_pointer(void)
{
    return &motor_yaw;
}

//Return a pointer to pitch motor data
const motor_feedback_t *get_pitch_motor_feedback_pointer(void)
{
    return &motor_pit;
}

//Return a pointer to trigger trigger motor data
const motor_feedback_t *get_trigger_motor_feedback_pointer(void)
{
    return &motor_trigger;
}

//Return a pointer to revolver motor data
const motor_feedback_t *get_hopper_motor_feedback_pointer(void)
{
    return &motor_hopper;
}

//Return a pointer to chassis motor data
const motor_feedback_t *get_chassis_motor_feedback_pointer(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}


/******************** Private Function Implementationss ********************/

/**
* @brief  CAN1 interrupt handler, used for reading gimbal messages
* @param  None
* @modify rx1_message
* @retval None
*/
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) // Ensure CAN channel is open to receive data
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);  // Receives CAN1 (gimbal) data and stores in rx1_message
        CAN_hook(&rx1_message); // Call to CAN_hook and allocates the received message to a data struct
    }
}


/**
* @brief  CAN2 interrupt handler, used for reading chassis messages
* @param  None
* @modify rx2_message
* @retval None
*/
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET) // Ensure CAN channel is open to receive data
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message); // Receives CAN2 (chassis) data and stores in rx1_messag
        CAN_hook(&rx2_message); // Call to CAN_hook and allocates the received message to a data struct
    }
}


/**
* @brief  Handles CAN messages received, takes motor ID and calls handling function of that motor
*					The method runs in the background and updates the relevant motor information automatically
* @param  rx_message: pointer to the raw data read on CAN
* @modify (one of) motor_yaw, motor_pit, motor_trigger, motor_revolver, and motor_chassis

* @retval None
*/
static void CAN_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId) //Gets the ID from the received CAN message
    {
    case CAN_YAW_MOTOR_ID:
    {
        fill_motor_readings(&motor_yaw, rx_message);
        break;
    }
    case CAN_PIT_MOTOR_ID:
    {
        fill_motor_readings(&motor_pit, rx_message);
        break;
    }
    case CAN_TRIGGER_MOTOR_ID:
    {
        fill_motor_readings(&motor_trigger, rx_message);
        break;
    }
		case CAN_HOPPER_MOTOR_ID:
    {
        fill_motor_readings(&motor_hopper, rx_message);
        break;
    }
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
        static uint8_t i = 0;
        i = rx_message->StdId - CAN_3508_M1_ID; //get motor ID
        fill_motor_readings(&motor_chassis[i], rx_message); //Use motor ID as index of array
        break;
    }

    default:
    {
        break;
    }
    }
}
