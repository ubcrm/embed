/**
  ******************************************************************************
  * @file    APP/CAN_receive
  * @date    15-January-2020
  * @brief   This file contains functions to write to chassis and gimbal motors 
	*          and read their information using CAN interrupt. 
	* @attention The initialization of CAN channels must be done by calling funtions
	*          in hardware/CAN.c. Additionally, the motor layout are as following:
	*          CAN 1: chassis  M3508  
	*									motor ID 1 2 3 4
	*									write ID 0x200 (for all of them)
	*                 read ID 0x201 0x202 0x203 0x204 
	*					 CAN 2: gimbal M6020_yaw M6020_pitch P36_revolver P36_shoot
	*                 motor ID 5 6 7 8
	*									write ID 0x1FF (for all of them)
	*									read ID 0x205 0x206 0x207 0x208
  ******************************************************************************
**/

#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CHASSIS_CAN CAN2
#define GIMBAL_CAN CAN1


// CAN send and receive IDs
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_SHOOT_MOTOR_ID = 0x207,
	  CAN_REVOLVER_MOTOR_ID = 0x208,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;


//Motor data struct for GM6020 and M3508
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;



/******************** Main Functions Called From Outside ********************/

//Resets chassis motor CAN ID
extern void CAN_CMD_CHASSIS_RESET_ID(void);
//Send to yaw, pitch , trigger, and revolver
extern void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//Send to chassis
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//Return a pointer to yaw motor data
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//Return a pointer to pitch motor data
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//Return a pointer to trigger motor data
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
//Return a pointer to trigger motor data
extern const motor_measure_t *get_Revolver_Motor_Measure_Point(void);
//Return a pointer to chassis motors data
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif

#endif
