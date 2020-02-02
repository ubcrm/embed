/**
  ******************************************************************************
    * @file    TASK/INS_task
    * @brief   This file contains functions to update and access data from the IMU.
    *          Currently using MPU 6500, which allows access to gyro and accelerometer data
    *          A vector of current yaw/pitch direction is also calculated
    *          DMA is used for data access unless otherwise specified
  ******************************************************************************
**/

#ifndef INS_Task_H
#define INS_Task_H
#include "main.h"

#define USE_IST8310 //�Ƿ�ʹ��IST8310�����ƣ���ʹ��ע�Ͷ���

#define MPU6500_USE_DATA_READY_EXIT //�Ƿ�ʹ��MPU6500���ⲿ�жϣ���ʹ��ע�Ͷ���

#define MPU6500_USE_SPI_DMA //�Ƿ�ʹ��SPI��DMA���䣬��ʹ��ע�Ͷ���

//�������IST8310��DMA����23���ֽڣ�������ã���7���ֽڣ�Ϊ16���ֽ�
#if defined(USE_IST8310)
#define DMA_RX_NUM 23
#else
#define DMA_RX_NUM 16
#endif

//mpu6500ԭʼ�����ڻ�����buf��λ��
#ifdef MPU6500_USE_SPI_DMA
#define MPU6500_RX_BUF_DATA_OFFSET 1
#else
#define MPU6500_RX_BUF_DATA_OFFSET 0
#endif

//ist83100ԭʼ�����ڻ�����buf��λ��
#ifdef MPU6500_USE_SPI_DMA
#define IST8310_RX_BUF_DATA_OFFSET 16
#else
#define IST8310_RX_BUF_DATA_OFFSET 15
#endif

#define MPU6500_TEMPERATURE_PID_KP 1600.0f //�¶ȿ���PID��kp
#define MPU6500_TEMPERATURE_PID_KI 0.2f    //�¶ȿ���PID��ki
#define MPU6500_TEMPERATURE_PID_KD 0.0f    //�¶ȿ���PID��kd

#define MPU6500_TEMPERATURE_PID_MAX_OUT 4500.0f  //�¶ȿ���PID��max_out
#define MPU6500_TEMPERATURE_PID_MAX_IOUT 4400.0f //�¶ȿ���PID��max_iout

#define INS_DELTA_TICK 1 //������õļ��

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500�����¶ȵ�����TIM������ֵ������PWM���Ϊ MPU6500_TEMP_PWM_MAX - 1

//Data index offset for angle vector
#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

//Data index offset for gyro pointer
#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

//Data index offset for accelerometer pointer
#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2



/******************** Accessor Functions Called from Outside ********************/

//Main IMU handler task
extern void INS_task(void *pvParameters);
//Gyro calibration
extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);
//Set gyro calibration constants
extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3]);
//Returns a pointer to a vector defining the current robot position
extern const fp32 *get_INS_angle_point(void);
//Returns a pointer to a vector of current gyro reading
extern const fp32 *get_MPU6500_Gyro_Data_Point(void);
//Returns a pointer to a vector of current accelerometer reading
extern const fp32 *get_MPU6500_Accel_Data_Point(void);

#endif
