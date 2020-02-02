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

#define USE_IST8310 //是否使用IST8310磁力计，不使用注释定义

#define MPU6500_USE_DATA_READY_EXIT //是否使用MPU6500的外部中断，不使用注释定义

#define MPU6500_USE_SPI_DMA //是否使用SPI的DMA传输，不使用注释定义

//如果用了IST8310，DMA传输23个字节，如果不用，少7个字节，为16个字节
#if defined(USE_IST8310)
#define DMA_RX_NUM 23
#else
#define DMA_RX_NUM 16
#endif

//mpu6500原始数据在缓冲区buf的位置
#ifdef MPU6500_USE_SPI_DMA
#define MPU6500_RX_BUF_DATA_OFFSET 1
#else
#define MPU6500_RX_BUF_DATA_OFFSET 0
#endif

//ist83100原始数据在缓冲区buf的位置
#ifdef MPU6500_USE_SPI_DMA
#define IST8310_RX_BUF_DATA_OFFSET 16
#else
#define IST8310_RX_BUF_DATA_OFFSET 15
#endif

#define MPU6500_TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define MPU6500_TEMPERATURE_PID_KI 0.2f    //温度控制PID的ki
#define MPU6500_TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd

#define MPU6500_TEMPERATURE_PID_MAX_OUT 4500.0f  //温度控制PID的max_out
#define MPU6500_TEMPERATURE_PID_MAX_IOUT 4400.0f //温度控制PID的max_iout

#define INS_DELTA_TICK 1 //任务调用的间隔

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1

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
