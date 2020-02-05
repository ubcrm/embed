/**
  ******************************************************************************
    * @file    TASK/test_task
    * @date    24-January/2020
    * @brief   This file contains tasks and functions used for hardware testing
    * @attention Leave the main loop blank
  ******************************************************************************
**/

#ifndef USER_TASK_H
#define USER_TASK_H

#include "CAN_receive.h"
#include "main.h"

//Reading angle, gyro, and accelerometer data and printing to serial
static void test_imu_readings(uint8_t angle, uint8_t gyro, uint8_t acce);
//Turns a gimbal motor (GM6020) and outputs its position, rpm, and current
static void test_GM6020(void);
//Enables Debug of P19 (chassis) motor
static void test_P19(int id);

/******************** Task/Functions Called Outside ********************/
//Task used for testing. Usually left blank.
void test_task(void *pvParameters);

#endif
