/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : gyro97B.c
 * Project  : Track Test Tool
 * Date     : 2023/7/10
 * Author   : WangYu
 *
 **********************************************************************/
 
/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "stm32f4xx_hal.h"
#include "calibrator.h"
#include "comm.h"

/* Private macro -------------------------------------------------------------*/
#define GYRO_UPDATE_FREQUENCY   300.0f    // Unit : Hz
#define GYRO_FIRST_BYTE         0xDD
#define CIRCULAR_ANGLE_DEGREE   360.0f    // Unit : degree

/* External Variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TRACK_MEAS_ITEM meas;
extern GYRO_MODE gyroMode;

/* Private Variables ---------------------------------------------------------*/
static uint8_t gyro1[8]; // gyro1 angle velocity raw data (significant 24 bit)
static uint8_t gyro2[8]; // gyro2 angle velocity raw data (significant 24 bit)

volatile float gyro_zero_offset[2] = {0.0f, 0.0f};
volatile uint64_t gyro_count[2] = {0, 0};

/* Private function prototypes -----------------------------------------------*/

/* Formal function definitions -----------------------------------------------*/
void startGyro(void)
{
  HAL_StatusTypeDef status;
  
  meas.omega1 = 0.0f - gyro_zero_offset[0];
  meas.omega2 = 0.0f - gyro_zero_offset[1];
  
  status = HAL_UART_Receive_DMA(&huart3, gyro1, 8);
  assert_param(status == HAL_OK);
  
  status = HAL_UART_Receive_DMA(&huart6, gyro2, 8);
  assert_param(status == HAL_OK);
}

// yaw
void uart3RxCallback(void)
{
  int32_t gyro_24bit;
  
  // Get raw data
  if (gyro1[0] != GYRO_FIRST_BYTE) return;
  gyro_24bit = (((int32_t)gyro1[2]) << 16) + (((int32_t)gyro1[3]) << 8) + (int32_t)gyro1[1];
  gyro_24bit = (gyro_24bit << 8) >> 8;
  
  // Transfer raw data to angular velocity
  meas.omega1 = (float)gyro_24bit / GYRO_SCALE_FACTOR1;
  
  // Process gyro zero drift
  if (gyroMode == GYRO_OFFSET_HOLD) {
    gyro_zero_offset[0] = gyro_zero_offset[0] * (float)gyro_count[0] + meas.omega1;
    gyro_zero_offset[0] /= gyro_count[0] + 1;
  }
  else if (gyroMode == GYRO_OFFSET_REMOVED) {
    meas.omega1 -= gyro_zero_offset[0];
  }
  
  // Integrate angular velocity values into angle values
  meas.yaw += meas.omega1 / GYRO_UPDATE_FREQUENCY;
  if (meas.yaw > CIRCULAR_ANGLE_DEGREE) meas.yaw -= CIRCULAR_ANGLE_DEGREE;
  
  gyro_count[0]++;
}

// pitch
void uart6RxCallback(void)
{
  int32_t gyro_24bit;
  
  // Get raw data
  if (gyro2[0] != GYRO_FIRST_BYTE) return;
  gyro_24bit = (((int32_t)gyro2[2]) << 16) + (((int32_t)gyro2[3]) << 8) + (int32_t)gyro2[1];
  gyro_24bit = (gyro_24bit << 8) >> 8;
  
  // Transfer raw data to angular velocity
  meas.omega2 = (float)gyro_24bit / GYRO_SCALE_FACTOR2;
  
  // Process gyro zero drift
  if (gyroMode == GYRO_OFFSET_HOLD) {
    gyro_zero_offset[1] = gyro_zero_offset[1] * (float)gyro_count[1] + meas.omega2;
    gyro_zero_offset[1] /= gyro_count[1] + 1;
  }
  else if (gyroMode == GYRO_OFFSET_REMOVED) {
    meas.omega2 -= gyro_zero_offset[1];
  }
  
  // Integrate angular velocity values into angle values
  meas.pitch += meas.omega2 / GYRO_UPDATE_FREQUENCY;
  if (meas.pitch > CIRCULAR_ANGLE_DEGREE) meas.pitch -= CIRCULAR_ANGLE_DEGREE;
  
  gyro_count[1]++;
}
