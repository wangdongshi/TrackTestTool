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
#define GYRO_RX_BUFFER_SIZE     8
#define GYRO_UPDATE_FREQUENCY   300.0f    // Unit : Hz
#define GYRO_FIRST_BYTE         0xDD
#define CIRCULAR_ANGLE_DEGREE   360.0f    // Unit : degree

/* Private typedef -----------------------------------------------------------*/
typedef enum {
  OK = 0,
  NG
} Status;

/* External Variables --------------------------------------------------------*/
//extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TRACK_MEAS_ITEM meas;
extern GYRO_MODE gyroMode;

/* Private Variables ---------------------------------------------------------*/
static volatile uint8_t gyro1[GYRO_RX_BUFFER_SIZE]; // gyro1 angle velocity raw data (significant 24 bit)
static volatile uint8_t gyro2[GYRO_RX_BUFFER_SIZE]; // gyro2 angle velocity raw data (significant 24 bit)

volatile float gyro_zero_offset[2] = {0.0f, 0.0f};
volatile uint64_t gyro_count[2] = {0, 0};

/* Private function prototypes -----------------------------------------------*/

/* Formal function definitions -----------------------------------------------*/
void startGyro(void)
{
  HAL_StatusTypeDef status;
  
  meas.omega1 = 0.0f - gyro_zero_offset[0];
  meas.omega2 = 0.0f - gyro_zero_offset[1];
  
  status = HAL_UART_Receive_DMA(&huart3, (uint8_t*)gyro1, GYRO_RX_BUFFER_SIZE);
  assert_param(status == HAL_OK);

  status = HAL_UART_Receive_DMA(&huart6, (uint8_t*)gyro2, GYRO_RX_BUFFER_SIZE);
  assert_param(status == HAL_OK);
}

// pitch
void uart3RxCallback(void)
{
  int32_t gyro_24bit;
  
  // Check leading byte
  if (gyro1[0] != GYRO_FIRST_BYTE) {
    // Integrate angular velocity by previous omega data
    PRINTF("Gyroscope1 data format is error!\r\n");
    meas.pitch += meas.omega1 / GYRO_UPDATE_FREQUENCY;
    if (meas.pitch > CIRCULAR_ANGLE_DEGREE) meas.pitch -= CIRCULAR_ANGLE_DEGREE;
    gyro_count[0]++;
    return;
  }
    
  // Get raw data
  gyro_24bit = (((int32_t)gyro1[2]) << 16) + (((int32_t)gyro1[3]) << 8) + (int32_t)gyro1[1];
  //gyro_24bit = (gyro_24bit << 8) >> 8;
  gyro_24bit = (int32_t)(gyro_24bit ^ 0x00800000) - (int32_t)0x00800000;
  
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
  meas.pitch += meas.omega1 / GYRO_UPDATE_FREQUENCY;
  if (meas.pitch > CIRCULAR_ANGLE_DEGREE) meas.pitch -= CIRCULAR_ANGLE_DEGREE;
  
  gyro_count[0]++;
}

// yaw
void uart6RxCallback(void)
{
  int32_t gyro_24bit;
  
  // Check leading byte
  if (gyro1[0] != GYRO_FIRST_BYTE) {
    // Integrate angular velocity by previous omega data
    PRINTF("Gyroscope2 data format is error!\r\n");
    meas.yaw += meas.omega2 / GYRO_UPDATE_FREQUENCY;
    if (meas.yaw > CIRCULAR_ANGLE_DEGREE) meas.yaw -= CIRCULAR_ANGLE_DEGREE;
    gyro_count[1]++;
    return;
  }
  
  // Get raw data
  gyro_24bit = (((int32_t)gyro2[2]) << 16) + (((int32_t)gyro2[3]) << 8) + (int32_t)gyro2[1];
  //gyro_24bit = (gyro_24bit << 8) >> 8;
  gyro_24bit = (int32_t)(gyro_24bit ^ 0x00800000) - (int32_t)0x00800000;
  
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
  meas.yaw += meas.omega2 / GYRO_UPDATE_FREQUENCY;
  if (meas.yaw > CIRCULAR_ANGLE_DEGREE) meas.yaw -= CIRCULAR_ANGLE_DEGREE;
  
  gyro_count[1]++;
}
