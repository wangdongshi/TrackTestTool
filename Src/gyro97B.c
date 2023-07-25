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

/* Private macro -------------------------------------------------------------*/
#define GYRO_SCALE_FACTOR1      50000.0f  // Unit : degree/s
#define GYRO_SCALE_FACTOR2      50000.0f  // Unit : degree/s
#define GYRO_UPDATE_FREQUENCY   300.0f    // Unit : Hz
#define GYRO_FIRST_BYTE         0xDD
#define CIRCULAR_ANGLE_DEGREE   360.0f    // Unit : degree

/* External Variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern float gyro[2]; // gyro integral data (angle, unit : degree)

/* Private Variables ---------------------------------------------------------*/
static uint8_t gyro1[8]; // gyro1 angle velocity raw data (significant 24 bit)
static uint8_t gyro2[8]; // gyro2 angle velocity raw data (significant 24 bit)

/* Private function prototypes -----------------------------------------------*/

/* Formal function definitions -----------------------------------------------*/
void startGyro(void)
{
  HAL_StatusTypeDef status;
  
  gyro[0] = 0.0f;
  gyro[1] = 0.0f;
  
  status = HAL_UART_Receive_DMA(&huart3, gyro1, 8);
  assert_param(status == HAL_OK);
  
  status = HAL_UART_Receive_DMA(&huart6, gyro2, 8);
  assert_param(status == HAL_OK);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  int32_t gyro_24bit;
  float angular_velocity;
  
  if (huart == &huart3) {
    if (gyro1[0] != GYRO_FIRST_BYTE) return;
    gyro_24bit = (((int32_t)gyro1[2]) << 16) + (((int32_t)gyro1[3]) << 8) + (int32_t)gyro1[1];
    gyro_24bit = (gyro_24bit << 8) >> 8;
    angular_velocity = (float)gyro_24bit / GYRO_SCALE_FACTOR1;
    //gyro[0] += angular_velocity / GYRO_UPDATE_FREQUENCY;
    //if (gyro[0] > CIRCULAR_ANGLE_DEGREE) gyro[0] -= CIRCULAR_ANGLE_DEGREE;
    gyro[0] = angular_velocity;
  }
  else if (huart == &huart6) {
    if (gyro2[0] != GYRO_FIRST_BYTE) return;
    gyro_24bit = (((int32_t)gyro2[2]) << 16) + (((int32_t)gyro2[3]) << 8) + (int32_t)gyro2[1];
    gyro_24bit = (gyro_24bit << 8) >> 8;
    angular_velocity = (float)gyro_24bit / GYRO_SCALE_FACTOR2;
    //gyro[1] += angular_velocity / GYRO_UPDATE_FREQUENCY;
    //if (gyro[1] > CIRCULAR_ANGLE_DEGREE) gyro[1] -= CIRCULAR_ANGLE_DEGREE;
    gyro[1] = angular_velocity;
  }
}
