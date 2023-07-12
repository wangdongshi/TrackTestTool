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
  gyro[0] = 0.0f;
  gyro[1] = 0.0f;
  
  HAL_UART_Receive_DMA(&huart3, gyro1, 8);
  HAL_UART_Receive_DMA(&huart6, gyro2, 8);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3) {
    if (gyro1[0] != GYRO_FIRST_BYTE) return;
    float angular_velocity1 = (gyro1[2] << 16) + (gyro1[3] << 8) + gyro1[1];
    angular_velocity1 /= GYRO_SCALE_FACTOR1;
    gyro[0] += angular_velocity1 / GYRO_UPDATE_FREQUENCY;
    if (gyro[0] > CIRCULAR_ANGLE_DEGREE) gyro[0] -= CIRCULAR_ANGLE_DEGREE;
  }
  else if (huart == &huart6) {
    if (gyro2[0] != GYRO_FIRST_BYTE) return;
    float angular_velocity2 = (gyro2[2] << 16) + (gyro2[3] << 8) + gyro2[1];
    angular_velocity2 /= GYRO_SCALE_FACTOR2;
    gyro[1] += angular_velocity2 / GYRO_UPDATE_FREQUENCY;
    if (gyro[1] > CIRCULAR_ANGLE_DEGREE) gyro[1] -= CIRCULAR_ANGLE_DEGREE;
  }
}
