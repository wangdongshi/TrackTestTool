/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : encoder.c
 * Project  : Track Test Tool
 * Date     : 2023/7/14
 * Author   : WangYu
 *
 **********************************************************************/
 
/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

/* Private macro -------------------------------------------------------------*/

/* External Variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern osSemaphoreId EncoderArriveSemHandle;

/* Private Variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Formal function definitions -----------------------------------------------*/
void startEncoder(void)
{
  __HAL_TIM_SET_AUTORELOAD(&htim1, 0x1234); // reset encoder counter
  
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

void encoderCallback(void)
{
  osSemaphoreRelease(EncoderArriveSemHandle);
}
