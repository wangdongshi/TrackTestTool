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
#include "main.h"
#include "comm.h"
#include "calibrator.h"

/* Private macro -------------------------------------------------------------*/
#define DIRECTION_ENCODER_INSTALL (1) // 1 or -1
#define TESTER_TRIGGER_DISTANCE   125.0f  // Unit : mm
#define ENCODER_PULSE_RATE        200     // Unit : pulse/r (Omron E6B2)
#define ENCODER_MULTI_FREQ        4       // A & B signal, rising & falling edge
#define PI                        3.1415926f

/* Private type definitions --------------------------------------------------*/
typedef enum {
  FORWARD = 0,
  BACKWARD
} DIRECTION_ENCODER; // encoder rotate direction

/* Private variables ---------------------------------------------------------*/
static int16_t direction;
static DIRECTION_ENCODER currentDirection, previousDirection;
static uint32_t counter = (uint32_t)(TESTER_TRIGGER_DISTANCE * 
  ENCODER_MULTI_FREQ * ENCODER_PULSE_RATE / MILEAGE_WHEEL_DIAMETER / PI);

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern osSemaphoreId EncoderArriveSemHandle;
extern TRACK_MEAS_ITEM meas;
extern DIRECTION_MODE directionMode;

/* Private function prototypes -----------------------------------------------*/
static void initEncoder(void);

/* Formal function definitions -----------------------------------------------*/
static void initEncoder(void)
{
  // initialize encoder direction flag
  currentDirection = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) ? BACKWARD : FORWARD;
  previousDirection = currentDirection;
  
  // initialize direction coefficient
  direction = (directionMode ? (-DIRECTION_ENCODER_INSTALL) : (DIRECTION_ENCODER_INSTALL));
  
  // reset encoder counter
  __HAL_TIM_SET_AUTORELOAD(&htim1, counter - 1);
}

void startEncoder(float mileage)
{
  meas.mileage = mileage;
  
  initEncoder();
  
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

void stopEncoder(void)
{
  HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
}

void encoderCallback(void)
{
  currentDirection = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) ? BACKWARD : FORWARD;
  if (currentDirection == previousDirection) {
    if (currentDirection == FORWARD) meas.mileage += (TESTER_TRIGGER_DISTANCE / 1000.f) * (float)direction;
    else meas.mileage -= (TESTER_TRIGGER_DISTANCE / 1000.f) * (float)direction;
    osSemaphoreRelease(EncoderArriveSemHandle);
  }
  previousDirection = currentDirection;
}
