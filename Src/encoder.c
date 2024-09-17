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
#define DIRECTION_ENCODER_INSTALL     (1)         // 1 or -1
#define SPEED_FACTOR                  3.6f        // mm/ms --> km/h
#define TESTER_TRIGGER_DISTANCE       125.0f      // Unit : mm
#define ENCODER_PULSE_RATE            200         // Unit : pulse/r (Omron E6B2)
#define ENCODER_MULTI_FREQ            4           // A & B signal, rising & falling edge
#define PI                            3.1415926f

/* Private type definitions --------------------------------------------------*/
typedef enum {
  FORWARD = 0,
  BACKWARD
} DIRECTION_ENCODER; // encoder rotate direction

/* Private variables ---------------------------------------------------------*/
static int16_t direction;
static uint8_t tim5Overflow = 0;
static DIRECTION_ENCODER currentDirection, previousDirection;
static uint32_t counter = (uint32_t)(TESTER_TRIGGER_DISTANCE * 
  ENCODER_MULTI_FREQ * ENCODER_PULSE_RATE / MILEAGE_WHEEL_DIAMETER / PI);

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern osSemaphoreId EncoderArriveSemHandle;
extern osMutexId EncoderDelayMutexHandle;
//extern osTimerId EncoderDelayTimerHandle;
extern TRACK_MEAS_ITEM meas;
extern uint16_t measBackupFlg;
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
  
  tim5Overflow = 0;
  HAL_TIM_Base_Start_IT(&htim5); // start 10s cyclic timer
  
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

void speedTimerOverflow(void)
{
  HAL_TIM_Base_Stop_IT(&htim5);
  __HAL_TIM_SET_COUNTER(&htim5, 0);
  tim5Overflow = 1;
  meas.speed = 0.0f;
}

void encoderCallback(void)
{
  float ms;
  // update speed
  uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim5);
  if (tim5Overflow) { // The encoder was triggered in the case of the last timeout. 
                      // The speed can't be calculated this time and it is still set to 0.
    meas.speed = 0.0f;
  }
  else {
    if (cnt == 0) {
      meas.speed = 100.0f; // The interval between two encoder interruptions is zero. 
                           // It means that the speed exceeds the upper limit. So it is set to 100km/h.
    }
    else {
      ms = (float)cnt * 0.2f; // 1 cnt = 0.2 ms
      meas.speed = TESTER_TRIGGER_DISTANCE / ms * SPEED_FACTOR;
    }
  }
  tim5Overflow = 0;
  HAL_TIM_Base_Stop_IT(&htim5);
  __HAL_TIM_SET_COUNTER(&htim5, 0);
  HAL_TIM_Base_Start_IT(&htim5);
  
  // update mileage and direction
  currentDirection = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) ? BACKWARD : FORWARD;
  if (currentDirection == previousDirection) {
    if (currentDirection == FORWARD) meas.mileage += (TESTER_TRIGGER_DISTANCE / 1000.f) * (float)direction;
    else meas.mileage -= (TESTER_TRIGGER_DISTANCE / 1000.f) * (float)direction;
    //osMutexWait(EncoderDelayMutexHandle, osWaitForever);  // TODO : Adding Mutex here will cause a crash
    measBackupFlg = 1;
    //osMutexRelease(EncoderDelayMutexHandle);
    HAL_TIM_Base_Start_IT(&htim7); // start encoder delay timer
  }
  else {
    meas.speed = 0.0f; // The rotation is reversed and the speed returns to zero.
  }
  previousDirection = currentDirection;
}
