/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : nst1001.c
 * Project  : Track Test Tool
 * Date     : 2023/11/1
 * Author   : WangYu
 *
 **********************************************************************/
 
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
#define CHECK_INTERVAL_TIME                5  // ms
#define FRAME_DATA_DETERMINATION_TIME      10 // ms
#define NST_FILTER_DEPTH                   32 // 5 x 32 = 160 ms

#define NST_PULSE_2_CELSIUS_FACTOR         0.0625f
#define NST_PULSE_2_CELSIUS_OFFSET         50.0625f
#define NST_COMP_POINT_1                   30.0f
#define NST_COMP_POINT_2                   100.0f
#define NST_COMP_POINT_1_COEFFICIENT       0.005f
#define NST_COMP_POINT_2_COEFFICIENT       0.012f

/* Private typedef -----------------------------------------------------------*/
/* External Variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
extern TRACK_MEAS_ITEM meas;

/* Private Variables ---------------------------------------------------------*/
static volatile uint16_t cntPulse = 0;

/* Private function prototypes -----------------------------------------------*/
static float calculateCelsius(uint16_t pulseCnt);

/* Formal function definitions -----------------------------------------------*/
void startTempSensor(void)
{
  LL_GPIO_SetOutputPin(EN_NST1_GPIO_Port, EN_NST1_Pin);
  LL_GPIO_ResetOutputPin(EN_NST2_GPIO_Port, EN_NST2_Pin);
  LL_GPIO_SetOutputPin(EN_NST3_GPIO_Port, EN_NST3_Pin);
	
  HAL_TIM_Base_Start(&htim4);
}

void sensorTask(void const * argument)
{
  uint16_t noPulseTime = 0;
  uint16_t prevCnt = 0;
  
  while(1) {
    uint16_t currCnt = __HAL_TIM_GET_COUNTER(&htim4);

    if(currCnt == 0) { // TIM4 is not started
      noPulseTime = 0;
    }
    else if(currCnt != prevCnt) { // NST1001 pulse counting time period
      noPulseTime = 0;
      prevCnt = currCnt;
    }
    else {
      noPulseTime += CHECK_INTERVAL_TIME;
      if(noPulseTime > FRAME_DATA_DETERMINATION_TIME) {
        meas.temperature = calculateCelsius(currCnt);
        noPulseTime = 0;
        prevCnt = 0;
        // reset TIM4
        HAL_TIM_Base_Stop_IT(&htim4);
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        HAL_TIM_Base_Start_IT(&htim4);
      }
    }
    
    // NST1001 frame min gap time is 16ms, so check nst1001 every 5ms is reasonable.
    osDelay(CHECK_INTERVAL_TIME);
  }
}

static float calculateCelsius(uint16_t pulseCnt)
{
  float celsius = 0.0f;
  
  //celsius = movingFilter(cnt_pulsNum) * 0.0625f - 50.0625f;
  celsius = (float)pulseCnt * NST_PULSE_2_CELSIUS_FACTOR - NST_PULSE_2_CELSIUS_OFFSET;

  if(celsius < NST_COMP_POINT_1) {
    celsius = celsius + (celsius - NST_COMP_POINT_1) * NST_COMP_POINT_1_COEFFICIENT;
  }
  else if(celsius > NST_COMP_POINT_2) {
    celsius = celsius + (NST_COMP_POINT_2 - celsius) * NST_COMP_POINT_2_COEFFICIENT;
  }

  return celsius;
}
