/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : ad7608.c
 * Project  : Track Test Tool
 * Date     : 2023/7/3
 * Author   : WangYu
 *
 **********************************************************************/
 
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "debug.h"
#include "cmsis_os.h"
#include "calibrator.h"
#include "ad7608.h"
#include "compute.h"
#include "comm.h"

/* Private macro -------------------------------------------------------------*/

/* STA  : PC04
 * STB  : short to PC04
 * SCLK : PA05
 * RST  : PA07
 * CS   : PA04
 * BUSY : PC05
 * DOA  : PA06
 */

/* Private macro definition */
#define AD7608_CONVSTA_H   LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0)
#define AD7608_CONVSTA_L LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0)
#define AD7608_RESET_H     LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define AD7608_RESET_L   LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define AD7608_BUSY       LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_5)

#define INERTIAL_SENSOR_ZERO_OUTPUT      2.50f              // 2.47 to 2.53
#define CIRCULAR_ANGLE_DEGREE            360.0f             // Unit : degree
#define STANDARD_TRACK_DISTANCE          1505.0f            // Unit : mm
#define PI                               3.1415926f

/* External Variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern SPI_HandleTypeDef hspi1;
extern osMutexId ADCSamplingMutexHandle;
extern osSemaphoreId AdcConvertStartSemHandle;
extern float vol[AD7608_CH_NUMBER];
extern TRACK_MEAS_ITEM meas;
extern WORK_MODE workMode;

/* Private Variables ---------------------------------------------------------*/
const  CAL_TBL tbl __attribute__((section(".ARM.__at_0x08060000"))) = CAL_TBL_DATA;
uint8_t buff[2][AD7608_DMA_BUFFER_LENGTH] __attribute__((aligned(4))) = {0};
uint8_t buffIndex = 0;
float filteredVol[AD7608_CH_NUMBER] = {0.0f}; // filtered ADC data (length = 18 bit) (Must define here!!)

/* Private function prototypes -----------------------------------------------*/
static void  resetADC(void);
static float calibrateADCData(ADC_CAL item, float raw);

/* Formal function definitions -----------------------------------------------*/
void adcTask(void const * argument)
{
  resetADC();
  
  buffIndex = 0;
  while(1) {
    // ADC sampling triggered by TIM2-CH1's PWM
    // ADC data reading triggered in the TIM3 interrupt (TIM2-CH1's slave)
    osSemaphoreWait(AdcConvertStartSemHandle, osWaitForever); // Send by TIM3 interrupt
    HAL_SPI_Receive_DMA(&hspi1, buff[buffIndex], sizeof(buff)); // read previous ADC data
    pretreatADCData();
    buffIndex = !buffIndex;
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
}

void startADC(void)
{
  initFilter();
  // Over Sampling    BUSY_time(Max.)
  //     OFF             4.15 us
  //     x2              9.1  us
  //     x4              18.8 us
  //     x8              39   us
  //     x16             78   us
  //     x32             158  us
  //     x64             315  us
  // During the waiting period for the BUSY signal, 
  // system control can be given up initially, 
  // but even if 64x oversampling is turned on, 
  // the longest waiting time does not exceed 1ms, 
  // which is less than the operating system scheduling interval, 
  // so 1ms delay is used for handing over system control.
  
  // Now the ADC has enabled 64 bit hardware over-sampling, 
  // and a single ADC sampling takes 315us. 
  // In order to facilitate the calculation of software filtering, 
  // we take 400us as a sampling period, and the data reading operation 
  // is also set in one sampling period (reading the previous sampling data).
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);
}

static void resetADC(void)
{
  AD7608_RESET_H;
  for (uint16_t i = 0xFF; i != 0; i--);
  AD7608_RESET_L;
}

// This function only processes the data of the analog sensor part. 
// For the gyroscope, because it is a digital sensor, and its 
// transformation value are closely related to the number of interruptions,
// so the conversion processing of the gyroscope data has been processed 
// in the serial port interrupt of the gyroscope.
void prepareSensorData(void)
{
  float sinRoll;
  
  // set normal valtage item
  osMutexWait(ADCSamplingMutexHandle, osWaitForever);
  meas.distance_comp= vol[TRACK_DIST_COMP];
  meas.height_comp  = vol[TRACK_HEIGHT_COMP];
  meas.distance     = vol[TRACK_DISTANCE];
  meas.battery      = vol[TRACK_BATTERY_VOLTAGE] * 2.0f;
  
  // calculate dip angle and track height
  // Angle = arcsin((E0-Eb)/SF)-Theta
  sinRoll = (vol[TRACK_DIP_A1] - vol[TRACK_DIP_0] - 
             INERTIAL_SENSOR_ZERO_OUTPUT) / TILT_SCALE_FACTOR;
  osMutexRelease(ADCSamplingMutexHandle);
  if (sinRoll > +1.0f) sinRoll = +1.0f;
  if (sinRoll < -1.0f) sinRoll = -1.0f;
  meas.roll = asin(sinRoll) * CIRCULAR_ANGLE_DEGREE / (2.0f * PI) - 
              TILT_AXIS_MISALIGNMENT_ANGLE;
  meas.height = sinRoll * STANDARD_TRACK_DISTANCE;
  
  // calibrate
  if (workMode == MODE_NORMAL_WORK) {
    meas.distance_comp= calibrateADCData(CAL_DIST_COMPENSATION, meas.distance_comp);
    meas.height_comp  = calibrateADCData(CAL_HEIGHT, meas.height_comp);
    meas.distance     = calibrateADCData(CAL_DISTANCE, meas.distance);
    meas.height       = calibrateADCData(CAL_DIP, meas.height);
  }
}

static float calibrateADCData(ADC_CAL item, float raw)
{
  assert_param(item < CAL_ITEMS);
  
  uint8_t index;
  float alpha, result;
  
  for (index = 0; index < CAL_POINTS; index++) {
    if (isnan(tbl[item][index].meas)) return tbl[item][index - 1].real; // overflow
    if (tbl[item][index].meas == raw) return tbl[item][index].real;
    if (tbl[item][index].meas > raw) break;
  }
  
  if (index == 0) return tbl[item][0].real; // underflow
  if (index == CAL_POINTS) return tbl[item][index - 1].real; // overflow
  
  alpha = (tbl[item][index].meas - raw) /
          (tbl[item][index].meas - tbl[item][index - 1].meas);
  result = tbl[item][index].real - alpha * 
          (tbl[item][index].real - tbl[item][index - 1].real);
  
  return result;
}
