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
#include "debug.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "ad7608.h"
#include "comm.h"
#include "calibrator.h"

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
#define AD7608_CH_DATA_RESOLUTION   18
#define AD7608_DMA_BUFFER_LENGTH    (AD7608_CH_NUMBER * AD7608_CH_DATA_RESOLUTION / 8)
#define AD7608_CONVSTA_H   LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4)
#define AD7608_CONVSTA_L LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4)
#define AD7608_RESET_H     LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define AD7608_RESET_L   LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define AD7608_BUSY       LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_5)

#define ADC_VOLTAGE_TRANSFER_FACTOR     (5.0f / 131072.0f)  // 131072 = 2^17
#define INERTIAL_SENSOR_ZERO_OUTPUT      2.50f              // 2.47 to 2.53
#define CIRCULAR_ANGLE_DEGREE            360.0f             // Unit : degree
#define STANDARD_TRACK_DISTANCE          1435.0f            // Unit : mm
#define PI                               3.1415926f

/* External Variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern osSemaphoreId AdcConvertStartSemHandle;
extern osSemaphoreId AdcConvertCompleteSemHandle;
extern uint32_t adc[AD7608_CH_NUMBER];
extern TRACK_MEAS_ITEM meas;
extern WORK_MODE workMode;

/* Private Variables ---------------------------------------------------------*/
const  CAL_TBL tbl __attribute__((section(".ARM.__at_0x08060000"))) = CAL_TBL_DATA;
static uint8_t buff[AD7608_DMA_BUFFER_LENGTH];

/* Private function prototypes -----------------------------------------------*/
static void AD7608_RESET(void);
static void AD7608_TRIGGER(void);
static void Delay(uint32_t nCount);
static float calibrateADCData(ADC_CAL item, float raw);

/* Formal function definitions -----------------------------------------------*/
void adcTask(void const * argument)
{
  AD7608_RESET();
  
  while(1) {
    osSemaphoreWait(AdcConvertStartSemHandle, osWaitForever);
    AD7608_TRIGGER();
    while(AD7608_BUSY) {};
    HAL_SPI_Receive_DMA(&hspi1, buff, sizeof(buff));
  }
}

static void AD7608_RESET(void)
{
  AD7608_RESET_H;
  Delay(0xFF);
  AD7608_RESET_L;
}

static void AD7608_TRIGGER(void)
{
  AD7608_CONVSTA_L;
  Delay(0xF);
  AD7608_CONVSTA_H;
}

static void Delay(uint32_t nCount) 
{ 
  for(; nCount != 0; nCount--); 
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  // Map the ADC data from DMA buffer
  adc[0] = ((buff[ 0] & 0xFF) << 10) + (buff[ 1] << 2) + (buff[ 2] >> 6);
  adc[1] = ((buff[ 2] & 0x3F) << 12) + (buff[ 3] << 4) + (buff[ 4] >> 4);
  adc[2] = ((buff[ 4] & 0x0F) << 14) + (buff[ 5] << 6) + (buff[ 6] >> 2);
  adc[3] = ((buff[ 6] & 0x03) << 16) + (buff[ 7] << 8) + (buff[ 8] >> 0);
  adc[4] = ((buff[ 9] & 0xFF) << 10) + (buff[10] << 2) + (buff[11] >> 6);
  adc[5] = ((buff[11] & 0x3F) << 12) + (buff[12] << 4) + (buff[13] >> 4);
  adc[6] = ((buff[13] & 0x0F) << 14) + (buff[14] << 6) + (buff[15] >> 2);
  adc[7] = ((buff[15] & 0x03) << 16) + (buff[16] << 8) + (buff[17] >> 0);
  
#if 0
  // Print debug info (raw DMA data)
  PRINTF("DMA_BUF[18] = 0x");
  for (uint8_t i = 0; i < 18; i++) PRINTF("%X", buff[i]);
  PRINTF("\r\n");
  
  // Print debug info (ADC data)
  for(uint8_t i = 0; i < 8; i++) {
    PRINTF("CH%d : %d, \t", i, adc[i]);
    if (i == 3 || i == 7) PRINTF("\r\n");
    if (i == 7) PRINTF("\r\n");
  }
#endif
  
  osSemaphoreRelease(AdcConvertCompleteSemHandle);
}

void changeADCData2ActualValue(void)
{
  float sin_roll;
  float vol[AD7608_CH_NUMBER];
  
  // ADC data --> voltage (-5V to +5V)
  for (uint32_t i = 0; i < AD7608_CH_NUMBER; i++) {
    //vol[i] = (float)((adc[i] << 14) >> 14) * ADC_VOLTAGE_TRANSFER_FACTOR;
    vol[i] = (float)((int32_t)(adc[i] ^ 0x00020000) - (int32_t)0x00020000) * 
             ADC_VOLTAGE_TRANSFER_FACTOR;
  }
  
  meas.distance_comp= vol[TRACK_DIST_COMPENSATION];
  meas.height_comp  = vol[TRACK_HEIGHT];
  meas.distance     = vol[TRACK_DISTANCE];
  meas.battery      = vol[TRACK_BATTERY_VOLTAGE];
  
  // calculate dip angle and track height
  // Angle = arcsin((E0-Eb)/SF)-Theta
  sin_roll = (vol[TRACK_DIP_A1] - vol[TRACK_DIP_0] - INERTIAL_SENSOR_ZERO_OUTPUT) /
              TILT_SCALE_FACTOR;
  if (sin_roll > +1.0f) sin_roll = +1.0f;
  if (sin_roll < -1.0f) sin_roll = -1.0f;
  meas.roll = asin(sin_roll) * CIRCULAR_ANGLE_DEGREE / (2.0f * PI) - 
              TILT_AXIS_MISALIGNMENT_ANGLE;
  meas.height = sin_roll * STANDARD_TRACK_DISTANCE;
  
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
