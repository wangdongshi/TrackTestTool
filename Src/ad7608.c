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

#define ADC_FILTER_MAX_DEPTH             64
#define ADC_VOLTAGE_NOISE_NUMBER         10                 // 10 max. points and 10 min. points each
#define ADC_VOLTAGE_TRANSFER_FACTOR     (5.0f / 131072.0f)  // 131072 = 2^17
#define INERTIAL_SENSOR_ZERO_OUTPUT      2.50f              // 2.47 to 2.53
#define CIRCULAR_ANGLE_DEGREE            360.0f             // Unit : degree
#define STANDARD_TRACK_DISTANCE          1435.0f            // Unit : mm
#define PI                               3.1415926f

/* External Variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim7;
extern SPI_HandleTypeDef hspi1;
extern osMutexId ADCSamplingMutexHandle;
extern osSemaphoreId AdcConvertStartSemHandle;
extern osSemaphoreId AdcConvertCompleteSemHandle;
extern TRACK_MEAS_ITEM meas;
extern WORK_MODE workMode;
extern DATA_MODE dataMode;

/* Private Variables ---------------------------------------------------------*/
const  CAL_TBL tbl __attribute__((section(".ARM.__at_0x08060000"))) = CAL_TBL_DATA;
static uint8_t buff[AD7608_DMA_BUFFER_LENGTH];

uint16_t filterDeepth = 0;
static uint16_t noisePtsNumber = 0;
float outputADVal[AD7608_CH_NUMBER]; // channel data for print to VOFA+
static int32_t adc[AD7608_CH_NUMBER]; // just received ADC value (raw ADC data)
static float filteredVol[AD7608_CH_NUMBER]; // filtered ADC data (length = 18 bit)


/* Private function prototypes -----------------------------------------------*/
static void AD7608_RESET(void);
static void AD7608_TRIGGER(void);
static void Delay(uint32_t nCount);
static void movingAverageFilter(const float* xn, float* yn);
static float getNoiseSum(const uint32_t deepth, const uint32_t noisePtsNum, const float* pArray);
static void filterVoltage(void);
static float calibrateADCData(ADC_CAL item, float raw);

/* Formal function definitions -----------------------------------------------*/
void adcTask(void const * argument)
{
  AD7608_RESET();
  
  while(1) {
    osSemaphoreWait(AdcConvertStartSemHandle, osWaitForever);
    //PRINTF("Timer7(5ms) come!\r\n");
    
    AD7608_TRIGGER();
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
    osDelay(1);
    while(AD7608_BUSY) {};
      
    HAL_SPI_Receive_DMA(&hspi1, buff, sizeof(buff));
    osSemaphoreWait(AdcConvertCompleteSemHandle, osWaitForever);
      
    filterVoltage();
  }
}

void startADC(void)
{
  initFilter();
  
  HAL_TIM_Base_Start_IT(&htim7); // start 5ms cyclic timer
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
  Delay(0xF); // TODO : If the running time is insufficient, 
              //        the delay need change to the OS delay here.  
  AD7608_CONVSTA_H;
}

static void Delay(uint32_t nCount) 
{ 
  for(; nCount != 0; nCount--); 
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  //adcCnt++;
  
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

static void filterVoltage(void)
{
  float vol[AD7608_CH_NUMBER];
  
  // ADC data(18 bit) --> voltage (-5V to +5V)
  for (uint32_t i = 0; i < AD7608_CH_NUMBER; i++) {
    //vol[i] = (float)((filteredADC[i] << 14) >> 14) * ADC_VOLTAGE_TRANSFER_FACTOR;
    vol[i] = (float)((int32_t)(adc[i] ^ 0x00020000) - (int32_t)0x00020000) * 
             ADC_VOLTAGE_TRANSFER_FACTOR;
  }
  
  // copy voltage value
  osMutexWait(ADCSamplingMutexHandle, osWaitForever);
  if (filterDeepth != 0) { // execute moving average filter
    // TODO : If other types of filters are used, past ADC sample values 
    //        need to be calculated together, and a certain length of 
    //        old sample values need to be cached. 
    //        Because a simple algorithm of moving average is used here, 
    //        past sample values are not cached for the time being.
    movingAverageFilter(vol, filteredVol);
  }
  else { // do not execute filter
    memcpy((void*)filteredVol, (void*)vol, sizeof(vol));
  }
  osMutexRelease(ADCSamplingMutexHandle);
  
  // copy data to print buffer
  switch (dataMode) {
    case DATA_ADC_RAW:
      memcpy((void*)outputADVal, (void*)adc, sizeof(adc));
      break;
    case DATA_VOL_RAW:
      memcpy((void*)outputADVal, (void*)vol, sizeof(vol));
      break;
    case DATA_VOL_FILTERED:
    default:
      memcpy((void*)outputADVal, (void*)filteredVol, sizeof(filteredVol));
      break;
  }
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
  meas.distance_comp= filteredVol[TRACK_DIST_COMP];
  meas.height_comp  = filteredVol[TRACK_HEIGHT_COMP];
  meas.distance     = filteredVol[TRACK_DISTANCE];
  meas.battery      = filteredVol[TRACK_BATTERY_VOLTAGE] * 2.0f;
  
  // calculate dip angle and track height
  // Angle = arcsin((E0-Eb)/SF)-Theta
  sinRoll = (filteredVol[TRACK_DIP_A1] - filteredVol[TRACK_DIP_0] - 
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

// an implementation of a moving average filter with noise removal
static int32_t filterIndex = -1;
static float filterSum[AD7608_CH_NUMBER] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static float filterBuffer[AD7608_CH_NUMBER][ADC_FILTER_MAX_DEPTH]; // ADC filter data buffer
static float sortArray[ADC_FILTER_MAX_DEPTH];

void initFilter(void)
{
  // initialize index 
  filterIndex = -1;
  
  // initialize noise number 
  switch (filterDeepth) {
    default:
    case 0:
    case 2:
      noisePtsNumber = 0;
      break;
    case 4:
      noisePtsNumber = 1;
      break;
    case 8:
      noisePtsNumber = 2;
      break;
    case 16:
      noisePtsNumber = 3;
      break;
    case 32:
      noisePtsNumber = 5;
      break;
    case 64:
      noisePtsNumber = 10;
      break;
  }
}

static void movingAverageFilter(const float* xn, float* yn)
{
  float noiseSum;
  
  if(filterIndex == -1) { // init moving average filter
    for(uint16_t i = 0; i < AD7608_CH_NUMBER; i++) {
      for(uint16_t j = 0; j < filterDeepth; j++) {
        filterBuffer[i][j] = xn[i];
      }
      filterSum[i] = xn[i] * filterDeepth;
      yn[i] = xn[i];
    }
    filterIndex = 0;
  }
  else {
    for(uint16_t i = 0; i < AD7608_CH_NUMBER; i++) {
      filterSum[i] -= filterBuffer[i][filterIndex];
      filterBuffer[i][filterIndex] = xn[i];
      filterSum[i] += xn[i];
      noiseSum = getNoiseSum(filterDeepth, noisePtsNumber, filterBuffer[i]);
      yn[i] = (filterSum[i] - noiseSum) / (filterDeepth - noisePtsNumber * 2);
    }
    filterIndex++;
    if (filterIndex >= filterDeepth) filterIndex = 0;
  }
}

static float getNoiseSum(const uint32_t deepth, const uint32_t noisePtsNum, const float* pArray)
{
  assert_param(deepth <= 64);
  assert_param(noisePtsNum <= 10);
  
  float temp = 0.0f;
  float noiseSum = 0.0f;
  uint32_t exchanged = 1;
  
  memcpy((void*)sortArray, (void*)pArray, deepth * sizeof(float));
  
  // bubble sort
  for(uint32_t i = 0; (i < deepth - 1) && exchanged; i++) {
    exchanged = 0;
    for(uint32_t j = 0; j < deepth - 1 - i; j++) {
      if (sortArray[j] > sortArray[j + 1]) {
        temp = sortArray[j];
        sortArray[j] = sortArray[j + 1];
        sortArray[j + 1] = temp;
        exchanged = 1;
      }
    }
  }
  
  // calculate noise points sum
  for(uint32_t i = 0; i < noisePtsNum; i++) {
    noiseSum += sortArray[i];
  }
  for(uint32_t i = deepth - 1; i > deepth - noisePtsNum - 1; i--) {
    noiseSum += sortArray[i];
  }
  
  return noiseSum;
}
