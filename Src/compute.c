/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : compute.c
 * Project  : Track Test Tool
 * Date     : 2024/3/22
 * Author   : WangYu
 *
 **********************************************************************/
 
/* Private Includes */
#include <string.h>
#include "debug.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "compute.h"
#include "ad7608.h"
#include "calibrator.h"
#include "comm.h"

/* Private Macro definition */
#define ADC_FILTER_MAX_DEPTH             64
#define ADC_VOLTAGE_NOISE_NUMBER         10                 // 10 max. points and 10 min. points each
#define ADC_VOLTAGE_TRANSFER_FACTOR     (5.0f / 131072.0f)  // 131072 = 2^17

/* External Variables */
extern osMutexId ADCSamplingMutexHandle;
extern uint8_t buff[2][AD7608_DMA_BUFFER_LENGTH];
extern uint8_t buffIndex;
extern float filteredVol[AD7608_CH_NUMBER]; // Can not define here!!
extern DATA_MODE dataMode;

/* Private Variables */
uint32_t rollADC = 0;
uint16_t filterDeepth = 0;
static uint16_t noisePtsNumber = 0;
float vol[AD7608_CH_NUMBER] = {0.0f};
float outputADVal[AD7608_CH_NUMBER] = {0.0f}; // channel data for print to VOFA+
int32_t adc[AD7608_CH_NUMBER]; // just received ADC value (raw ADC data)

/* Private function prototypes */
static void  cacheADCData(void);
static void  filterVoltage(void);
static void  movingAverageFilter(const float* xn, float* yn);
static float getNoiseSum(const uint32_t deepth, const uint32_t noisePtsNum, const float* pArray);

/* Formal function definitions */
void pretreatADCData(void)
{
  cacheADCData();
  
  osMutexWait(ADCSamplingMutexHandle, osWaitForever);
  
  filterVoltage();
  
  osMutexRelease(ADCSamplingMutexHandle);
}

static void filterVoltage(void)
{
  // ADC data(18 bit) --> voltage (-5V to +5V)
  for (uint32_t i = 0; i < AD7608_CH_NUMBER; i++) {
    //vol[i] = (float)((filteredADC[i] << 14) >> 14) * ADC_VOLTAGE_TRANSFER_FACTOR;
    vol[i] = (float)((int32_t)(adc[i] ^ 0x00020000) - (int32_t)0x00020000) * 
             ADC_VOLTAGE_TRANSFER_FACTOR;
  }
  
  // backup roll ADC data
  rollADC = adc[TRACK_DIP_A1];
  
  // filter process
  if (filterDeepth != 0) { // execute moving average filter
    // TODO : If other types of filters are used, past ADC sample values 
    //        need to be calculated together, and a certain length of 
    //        old sample values need to be cached. 
    //        Because a simple algorithm of moving average is used here, 
    //        past sample values are not cached for the time being.
    movingAverageFilter(vol, filteredVol);
  }
  
  // copy data to print buffer
  if (dataMode == DATA_ADC_RAW) {
    memcpy((void*)outputADVal, (void*)adc, sizeof(adc));
  }
  else if (dataMode == DATA_VOL_RAW || filterDeepth == 0) {
    memcpy((void*)outputADVal, (void*)vol, sizeof(vol));
  }
  else {
    memcpy((void*)outputADVal, (void*)filteredVol, sizeof(filteredVol));
  }
  
  // copy filtered data to voltage buffer
  if (filterDeepth != 0) memcpy((void*)vol, (void*)filteredVol, sizeof(filteredVol));
}

static void cacheADCData(void)
{
  //adcCnt++;
  uint8_t i = !buffIndex;
  
  // Map the ADC data from DMA buffer
  adc[0] = ((buff[i][ 0] & 0xFF) << 10) + (buff[i][ 1] << 2) + (buff[i][ 2] >> 6);
  adc[1] = ((buff[i][ 2] & 0x3F) << 12) + (buff[i][ 3] << 4) + (buff[i][ 4] >> 4);
  adc[2] = ((buff[i][ 4] & 0x0F) << 14) + (buff[i][ 5] << 6) + (buff[i][ 6] >> 2);
  adc[3] = ((buff[i][ 6] & 0x03) << 16) + (buff[i][ 7] << 8) + (buff[i][ 8] >> 0);
  adc[4] = ((buff[i][ 9] & 0xFF) << 10) + (buff[i][10] << 2) + (buff[i][11] >> 6);
  adc[5] = ((buff[i][11] & 0x3F) << 12) + (buff[i][12] << 4) + (buff[i][13] >> 4);
  adc[6] = ((buff[i][13] & 0x0F) << 14) + (buff[i][14] << 6) + (buff[i][15] >> 2);
  adc[7] = ((buff[i][15] & 0x03) << 16) + (buff[i][16] << 8) + (buff[i][17] >> 0);
  
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

static float getNoiseSum(const uint32_t deepth, const uint32_t noiseNum, const float* pArray)
{
  assert_param(deepth <= 64);
  assert_param(noiseNum <= 10);
  
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
  for(uint32_t i = 0; i < noiseNum; i++) {
    noiseSum += sortArray[i];
  }
  for(uint32_t i = deepth - noiseNum; i < deepth; i++) {
    noiseSum += sortArray[i];
  }
  
  return noiseSum;
}
