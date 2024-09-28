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
#include "stm32f407xx.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "cmsis_os.h"
#include "compute.h"
#include "ad7608.h"
#include "calibrator.h"
#include "comm.h"

#include "../Drivers/CMSIS/DSP_Lib/Source/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c"
#include "../Drivers/CMSIS/DSP_Lib/Source/FilteringFunctions/arm_biquad_cascade_df1_f32.c"

/* Private Macro definition */
#define USE_LOW_PASS_FILTER                0

#define DENOISE_BUF_DEPTH                  10
#define ADC_VOLTAGE_UPPER_LIMIT            4.9f
#define ADC_VOLTAGE_LOWER_LIMIT            0.1f

#define STAGE_NUMBER                       2                   // The number of 2nd order biquad filters
#define DIP_VOL_DATA_BUFFER_LENGTH         9                   // internal time = 50 ms
#define ADC_VOLTAGE_TRANSFER_FACTOR        (5.0f / 131072.0f)  // 131072 = 2^17
#define IIR_SCALE_VALUE                    (0.0000098458979119984636f * 0.0000098126110186609252f) // cutoff = 1Hz

#define ULTRA_HIGH_BUFFER_LENGTH           40
#define ULTRA_HIGH_JUDGE_LENGTH            20     // TODO : must test in actual track
#define ULTRA_HIGH_SURGE_LIMIT             50.0f  // TODO : must test in actual track
#define ULTRA_HIGH_SURGE_REPLACE_LENGTH    350    // TODO : must test in actual track

// cutoff = 1Hz
const float iirCoeffs32LP[5 * STAGE_NUMBER] = {                                                                                 
	1.0f,  2.0f,  1.0f,  1.9951632412838627f,  -0.99520262487551059f,
	1.0f,  2.0f,  1.0f,  1.9884180173746586f,  -0.98845726781873322f                                                                                                
};

/*
#define IIR_SCALE_VALUE                  (0.0000392889647183107440f * 0.0000390248389489221170f)   // cutoff = 2Hz
// cutoff = 2Hz
const float iirCoeffs32LP[5 * STAGE_NUMBER] = {                                                                                 
	1.0f,  2.0f,  1.0f,  1.9902712416617285f,  -0.99042839752060186f,
	1.0f,  2.0f,  1.0f,  1.9768913542871200f,  -0.97704745364291579f                                                                                                
};
*/

/* External Variables */
extern osMutexId ADCSamplingMutexHandle;
extern osMutexId EncoderDelayMutexHandle;
extern uint8_t buff[2][AD7608_DMA_BUFFER_LENGTH];
extern uint8_t buffIndex;
extern float filteredVol[AD7608_CH_NUMBER]; // Can not define here!!
extern WORK_MODE workMode;
extern DATA_MODE dataMode;

/* Private Variables */
uint32_t rollADC = 0;
uint16_t filterDeepth = 2;
uint16_t measBackupFlg = 0;
float vol[AD7608_CH_NUMBER] = {0.0f};
float volDelayBuf[AD7608_CH_NUMBER] = {0.0f};
float outputADVal[AD7608_CH_NUMBER] = {0.0f}; // channel data for print to VOFA+
int32_t adc[AD7608_CH_NUMBER]; // just received ADC value (raw ADC data)
float ultraHighReplaceVal = 0.0f;
uint16_t ultraHighReplaceCnt = 0;
static float ultraHighBuf[ULTRA_HIGH_BUFFER_LENGTH] = {0.0f};
static float ultraHighCompBuf[ULTRA_HIGH_BUFFER_LENGTH] = {0.0f};
static arm_biquad_casd_df1_inst_f32 S;
static float iirInputBuf[DIP_VOL_DATA_BUFFER_LENGTH]  =  {0.0f};
static float iirOutputBuf[DIP_VOL_DATA_BUFFER_LENGTH] =  {0.0f};
static float iirStateF32[4 * STAGE_NUMBER]; // state buffer

/* Private function prototypes */
static void  cacheADCData(void);
static void  treatVolData(void);
static void  removeNoise(void);
static void  backupUltraHigh(void);
static void  replaceUltraHigh(void);
static void  backupMeasData(void);
float filterRollVol(float);

/* Formal function definitions */
void pretreatADCData(void)
{
  cacheADCData();
  
  osMutexWait(ADCSamplingMutexHandle, osWaitForever);
  treatVolData();
  osMutexRelease(ADCSamplingMutexHandle);
  
  backupUltraHigh();
  replaceUltraHigh();
  backupMeasData();
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

static void treatVolData(void)
{
  // ADC data(18 bit) --> voltage (-5V to +5V)
  for (uint32_t i = 0; i < AD7608_CH_NUMBER; i++) {
    vol[i] = (float)((int32_t)(adc[i] ^ 0x00020000) - (int32_t)0x00020000) * ADC_VOLTAGE_TRANSFER_FACTOR;
  }
  
  // backup roll ADC data
  rollADC = adc[TRACK_DIP_A1];
  
  // remove noise point
  if (workMode == MODE_NORMAL_WORK) removeNoise();
  
  // filter process
  memcpy((void*)filteredVol, (void*)vol, sizeof(vol));
#if USE_LOW_PASS_FILTER
  if (filterDeepth != 0) {
    filteredVol[TRACK_DIP_A1] = filterRollVol(vol[TRACK_DIP_A1]);
  }
#endif
  
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

static float denoiseBuf[AD7608_CH_NUMBER][DENOISE_BUF_DEPTH]; // denoise data buffer
static void removeNoise(void)
{
  for (uint32_t i = 0; i < AD7608_CH_NUMBER; i++) {
		if (vol[i] < ADC_VOLTAGE_LOWER_LIMIT || vol[i] > ADC_VOLTAGE_UPPER_LIMIT) {
			vol[i] = denoiseBuf[i][DENOISE_BUF_DEPTH - 1];
		}
    // shift old denoise buffer data
    for (uint32_t j = 1; j < DENOISE_BUF_DEPTH - 1; j++) denoiseBuf[i][j] = denoiseBuf[i][j - 1];
    // add current sampling voltage data
    denoiseBuf[i][DENOISE_BUF_DEPTH - 1] = vol[i];
  }
}

static void backupUltraHigh(void)
{
  size_t size = (ULTRA_HIGH_BUFFER_LENGTH - 1) * sizeof(float);
  memcpy((void*)&ultraHighBuf[0],     (void*)&ultraHighBuf[1],     size);
  memcpy((void*)&ultraHighCompBuf[0], (void*)&ultraHighCompBuf[1], size);
  
  ultraHighBuf[ULTRA_HIGH_BUFFER_LENGTH - 1]     = vol[TRACK_DIP_A1];
  ultraHighCompBuf[ULTRA_HIGH_BUFFER_LENGTH - 1] = vol[TRACK_HEIGHT_COMP];
}

static void  replaceUltraHigh(void)
{
  // TODO : Theoretically, the mark of super-high replacement needs to be protected by mutex, 
  //        but it is not protected here for the time being.
  if (ultraHighReplaceCnt == 0) {
    float ultraHighCompDiff = ultraHighCompBuf[ULTRA_HIGH_BUFFER_LENGTH - 1] - 
      ultraHighCompBuf[ULTRA_HIGH_BUFFER_LENGTH - ULTRA_HIGH_JUDGE_LENGTH];
    if (fabs(ultraHighCompDiff) > ULTRA_HIGH_SURGE_LIMIT) {
      ultraHighReplaceCnt = ULTRA_HIGH_SURGE_REPLACE_LENGTH;
      ultraHighReplaceVal = ultraHighBuf[ULTRA_HIGH_BUFFER_LENGTH - ULTRA_HIGH_JUDGE_LENGTH];
    }
  }
  else {
    ultraHighReplaceCnt--;
  }
}
  
static void backupMeasData(void)
{
  osMutexWait(EncoderDelayMutexHandle, osWaitForever);
  
  if (measBackupFlg != 0) {
    memcpy(&volDelayBuf[0], &vol[0], AD7608_CH_NUMBER * sizeof(float));
    measBackupFlg = 0;
  }
  
  osMutexRelease(EncoderDelayMutexHandle);
}

void initFilter(void)
{
  // clear denoise buffer
  for (uint32_t i = 0; i < AD7608_CH_NUMBER; i++) {
    for (int j = 0; j < DENOISE_BUF_DEPTH; j++) {
      denoiseBuf[i][j] = 2.5f;
    }
  }
  
  // clear filter buffer
  for (uint32_t i = 0; i < DIP_VOL_DATA_BUFFER_LENGTH; i++) {
    iirInputBuf[i]  = 1.65f;
    iirOutputBuf[i] = 1.65f;
  }
  
  // initialize biquad state
	arm_biquad_cascade_df1_init_f32(&S, STAGE_NUMBER, (float*)&iirCoeffs32LP[0], (float*)&iirStateF32[0]);
}

float filterRollVol(float inVol)
{
	float *inputF32  = &iirInputBuf[0];
  float *outputF32 = &iirOutputBuf[0];
  
  // shift buffer and push new voltage data
  size_t copySize = (DIP_VOL_DATA_BUFFER_LENGTH - 1) * sizeof(float);
  memcpy(inputF32,  (inputF32  + 1), copySize);
  memcpy(outputF32, (outputF32 + 1), copySize);
  inputF32[DIP_VOL_DATA_BUFFER_LENGTH - 1] = inVol;
  
  // filter process
	arm_biquad_cascade_df1_f32(&S, inputF32, outputF32, 1);
  
  return iirOutputBuf[0] * IIR_SCALE_VALUE;
}
