/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : ad7608.h
 * Project  : Track Test Tool
 * Date     : 2023/7/3
 * Author   : WangYu
 *
 **********************************************************************/
#ifndef __ADC7608_H__
#define __ADC7608_H__

#include "stm32f4xx_hal.h"

#define AD7608_CH_NUMBER            8
#define AD7608_CH_DATA_RESOLUTION   18
#define AD7608_DMA_BUFFER_LENGTH    (AD7608_CH_NUMBER * AD7608_CH_DATA_RESOLUTION / 8)

typedef enum {
  TRACK_REVERSE1 = 0,
  TRACK_REVERSE2,
  TRACK_REVERSE3,
  TRACK_DIP_A1,
  TRACK_HEIGHT_COMP,
  TRACK_DIST_COMP,
  TRACK_DISTANCE,
  NONE
} ADC_CH;

void ADC_Task(void const * argument);
void startADC(void);
void initFilter(void);
void prepareSensorData(void);

#endif // __ADC7608_H__
