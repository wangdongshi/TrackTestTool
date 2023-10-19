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

#define AD7608_CH_NUMBER 8

typedef enum {
  TRACK_DIST_COMPENSATION = 0,
  TRACK_HEIGHT,
  TRACK_DIP_0,
  TRACK_DIP_A1,
  TRACK_DISTANCE,
  TRACK_RESERVE,
  TRACK_BATTERY_VOLTAGE,
  NONE
} ADC_CH;

void ADC_Task(void const * argument);
void startADC(void);
void clearADCFilterData(void);
void prepareSensorData(void);

#endif // __ADC7608_H__
