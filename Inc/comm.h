/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : comm.h
 * Project  : Track Test Tool
 * Date     : 2023/7/31
 * Author   : WangYu
 *
 **********************************************************************/
#ifndef __COMM_H__
#define __COMM_H__

#include <stdint.h>

typedef enum {
  COMM_CHANGE_TO_NORMAL_MODE = 0, // change mode from pre-work to work
  COMM_SET_MILAGE,
  COMM_SET_DISPLAY_MODE,
  COMM_SET_TRIGGER_MODE,
  COMM_SET_GYRO_ZERO_DRIFT_MODE,
  COMM_SET_OUTPUT_ADC_DATA_MODE,
  COMM_SET_FILTER_MODE,
  COMM_NUMBER_MAX
} COMM_TYPE;

typedef enum {
  MODE_PRE_WORK = 0,
  MODE_NORMAL_WORK,
  MODE_NUMBER_MAX
} WORK_MODE;

typedef enum {
  TRIG_CYCLIC = 0,
  TRIG_ENCODER,
} TRIG_MODE;

typedef enum {
  GYRO_OFFSET_HOLD = 0,
  GYRO_OFFSET_REMOVED,
  GYRO_MODE_NUMBER_MAX
} GYRO_MODE;

typedef enum {
  DATA_ADC_RAW = 0,
  DATA_VOL_RAW,
  DATA_VOL_FILTERED,
  DATA_MEASURE = 0,
  DATA_MODE_NUMBER_MAX
} DATA_MODE;

typedef enum {
  FILTER_SOFT_ON = 0,
  FILTER_SOFT_OFF,
  FILTER_MODE_NUMBER_MAX
} FILTER_MODE;

typedef enum {
  OUTPUT_JUSTFLOAT = 0,
  OUTPUT_FIREWATER,
  OUTPUT_NUMBER_MAX
} OUTPUT_MODE;

typedef struct {
  uint32_t  length;
  uint16_t  type;
  float     startPoint;  // Unit : mm
  uint16_t  outFormat;
  uint16_t  gyroMode;
  uint16_t  trigMode;
  uint16_t  dataMode;
  uint16_t  filterSW;
} COMM_MSG;

void startCommunication(void);
void PRINTF2(const char *format, ...);
void TRACEFLOAT(float* input, unsigned short length);
void uart2RxCallback(void);

#endif // __COMM_H__
