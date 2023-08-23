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
  COMM_RESET_MILAGE,
  COMM_CHANGE_OUTPUT_FORMAT,
  COMM_REMOVE_GYRO_ZERO_DRIFT,
  COMM_NUMBER_MAX
} COMM_TYPE;

typedef enum {
  MODE_PRE_WORK = 0,
  MODE_NORMAL_WORK,
  MODE_NUMBER_MAX
} WORK_MODE;

typedef enum {
  GYRO_OFFSET_HOLD = 0,
  GYRO_OFFSET_REMOVED,
  GYRO_MODE_NUMBER_MAX
} GYRO_MODE;

typedef enum {
  OUTPUT_JUSTFLOAT = 0,
  OUTPUT_FIREWATER,
  OUTPUT_NUMBER_MAX
} OUTPUT_MODE;

typedef struct {
  uint32_t  length;
  uint16_t  type;
  float     startPoint;  // Unit : mm
  float     startAngle1; // gyro1
  float     startAngle2; // gyro2
  uint16_t  outFormat;
  uint16_t  gyroMode;
} COMM_MSG;

void startCommunication(void);
void PRINTF2(const char *format, ...);
void TRACEFLOAT(float* input, unsigned short length);
void uart2RxCallback(void);

#endif // __COMM_H__
