/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : calibrate.c
 * Project  : Track Test Tool
 * Date     : 2024/12/11
 * Author   : WangYu
 *
 **********************************************************************/
 
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "flash.h"
#include "calibrate.h"

/* Private macro -------------------------------------------------------------*/
#define CAL_DATA_SECTOR    (FLASH_OPT_MAX_SECTOR)
#define CAL_DATA_ADDRESS   (FLASH_MIN_SECTOR_ADDRESS + (CAL_DATA_SECTOR - FLASH_OPT_MIN_SECTOR) * BYTE_NUM_PER_SECTOR)

/* External Variables --------------------------------------------------------*/

/* Private Variables ---------------------------------------------------------*/
CAL_TBL tmpTbl = CAL_TBL_DATA;
CAL_TBL calTbl;

/* Private function prototypes -----------------------------------------------*/

/* Formal function definitions -----------------------------------------------*/
int initCalibrateData(void)
{
  int result = checkSectorWithCRC16(CAL_DATA_SECTOR, sizeof(CAL_TBL));
  
  if (result) {
    memcpy((void*)&calTbl, (void*)CAL_DATA_ADDRESS, sizeof(CAL_TBL));
  }
  else {
    memcpy((void*)&calTbl, (void*)&tmpTbl, sizeof(CAL_TBL));
  }
  
  return result;
}

void insertCalADCRecord(
  const ADC_CAL type,
  const unsigned short seq,
  const float standVal,
  const float calibVal
)
{
  CAL_PAIR pair = {standVal, calibVal};
  void* targetAddr = (void*)((unsigned int)&tmpTbl + 
                     (type * CAL_POINTS * sizeof(CAL_PAIR)) + 
                     (seq * sizeof(CAL_PAIR)));
  memcpy(targetAddr, &pair, sizeof(CAL_PAIR));
}

void updateCalWheelDiameter(const float diameter)
{
  void* targetAddr = (void*)((unsigned int)&tmpTbl + 
                     sizeof(CAL_PAIR) * CAL_ITEMS * CAL_POINTS);
  memcpy(targetAddr, &diameter, sizeof(float));
}

void updateCalGyroScale(const unsigned short no, const float scale)
{
  void* targetAddr = (void*)((unsigned int)&tmpTbl + 
                     sizeof(CAL_PAIR) * CAL_ITEMS * CAL_POINTS +
                     sizeof(float) /*wheel diameter*/ + sizeof(float) * no);
  memcpy(targetAddr, &scale, sizeof(float));
}

void updateCalTiltScale(const float scale)
{
  void* targetAddr = (void*)((unsigned int)&tmpTbl + 
                     sizeof(CAL_PAIR) * CAL_ITEMS * CAL_POINTS +
                     sizeof(float) * 3);
  memcpy(targetAddr, &scale, sizeof(float));
}

int writeCalibrateData(void)
{
  return writeSectorWithCRC16(CAL_DATA_SECTOR, (unsigned char*)&tmpTbl, sizeof(CAL_TBL));
}

int eraseCalibrateData(void)
{
  return writeSectorWithCRC16(CAL_DATA_SECTOR, (unsigned char*)&tmpTbl, sizeof(CAL_TBL));
}
