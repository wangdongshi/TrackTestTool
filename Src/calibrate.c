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
const CAL_TBL initCalTbl __attribute__((section(".ARM.__at_0x08060000"))) = CAL_TBL_DATA;
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
    memcpy((void*)&calTbl, (void*)&initCalTbl, sizeof(CAL_TBL));
  }
  
  return result;
}

int insertCalibrateRecord(
  const unsigned short type,
  const unsigned short seq,
  const float standVal,
  const float calibVal
)
{
  return 1;
}

int writeCalibrateData(void)
{
  return writeSectorWithCRC16(CAL_DATA_SECTOR, (unsigned char*)&calTbl, sizeof(CAL_TBL));
}

int eraseCalibrateData(void)
{
  memcpy((void*)&calTbl, (void*)&initCalTbl, sizeof(CAL_TBL));
  return writeSectorWithCRC16(CAL_DATA_SECTOR, (unsigned char*)&calTbl, sizeof(CAL_TBL));
}
