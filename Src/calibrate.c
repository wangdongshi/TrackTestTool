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
CAL_TBL tbl = CAL_TBL_DATA;

/* Private function prototypes -----------------------------------------------*/

/* Formal function definitions -----------------------------------------------*/
void initCalibrateData(void)
{
  if (CheckSectorWithCRC16(CAL_DATA_SECTOR, sizeof(CAL_TBL))) {
    memcpy(&tbl, (void*)CAL_DATA_ADDRESS, sizeof(CAL_TBL));
  }
}
