/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : flash.c
 * Project  : Track Test Tool
 * Date     : 2024/11/26
 * Author   : WangYu
 *
 **********************************************************************/
 
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "flash.h"

/* Private macro -------------------------------------------------------------*/
# define FLASH_FLAG    FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | \
                       FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR

/* Private typedef -----------------------------------------------------------*/

/* External Variables --------------------------------------------------------*/

/* Private Variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Formal function definitions -----------------------------------------------*/
int EraseSector(const unsigned short sectorNo)
{
  assert_param(sectorNo > 4 && sectorNo < 11);
  
  FLASH_EraseInitTypeDef initFlash = {FLASH_TYPEERASE_SECTORS, FLASH_BANK_1, sectorNo, 1, VOLTAGE_RANGE_3};
  uint32_t error = 0xFFFFFFFFU;
  
  HAL_StatusTypeDef status;
  
  if (HAL_FLASH_Unlock() != HAL_OK) return 0;
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG);
  status = HAL_FLASHEx_Erase(&initFlash, &error);
  if (HAL_FLASH_Lock() != HAL_OK) return 0;
  
  if (status != HAL_OK) {
    if (HAL_FLASH_Unlock() != HAL_OK) return 0;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG);
    status = HAL_FLASHEx_Erase(&initFlash, &error);
    if (HAL_FLASH_Lock() != HAL_OK) return 0;
    return (status == HAL_OK);
  }
  else {
    return 1;
  }
}
