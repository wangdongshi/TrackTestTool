/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : flash.h
 * Project  : Track Test Tool
 * Date     : 2024/11/26
 * Author   : WangYu
 *
 **********************************************************************/
#ifndef __FLASH_H__
#define __FLASH_H__

# define FLASH_OPT_BANK             1
# define FLASH_OPT_MIN_SECTOR       5
# define FLASH_OPT_MAX_SECTOR       11
# define PAGE_NUM_PER_SECTOR        128
# define BYTE_NUM_PER_PAGE          1024
# define BYTE_NUM_PER_SECTOR        (PAGE_NUM_PER_SECTOR * BYTE_NUM_PER_PAGE)
# define FLASH_MIN_SECTOR_ADDRESS   0x08020000

int EraseSector(const unsigned short sectorNo);
int WriteSectorData(const unsigned short sectorNo, unsigned char* data, const unsigned short size);
int WriteSectorWithCRC16(const unsigned short sectorNo, unsigned char* data, const unsigned short size);
int CheckSectorWithCRC16(const unsigned short sectorNo, const unsigned short size);

#endif // __FLASH_H__
