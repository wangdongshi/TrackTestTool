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

int EraseSector(const unsigned short sectorNo);
int WriteSectorData(const unsigned short sectorNo, unsigned char* data, const unsigned short size);
int WriteSectorWithCRC16(const unsigned short sectorNo, unsigned char* data, const unsigned short size);
int CheckSectorWithCRC16(const unsigned short sectorNo, const unsigned short size);

#endif // __FLASH_H__
