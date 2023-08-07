/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : crc.h
 * Project  : Track Test Tool
 * Date     : 2023/8/7
 * Author   : WangYu
 *
 **********************************************************************/
#ifndef __CRC_H__
#define __CRC_H__

#include <stdint.h>

uint16_t calcCRC16(uint8_t *pbuf, uint8_t nlen);

#endif // __CRC_H__
