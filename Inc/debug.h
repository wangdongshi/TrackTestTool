/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : debug.h
 * Project  : Track Test Tool
 * Date     : 2023/7/3
 * Author   : WangYu
 *
 **********************************************************************/
#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdio.h>

#define ASSERT
#define DEBUG

#ifdef DEBUG

#define PRINTF   printf

#else

void PRINTF(const char *format, ...);

#endif /* DEBUG */

#endif // __DEBUG_H__
