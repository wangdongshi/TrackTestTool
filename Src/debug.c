/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : debug.c
 * Project  : Track Test Tool
 * Date     : 2023/7/3
 * Author   : WangYu
 *
 **********************************************************************/
#include <stdarg.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
static char buffer[200];

void PRINTF2(const char *format,...) // UART2 for RS485/RS232/USB/Ethernet
{
  unsigned short length;
  va_list args;
  va_start(args, format);
  length = vsnprintf((char*)buffer, sizeof(buffer) + 1, (char*)format, args);
  va_end(args);
  
  HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, length);
}

int fputc(int ch, FILE *stream)
{
  assert_param(ch <= 0x1FF);
  
  while((USART1->SR & UART_FLAG_TXE) != UART_FLAG_TXE);
  USART1->DR = (ch & (uint16_t)0x01FF);
  
	return ((uint16_t)(ch));
}

#ifndef DEBUG

void PRINTF(const char *format, ...)
{
}

#endif
