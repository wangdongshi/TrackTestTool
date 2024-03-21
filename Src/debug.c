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

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;

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

#endif /* DEBUG */
