/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : redirection.c
 * Project  : Track Test Tool
 * Date     : 2023/7/3
 * Author   : WangYu
 *
 **********************************************************************/
#include <stdio.h>
#include "stm32f4xx_hal.h"

int fputc(int ch, FILE *stream)
{
  assert_param(ch <= 0x1FF);
  
  while((USART1->SR & USART_FLAG_TXE) != USART_FLAG_TXE);
  USART1->DR = (ch & (uint16_t)0x01FF);
  
	return ((uint16_t)(ch));
}
