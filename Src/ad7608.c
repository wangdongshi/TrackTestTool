/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : ad7608.c
 * Project  : Track Test Tool
 * Date     : 2023/7/3
 * Author   : WangYu
 *
 **********************************************************************/
 
/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#if 1

/* Private macro -------------------------------------------------------------*/

/* STA  : PC04
 * STB  : short to PC04
 * SCLK : PA05
 * RST  : PA07
 * CS   : PA04
 * BUSY : PC05
 * DOA  : PA06
 */

/* Output definition */
#define AD7608_CONVSTA_H   LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4)
#define AD7608_CONVSTA_L LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4)
#define AD7608_SCLK_H      LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5)
#define AD7608_SCLK_L    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5)
#define AD7608_RESET_H     LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define AD7608_RESET_L   LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define AD7608_CS_H        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4)
#define AD7608_CS_L      LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4)

#define AD7608_BUSY       LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_5)
#define AD7608_DOUTA      LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6)

/* Private Variables ---------------------------------------------------------*/
uint32_t adc_data[8]; // data length = 18 bit

/* Private function prototypes -----------------------------------------------*/
void GPIO_Configuration(void);
void AD7608_SETOS(uint8_t osv);
void AD7608_RESET(void);
void AD7608_STARTCONV(void);
void Delay(uint32_t nCount);
uint32_t AD7608_READDATA(void);

/* Formal function definitions -----------------------------------------------*/
int ADC_Task(void) 
{
  // initialize AD7608 GPIO
  GPIO_Configuration();

  AD7608_RESET();
  
  AD7608_CONVSTA_H;
  
  // trigger ADC transfer and read data 
  while (1) {
    if(!AD7608_BUSY) {
      AD7608_CS_L;
      for(uint8_t i = 0; i < 8; i++) {
        adc_data[i] = AD7608_READDATA();
        PRINTF("CH%d : %d, \t", i, adc_data[i]);
        if (i == 3 || i == 7) PRINTF("\r\n");
        if (i == 7) PRINTF("\r\n");
      }
      AD7608_CS_H;
      osDelay(2000);
      AD7608_STARTCONV();
      while(AD7608_BUSY);
    }
  } 
}

void AD7608_STARTCONV(void)
{
  AD7608_CONVSTA_L;
  Delay(0xF);
  AD7608_CONVSTA_H;
}

uint32_t AD7608_READDATA(void)
{
  uint32_t usData = 0;
  for (uint8_t i = 0; i < 18; i++) {
    AD7608_SCLK_L;
    usData = usData << 1;
    if(AD7608_DOUTA) usData |= 0x0001;
    AD7608_SCLK_H;
  }
  return usData;		
}

void AD7608_RESET(void)
{
  AD7608_RESET_H;
  Delay(0xFF);
  AD7608_RESET_L;
}

void GPIO_Configuration(void) 
{
  LL_GPIO_InitTypeDef GPIO_InitStructure;
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  
  /* STA  : PC04
   * STB  : short to PC04
   * SCLK : PA05
   * RST  : PA07
   * CS   : PA04
   * BUSY : PC05
   * DOA  : PA06
   */
  
  /* Set output I/O */
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4);
  
  GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT;
  
  GPIO_InitStructure.Pin = LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_7;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = LL_GPIO_PIN_4;
  LL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* Set input I/O */
  GPIO_InitStructure.Mode = LL_GPIO_MODE_INPUT;
  
  GPIO_InitStructure.Pin = LL_GPIO_PIN_6;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = LL_GPIO_PIN_5;
  LL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Delay(uint32_t nCount) 
{ 
  for(; nCount != 0; nCount--); 
}

#else

void ADC_Task(void const * argument)
{
  while(1) {
    PRINTF("ADC is Working...\r\n");
    osDelay(1000);
  }
}

#endif
