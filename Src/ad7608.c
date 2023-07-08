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
#define AD7608_RESET_H     LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define AD7608_RESET_L   LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7)
#define AD7608_BUSY       LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_5)

/* External Variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

/* Private Variables ---------------------------------------------------------*/
uint8_t  buff[18];
uint32_t adc_data[8]; // data length = 18 bit

/* Private function prototypes -----------------------------------------------*/
void AD7608_RESET(void);
void AD7608_TRIGGER(void);
void Delay(uint32_t nCount);

/* Formal function definitions -----------------------------------------------*/
int ADC_Task(void) 
{
  AD7608_RESET();
  
  do {
    AD7608_TRIGGER();
    while(AD7608_BUSY) {};
    HAL_SPI_Receive_DMA(&hspi1, buff, 18);
    osDelay(2000);
  }
  while(1);
}

void AD7608_RESET(void)
{
  AD7608_RESET_H;
  Delay(0xFF);
  AD7608_RESET_L;
}

void AD7608_TRIGGER(void)
{
  AD7608_CONVSTA_L;
  Delay(0xF);
  AD7608_CONVSTA_H;
}

void Delay(uint32_t nCount) 
{ 
  for(; nCount != 0; nCount--); 
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  // Map the ADC data from DMA buffer
  adc_data[0] = ((buff[ 0] & 0xFF) << 10) + (buff[ 1] << 2) + (buff[ 2] >> 6);
  adc_data[1] = ((buff[ 2] & 0x3F) << 12) + (buff[ 3] << 4) + (buff[ 4] >> 4);
  adc_data[2] = ((buff[ 4] & 0x0F) << 14) + (buff[ 5] << 6) + (buff[ 6] >> 2);
  adc_data[3] = ((buff[ 6] & 0x03) << 16) + (buff[ 7] << 8) + (buff[ 8] >> 0);
  adc_data[4] = ((buff[ 9] & 0xFF) << 10) + (buff[10] << 2) + (buff[11] >> 6);
  adc_data[5] = ((buff[11] & 0x3F) << 12) + (buff[12] << 4) + (buff[13] >> 4);
  adc_data[6] = ((buff[13] & 0x0F) << 14) + (buff[14] << 6) + (buff[15] >> 2);
  adc_data[7] = ((buff[15] & 0x03) << 16) + (buff[16] << 8) + (buff[17] >> 0);
  
  // Print raw DMA data
  PRINTF("DMA_BUF[18] = 0x");
  for (uint8_t i = 0; i < 18; i++) PRINTF("%X", buff[i]);
  PRINTF("\r\n");
  
  // Print ADC data
  for(uint8_t i = 0; i < 8; i++) {
    PRINTF("CH%d : %d, \t", i, adc_data[i]);
    if (i == 3 || i == 7) PRINTF("\r\n");
    if (i == 7) PRINTF("\r\n");
  }
}
