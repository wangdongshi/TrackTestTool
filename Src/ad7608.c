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
#include "debug.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#if 0
#define AD7608OS0_H LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_13)
#define AD7608OS0_L LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_13)
#define AD7608OS1_H LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_14)
#define AD7608OS1_L LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_14)
#define AD7608OS2_H LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15)
#define AD7608OS2_L LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15)

#define AD7608_CONVST_A_H LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1)
#define AD7608_CONVST_A_L LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1)
#define AD7608_CONVST_B_H LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2)
#define AD7608_CONVST_B_L LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2)

#define AD7608_SCLK_H LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0)
#define AD7608_SCLK_L LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0)
#define AD7608_RESET_H LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1)
#define AD7608_RESET_L LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1)
#define AD7608_CS_H LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2)
#define AD7608_CS_L LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2)

#define AD7608_BUSY GPIO_ReadInputDataBit(GPIOA, LL_GPIO_PIN_3)
#define AD7608_DOUTA GPIO_ReadInputDataBit(GPIOA, LL_GPIO_PIN_4)
#define AD7608_DOUTB GPIO_ReadInputDataBit(GPIOA, LL_GPIO_PIN_5)

//#define AD7608_SYNC_H LL_GPIO_SetOutputPin(GPIOA, GPIO_Pin_6)
//#define AD7608_SYNC_L LL_GPIO_ResetOutputPin(GPIOA, GPIO_Pin_6)

extern UART_HandleTypeDef huart1;

void RCC_Configuration(void); 
void GPIO_Configuration(void);
void NVIC_Configuration(void); 
void Delay(uint32_t nCount);
void AD7608_SETOS(uint8_t osv);
void AD7608_RESET(void);
uint32_t ad7606_ReadDATA(void);
void AD7608_STARTCONV(void);


uint32_t datatemp[8];

int ADC_Task(void) 
{
  // initialize AD7608
  RCC_Configuration();
  GPIO_Configuration();
  NVIC_Configuration();

  AD7608_SETOS(0X00);
  AD7608_RESET();
  AD7608_CONVST_A_H;
  AD7608_CONVST_B_H;
  
  // trigger ADC transfer and read data 
  while (1) {
    if(!AD7608_BUSY) {
      AD7608_CS_L;
      for(uint8_t i = 0; i < 8; i++) {
        datatemp[i] = ad7606_ReadDATA();
        HAL_UART_Transmit(&huart1, "CH%d : %d\r\n", i, datatemp[i]);
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
  AD7608_CONVST_A_L;
  AD7608_CONVST_B_L;
  Delay(0xF);
  AD7608_CONVST_A_H;
  AD7608_CONVST_B_H;
}

uint32_t ad7606_ReadDATA(void)
{
  uint32_t usData = 0;
  for (uint8_t i = 0; i < 18; i++)
  {
    AD7608_SCLK_L;
    usData = usData << 1;
    if(AD7608_DOUTA)
    {
      usData |= 0x0001;
    }
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

void AD7608_SETOS(uint8_t osv)
{
  switch(osv)
  {
  case 0://000
    AD7608OS0_L;
    AD7608OS1_L;
    AD7608OS2_L;
    break;
  case 1://001
    AD7608OS0_H;
    AD7608OS1_L;
    AD7608OS2_L;
    break;
  case 2://010
    AD7608OS0_L;
    AD7608OS1_H;
    AD7608OS2_L;
    break;
  case 3://011
    AD7608OS0_H;
    AD7608OS1_H;
    AD7608OS2_L;
    break;
  case 4://100
    AD7608OS0_L;
    AD7608OS1_L;
    AD7608OS2_H;
    break;
  case 5://101
    AD7608OS0_H;
    AD7608OS1_L;
    AD7608OS2_H;
    break;
  case 6://110
    AD7608OS0_L;
    AD7608OS1_H;
    AD7608OS2_H;
    break;
  }
}

void RCC_Configuration(void) 
{    
  ErrorStatus HSEStartUpStatus;
  RCC_DeInit(); 
  RCC_HSEConfig(RCC_HSE_ON); 
  HSEStartUpStatus = RCC_WaitForHSEStartUp(); 
  if(HSEStartUpStatus == SUCCESS) 
  { 
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); 
    FLASH_SetLatency(FLASH_Latency_2); 
    RCC_HCLKConfig(RCC_SYSCLK_Div1);  
    RCC_PCLK2Config(RCC_HCLK_Div1);  
    RCC_PCLK1Config(RCC_HCLK_Div2); 
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); 
    RCC_PLLCmd(ENABLE); 
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) { } 
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
    while(RCC_GetSYSCLKSource() != 0x08) { } 
  } 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC| RCC_APB2Periph_GPIOA, ENABLE); 
} 

void GPIO_Configuration(void) 
{ 
  GPIO_InitTypeDef GPIO_InitStructure; 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_11|GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
} 


void NVIC_Configuration(void) 
{ 
#ifdef  VECT_TAB_RAM   
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);  
#else
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);    
#endif 
} 

void Delay(vu32 nCount) 
{ 
  for(; nCount != 0; nCount--); 
}

#endif


void ADC_Task(void const * argument)
{
  while(1)
  {
    PRINTF("ADC is Working...\r\n");
    osDelay(1000);
  }
}
