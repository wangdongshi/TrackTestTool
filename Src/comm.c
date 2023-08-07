/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : comm.c
 * Project  : Track Test Tool
 * Date     : 2023/7/31
 * Author   : WangYu
 *
 **********************************************************************/
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "stm32f4xx_hal_uart.h"
#include "comm.h"
#include "crc.h"

#define COMM_LEADING_BYTE    0xFFFF

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern osMessageQId UserCommandQueueHandle;

extern TRACK_MEAS_ITEM meas;
extern float gyro_zero_offset[2];

static uint8_t txBuffer[200];
uint8_t rxBuffer[COMM_RX_BUFFER_SIZE];
COMM_MSG *pBuf;
uint8_t mode = MODE_PRE_WORK;

osPoolId mpool;
osPoolDef(mpool, 4, COMM_MSG);

static HAL_StatusTypeDef HAL_UART_RX_DMAStop(UART_HandleTypeDef *huart);


void startCommunication(void)
{
  HAL_StatusTypeDef status;
  
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  
  status = HAL_UART_Receive_DMA(&huart2, rxBuffer, COMM_RX_BUFFER_SIZE);
  assert_param(status == HAL_OK);
}

void commTask(void const * argument)
{
  while(1) {
    osMessageGet(UserCommandQueueHandle, osWaitForever);
    
    switch (pBuf->type) {
      case COMM_CHANGE_TO_NORMAL_MODE:
        mode = MODE_NORMAL_WORK;
        meas.mileage = pBuf->startPoint;
        gyro_zero_offset[0] = pBuf->zeroOffset1;
        gyro_zero_offset[1] = pBuf->zeroOffset2;
        meas.yaw   = pBuf->startAngle1;
        meas.pitch = pBuf->startAngle2;
        break;
      case COMM_RESET_MILAGE:
        meas.mileage = pBuf->startPoint;
        break;
      default:
        break;
    }
    
    osPoolFree(mpool, pBuf);
  }
}

void uart2RxCallback(void)
{
  // Check interrupt type
  uint32_t isrFlags   = READ_REG(huart2.Instance->SR);
  uint32_t cr1its     = READ_REG(huart2.Instance->CR1);
  if(((isrFlags & USART_SR_RXNE) == RESET) || ((cr1its & USART_CR1_RXNEIE) == RESET)) return; // It's not RX interrupt.
  
  // Check error
  uint32_t errorFlags = (isrFlags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
  uint32_t idleFlag   = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE);
  if(errorFlags != RESET) return; // There are normal errors in interrupt.
  if(idleFlag == (uint32_t)(RESET)) return; // There are some errors in interrupt.
  
  // Check message content
  if ((rxBuffer[0] || rxBuffer[1] << 8) != COMM_LEADING_BYTE) return; // Pre-guide code is error.
  uint16_t remainder  = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
  uint16_t length     = COMM_RX_BUFFER_SIZE - remainder;
  uint16_t crc16      = calcCRC16(&rxBuffer[2], (length - 4));
  if (crc16 != ((rxBuffer[length-2] << 8) | rxBuffer[length-1])) return;
  
  // Stop UART2 RX DMA transfer
  __HAL_UART_CLEAR_IDLEFLAG(&huart2);
  HAL_UART_RX_DMAStop(&huart2);
  
  // Organize message content
  pBuf = (COMM_MSG*)osPoolCAlloc(mpool);
  
  pBuf->length  = length;
  pBuf->type    = *(uint16_t*)(&rxBuffer[2]);
  switch (pBuf->type) {
    case COMM_CHANGE_TO_NORMAL_MODE:
      pBuf->startPoint  = *(float*)(&rxBuffer[4]);
      pBuf->zeroOffset1 = *(float*)(&rxBuffer[8]);
      pBuf->zeroOffset2 = *(float*)(&rxBuffer[12]);
      pBuf->startAngle1 = *(float*)(&rxBuffer[16]);
      pBuf->startAngle2 = *(float*)(&rxBuffer[20]);
      break;
    case COMM_RESET_MILAGE:
      pBuf->startPoint  = *(float*)(&rxBuffer[4]);
      break;
    default:
      break;
  }
  
  // Notify command task to process new command
  osMessagePut(UserCommandQueueHandle, (uint32_t)pBuf, osWaitForever);

  // Recover UART2 RX DMA transfer
  memset(rxBuffer, 0, COMM_RX_BUFFER_SIZE);
  HAL_UART_Receive_DMA(&huart2, rxBuffer, COMM_RX_BUFFER_SIZE);
  
  /*
	uint32_t tmpFlag = 0;
	uint32_t temp;
  
	tmpFlag =__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE);
	if((tmpFlag != RESET)) { 
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
		
		HAL_UART_DMAStop(&huart2);
		temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		recvLength =  COMM_RX_BUFFER_SIZE - temp;
		//recvDndFlag = 1;
		//HAL_UART_Transmit_DMA(&huart2, rxBuffer, recvLength);
		//recvLength = 0;
		//recvDndFlag = 0;
    osMessagePut(UserCommandQueueHandle, );

		memset(rxBuffer, 0, recvLength);
		HAL_UART_Receive_DMA(&huart2, rxBuffer, COMM_RX_BUFFER_SIZE);
  }
  */
}

void PRINTF2(const char *format, ...) // UART2 for RS485/RS232/USB/Ethernet
{
  unsigned short length;
  va_list args;
  va_start(args, format);
  length = vsnprintf((char*)txBuffer, sizeof(txBuffer) + 1, (char*)format, args);
  va_end(args);
  
  HAL_UART_Transmit_DMA(&huart2, txBuffer, length);
}

void TRACEFLOAT(float* input, unsigned short length) // UART2 for VOFA+
{
  memcpy((void*)txBuffer, (void*)input, (size_t)(length * sizeof(float)));
  *((uint32_t*)txBuffer + length) = 0x7F800000;
  
  HAL_UART_Transmit_DMA(&huart2, txBuffer, (length + 1) * sizeof(float));
}

static HAL_StatusTypeDef HAL_UART_RX_DMAStop(UART_HandleTypeDef *huart)
{
  uint32_t dmarequest = 0x00U;
  /* The Lock is not implemented on this API to allow the user application
     to call the HAL UART API under callbacks HAL_UART_TxCpltCallback() / HAL_UART_RxCpltCallback():
     when calling HAL_DMA_Abort() API the DMA TX/RX Transfer complete interrupt is generated
     and the correspond call back is executed HAL_UART_TxCpltCallback() / HAL_UART_RxCpltCallback()
     */

  /* Stop UART DMA Rx request if ongoing */
  dmarequest = HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR);
  if((huart->RxState == HAL_UART_STATE_BUSY_RX) && dmarequest)
  {
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Abort the UART DMA Rx channel */
    if(huart->hdmarx != NULL)
    {
      HAL_DMA_Abort(huart->hdmarx);
    }
    
    /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
    CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

    /* At end of Rx process, restore huart->RxState to Ready */
    huart->RxState = HAL_UART_STATE_READY;
  }

  return HAL_OK;
}
