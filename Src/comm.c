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
 
 /* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "stm32f4xx_hal_uart.h"
#include "comm.h"
#include "crc.h"

/* Private macro -------------------------------------------------------------*/
#define COMM_TX_BUFFER_SIZE     200
#define COMM_RX_BUFFER_SIZE     100
#define COMM_LEADING_BYTE       0xFFFF
#define COMM_FLOAT_NAN          0x7F800000

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern osMutexId UserCommandMutexHandle;
extern osSemaphoreId UserCommandArriveSemHandle;

extern TRACK_MEAS_ITEM meas;
extern float gyro_zero_offset[2];
extern uint8_t format;

/* Private variables ---------------------------------------------------------*/
static uint8_t txBuffer[COMM_TX_BUFFER_SIZE];
static uint8_t rxBuffer[COMM_RX_BUFFER_SIZE];

COMM_MSG command; // It's a shared memory parameter which is protected by mutex.

uint8_t mode = MODE_PRE_WORK;

/* Private function prototypes -----------------------------------------------*/
static uint16_t swapUint16(uint16_t value);
static uint32_t swapUint32(uint32_t value);
static float swapFloat(float value);

static HAL_StatusTypeDef HAL_UART_RX_DMAStop(UART_HandleTypeDef *huart);

/* Formal function definitions -----------------------------------------------*/
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
    osSemaphoreWait(UserCommandArriveSemHandle, osWaitForever);
    if (command.length == 0) continue;
    
    COMM_MSG msg;
    
    osMutexWait(UserCommandMutexHandle, osWaitForever);
    memcpy((void*)&msg, (void*)&command, sizeof(command));
    osMutexRelease(UserCommandMutexHandle);
    
    switch (msg.type) {
      case COMM_CHANGE_TO_NORMAL_MODE:
        mode = MODE_NORMAL_WORK;
        meas.mileage = msg.startPoint;
        gyro_zero_offset[0] = msg.zeroOffset1;
        gyro_zero_offset[1] = msg.zeroOffset2;
        meas.yaw   = msg.startAngle1;
        meas.pitch = msg.startAngle2;
        break;
      case COMM_RESET_MILAGE:
        meas.mileage = msg.startPoint;
        break;
      case COMM_CHANGE_OUTPUT_FORMAT:
        format = msg.outFormat;
        break;
      default:
        break;
    }    
  }
}

void uart2RxCallback(void)
{
  // Check error
  uint32_t isrFlags   = READ_REG(huart2.Instance->SR);
  uint32_t errorFlags = (isrFlags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
  if(errorFlags != RESET) return; // There are normal errors in interrupt.
  
  // Check interrupt type
  uint32_t idleFlag   = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE);
  if(idleFlag == (uint32_t)(RESET)) return; // Only UART2 idle interrupt is handled here.
  
  // Check message content
  if (((rxBuffer[0] << 8) | rxBuffer[1]) != COMM_LEADING_BYTE) return; // Pre-guide code is error.
  uint16_t remainder  = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
  uint16_t length     = COMM_RX_BUFFER_SIZE - remainder;
  uint16_t crc16      = swapUint16(calcCRC16(&rxBuffer[2], (length - 4))); // // Add modbus CRC
  if (crc16 != ((rxBuffer[length - 2] << 8) | rxBuffer[length - 1])) return;
  
  // Stop UART2 RX DMA transfer
  __HAL_UART_CLEAR_IDLEFLAG(&huart2);
  HAL_UART_RX_DMAStop(&huart2);
  
  // Organize message content
  osMutexWait(UserCommandMutexHandle, osWaitForever);
  command.length  = length;
  command.type    = swapUint16(*(uint16_t*)(&rxBuffer[2]));
  switch (command.type) {
    case COMM_CHANGE_TO_NORMAL_MODE:
      command.startPoint   = swapFloat(*(float*)(&rxBuffer[4]));
      command.zeroOffset1  = swapFloat(*(float*)(&rxBuffer[8]));
      command.zeroOffset2  = swapFloat(*(float*)(&rxBuffer[12]));
      command.startAngle1  = swapFloat(*(float*)(&rxBuffer[16]));
      command.startAngle2  = swapFloat(*(float*)(&rxBuffer[20]));
      break;
    case COMM_RESET_MILAGE:
      command.startPoint   = swapFloat(*(float*)(&rxBuffer[4]));
      break;
    case COMM_CHANGE_OUTPUT_FORMAT:
      command.outFormat = swapUint16(*(uint16_t*)(&rxBuffer[4]));
      break;
    default:
      break;
  }
  osMutexRelease(UserCommandMutexHandle);
  
  // Notify command task to process new command
  osSemaphoreRelease(UserCommandArriveSemHandle);

  // Recover UART2 RX DMA transfer
  HAL_StatusTypeDef status;
  memset(rxBuffer, 0, COMM_RX_BUFFER_SIZE);
  status = HAL_UART_Receive_DMA(&huart2, rxBuffer, COMM_RX_BUFFER_SIZE);
  assert_param(status == HAL_OK);
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
  *((uint32_t*)txBuffer + length) = COMM_FLOAT_NAN;
  
  HAL_UART_Transmit_DMA(&huart2, txBuffer, (length + 1) * sizeof(float));
}

static uint16_t swapUint16(uint16_t value)
{
  return ((value & 0x00FF) << 8) | ((value & 0xFF00) >> 8);
}

static uint32_t swapUint32(uint32_t value)
{
  uint32_t temp = 0;
  
  temp = ((value & 0x000000FF) << 24) |
         ((value & 0x0000FF00) << 8)  |
         ((value & 0x00FF0000) >> 8)  |
         ((value & 0xFF000000) >> 24);
  
  return temp;
}

static float swapFloat(float value)
{
  union SWAP_UNION {
    float    unionFloat;
    uint32_t unionUint32;
  } swap;
   
  swap.unionFloat = value;
  swap.unionUint32 = swapUint32(swap.unionUint32);

  return swap.unionFloat;
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
