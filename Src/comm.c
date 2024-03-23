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
#include "ad7608.h"
#include "encoder.h"
#include "comm.h"
#include "crc.h"
#include "stm32f4xx_hal_uart.h"

/* Private macro -------------------------------------------------------------*/
#define COMM_TX_BUFFER_SIZE     200
#define COMM_RX_BUFFER_SIZE     100
#define COMM_LEADING_BYTE       0xFFFF
#define COMM_FLOAT_NAN          0x7F800000

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern osSemaphoreId UserCommandArriveSemHandle;
extern osSemaphoreId UserCommandProcessSemHandle;
extern osSemaphoreId EncoderArriveSemHandle;

extern TRACK_MEAS_ITEM meas;
extern volatile float gyro_zero_offset[2];
extern volatile uint64_t gyro_count[2];
extern uint8_t format;
extern uint16_t filterDeepth;

/* Private variables ---------------------------------------------------------*/
static uint8_t txBuffer[COMM_TX_BUFFER_SIZE];
static uint8_t rxBuffer[COMM_RX_BUFFER_SIZE];
static uint8_t rxMsgBuf[COMM_RX_BUFFER_SIZE];

WORK_MODE workMode = MODE_PRE_WORK;
TRIG_MODE trigMode = TRIG_CYCLIC;
GYRO_MODE gyroMode = GYRO_OFFSET_HOLD;
DIRECTION_MODE directionMode = DIRECTION_INCREASE;
DATA_MODE dataMode = DATA_MEASURE;

/* Private function prototypes -----------------------------------------------*/
static uint16_t swapUint16(uint16_t value);
static uint32_t swapUint32(uint32_t value);
static float swapFloat(float value);

static HAL_StatusTypeDef HAL_UART_RX_DMAStop(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef HAL_UART_RX_Reset(UART_HandleTypeDef *huart, uint8_t *pData);

/* Formal function definitions -----------------------------------------------*/
void startCommunication(void)
{
  volatile HAL_StatusTypeDef status;
  
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  
  status = HAL_UART_Receive_DMA(&huart2, rxBuffer, COMM_RX_BUFFER_SIZE);
  assert_param(status == HAL_OK);
}

void commTask(void const * argument)
{
  while(1) {
    osSemaphoreWait(UserCommandArriveSemHandle, osWaitForever);
    // analysis command message
    osSemaphoreWait(UserCommandProcessSemHandle, osWaitForever);
    if (*((uint16_t*)rxMsgBuf) == 0) continue; // remove interference message on UART2
    COMM_TYPE type = (COMM_TYPE)swapUint16(*(uint16_t*)(&rxMsgBuf[2]));
    switch (type) {
      case COMM_CHANGE_TO_NORMAL_MODE:
        workMode = MODE_NORMAL_WORK;
        meas.mileage = swapFloat(*(float*)(&rxMsgBuf[4]));
        initFilter();
        break;
      case COMM_SET_MILAGE:
        stopEncoder();
        directionMode = (DIRECTION_MODE)swapUint16(*(uint16_t*)(&rxMsgBuf[8]));
        startEncoder(swapFloat(*(float*)(&rxMsgBuf[4])));
        osSemaphoreRelease(EncoderArriveSemHandle);
        break;
      case COMM_SET_DISPLAY_MODE:
        format = swapUint16(*(uint16_t*)(&rxMsgBuf[4]));
        break;
      case COMM_SET_TRIGGER_MODE:
        trigMode = (TRIG_MODE)swapUint16(*(uint16_t*)(&rxMsgBuf[4]));
        if (workMode == MODE_NORMAL_WORK && trigMode == TRIG_CYCLIC) {
          osSemaphoreRelease(EncoderArriveSemHandle);
        }
        break;
      case COMM_SET_GYRO_ZERO_DRIFT_MODE:
        gyroMode = (GYRO_MODE)swapUint16(*(uint16_t*)(&rxMsgBuf[4]));
        if (gyroMode == GYRO_OFFSET_HOLD) {
          gyro_zero_offset[0] = 0.0f;
          gyro_zero_offset[1] = 0.0f;
          gyro_count[0] = 0;
          gyro_count[1] = 0;
        }
        else if (gyroMode == GYRO_OFFSET_REMOVED) {
          meas.yaw = 0.0f;
          meas.pitch = 0.0f;
        }
        break;
      case COMM_SET_OUTPUT_ADC_DATA_MODE:
        dataMode = (DATA_MODE)swapUint16(*(uint16_t*)(&rxMsgBuf[4]));
        initFilter();
        break;
      case COMM_SET_FILTER_MODE:
        filterDeepth = swapUint16(*(uint16_t*)(&rxMsgBuf[4]));
        filterDeepth = (filterDeepth > 32) ? 32 : filterDeepth;
        filterDeepth = ((filterDeepth & (filterDeepth - 1)) == 0) ? filterDeepth : 0; // must be 2 to the Nth power
        initFilter();
        break;
      default:
        break;
    }
    osSemaphoreRelease(UserCommandProcessSemHandle);
  }
}

void uart2RxCallback(void)
{
  // Check error
  uint32_t isrFlags   = READ_REG(huart2.Instance->SR);
  uint32_t errorFlags = (isrFlags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
  if(errorFlags != RESET) {
    volatile uint32_t dr = READ_REG(huart2.Instance->DR);
    return; // There are normal errors in interrupt.
  }
  
  // Check interrupt type
  uint32_t idleFlag   = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE);
  if(idleFlag == (uint32_t)(RESET)) return; // Only UART2 idle interrupt is handled here.
  
  // Check message content
  if (((rxBuffer[0] << 8) | rxBuffer[1]) != COMM_LEADING_BYTE) { // Pre-guide code is error.
    HAL_UART_RX_Reset(&huart2, rxBuffer);
    return;
  }
  uint16_t remainder  = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
  uint16_t length     = COMM_RX_BUFFER_SIZE - remainder;
  uint16_t crc16      = swapUint16(calcCRC16(&rxBuffer[2], (length - 4))); // // Add modbus CRC
  if (crc16 != ((rxBuffer[length - 2] << 8) | rxBuffer[length - 1])) {
    HAL_UART_RX_Reset(&huart2, rxBuffer);
    return;
  }
  
  // Stop UART2 RX DMA transfer
  __HAL_UART_CLEAR_IDLEFLAG(&huart2);
  HAL_UART_RX_DMAStop(&huart2);
  
  // Organize message content
  osSemaphoreWait(UserCommandProcessSemHandle, osWaitForever);
  memcpy((void*)rxMsgBuf, (void*)rxBuffer, sizeof(rxBuffer));
  osSemaphoreRelease(UserCommandProcessSemHandle);
  
  // Notify command task to process new command
  osSemaphoreRelease(UserCommandArriveSemHandle);

  // Recover UART2 RX DMA transfer
  volatile HAL_StatusTypeDef status;
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
    if(huart->hdmarx != NULL) {
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

static HAL_StatusTypeDef HAL_UART_RX_Reset(UART_HandleTypeDef *huart, uint8_t *pData)
{
  HAL_StatusTypeDef status;
  
  // Stop UART2 RX DMA transfer
  __HAL_UART_CLEAR_IDLEFLAG(huart);
  HAL_UART_RX_DMAStop(huart);
  
  // Recover UART2 RX DMA transfer
  memset(pData, 0, COMM_RX_BUFFER_SIZE);
  status = HAL_UART_Receive_DMA(huart, pData, COMM_RX_BUFFER_SIZE);
  assert_param(status == HAL_OK);
  
  return status;
}
