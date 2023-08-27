/**********************************************************************
 * Copyright (c) 2023 - 2035 by WangYu
 * All rights reserved
 *
 * Filename : gyro97B.c
 * Project  : Track Test Tool
 * Date     : 2023/7/10
 * Author   : WangYu
 *
 **********************************************************************/
 
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "debug.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "calibrator.h"
#include "comm.h"
#include "gyro97B.h"

/* Private macro -------------------------------------------------------------*/
#define GYRO_UPDATE_FREQUENCY   300.0f    // Unit : Hz
#define GYRO_FIRST_BYTE         0xDD
#define CIRCULAR_ANGLE_DEGREE   360.0f    // Unit : degree

/* External Variables --------------------------------------------------------*/
//extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern TRACK_MEAS_ITEM meas;
extern GYRO_MODE gyroMode;

/* Private Variables ---------------------------------------------------------*/
volatile uint8_t gyro1[GYRO_RX_BUFFER_SIZE]; // gyro1 angle velocity raw data (significant 24 bit)
volatile uint8_t gyro2[GYRO_RX_BUFFER_SIZE]; // gyro2 angle velocity raw data (significant 24 bit)

volatile float gyro_zero_offset[2] = {0.0f, 0.0f};
volatile uint64_t gyro_count[2] = {0, 0};

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef HAL_UART_RX_DMAStop(UART_HandleTypeDef *huart);

/* Formal function definitions -----------------------------------------------*/
void startGyro(void)
{
  HAL_StatusTypeDef status;
  
  meas.omega1 = 0.0f - gyro_zero_offset[0];
  meas.omega2 = 0.0f - gyro_zero_offset[1];
  
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  
  status = HAL_UART_Receive_DMA(&huart3, (uint8_t*)gyro1, GYRO_RX_BUFFER_SIZE);
  assert_param(status == HAL_OK);
  
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
  
  status = HAL_UART_Receive_DMA(&huart6, (uint8_t*)gyro2, GYRO_RX_BUFFER_SIZE);
  assert_param(status == HAL_OK);
}

// yaw
void uart3RxCallback(void)
{
  int32_t gyro_24bit;
  
  // Check error
  uint32_t isrFlags   = READ_REG(huart3.Instance->SR);
  uint32_t errorFlags = (isrFlags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
  if(errorFlags != RESET) return; // There are normal errors in interrupt.
  
  // Check interrupt type
  uint32_t idleFlag   = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE);
  if(idleFlag == (uint32_t)(RESET)) return; // Only UART3 idle interrupt is handled here.
  
  // Check RX frame length
  uint16_t remainder  = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
  uint16_t length     = GYRO_RX_BUFFER_SIZE - remainder;
  if (length != 8) return;
  
  // Check pre-guide code
  if (gyro1[0] != GYRO_FIRST_BYTE) return;
  
  // Stop UART3 RX DMA transfer
  __HAL_UART_CLEAR_IDLEFLAG(&huart3);
  HAL_UART_RX_DMAStop(&huart3);
    
  // Get raw data
  gyro_24bit = (((int32_t)gyro1[2]) << 16) + (((int32_t)gyro1[3]) << 8) + (int32_t)gyro1[1];
  //gyro_24bit = (gyro_24bit << 8) >> 8;
  gyro_24bit = (int32_t)(gyro_24bit ^ 0x00800000) - (int32_t)0x00800000;
  
  // Transfer raw data to angular velocity
  meas.omega1 = (float)gyro_24bit / GYRO_SCALE_FACTOR1;
  
  // Process gyro zero drift
  if (gyroMode == GYRO_OFFSET_HOLD) {
    gyro_zero_offset[0] = gyro_zero_offset[0] * (float)gyro_count[0] + meas.omega1;
    gyro_zero_offset[0] /= gyro_count[0] + 1;
  }
  else if (gyroMode == GYRO_OFFSET_REMOVED) {
    meas.omega1 -= gyro_zero_offset[0];
  }
  
  // Integrate angular velocity values into angle values
  meas.yaw += meas.omega1 / GYRO_UPDATE_FREQUENCY;
  if (meas.yaw > CIRCULAR_ANGLE_DEGREE) meas.yaw -= CIRCULAR_ANGLE_DEGREE;
  
  gyro_count[0]++;
  
  // Recover UART3 RX DMA transfer
  HAL_StatusTypeDef status;
  memset((void*)gyro1, 0, GYRO_RX_BUFFER_SIZE);
  status = HAL_UART_Receive_DMA(&huart3, (uint8_t*)gyro1, GYRO_RX_BUFFER_SIZE);
  assert_param(status == HAL_OK);
}

// pitch
void uart6RxCallback(void)
{
  int32_t gyro_24bit;
  
  // Check error
  uint32_t isrFlags   = READ_REG(huart6.Instance->SR);
  uint32_t errorFlags = (isrFlags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
  if(errorFlags != RESET) return; // There are normal errors in interrupt.
  
  // Check interrupt type
  uint32_t idleFlag   = __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE);
  if(idleFlag == (uint32_t)(RESET)) return; // Only UART6 idle interrupt is handled here.
  
  // Check RX frame length
  uint16_t remainder  = __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
  uint16_t length     = GYRO_RX_BUFFER_SIZE - remainder;
  if (length != 8) return;
  
  // Check pre-guide code
  if (gyro2[0] != GYRO_FIRST_BYTE) return;
  
  // Stop UART3 RX DMA transfer
  __HAL_UART_CLEAR_IDLEFLAG(&huart6);
  HAL_UART_RX_DMAStop(&huart6);
  
  // Get raw data
  gyro_24bit = (((int32_t)gyro2[2]) << 16) + (((int32_t)gyro2[3]) << 8) + (int32_t)gyro2[1];
  //gyro_24bit = (gyro_24bit << 8) >> 8;
  gyro_24bit = (int32_t)(gyro_24bit ^ 0x00800000) - (int32_t)0x00800000;
  
  // Transfer raw data to angular velocity
  meas.omega2 = (float)gyro_24bit / GYRO_SCALE_FACTOR2;
  
  // Process gyro zero drift
  if (gyroMode == GYRO_OFFSET_HOLD) {
    gyro_zero_offset[1] = gyro_zero_offset[1] * (float)gyro_count[1] + meas.omega2;
    gyro_zero_offset[1] /= gyro_count[1] + 1;
  }
  else if (gyroMode == GYRO_OFFSET_REMOVED) {
    meas.omega2 -= gyro_zero_offset[1];
  }
  
  // Integrate angular velocity values into angle values
  meas.pitch += meas.omega2 / GYRO_UPDATE_FREQUENCY;
  if (meas.pitch > CIRCULAR_ANGLE_DEGREE) meas.pitch -= CIRCULAR_ANGLE_DEGREE;
  
  gyro_count[1]++;
  
  // Recover UART6 RX DMA transfer
  HAL_StatusTypeDef status;
  memset((void*)gyro2, 0, GYRO_RX_BUFFER_SIZE);
  status = HAL_UART_Receive_DMA(&huart6, (uint8_t*)gyro2, GYRO_RX_BUFFER_SIZE);
  assert_param(status == HAL_OK);
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
