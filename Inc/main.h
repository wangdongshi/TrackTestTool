/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2023 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define NST3_Pin LL_GPIO_PIN_5
#define NST3_GPIO_Port GPIOE
#define LED1_Pin LL_GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define RS485_REDE_Pin LL_GPIO_PIN_1
#define RS485_REDE_GPIO_Port GPIOA
#define RS485_TX_Pin LL_GPIO_PIN_2
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin LL_GPIO_PIN_3
#define RS485_RX_GPIO_Port GPIOA
#define ADC_CS_Pin LL_GPIO_PIN_4
#define ADC_CS_GPIO_Port GPIOA
#define ADC_SCLK_Pin LL_GPIO_PIN_5
#define ADC_SCLK_GPIO_Port GPIOA
#define ADC_DOUTA_Pin LL_GPIO_PIN_6
#define ADC_DOUTA_GPIO_Port GPIOA
#define ADC_RESET_Pin LL_GPIO_PIN_7
#define ADC_RESET_GPIO_Port GPIOA
#define ADC_CONVST_Pin LL_GPIO_PIN_4
#define ADC_CONVST_GPIO_Port GPIOC
#define ADC_BUSY_Pin LL_GPIO_PIN_5
#define ADC_BUSY_GPIO_Port GPIOC
#define ENCODER_A_Pin LL_GPIO_PIN_9
#define ENCODER_A_GPIO_Port GPIOE
#define ENCODER_B_Pin LL_GPIO_PIN_11
#define ENCODER_B_GPIO_Port GPIOE
#define GYRO1_TX_Pin LL_GPIO_PIN_10
#define GYRO1_TX_GPIO_Port GPIOB
#define GYRO1_RX_Pin LL_GPIO_PIN_11
#define GYRO1_RX_GPIO_Port GPIOB
#define IN_DI_A_Pin LL_GPIO_PIN_14
#define IN_DI_A_GPIO_Port GPIOB
#define IN_DI_B_Pin LL_GPIO_PIN_9
#define IN_DI_B_GPIO_Port GPIOD
#define NST2_Pin LL_GPIO_PIN_12
#define NST2_GPIO_Port GPIOD
#define GYRO2_TX_Pin LL_GPIO_PIN_6
#define GYRO2_TX_GPIO_Port GPIOC
#define GYRO2_RX_Pin LL_GPIO_PIN_7
#define GYRO2_RX_GPIO_Port GPIOC
#define DEBUG_TX_Pin LL_GPIO_PIN_9
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin LL_GPIO_PIN_10
#define DEBUG_RX_GPIO_Port GPIOA
#define NST1_Pin LL_GPIO_PIN_4
#define NST1_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

typedef struct _meas {
  float mileage;      // measure by encoder
  float distance;
  float distance_comp;
  float height;
  float height_comp;
  float roll;         // measure by dip sensor
  float yaw;          // integral by gyro1
  float pitch;        // integral by gyro2
  float battery;
  float omega1;       // measure by gyro1
  float omega2;       // measure by gyro2
  uint64_t sequence;
} TRACK_MEAS_ITEM;

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
