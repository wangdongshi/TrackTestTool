#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart1;

void ADC_Task(void const * argument)
{
  uint8_t tx_buffer[19] = "ADC is Working...\r\n";
 
  while(1)
  {
    HAL_UART_Transmit(&huart1, tx_buffer, sizeof(tx_buffer), 0xffff);
    osDelay(1000);
  }
}
