#include "stm32l0xx_hal.h"

__weak void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UARTEx_WakeupCallback can be implemented in the user file.
   */
}
