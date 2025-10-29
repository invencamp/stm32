#include "main.h"
#include "stm32l0xx_it.h"
extern UART_HandleTypeDef huart2;
void SysTick_Handler(void)
{
  HAL_IncTick();
}
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}
