#include "stm32l0xx_hal.h"

void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{ 
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(PreemptPriority));
  NVIC_SetPriority(IRQn,PreemptPriority);
}

void HAL_NVIC_EnableIRQ(IRQn_Type IRQn)
{
  assert_param(IS_NVIC_DEVICE_IRQ(IRQn));
  NVIC_EnableIRQ(IRQn);
}
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb)
{
   return SysTick_Config(TicksNumb);
}





