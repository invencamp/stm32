#include "stm32l0xx_hal.h"
__IO uint32_t uwTick;
uint32_t uwTickPrio   = (1UL << __NVIC_PRIO_BITS); /* Invalid PRIO */
HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_DEFAULT;  /* 1KHz */
HAL_StatusTypeDef HAL_Init(void)
{
  HAL_StatusTypeDef  status = HAL_OK;

  /* Configure Buffer cache, Flash prefetch,  Flash preread */
#if (BUFFER_CACHE_DISABLE != 0)
  __HAL_FLASH_BUFFER_CACHE_DISABLE();
#endif /* BUFFER_CACHE_DISABLE */

#if (PREREAD_ENABLE != 0)
  __HAL_FLASH_PREREAD_BUFFER_ENABLE();
#endif /* PREREAD_ENABLE */

#if (PREFETCH_ENABLE != 0)
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif /* PREFETCH_ENABLE */

  /* Use SysTick as time base source and configure 1ms tick (default clock after Reset is MSI) */
  if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
  {
    status = HAL_ERROR;
  }
  else
  {
    /* Init the low level hardware */
    HAL_MspInit();
  }

  /* Return function status */
  return status;
}

__weak HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  /* Configure the SysTick to have interrupt in 1ms time basis*/
  if (HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq)) > 0U)
  {
    return HAL_ERROR;
  }

  /* Configure the SysTick IRQ priority */
  if (TickPriority < (1UL << __NVIC_PRIO_BITS))
  {
    HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0U);
    uwTickPrio = TickPriority;
  }
  else
  {
    return HAL_ERROR;
  }

  /* Return function status */
  return HAL_OK;
}

__weak void HAL_IncTick(void)
{
  uwTick += uwTickFreq;
}

__weak uint32_t HAL_GetTick(void)
{
  return uwTick;
}




