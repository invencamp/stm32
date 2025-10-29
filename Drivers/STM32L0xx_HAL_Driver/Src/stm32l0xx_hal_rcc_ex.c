#include "stm32l0xx_hal.h"

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tickstart;
  uint32_t temp_reg;
  FlagStatus       pwrclkchanged = RESET;

  /* Check the parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClkInit->PeriphClockSelection));

  /*------------------------------- RTC/LCD Configuration ------------------------*/
  if ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == RCC_PERIPHCLK_RTC)
#if defined(LCD)
   || (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LCD) == RCC_PERIPHCLK_LCD)
#endif /* LCD */
     )
  {
    /* check for RTC Parameters used to output RTCCLK */
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == RCC_PERIPHCLK_RTC)
    {
      assert_param(IS_RCC_RTCCLKSOURCE(PeriphClkInit->RTCClockSelection));
    }

#if defined(LCD)
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LCD) == RCC_PERIPHCLK_LCD)
    {
      assert_param(IS_RCC_RTCCLKSOURCE(PeriphClkInit->LCDClockSelection));
    }
#endif /* LCD */

    /* As soon as function is called to change RTC clock source, activation of the
       power domain is done. */
    /* Requires to enable write access to Backup Domain of necessary */
    if(__HAL_RCC_PWR_IS_CLK_DISABLED())
    {
      __HAL_RCC_PWR_CLK_ENABLE();
      pwrclkchanged = SET;
    }

    if(HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
    {
      /* Enable write access to Backup domain */
      SET_BIT(PWR->CR, PWR_CR_DBP);

      /* Wait for Backup domain Write protection disable */
      tickstart = HAL_GetTick();

      while(HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
      {
        if((HAL_GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }

    /* Check if user wants to change HSE RTC prescaler whereas HSE is enabled */
    temp_reg = (RCC->CR & RCC_CR_RTCPRE);
    if ((temp_reg != (PeriphClkInit->RTCClockSelection & RCC_CR_RTCPRE))
#if defined (LCD)
     || (temp_reg != (PeriphClkInit->LCDClockSelection & RCC_CR_RTCPRE))
#endif /* LCD */
       )
    { /* Check HSE State */
      if ((PeriphClkInit->RTCClockSelection & RCC_CSR_RTCSEL) == RCC_CSR_RTCSEL_HSE)
      {
        if (HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSERDY))
        {
          /* To update HSE divider, first switch-OFF HSE clock oscillator*/
          return HAL_ERROR;
        }
      }
    }

    /* Reset the Backup domain only if the RTC Clock source selection is modified from reset value */
    temp_reg = (RCC->CSR & RCC_CSR_RTCSEL);

    if((temp_reg != 0x00000000U) && (((temp_reg != (PeriphClkInit->RTCClockSelection & RCC_CSR_RTCSEL)) \
      && (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == RCC_PERIPHCLK_RTC))
#if defined(LCD)
      || ((temp_reg != (PeriphClkInit->LCDClockSelection & RCC_CSR_RTCSEL)) \
       && (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LCD) == RCC_PERIPHCLK_LCD))
#endif /* LCD */
     ))
    {
      /* Store the content of CSR register before the reset of Backup Domain */
      temp_reg = (RCC->CSR & ~(RCC_CSR_RTCSEL));

      /* RTC Clock selection can be changed only if the Backup Domain is reset */
      __HAL_RCC_BACKUPRESET_FORCE();
      __HAL_RCC_BACKUPRESET_RELEASE();

      /* Restore the Content of CSR register */
      RCC->CSR = temp_reg;

       /* Wait for LSERDY if LSE was enabled */
      if (HAL_IS_BIT_SET(temp_reg, RCC_CSR_LSEON))
      {
        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till LSE is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == 0U)
        {
          if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
#if defined(LCD)
    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LCD) == RCC_PERIPHCLK_LCD)
    {
      __HAL_RCC_LCD_CONFIG(PeriphClkInit->LCDClockSelection);
    } 
#endif /* LCD */

    if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == RCC_PERIPHCLK_RTC)
    {
      __HAL_RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection);
    }

    /* Require to disable power clock if necessary */
    if(pwrclkchanged == SET)
    {
      __HAL_RCC_PWR_CLK_DISABLE();
    }
  }

#if defined (RCC_CCIPR_USART1SEL)
  /*------------------------------- USART1 Configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USART1) == RCC_PERIPHCLK_USART1)
  {
    /* Check the parameters */
    assert_param(IS_RCC_USART1CLKSOURCE(PeriphClkInit->Usart1ClockSelection));

    /* Configure the USART1 clock source */
    __HAL_RCC_USART1_CONFIG(PeriphClkInit->Usart1ClockSelection);
  }
#endif /* RCC_CCIPR_USART1SEL */

  /*----------------------------- USART2 Configuration --------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USART2) == RCC_PERIPHCLK_USART2)
  {
    /* Check the parameters */
    assert_param(IS_RCC_USART2CLKSOURCE(PeriphClkInit->Usart2ClockSelection));

    /* Configure the USART2 clock source */
    __HAL_RCC_USART2_CONFIG(PeriphClkInit->Usart2ClockSelection);
  }

  /*------------------------------ LPUART1 Configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPUART1) == RCC_PERIPHCLK_LPUART1)
  {
    /* Check the parameters */
    assert_param(IS_RCC_LPUART1CLKSOURCE(PeriphClkInit->Lpuart1ClockSelection));

    /* Configure the LPUAR1 clock source */
    __HAL_RCC_LPUART1_CONFIG(PeriphClkInit->Lpuart1ClockSelection);
  }

  /*------------------------------ I2C1 Configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2C1) == RCC_PERIPHCLK_I2C1)
  {
    /* Check the parameters */
    assert_param(IS_RCC_I2C1CLKSOURCE(PeriphClkInit->I2c1ClockSelection));

    /* Configure the I2C1 clock source */
    __HAL_RCC_I2C1_CONFIG(PeriphClkInit->I2c1ClockSelection);
  }

#if defined (RCC_CCIPR_I2C3SEL)
    /*------------------------------ I2C3 Configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2C3) == RCC_PERIPHCLK_I2C3)
  {
    /* Check the parameters */
    assert_param(IS_RCC_I2C3CLKSOURCE(PeriphClkInit->I2c3ClockSelection));

    /* Configure the I2C3 clock source */
    __HAL_RCC_I2C3_CONFIG(PeriphClkInit->I2c3ClockSelection);
  }
#endif /* RCC_CCIPR_I2C3SEL */

#if defined(USB)
 /*---------------------------- USB and RNG configuration --------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USB) == (RCC_PERIPHCLK_USB))
  {
    assert_param(IS_RCC_USBCLKSOURCE(PeriphClkInit->UsbClockSelection));
    __HAL_RCC_USB_CONFIG(PeriphClkInit->UsbClockSelection);
  }
#endif /* USB */

  /*---------------------------- LPTIM1 configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPTIM1) == (RCC_PERIPHCLK_LPTIM1))
  {
    assert_param(IS_RCC_LPTIMCLK(PeriphClkInit->LptimClockSelection));
    __HAL_RCC_LPTIM1_CONFIG(PeriphClkInit->LptimClockSelection);
  }

  return HAL_OK;
}
