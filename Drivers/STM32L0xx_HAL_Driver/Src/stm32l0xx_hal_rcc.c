#include "stm32l0xx_hal.h"
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct)
{
  uint32_t tickstart;
  uint32_t hsi_state;
  HAL_StatusTypeDef status;
  uint32_t sysclk_source, pll_config;

  /* Check Null pointer */
  if(RCC_OscInitStruct == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_RCC_OSCILLATORTYPE(RCC_OscInitStruct->OscillatorType));

  sysclk_source = __HAL_RCC_GET_SYSCLK_SOURCE();
  pll_config = __HAL_RCC_GET_PLL_OSCSOURCE();

  /*------------------------------- HSE Configuration ------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
  {
    /* Check the parameters */
    assert_param(IS_RCC_HSE(RCC_OscInitStruct->HSEState));

    /* When the HSE is used as system clock or clock source for PLL in these cases it is not allowed to be disabled */
    if((sysclk_source == RCC_SYSCLKSOURCE_STATUS_HSE)
       || ((sysclk_source == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && (pll_config == RCC_PLLSOURCE_HSE)))
    {
      if((__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != 0U) && (RCC_OscInitStruct->HSEState == RCC_HSE_OFF))
      {
        return HAL_ERROR;
      }
    }
    else
    {
      /* Set the new HSE configuration ---------------------------------------*/
      __HAL_RCC_HSE_CONFIG(RCC_OscInitStruct->HSEState);

      /* Check the HSE State */
      if(RCC_OscInitStruct->HSEState != RCC_HSE_OFF)
      {
        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till HSE is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == 0U)
        {
          if((HAL_GetTick() - tickstart ) > HSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till HSE is disabled */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != 0U)
        {
           if((HAL_GetTick() - tickstart ) > HSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }
  /*----------------------------- HSI Configuration --------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI)
  {
    /* Check the parameters */
    assert_param(IS_RCC_HSI(RCC_OscInitStruct->HSIState));
    assert_param(IS_RCC_CALIBRATION_VALUE(RCC_OscInitStruct->HSICalibrationValue));

    hsi_state = RCC_OscInitStruct->HSIState;

#if defined(RCC_CR_HSIOUTEN)
    if((hsi_state & RCC_HSI_OUTEN) != 0U)
    {
      /* HSI Output enable for timer requested */
      SET_BIT(RCC->CR, RCC_CR_HSIOUTEN);

      hsi_state &= ~RCC_CR_HSIOUTEN;
    }
#endif

    /* Check if HSI is used as system clock or as PLL source when PLL is selected as system clock */
    if((sysclk_source == RCC_SYSCLKSOURCE_STATUS_HSI)
       || ((sysclk_source == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && (pll_config == RCC_PLLSOURCE_HSI)))
    {
      /* When HSI is used as system clock it will not disabled */
      if((__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) != 0U) && (hsi_state == RCC_HSI_OFF))
      {
        return HAL_ERROR;
      }
      /* Otherwise, just the calibration and HSI or HSIdiv4 are allowed */
      else
      {
        /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
        __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);

        /* Enable the Internal High Speed oscillator (HSI or HSIdiv4) */
        __HAL_RCC_HSI_CONFIG(hsi_state);
      }

      /* Update the SystemCoreClock global variable */
      SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE)>> RCC_CFGR_HPRE_Pos];

      /* Configure the source of time base considering new system clocks settings*/
      status = HAL_InitTick (uwTickPrio);
      if(status != HAL_OK)
      {
        return status;
      }
    }
    else
    {
      /* Check the HSI State */
      if(hsi_state != RCC_HSI_OFF)
      {
        /* Enable the Internal High Speed oscillator (HSI or HSIdiv4) */
        __HAL_RCC_HSI_CONFIG(hsi_state);

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till HSI is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == 0U)
        {
          if((HAL_GetTick() - tickstart ) > HSI_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }

        /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
        __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
      }
      else
      {
        /* Disable the Internal High Speed oscillator (HSI). */
        __HAL_RCC_HSI_DISABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till HSI is disabled */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) != 0U)
        {
          if((HAL_GetTick() - tickstart ) > HSI_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }
  /*----------------------------- MSI Configuration --------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_MSI) == RCC_OSCILLATORTYPE_MSI)
  {
    /* When the MSI is used as system clock it will not be disabled */
    if(sysclk_source == RCC_CFGR_SWS_MSI)
    {
      if((__HAL_RCC_GET_FLAG(RCC_FLAG_MSIRDY) != 0U) && (RCC_OscInitStruct->MSIState == RCC_MSI_OFF))
      {
        return HAL_ERROR;
      }
      /* Otherwise, just the calibration and MSI range change are allowed */
      else
      {
        /* Check MSICalibrationValue and MSIClockRange input parameters */
        assert_param(IS_RCC_MSICALIBRATION_VALUE(RCC_OscInitStruct->MSICalibrationValue));
        assert_param(IS_RCC_MSI_CLOCK_RANGE(RCC_OscInitStruct->MSIClockRange));

        /* Selects the Multiple Speed oscillator (MSI) clock range .*/
        __HAL_RCC_MSI_RANGE_CONFIG(RCC_OscInitStruct->MSIClockRange);
        /* Adjusts the Multiple Speed oscillator (MSI) calibration value.*/
        __HAL_RCC_MSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->MSICalibrationValue);


        /* Update the SystemCoreClock global variable */
        SystemCoreClock =  (32768U * (1UL << ((RCC_OscInitStruct->MSIClockRange >> RCC_ICSCR_MSIRANGE_Pos) + 1U)))
                           >> AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos)];

        /* Configure the source of time base considering new system clocks settings*/
        status = HAL_InitTick (uwTickPrio);
        if(status != HAL_OK)
        {
          return status;
        }
      }
    }
    else
    {
      /* Check MSI State */
      assert_param(IS_RCC_MSI(RCC_OscInitStruct->MSIState));

      /* Check the MSI State */
      if(RCC_OscInitStruct->MSIState != RCC_MSI_OFF)
      {
        /* Enable the Multi Speed oscillator (MSI). */
        __HAL_RCC_MSI_ENABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till MSI is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_MSIRDY) == 0U)
        {
          if((HAL_GetTick() - tickstart) > MSI_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
        /* Check MSICalibrationValue and MSIClockRange input parameters */
        assert_param(IS_RCC_MSICALIBRATION_VALUE(RCC_OscInitStruct->MSICalibrationValue));
        assert_param(IS_RCC_MSI_CLOCK_RANGE(RCC_OscInitStruct->MSIClockRange));

        /* Selects the Multiple Speed oscillator (MSI) clock range .*/
        __HAL_RCC_MSI_RANGE_CONFIG(RCC_OscInitStruct->MSIClockRange);
         /* Adjusts the Multiple Speed oscillator (MSI) calibration value.*/
        __HAL_RCC_MSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->MSICalibrationValue);
      }
      else
      {
        /* Disable the Multi Speed oscillator (MSI). */
        __HAL_RCC_MSI_DISABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till MSI is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_MSIRDY) != 0U)
        {
          if((HAL_GetTick() - tickstart) > MSI_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }
  /*------------------------------ LSI Configuration -------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI)
  {
    /* Check the parameters */
    assert_param(IS_RCC_LSI(RCC_OscInitStruct->LSIState));

    /* Check the LSI State */
    if(RCC_OscInitStruct->LSIState != RCC_LSI_OFF)
    {
      /* Enable the Internal Low Speed oscillator (LSI). */
      __HAL_RCC_LSI_ENABLE();

      /* Get Start Tick */
      tickstart = HAL_GetTick();

      /* Wait till LSI is ready */
      while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == 0U)
      {
        if((HAL_GetTick() - tickstart ) > LSI_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else
    {
      /* Disable the Internal Low Speed oscillator (LSI). */
      __HAL_RCC_LSI_DISABLE();

      /* Get Start Tick */
      tickstart = HAL_GetTick();

      /* Wait till LSI is disabled */
      while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) != 0U)
      {
        if((HAL_GetTick() - tickstart ) > LSI_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
  }
  /*------------------------------ LSE Configuration -------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
  {
    FlagStatus       pwrclkchanged = RESET;

    /* Check the parameters */
    assert_param(IS_RCC_LSE(RCC_OscInitStruct->LSEState));

    /* Update LSE configuration in Backup Domain control register    */
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

    /* Set the new LSE configuration -----------------------------------------*/
    __HAL_RCC_LSE_CONFIG(RCC_OscInitStruct->LSEState);

    /* Check the LSE State */
    if(RCC_OscInitStruct->LSEState != RCC_LSE_OFF)
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
    else
    {
      /* Get Start Tick */
      tickstart = HAL_GetTick();

      /* Wait till LSE is disabled */
      while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != 0U)
      {
        if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }

    /* Require to disable power clock if necessary */
    if(pwrclkchanged == SET)
    {
      __HAL_RCC_PWR_CLK_DISABLE();
    }
  }

#if defined(RCC_HSI48_SUPPORT)
  /*----------------------------- HSI48 Configuration --------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI48) == RCC_OSCILLATORTYPE_HSI48)
  {
    /* Check the parameters */
    assert_param(IS_RCC_HSI48(RCC_OscInitStruct->HSI48State));

      /* Check the HSI48 State */
      if(RCC_OscInitStruct->HSI48State != RCC_HSI48_OFF)
      {
        /* Enable the Internal High Speed oscillator (HSI48). */
        __HAL_RCC_HSI48_ENABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till HSI48 is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSI48RDY) == 0U)
        {
          if((HAL_GetTick() - tickstart) > HSI48_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
        /* Disable the Internal High Speed oscillator (HSI48). */
        __HAL_RCC_HSI48_DISABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till HSI48 is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSI48RDY) != 0U)
        {
          if((HAL_GetTick() - tickstart) > HSI48_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
  }
#endif /* RCC_HSI48_SUPPORT */

  /*-------------------------------- PLL Configuration -----------------------*/
  /* Check the parameters */
  assert_param(IS_RCC_PLL(RCC_OscInitStruct->PLL.PLLState));
  if ((RCC_OscInitStruct->PLL.PLLState) != RCC_PLL_NONE)
  {
    /* Check if the PLL is used as system clock or not */
    if(sysclk_source != RCC_SYSCLKSOURCE_STATUS_PLLCLK)
    {
      if((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON)
      {
        /* Check the parameters */
        assert_param(IS_RCC_PLLSOURCE(RCC_OscInitStruct->PLL.PLLSource));
        assert_param(IS_RCC_PLL_MUL(RCC_OscInitStruct->PLL.PLLMUL));
        assert_param(IS_RCC_PLL_DIV(RCC_OscInitStruct->PLL.PLLDIV));

        /* Disable the main PLL. */
        __HAL_RCC_PLL_DISABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  != 0U)
        {
          if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }

        /* Configure the main PLL clock source, multiplication and division factors. */
        __HAL_RCC_PLL_CONFIG(RCC_OscInitStruct->PLL.PLLSource,
                             RCC_OscInitStruct->PLL.PLLMUL,
                             RCC_OscInitStruct->PLL.PLLDIV);
        /* Enable the main PLL. */
        __HAL_RCC_PLL_ENABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL is ready */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  == 0U)
        {
          if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
        /* Disable the main PLL. */
        __HAL_RCC_PLL_DISABLE();

        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till PLL is disabled */
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  != 0U)
        {
          if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    else
    {
      /* Check if there is a request to disable the PLL used as System clock source */
      if((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_OFF)
      {
        return HAL_ERROR;
      }
      else
      {
        /* Do not return HAL_ERROR if request repeats the current configuration */
        pll_config = RCC->CFGR;
        if((READ_BIT(pll_config, RCC_CFGR_PLLSRC) != RCC_OscInitStruct->PLL.PLLSource) ||
           (READ_BIT(pll_config, RCC_CFGR_PLLMUL) != RCC_OscInitStruct->PLL.PLLMUL) ||
           (READ_BIT(pll_config, RCC_CFGR_PLLDIV) != RCC_OscInitStruct->PLL.PLLDIV))
        {
          return HAL_ERROR;
        }
      }
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency)
{
  uint32_t tickstart;
  HAL_StatusTypeDef status;

  /* Check Null pointer */
  if(RCC_ClkInitStruct == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_RCC_CLOCKTYPE(RCC_ClkInitStruct->ClockType));
  assert_param(IS_FLASH_LATENCY(FLatency));

  /* To correctly read data from FLASH memory, the number of wait states (LATENCY)
  must be correctly programmed according to the frequency of the CPU clock
  (HCLK) and the supply voltage of the device. */

  /* Increasing the number of wait states because of higher CPU frequency */
  if(FLatency > __HAL_FLASH_GET_LATENCY())
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    __HAL_FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by polling the FLASH_ACR register */
    tickstart = HAL_GetTick();

    while (__HAL_FLASH_GET_LATENCY() != FLatency)
    {
      if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
  }

  /*-------------------------- HCLK Configuration --------------------------*/
  if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
  {
    assert_param(IS_RCC_HCLK(RCC_ClkInitStruct->AHBCLKDivider));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct->AHBCLKDivider);
  }

  /*------------------------- SYSCLK Configuration ---------------------------*/
  if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
  {
    assert_param(IS_RCC_SYSCLKSOURCE(RCC_ClkInitStruct->SYSCLKSource));

    /* HSE is selected as System Clock Source */
    if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE)
    {
      /* Check the HSE ready flag */
      if(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == 0U)
      {
        return HAL_ERROR;
      }
    }
    /* PLL is selected as System Clock Source */
    else if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK)
    {
      /* Check the PLL ready flag */
      if(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == 0U)
      {
        return HAL_ERROR;
      }
    }
    /* HSI is selected as System Clock Source */
    else if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSI)
    {
      /* Check the HSI ready flag */
      if(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == 0U)
      {
        return HAL_ERROR;
      }
    }
    /* MSI is selected as System Clock Source */
    else
    {
      /* Check the MSI ready flag */
      if(__HAL_RCC_GET_FLAG(RCC_FLAG_MSIRDY) == 0U)
      {
        return HAL_ERROR;
      }
    }
    __HAL_RCC_SYSCLK_CONFIG(RCC_ClkInitStruct->SYSCLKSource);

    /* Get Start Tick */
    tickstart = HAL_GetTick();

    if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE)
    {
      while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSE)
      {
        if((HAL_GetTick() - tickstart ) > CLOCKSWITCH_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK)
    {
      while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK)
      {
        if((HAL_GetTick() - tickstart ) > CLOCKSWITCH_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSI)
    {
      while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSI)
      {
        if((HAL_GetTick() - tickstart ) > CLOCKSWITCH_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else
    {
      while(__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_MSI)
      {
        if((HAL_GetTick() - tickstart ) > CLOCKSWITCH_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }
  }
  /* Decreasing the number of wait states because of lower CPU frequency */
  if(FLatency < __HAL_FLASH_GET_LATENCY())
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    __HAL_FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by polling the FLASH_ACR register */
    tickstart = HAL_GetTick();

    while (__HAL_FLASH_GET_LATENCY() != FLatency)
    {
      if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
  }

  /*-------------------------- PCLK1 Configuration ---------------------------*/
  if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
  {
    assert_param(IS_RCC_PCLK(RCC_ClkInitStruct->APB1CLKDivider));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);
  }

  /*-------------------------- PCLK2 Configuration ---------------------------*/
  if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
  {
    assert_param(IS_RCC_PCLK(RCC_ClkInitStruct->APB2CLKDivider));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_ClkInitStruct->APB2CLKDivider) << 3));
  }

  /* Update the SystemCoreClock global variable */
  SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE)>> RCC_CFGR_HPRE_Pos];

  /* Configure the source of time base considering new system clocks settings*/
  status = HAL_InitTick(uwTickPrio);
  if(status != HAL_OK)
  {
    return status;
  }

  return HAL_OK;
}

uint32_t HAL_RCC_GetSysClockFreq(void)
{
  uint32_t tmpreg, pllm, plld, pllvco, msiclkrange;    /* no init needed */
  uint32_t sysclockfreq;

  tmpreg = RCC->CFGR;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (tmpreg & RCC_CFGR_SWS)
  {
    case RCC_SYSCLKSOURCE_STATUS_HSI:  /* HSI used as system clock source */
    {
      if ((RCC->CR & RCC_CR_HSIDIVF) != 0U)
      {
        sysclockfreq =  (HSI_VALUE >> 2);
      }
      else
      {
        sysclockfreq =  HSI_VALUE;
      }
      break;
    }
    case RCC_SYSCLKSOURCE_STATUS_HSE:  /* HSE used as system clock */
    {
      sysclockfreq = HSE_VALUE;
      break;
    }
    case RCC_SYSCLKSOURCE_STATUS_PLLCLK:  /* PLL used as system clock */
    {
      pllm = PLLMulTable[(uint32_t)(tmpreg & RCC_CFGR_PLLMUL) >> RCC_CFGR_PLLMUL_Pos];
      plld = ((uint32_t)(tmpreg & RCC_CFGR_PLLDIV) >> RCC_CFGR_PLLDIV_Pos) + 1U;
      if (__HAL_RCC_GET_PLL_OSCSOURCE() != RCC_PLLSOURCE_HSI)
      {
        /* HSE used as PLL clock source */
        pllvco = (uint32_t)(((uint64_t)HSE_VALUE * (uint64_t)pllm) / (uint64_t)plld);
      }
      else
      {
        if ((RCC->CR & RCC_CR_HSIDIVF) != 0U)
        {
          pllvco = (uint32_t)((((uint64_t)(HSI_VALUE >> 2)) * (uint64_t)pllm) / (uint64_t)plld);
        }
        else
        {
         pllvco = (uint32_t)(((uint64_t)HSI_VALUE * (uint64_t)pllm) / (uint64_t)plld);
        }
      }
      sysclockfreq = pllvco;
      break;
    }
    case RCC_SYSCLKSOURCE_STATUS_MSI:  /* MSI used as system clock source */
    default: /* MSI used as system clock */
    {
      msiclkrange = (RCC->ICSCR & RCC_ICSCR_MSIRANGE ) >> RCC_ICSCR_MSIRANGE_Pos;
      sysclockfreq = (32768U * (1UL << (msiclkrange + 1U)));
      break;
    }
  }
  return sysclockfreq;
}
uint32_t HAL_RCC_GetHCLKFreq(void)
{
  return SystemCoreClock;
}
uint32_t HAL_RCC_GetPCLK1Freq(void)
{
  /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
  return (HAL_RCC_GetHCLKFreq() >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
}

uint32_t HAL_RCC_GetPCLK2Freq(void)
{
  /* Get HCLK source and Compute PCLK2 frequency ---------------------------*/
  return (HAL_RCC_GetHCLKFreq()>> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]);
}