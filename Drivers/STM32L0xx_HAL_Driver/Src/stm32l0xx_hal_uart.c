#include "stm32l0xx_hal.h"
#define UART_BRR_MIN    0x10U        /* UART BRR minimum authorized value */
#define UART_BRR_MAX    0x0000FFFFU
#define USART_CR1_FIELDS  ((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE | \
                                      USART_CR1_OVER8)) 
																			#define USART_CR3_FIELDS  ((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE |\
                                      USART_CR3_ONEBIT)) /*!< UART or USART CR3 fields of parameters set by UART_SetConfig API */

#define LPUART_BRR_MIN  0x00000300U  /* LPUART BRR minimum authorized value */
#define LPUART_BRR_MAX  0x000FFFFFU
void UART_AdvFeatureConfig(UART_HandleTypeDef *huart)
{
  /* Check whether the set of advanced features to configure is properly set */
  assert_param(IS_UART_ADVFEATURE_INIT(huart->AdvancedInit.AdvFeatureInit));

  /* if required, configure TX pin active level inversion */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_TXINVERT_INIT))
  {
    assert_param(IS_UART_ADVFEATURE_TXINV(huart->AdvancedInit.TxPinLevelInvert));
    MODIFY_REG(huart->Instance->CR2, USART_CR2_TXINV, huart->AdvancedInit.TxPinLevelInvert);
  }

  /* if required, configure RX pin active level inversion */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_RXINVERT_INIT))
  {
    assert_param(IS_UART_ADVFEATURE_RXINV(huart->AdvancedInit.RxPinLevelInvert));
    MODIFY_REG(huart->Instance->CR2, USART_CR2_RXINV, huart->AdvancedInit.RxPinLevelInvert);
  }

  /* if required, configure data inversion */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_DATAINVERT_INIT))
  {
    assert_param(IS_UART_ADVFEATURE_DATAINV(huart->AdvancedInit.DataInvert));
    MODIFY_REG(huart->Instance->CR2, USART_CR2_DATAINV, huart->AdvancedInit.DataInvert);
  }

  /* if required, configure RX/TX pins swap */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_SWAP_INIT))
  {
    assert_param(IS_UART_ADVFEATURE_SWAP(huart->AdvancedInit.Swap));
    MODIFY_REG(huart->Instance->CR2, USART_CR2_SWAP, huart->AdvancedInit.Swap);
  }

  /* if required, configure RX overrun detection disabling */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_RXOVERRUNDISABLE_INIT))
  {
    assert_param(IS_UART_OVERRUN(huart->AdvancedInit.OverrunDisable));
    MODIFY_REG(huart->Instance->CR3, USART_CR3_OVRDIS, huart->AdvancedInit.OverrunDisable);
  }

  /* if required, configure DMA disabling on reception error */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_DMADISABLEONERROR_INIT))
  {
    assert_param(IS_UART_ADVFEATURE_DMAONRXERROR(huart->AdvancedInit.DMADisableonRxError));
    MODIFY_REG(huart->Instance->CR3, USART_CR3_DDRE, huart->AdvancedInit.DMADisableonRxError);
  }

  /* if required, configure auto Baud rate detection scheme */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_AUTOBAUDRATE_INIT))
  {
    assert_param(IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(huart->Instance));
    assert_param(IS_UART_ADVFEATURE_AUTOBAUDRATE(huart->AdvancedInit.AutoBaudRateEnable));
    MODIFY_REG(huart->Instance->CR2, USART_CR2_ABREN, huart->AdvancedInit.AutoBaudRateEnable);
    /* set auto Baudrate detection parameters if detection is enabled */
    if (huart->AdvancedInit.AutoBaudRateEnable == UART_ADVFEATURE_AUTOBAUDRATE_ENABLE)
    {
      assert_param(IS_UART_ADVFEATURE_AUTOBAUDRATEMODE(huart->AdvancedInit.AutoBaudRateMode));
      MODIFY_REG(huart->Instance->CR2, USART_CR2_ABRMODE, huart->AdvancedInit.AutoBaudRateMode);
    }
  }

  /* if required, configure MSB first on communication line */
  if (HAL_IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_MSBFIRST_INIT))
  {
    assert_param(IS_UART_ADVFEATURE_MSBFIRST(huart->AdvancedInit.MSBFirst));
    MODIFY_REG(huart->Instance->CR2, USART_CR2_MSBFIRST, huart->AdvancedInit.MSBFirst);
  }
}
HAL_StatusTypeDef UART_SetConfig(UART_HandleTypeDef *huart)
{
  uint32_t tmpreg;
  uint16_t brrtemp;
  UART_ClockSourceTypeDef clocksource;
  uint32_t usartdiv;
  HAL_StatusTypeDef ret               = HAL_OK;
  uint32_t pclk;

  /* Check the parameters */
  assert_param(IS_UART_BAUDRATE(huart->Init.BaudRate));
  assert_param(IS_UART_WORD_LENGTH(huart->Init.WordLength));
  if (UART_INSTANCE_LOWPOWER(huart))
  {
    assert_param(IS_LPUART_STOPBITS(huart->Init.StopBits));
  }
  else
  {
    assert_param(IS_UART_STOPBITS(huart->Init.StopBits));
    assert_param(IS_UART_ONE_BIT_SAMPLE(huart->Init.OneBitSampling));
  }

  assert_param(IS_UART_PARITY(huart->Init.Parity));
  assert_param(IS_UART_MODE(huart->Init.Mode));
  assert_param(IS_UART_HARDWARE_FLOW_CONTROL(huart->Init.HwFlowCtl));
  assert_param(IS_UART_OVERSAMPLING(huart->Init.OverSampling));

  /*-------------------------- USART CR1 Configuration -----------------------*/
  /* Clear M, PCE, PS, TE, RE and OVER8 bits and configure
  *  the UART Word Length, Parity, Mode and oversampling:
  *  set the M bits according to huart->Init.WordLength value
  *  set PCE and PS bits according to huart->Init.Parity value
  *  set TE and RE bits according to huart->Init.Mode value
  *  set OVER8 bit according to huart->Init.OverSampling value */
  tmpreg = (uint32_t)huart->Init.WordLength | huart->Init.Parity | huart->Init.Mode | huart->Init.OverSampling ;
  MODIFY_REG(huart->Instance->CR1, USART_CR1_FIELDS, tmpreg);

  /*-------------------------- USART CR2 Configuration -----------------------*/
  /* Configure the UART Stop Bits: Set STOP[13:12] bits according
  * to huart->Init.StopBits value */
  MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, huart->Init.StopBits);

  /*-------------------------- USART CR3 Configuration -----------------------*/
  /* Configure
  * - UART HardWare Flow Control: set CTSE and RTSE bits according
  *   to huart->Init.HwFlowCtl value
  * - one-bit sampling method versus three samples' majority rule according
  *   to huart->Init.OneBitSampling (not applicable to LPUART) */
  tmpreg = (uint32_t)huart->Init.HwFlowCtl;

  if (!(UART_INSTANCE_LOWPOWER(huart)))
  {
    tmpreg |= huart->Init.OneBitSampling;
  }
  MODIFY_REG(huart->Instance->CR3, USART_CR3_FIELDS, tmpreg);


  /*-------------------------- USART BRR Configuration -----------------------*/
  UART_GETCLOCKSOURCE(huart, clocksource);

  /* Check LPUART instance */
  if (UART_INSTANCE_LOWPOWER(huart))
  {
    /* Retrieve frequency clock */
    switch (clocksource)
    {
      case UART_CLOCKSOURCE_PCLK1:
        pclk = HAL_RCC_GetPCLK1Freq();
        break;
      case UART_CLOCKSOURCE_HSI:
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIDIV) != 0U)
        {
          pclk = (uint32_t)(HSI_VALUE >> 2U);
        }
        else
        {
          pclk = (uint32_t) HSI_VALUE;
        }
        break;
      case UART_CLOCKSOURCE_SYSCLK:
        pclk = HAL_RCC_GetSysClockFreq();
        break;
      case UART_CLOCKSOURCE_LSE:
        pclk = (uint32_t) LSE_VALUE;
        break;
      default:
        pclk = 0U;
        ret = HAL_ERROR;
        break;
    }

    /* If proper clock source reported */
    if (pclk != 0U)
    {
      /* No Prescaler applicable */
      /* Ensure that Frequency clock is in the range [3 * baudrate, 4096 * baudrate] */
      if ((pclk < (3U * huart->Init.BaudRate)) ||
          (pclk > (4096U * huart->Init.BaudRate)))
      {
        ret = HAL_ERROR;
      }
      else
      {
        usartdiv = (uint32_t)(UART_DIV_LPUART(pclk, huart->Init.BaudRate));
        if ((usartdiv >= LPUART_BRR_MIN) && (usartdiv <= LPUART_BRR_MAX))
        {
          huart->Instance->BRR = usartdiv;
        }
        else
        {
          ret = HAL_ERROR;
        }
      } /* if ( (pclk < (3 * huart->Init.BaudRate) ) || (pclk > (4096 * huart->Init.BaudRate) )) */
    } /* if (pclk != 0) */
  }
  /* Check UART Over Sampling to set Baud Rate Register */
  else if (huart->Init.OverSampling == UART_OVERSAMPLING_8)
  {
    switch (clocksource)
    {
      case UART_CLOCKSOURCE_PCLK1:
        pclk = HAL_RCC_GetPCLK1Freq();
        break;
      case UART_CLOCKSOURCE_PCLK2:
        pclk = HAL_RCC_GetPCLK2Freq();
        break;
      case UART_CLOCKSOURCE_HSI:
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIDIV) != 0U)
        {
          pclk = (uint32_t)(HSI_VALUE >> 2U);
        }
        else
        {
          pclk = (uint32_t) HSI_VALUE;
        }
        break;
      case UART_CLOCKSOURCE_SYSCLK:
        pclk = HAL_RCC_GetSysClockFreq();
        break;
      case UART_CLOCKSOURCE_LSE:
        pclk = (uint32_t) LSE_VALUE;
        break;
      default:
        pclk = 0U;
        ret = HAL_ERROR;
        break;
    }

    /* USARTDIV must be greater than or equal to 0d16 */
    if (pclk != 0U)
    {
      usartdiv = (uint32_t)(UART_DIV_SAMPLING8(pclk, huart->Init.BaudRate));
      if ((usartdiv >= UART_BRR_MIN) && (usartdiv <= UART_BRR_MAX))
      {
        brrtemp = (uint16_t)(usartdiv & 0xFFF0U);
        brrtemp |= (uint16_t)((usartdiv & (uint16_t)0x000FU) >> 1U);
        huart->Instance->BRR = brrtemp;
      }
      else
      {
        ret = HAL_ERROR;
      }
    }
  }
  else
  {
    switch (clocksource)
    {
      case UART_CLOCKSOURCE_PCLK1:
        pclk = HAL_RCC_GetPCLK1Freq();
        break;
      case UART_CLOCKSOURCE_PCLK2:
        pclk = HAL_RCC_GetPCLK2Freq();
        break;
      case UART_CLOCKSOURCE_HSI:
        if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIDIV) != 0U)
        {
          pclk = (uint32_t)(HSI_VALUE >> 2U);
        }
        else
        {
          pclk = (uint32_t) HSI_VALUE;
        }
        break;
      case UART_CLOCKSOURCE_SYSCLK:
        pclk = HAL_RCC_GetSysClockFreq();
        break;
      case UART_CLOCKSOURCE_LSE:
        pclk = (uint32_t) LSE_VALUE;
        break;
      default:
        pclk = 0U;
        ret = HAL_ERROR;
        break;
    }

    if (pclk != 0U)
    {
      /* USARTDIV must be greater than or equal to 0d16 */
      usartdiv = (uint32_t)(UART_DIV_SAMPLING16(pclk, huart->Init.BaudRate));
      if ((usartdiv >= UART_BRR_MIN) && (usartdiv <= UART_BRR_MAX))
      {
        huart->Instance->BRR = (uint16_t)usartdiv;
      }
      else
      {
        ret = HAL_ERROR;
      }
    }
  }


  /* Clear ISR function pointers */
  huart->RxISR = NULL;
  huart->TxISR = NULL;

  return ret;
}

static void UART_RxISR_16BIT(UART_HandleTypeDef *huart)
{
  uint16_t *tmp;
  uint16_t uhMask = huart->Mask;
  uint16_t  uhdata;

  /* Check that a Rx process is ongoing */
  if (huart->RxState == HAL_UART_STATE_BUSY_RX)
  {
    uhdata = (uint16_t) READ_REG(huart->Instance->RDR);
    tmp = (uint16_t *) huart->pRxBuffPtr ;
    *tmp = (uint16_t)(uhdata & uhMask);
    huart->pRxBuffPtr += 2U;
    huart->RxXferCount--;

    if (huart->RxXferCount == 0U)
    {
      /* Disable the UART Parity Error Interrupt and RXNE interrupt*/
      ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));

      /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
      ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

      /* Rx process is completed, restore huart->RxState to Ready */
      huart->RxState = HAL_UART_STATE_READY;

      /* Clear RxISR function pointer */
      huart->RxISR = NULL;

      /* Initialize type of RxEvent to Transfer Complete */
      huart->RxEventType = HAL_UART_RXEVENT_TC;

      if (!(IS_LPUART_INSTANCE(huart->Instance)))
      {
        /* Check that USART RTOEN bit is set */
        if (READ_BIT(huart->Instance->CR2, USART_CR2_RTOEN) != 0U)
        {
          /* Enable the UART Receiver Timeout Interrupt */
          ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_RTOIE);
        }
      }

      /* Check current reception Mode :
         If Reception till IDLE event has been selected : */
      if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
      {
        /* Set reception type to Standard */
        huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;

        /* Disable IDLE interrupt */
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);

        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == SET)
        {
          /* Clear IDLE Flag */
          __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
        }

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Rx Event callback*/
        huart->RxEventCallback(huart, huart->RxXferSize);
#else
        /*Call legacy weak Rx Event callback*/
        HAL_UARTEx_RxEventCallback(huart, huart->RxXferSize);
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS) */
      }
      else
      {
        /* Standard reception API called */
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Rx complete callback*/
        huart->RxCpltCallback(huart);
#else
        /*Call legacy weak Rx complete callback*/
        HAL_UART_RxCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
      }
    }
  }
  else
  {
    /* Clear RXNE interrupt flag */
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
  }
}
static void UART_EndRxTransfer(UART_HandleTypeDef *huart)
{
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
  ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

  /* In case of reception waiting for IDLE event, disable also the IDLE IE interrupt source */
  if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
  }

  /* At end of Rx process, restore huart->RxState to Ready */
  huart->RxState = HAL_UART_STATE_READY;
  huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;

  /* Reset RxIsr function pointer */
  huart->RxISR = NULL;
}

static void UART_RxISR_8BIT(UART_HandleTypeDef *huart)
{
  uint16_t uhMask = huart->Mask;
  uint16_t  uhdata;

  /* Check that a Rx process is ongoing */
  if (huart->RxState == HAL_UART_STATE_BUSY_RX)
  {
    uhdata = (uint16_t) READ_REG(huart->Instance->RDR);
    *huart->pRxBuffPtr = (uint8_t)(uhdata & (uint8_t)uhMask);
    huart->pRxBuffPtr++;
    huart->RxXferCount--;

    if (huart->RxXferCount == 0U)
    {
      /* Disable the UART Parity Error Interrupt and RXNE interrupts */
      ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));

      /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
      ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

      /* Rx process is completed, restore huart->RxState to Ready */
      huart->RxState = HAL_UART_STATE_READY;

      /* Clear RxISR function pointer */
      huart->RxISR = NULL;

      /* Initialize type of RxEvent to Transfer Complete */
      huart->RxEventType = HAL_UART_RXEVENT_TC;

      if (!(IS_LPUART_INSTANCE(huart->Instance)))
      {
        /* Check that USART RTOEN bit is set */
        if (READ_BIT(huart->Instance->CR2, USART_CR2_RTOEN) != 0U)
        {
          /* Enable the UART Receiver Timeout Interrupt */
          ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_RTOIE);
        }
      }

      /* Check current reception Mode :
         If Reception till IDLE event has been selected : */
      if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
      {
        /* Set reception type to Standard */
        huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;

        /* Disable IDLE interrupt */
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);

        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == SET)
        {
          /* Clear IDLE Flag */
          __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);
        }

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Rx Event callback*/
        huart->RxEventCallback(huart, huart->RxXferSize);
#else
        /*Call legacy weak Rx Event callback*/
        HAL_UARTEx_RxEventCallback(huart, huart->RxXferSize);
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS) */
      }
      else
      {
        /* Standard reception API called */
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Rx complete callback*/
        huart->RxCpltCallback(huart);
#else
        /*Call legacy weak Rx complete callback*/
        HAL_UART_RxCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
      }
    }
  }
  else
  {
    /* Clear RXNE interrupt flag */
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
  }
}

__weak void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_ErrorCallback can be implemented in the user file.
   */
}
__weak void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  UNUSED(Size);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UARTEx_RxEventCallback can be implemented in the user file.
   */
}
__weak void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_TxCpltCallback can be implemented in the user file.
   */
}
HAL_StatusTypeDef UART_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  huart->pRxBuffPtr  = pData;
  huart->RxXferSize  = Size;
  huart->RxXferCount = Size;
  huart->RxISR       = NULL;

  /* Computation of UART mask to apply to RDR register */
  UART_MASK_COMPUTATION(huart);

  huart->ErrorCode = HAL_UART_ERROR_NONE;
  huart->RxState = HAL_UART_STATE_BUSY_RX;

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

  /* Set the Rx ISR function pointer according to the data word length */
  if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
  {
    huart->RxISR = UART_RxISR_16BIT;
  }
  else
  {
    huart->RxISR = UART_RxISR_8BIT;
  }

  /* Enable the UART Parity Error interrupt and Data Register Not Empty interrupt */
  if (huart->Init.Parity != UART_PARITY_NONE)
  {
    ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
  }
  else
  {
    ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);
  }
  return HAL_OK;
}

HAL_StatusTypeDef UART_CheckIdleState(UART_HandleTypeDef *huart)
{
  uint32_t tickstart;

  /* Initialize the UART ErrorCode */
  huart->ErrorCode = HAL_UART_ERROR_NONE;

  /* Init tickstart for timeout management */
  tickstart = HAL_GetTick();

  /* Check if the Transmitter is enabled */
  if ((huart->Instance->CR1 & USART_CR1_TE) == USART_CR1_TE)
  {
    /* Wait until TEACK flag is set */
    if (UART_WaitOnFlagUntilTimeout(huart, USART_ISR_TEACK, RESET, tickstart, HAL_UART_TIMEOUT_VALUE) != HAL_OK)
    {
      /* Disable TXE interrupt for the interrupt process */
      ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TXEIE));

      huart->gState = HAL_UART_STATE_READY;

      __HAL_UNLOCK(huart);

      /* Timeout occurred */
      return HAL_TIMEOUT;
    }
  }

  /* Check if the Receiver is enabled */
  if ((huart->Instance->CR1 & USART_CR1_RE) == USART_CR1_RE)
  {
    /* Wait until REACK flag is set */
    if (UART_WaitOnFlagUntilTimeout(huart, USART_ISR_REACK, RESET, tickstart, HAL_UART_TIMEOUT_VALUE) != HAL_OK)
    {
      /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error)
      interrupts for the interrupt process */
      ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
      ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

      huart->RxState = HAL_UART_STATE_READY;

      __HAL_UNLOCK(huart);

      /* Timeout occurred */
      return HAL_TIMEOUT;
    }
  }

  /* Initialize the UART State */
  huart->gState = HAL_UART_STATE_READY;
  huart->RxState = HAL_UART_STATE_READY;
  huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
  huart->RxEventType = HAL_UART_RXEVENT_TC;

  __HAL_UNLOCK(huart);

  return HAL_OK;
}
HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
                                              uint32_t Tickstart, uint32_t Timeout)
{
  /* Wait until flag is set */
  while ((__HAL_UART_GET_FLAG(huart, Flag) ? SET : RESET) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
      {

        return HAL_TIMEOUT;
      }

      if (READ_BIT(huart->Instance->CR1, USART_CR1_RE) != 0U)
      {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) == SET)
        {
           /* Clear Overrun Error flag*/
           __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);

           /* Blocking error : transfer is aborted
           Set the UART state ready to be able to start again the process,
           Disable Rx Interrupts if ongoing */
           UART_EndRxTransfer(huart);

           huart->ErrorCode = HAL_UART_ERROR_ORE;

           /* Process Unlocked */
           __HAL_UNLOCK(huart);

           return HAL_ERROR;
        }
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RTOF) == SET)
        {
          /* Clear Receiver Timeout flag*/
          __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_RTOF);

          /* Blocking error : transfer is aborted
          Set the UART state ready to be able to start again the process,
          Disable Rx Interrupts if ongoing */
          UART_EndRxTransfer(huart);

          huart->ErrorCode = HAL_UART_ERROR_RTO;

          /* Process Unlocked */
          __HAL_UNLOCK(huart);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart)
{
  /* Check the UART handle allocation */
  if (huart == NULL)
  {
    return HAL_ERROR;
  }

  if (huart->Init.HwFlowCtl != UART_HWCONTROL_NONE)
  {
    /* Check the parameters */
    assert_param(IS_UART_HWFLOW_INSTANCE(huart->Instance));
  }
  else
  {
    /* Check the parameters */
    assert_param((IS_UART_INSTANCE(huart->Instance)) || (IS_LPUART_INSTANCE(huart->Instance)));
  }

  if (huart->gState == HAL_UART_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    huart->Lock = HAL_UNLOCKED;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    UART_InitCallbacksToDefault(huart);

    if (huart->MspInitCallback == NULL)
    {
      huart->MspInitCallback = HAL_UART_MspInit;
    }

    /* Init the low level hardware */
    huart->MspInitCallback(huart);
#else
    /* Init the low level hardware : GPIO, CLOCK */
    HAL_UART_MspInit(huart);
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS) */
  }

  huart->gState = HAL_UART_STATE_BUSY;

  __HAL_UART_DISABLE(huart);

  /* Set the UART Communication parameters */
  if (UART_SetConfig(huart) == HAL_ERROR)
  {
    return HAL_ERROR;
  }

  if (huart->AdvancedInit.AdvFeatureInit != UART_ADVFEATURE_NO_INIT)
  {
    UART_AdvFeatureConfig(huart);
  }

  /* In asynchronous mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CR2 register,
  - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
  CLEAR_BIT(huart->Instance->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  CLEAR_BIT(huart->Instance->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

  __HAL_UART_ENABLE(huart);

  /* TEACK and/or REACK to check before moving huart->gState and huart->RxState to Ready */
  return (UART_CheckIdleState(huart));
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  const uint8_t  *pdata8bits;
  const uint16_t *pdata16bits;
  uint32_t tickstart;

  /* Check that a Tx process is not already ongoing */
  if (huart->gState == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }

    /* In case of 9bits/No Parity transfer, pData buffer provided as input parameter
       should be aligned on a u16 frontier, as data to be filled into TDR will be
       handled through a u16 cast. */
    if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
    {
      if ((((uint32_t)pData) & 1U) != 0U)
      {
        return  HAL_ERROR;
      }
    }

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->gState = HAL_UART_STATE_BUSY_TX;

    /* Init tickstart for timeout management */
    tickstart = HAL_GetTick();

    huart->TxXferSize  = Size;
    huart->TxXferCount = Size;

    /* In case of 9bits/No Parity transfer, pData needs to be handled as a uint16_t pointer */
    if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
    {
      pdata8bits  = NULL;
      pdata16bits = (const uint16_t *) pData;
    }
    else
    {
      pdata8bits  = pData;
      pdata16bits = NULL;
    }

    while (huart->TxXferCount > 0U)
    {
      if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXE, RESET, tickstart, Timeout) != HAL_OK)
      {

        huart->gState = HAL_UART_STATE_READY;

        return HAL_TIMEOUT;
      }
      if (pdata8bits == NULL)
      {
        huart->Instance->TDR = (uint16_t)(*pdata16bits & 0x01FFU);
        pdata16bits++;
      }
      else
      {
        huart->Instance->TDR = (uint8_t)(*pdata8bits & 0xFFU);
        pdata8bits++;
      }
      huart->TxXferCount--;
    }

    if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TC, RESET, tickstart, Timeout) != HAL_OK)
    {
      huart->gState = HAL_UART_STATE_READY;

      return HAL_TIMEOUT;
    }

    /* At end of Tx process, restore huart->gState to Ready */
    huart->gState = HAL_UART_STATE_READY;

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  /* Check that a Rx process is not already ongoing */
  if (huart->RxState == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return HAL_ERROR;
    }

    /* In case of 9bits/No Parity transfer, pData buffer provided as input parameter
       should be aligned on a u16 frontier, as data to be received from RDR will be
       handled through a u16 cast. */
    if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
    {
      if ((((uint32_t)pData) & 1U) != 0U)
      {
        return  HAL_ERROR;
      }
    }

    /* Set Reception type to Standard reception */
    huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;

    if (!(IS_LPUART_INSTANCE(huart->Instance)))
    {
      /* Check that USART RTOEN bit is set */
      if (READ_BIT(huart->Instance->CR2, USART_CR2_RTOEN) != 0U)
      {
        /* Enable the UART Receiver Timeout Interrupt */
        ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_RTOIE);
      }
    }

    return (UART_Start_Receive_IT(huart, pData, Size));
  }
  else
  {
    return HAL_BUSY;
  }
}

static void UART_EndTransmit_IT(UART_HandleTypeDef *huart)
{
  /* Disable the UART Transmit Complete Interrupt */
  ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TCIE);

  /* Tx process is ended, restore huart->gState to Ready */
  huart->gState = HAL_UART_STATE_READY;

  /* Cleat TxISR function pointer */
  huart->TxISR = NULL;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  /*Call registered Tx complete callback*/
  huart->TxCpltCallback(huart);
#else
  /*Call legacy weak Tx complete callback*/
  HAL_UART_TxCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}
static void UART_DMAAbortOnError(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)(hdma->Parent);
  huart->RxXferCount = 0U;
  huart->TxXferCount = 0U;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  /*Call registered error callback*/
  huart->ErrorCallback(huart);
#else
  /*Call legacy weak error callback*/
  HAL_UART_ErrorCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t isrflags   = READ_REG(huart->Instance->ISR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its     = READ_REG(huart->Instance->CR3);

  uint32_t errorflags;
  uint32_t errorcode;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF));
  if (errorflags == 0U)
  {
    /* UART in mode Receiver ---------------------------------------------------*/
    if (((isrflags & USART_ISR_RXNE) != 0U)
        && ((cr1its & USART_CR1_RXNEIE) != 0U))
    {
      if (huart->RxISR != NULL)
      {
        huart->RxISR(huart);
      }
      return;
    }
  }

  /* If some errors occur */
  if ((errorflags != 0U)
      && (((cr3its & USART_CR3_EIE) != 0U)
          || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_RTOIE)) != 0U)))
  {
    /* UART parity error interrupt occurred -------------------------------------*/
    if (((isrflags & USART_ISR_PE) != 0U) && ((cr1its & USART_CR1_PEIE) != 0U))
    {
      __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);

      huart->ErrorCode |= HAL_UART_ERROR_PE;
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if (((isrflags & USART_ISR_FE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U))
    {
      __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);

      huart->ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if (((isrflags & USART_ISR_NE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U))
    {
      __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);

      huart->ErrorCode |= HAL_UART_ERROR_NE;
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if (((isrflags & USART_ISR_ORE) != 0U)
        && (((cr1its & USART_CR1_RXNEIE) != 0U) ||
            ((cr3its & USART_CR3_EIE) != 0U)))
    {
      __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);

      huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }

    /* UART Receiver Timeout interrupt occurred ---------------------------------*/
    if (((isrflags & USART_ISR_RTOF) != 0U) && ((cr1its & USART_CR1_RTOIE) != 0U))
    {
      __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_RTOF);

      huart->ErrorCode |= HAL_UART_ERROR_RTO;
    }

    /* Call UART Error Call back function if need be ----------------------------*/
    if (huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
      /* UART in mode Receiver --------------------------------------------------*/
      if (((isrflags & USART_ISR_RXNE) != 0U)
          && ((cr1its & USART_CR1_RXNEIE) != 0U))
      {
        if (huart->RxISR != NULL)
        {
          huart->RxISR(huart);
        }
      }

      /* If Error is to be considered as blocking :
          - Receiver Timeout error in Reception
          - Overrun error in Reception
          - any error occurs in DMA mode reception
      */
      errorcode = huart->ErrorCode;
      if ((HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR)) ||
          ((errorcode & (HAL_UART_ERROR_RTO | HAL_UART_ERROR_ORE)) != 0U))
      {
        /* Blocking error : transfer is aborted
           Set the UART state ready to be able to start again the process,
           Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
        UART_EndRxTransfer(huart);

        /* Abort the UART DMA Rx channel if enabled */
        if (HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR))
        {
          /* Disable the UART DMA Rx request if enabled */
          ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

          /* Abort the UART DMA Rx channel */
          if (huart->hdmarx != NULL)
          {
            /* Set the UART DMA Abort callback :
               will lead to call HAL_UART_ErrorCallback() at end of DMA abort procedure */
            huart->hdmarx->XferAbortCallback = UART_DMAAbortOnError;

            /* Abort DMA RX */
            if (HAL_DMA_Abort_IT(huart->hdmarx) != HAL_OK)
            {
              /* Call Directly huart->hdmarx->XferAbortCallback function in case of error */
              huart->hdmarx->XferAbortCallback(huart->hdmarx);
            }
          }
          else
          {
            /* Call user error callback */
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
            /*Call registered error callback*/
            huart->ErrorCallback(huart);
#else
            /*Call legacy weak error callback*/
            HAL_UART_ErrorCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

          }
        }
        else
        {
          /* Call user error callback */
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
          /*Call registered error callback*/
          huart->ErrorCallback(huart);
#else
          /*Call legacy weak error callback*/
          HAL_UART_ErrorCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
        }
      }
      else
      {
        /* Non Blocking error : transfer could go on.
           Error is notified to user through user error callback */
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered error callback*/
        huart->ErrorCallback(huart);
#else
        /*Call legacy weak error callback*/
        HAL_UART_ErrorCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
        huart->ErrorCode = HAL_UART_ERROR_NONE;
      }
    }
    return;

  } /* End if some error occurs */

  /* Check current reception Mode :
     If Reception till IDLE event has been selected : */
  if ((huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
      && ((isrflags & USART_ISR_IDLE) != 0U)
      && ((cr1its & USART_ISR_IDLE) != 0U))
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF);

    /* Check if DMA mode is enabled in UART */
    if (HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR))
    {
      /* DMA mode enabled */
      /* Check received length : If all expected data are received, do nothing,
         (DMA cplt callback will be called).
         Otherwise, if at least one data has already been received, IDLE event is to be notified to user */
      uint16_t nb_remaining_rx_data = (uint16_t) __HAL_DMA_GET_COUNTER(huart->hdmarx);
      if ((nb_remaining_rx_data > 0U)
          && (nb_remaining_rx_data < huart->RxXferSize))
      {
        /* Reception is not complete */
        huart->RxXferCount = nb_remaining_rx_data;

        /* In Normal mode, end DMA xfer and HAL UART Rx process*/
        if (HAL_IS_BIT_CLR(huart->hdmarx->Instance->CCR, DMA_CCR_CIRC))
        {
          /* Disable PE and ERR (Frame error, noise error, overrun error) interrupts */
          ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);
          ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

          /* Disable the DMA transfer for the receiver request by resetting the DMAR bit
             in the UART CR3 register */
          ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

          /* At end of Rx process, restore huart->RxState to Ready */
          huart->RxState = HAL_UART_STATE_READY;
          huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;

          ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);

          /* Last bytes received, so no need as the abort is immediate */
          (void)HAL_DMA_Abort(huart->hdmarx);
        }

        /* Initialize type of RxEvent that correspond to RxEvent callback execution;
           In this case, Rx Event type is Idle Event */
        huart->RxEventType = HAL_UART_RXEVENT_IDLE;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Rx Event callback*/
        huart->RxEventCallback(huart, (huart->RxXferSize - huart->RxXferCount));
#else
        /*Call legacy weak Rx Event callback*/
        HAL_UARTEx_RxEventCallback(huart, (huart->RxXferSize - huart->RxXferCount));
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS) */
      }
      return;
    }
    else
    {
      /* DMA mode not enabled */
      /* Check received length : If all expected data are received, do nothing.
         Otherwise, if at least one data has already been received, IDLE event is to be notified to user */
      uint16_t nb_rx_data = huart->RxXferSize - huart->RxXferCount;
      if ((huart->RxXferCount > 0U)
          && (nb_rx_data > 0U))
      {
        /* Disable the UART Parity Error Interrupt and RXNE interrupts */
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));

        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        /* Rx process is completed, restore huart->RxState to Ready */
        huart->RxState = HAL_UART_STATE_READY;
        huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;

        /* Clear RxISR function pointer */
        huart->RxISR = NULL;

        ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);

        /* Initialize type of RxEvent that correspond to RxEvent callback execution;
           In this case, Rx Event type is Idle Event */
        huart->RxEventType = HAL_UART_RXEVENT_IDLE;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Rx complete callback*/
        huart->RxEventCallback(huart, nb_rx_data);
#else
        /*Call legacy weak Rx Event callback*/
        HAL_UARTEx_RxEventCallback(huart, nb_rx_data);
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS) */
      }
      return;
    }
  }

  /* UART wakeup from Stop mode interrupt occurred ---------------------------*/
  if (((isrflags & USART_ISR_WUF) != 0U) && ((cr3its & USART_CR3_WUFIE) != 0U))
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_WUF);

    /* UART Rx state is not reset as a reception process might be ongoing.
       If UART handle state fields need to be reset to READY, this could be done in Wakeup callback */

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    /* Call registered Wakeup Callback */
    huart->WakeupCallback(huart);
#else
    /* Call legacy weak Wakeup Callback */
    HAL_UARTEx_WakeupCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
    return;
  }

  /* UART in mode Transmitter ------------------------------------------------*/
  if (((isrflags & USART_ISR_TXE) != 0U)
      && ((cr1its & USART_CR1_TXEIE) != 0U))
  {
    if (huart->TxISR != NULL)
    {
      huart->TxISR(huart);
    }
    return;
  }

  /* UART in mode Transmitter (transmission end) -----------------------------*/
  if (((isrflags & USART_ISR_TC) != 0U) && ((cr1its & USART_CR1_TCIE) != 0U))
  {
    UART_EndTransmit_IT(huart);
    return;
  }

}
