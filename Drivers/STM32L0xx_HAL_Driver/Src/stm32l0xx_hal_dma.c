#include "stm32l0xx_hal.h"
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1cU));
  hdma->Instance->CNDTR = DataLength;
  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {
    hdma->Instance->CPAR = DstAddress;
    hdma->Instance->CMAR = SrcAddress;
  }
  else
  {
    hdma->Instance->CPAR = SrcAddress;
    hdma->Instance->CMAR = DstAddress;
  }
}
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;
  assert_param(IS_DMA_BUFFER_SIZE(DataLength));
  __HAL_LOCK(hdma);

  if(HAL_DMA_STATE_READY == hdma->State)
  {
    hdma->State = HAL_DMA_STATE_BUSY;
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;
    __HAL_DMA_DISABLE(hdma);
    DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);
    if(NULL != hdma->XferHalfCpltCallback )
    {
      __HAL_DMA_ENABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));
    }
    else
    {
      __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
      __HAL_DMA_ENABLE_IT(hdma, (DMA_IT_TC | DMA_IT_TE));
    }
    __HAL_DMA_ENABLE(hdma);
  }
  else
  {
    __HAL_UNLOCK(hdma);
    status = HAL_BUSY;
  }
  return status;
}

HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma)
{
  HAL_StatusTypeDef status = HAL_OK;
  if(hdma->State != HAL_DMA_STATE_BUSY)
  {
    hdma->ErrorCode = HAL_DMA_ERROR_NO_XFER;
    __HAL_UNLOCK(hdma);

    return HAL_ERROR;
  }
  else
  {
    __HAL_DMA_DISABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));
    __HAL_DMA_DISABLE(hdma);
    hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1cU));
    hdma->State = HAL_DMA_STATE_READY;
    __HAL_UNLOCK(hdma);

    return status;
  }
}

HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma)
{
  HAL_StatusTypeDef status = HAL_OK;

  if(HAL_DMA_STATE_BUSY != hdma->State)
  {
    hdma->ErrorCode = HAL_DMA_ERROR_NO_XFER;

    status = HAL_ERROR;
  }
  else
  {
    __HAL_DMA_DISABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));
    __HAL_DMA_DISABLE(hdma);
    hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1cU));
    hdma->State = HAL_DMA_STATE_READY;
    __HAL_UNLOCK(hdma);
    if(hdma->XferAbortCallback != NULL)
    {
      hdma->XferAbortCallback(hdma);
    }
  }
  return status;
}

uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *hdma)
{
  return hdma->ErrorCode;
}

