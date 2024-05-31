/*
 ******************************************************************************
 * @file    i3c_api.c
 * @author  Sensors Software Solution Team
 * @brief   Set of useful APIs to abstract common I3C bus operations
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "i3c_api.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

int32_t i3c_rstdaa(I3C_HandleTypeDef *handle)
{
  int32_t ret = 0;

  uint32_t aControlBuffer[0xFF];
  I3C_XferTypeDef aContextBuffers;
  uint8_t aTxBuffer[0x0F];

  I3C_CCCTypeDef aSET_CCC_RST[] = {
    { 0x00, 0x06, { NULL, 0 }, LL_I3C_DIRECTION_WRITE },
  };

  aContextBuffers.CtrlBuf.pBuffer = aControlBuffer;
  aContextBuffers.CtrlBuf.Size = COUNTOF(aControlBuffer);
  aContextBuffers.TxBuf.pBuffer = aTxBuffer;
  aContextBuffers.TxBuf.Size = 4;

  if (HAL_I3C_AddDescToFrame(handle, aSET_CCC_RST, NULL, &aContextBuffers, COUNTOF(aSET_CCC_RST), I3C_BROADCAST_WITHOUT_DEFBYTE_RESTART) != HAL_OK)
    ret = -1;
  else if (HAL_I3C_Ctrl_TransmitCCC(handle, &aContextBuffers, 1000) != HAL_OK)
    ret = -1;

  return ret;
}

int32_t i3c_write(I3C_HandleTypeDef *handle, uint16_t addr, uint16_t reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = 0;

  I3C_XferTypeDef contextBuffer;
  uint32_t controlBuffer[0xF];
  I3C_PrivateTypeDef privateDescriptor;
  uint8_t data[32] = { 0 };
  data[0] = (uint8_t)reg;
  memcpy(&data[1], pdata, len);

  addr >>= 1;

  privateDescriptor.TargetAddr = addr;
  privateDescriptor.TxBuf.pBuffer = data;
  privateDescriptor.TxBuf.Size = (1 + len);
  privateDescriptor.RxBuf.pBuffer = NULL;
  privateDescriptor.RxBuf.Size = 0;
  privateDescriptor.Direction = HAL_I3C_DIRECTION_WRITE;

  memset((void *)&contextBuffer, 0x0, sizeof(I3C_XferTypeDef));
  contextBuffer.CtrlBuf.pBuffer = controlBuffer;
  contextBuffer.CtrlBuf.Size = 1;
  contextBuffer.TxBuf.pBuffer = data;
  contextBuffer.TxBuf.Size = (1 + len);

  if (HAL_I3C_AddDescToFrame(handle, NULL, &privateDescriptor, &contextBuffer, contextBuffer.CtrlBuf.Size, I3C_PRIVATE_WITH_ARB_RESTART) != HAL_OK)
    ret = -1;
  else if (HAL_I3C_Ctrl_Transmit(handle, &contextBuffer, 1000) != HAL_OK)
    ret = -1;

  return ret;
}

int32_t i3c_read(I3C_HandleTypeDef *handle, uint16_t addr, uint16_t reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = 0;

  I3C_XferTypeDef contextBuffer;
  uint32_t controlBuffer[0xF];
  I3C_PrivateTypeDef privateDescriptor;
  uint8_t myReg[1] = { (uint8_t)reg };
  uint8_t data[32] = { 0 };

  addr >>= 1;

  privateDescriptor.TargetAddr = addr;
  privateDescriptor.TxBuf.pBuffer = myReg;
  privateDescriptor.TxBuf.Size = 1;
  privateDescriptor.RxBuf.pBuffer = NULL;
  privateDescriptor.RxBuf.Size = 0;
  privateDescriptor.Direction = HAL_I3C_DIRECTION_WRITE;

  memset((void *)&contextBuffer, 0x0, sizeof(I3C_XferTypeDef));
  contextBuffer.CtrlBuf.pBuffer = controlBuffer;
  contextBuffer.CtrlBuf.Size = 1;
  contextBuffer.TxBuf.pBuffer = myReg;
  contextBuffer.TxBuf.Size = 1;

  if (HAL_I3C_AddDescToFrame(handle, NULL, &privateDescriptor, &contextBuffer, contextBuffer.CtrlBuf.Size, I3C_PRIVATE_WITH_ARB_RESTART) != HAL_OK)
    ret = -1;
  else if (HAL_I3C_Ctrl_Transmit(handle, &contextBuffer, 1000) != HAL_OK)
    ret = -1;

  if (ret == 0) {
    privateDescriptor.TargetAddr = addr;
    privateDescriptor.TxBuf.pBuffer = NULL;
    privateDescriptor.TxBuf.Size = 0;
    privateDescriptor.RxBuf.pBuffer = data;
    privateDescriptor.RxBuf.Size = len;
    privateDescriptor.Direction = HAL_I3C_DIRECTION_READ;

    memset((void *)&contextBuffer, 0x0, sizeof(I3C_XferTypeDef));
    contextBuffer.CtrlBuf.pBuffer = controlBuffer;
    contextBuffer.CtrlBuf.Size = 1;
    contextBuffer.RxBuf.pBuffer = data;
    contextBuffer.RxBuf.Size = len;

    if (HAL_I3C_AddDescToFrame(handle, NULL, &privateDescriptor, &contextBuffer, contextBuffer.CtrlBuf.Size, I3C_PRIVATE_WITH_ARB_STOP) != HAL_OK)
      ret = -1;
    else if (HAL_I3C_Ctrl_Receive(handle, &contextBuffer, 1000) != HAL_OK)
      ret = -1;
    else
      memcpy(pdata, data, len);
  }

  return ret;
}

int32_t i3c_setdasa(I3C_HandleTypeDef *handle, uint8_t addr, uint8_t *cccdata, uint16_t len)
{
  int32_t ret = 0;

  uint32_t aControlBuffer[0xFF];
  I3C_XferTypeDef aContextBuffers;
  uint8_t aTxBuffer[0x0F];

  I3C_CCCTypeDef aSET_DASA_Desc[] = {
    { addr >> 1, 0x87, { cccdata, len }, LL_I3C_DIRECTION_WRITE},
  };

  aContextBuffers.CtrlBuf.pBuffer = aControlBuffer;
  aContextBuffers.CtrlBuf.Size = COUNTOF(aControlBuffer);
  aContextBuffers.TxBuf.pBuffer = aTxBuffer;
  aContextBuffers.TxBuf.Size = 4;

  if (HAL_I3C_AddDescToFrame(handle, aSET_DASA_Desc, NULL, &aContextBuffers, COUNTOF(aSET_DASA_Desc), I3C_DIRECT_WITHOUT_DEFBYTE_RESTART) != HAL_OK)
    ret = -1;
  else if (HAL_I3C_Ctrl_TransmitCCC(handle, &aContextBuffers, 1000) != HAL_OK)
    ret = -1;

  return ret;
}

int32_t i3c_set_bus_frequency(I3C_HandleTypeDef *handle, uint32_t i3c_freq)
{
  const I3C_CtrlTimingTypeDef input_timing = {
    .clockSrcFreq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I3C1),
    .i3cPPFreq = i3c_freq,
    .i2cODFreq = 0,
    .dutyCycle = 50,
    .busType = I3C_PURE_I3C_BUS
  };

  LL_I3C_CtrlBusConfTypeDef ctrl_bus_conf;

  if (I3C_CtrlTimingComputation(&input_timing, &ctrl_bus_conf))
    return -1;
  else if (HAL_I3C_Ctrl_BusCharacteristicConfig(handle, &ctrl_bus_conf) != 0)
    return -1;

  return 0;
}
