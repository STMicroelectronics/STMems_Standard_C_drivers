/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i3c.c
  * @brief   This file provides code for the configuration
  *          of the I3C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i3c.h"

/* USER CODE BEGIN 0 */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "stm32h5xx_util_i3c.h"

/* USER CODE END 0 */

I3C_HandleTypeDef hi3c1;

/* I3C1 init function */
void MX_I3C1_Init(void)
{

  /* USER CODE BEGIN I3C1_Init 0 */

  /* USER CODE END I3C1_Init 0 */

  I3C_FifoConfTypeDef sFifoConfig = {0};
  I3C_CtrlConfTypeDef sCtrlConfig = {0};

  /* USER CODE BEGIN I3C1_Init 1 */

  /* USER CODE END I3C1_Init 1 */
  hi3c1.Instance = I3C1;
  hi3c1.Mode = HAL_I3C_MODE_CONTROLLER;
  hi3c1.Init.CtrlBusCharacteristic.SDAHoldTime = HAL_I3C_SDA_HOLD_TIME_1_5;
  hi3c1.Init.CtrlBusCharacteristic.WaitTime = HAL_I3C_OWN_ACTIVITY_STATE_0;
  hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x09;
  hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x09;
  hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0x59;
  hi3c1.Init.CtrlBusCharacteristic.SCLI2CHighDuration = 0x00;
  hi3c1.Init.CtrlBusCharacteristic.BusFreeDuration = 0x32;
  hi3c1.Init.CtrlBusCharacteristic.BusIdleDuration = 0xf8;
  if (HAL_I3C_Init(&hi3c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure FIFO
  */
  sFifoConfig.RxFifoThreshold = HAL_I3C_RXFIFO_THRESHOLD_1_4;
  sFifoConfig.TxFifoThreshold = HAL_I3C_TXFIFO_THRESHOLD_1_4;
  sFifoConfig.ControlFifo = HAL_I3C_CONTROLFIFO_DISABLE;
  sFifoConfig.StatusFifo = HAL_I3C_STATUSFIFO_DISABLE;
  if (HAL_I3C_SetConfigFifo(&hi3c1, &sFifoConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure controller
  */
  sCtrlConfig.DynamicAddr = 0;
  sCtrlConfig.StallTime = 0x00;
  sCtrlConfig.HotJoinAllowed = DISABLE;
  sCtrlConfig.ACKStallState = DISABLE;
  sCtrlConfig.CCCStallState = DISABLE;
  sCtrlConfig.TxStallState = DISABLE;
  sCtrlConfig.RxStallState = DISABLE;
  sCtrlConfig.HighKeeperSDA = DISABLE;
  if (HAL_I3C_Ctrl_Config(&hi3c1, &sCtrlConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I3C1_Init 2 */

  /* USER CODE END I3C1_Init 2 */

}

void HAL_I3C_MspInit(I3C_HandleTypeDef* i3cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(i3cHandle->Instance==I3C1)
  {
  /* USER CODE BEGIN I3C1_MspInit 0 */

  /* USER CODE END I3C1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I3C1;
    PeriphClkInitStruct.I3c1ClockSelection = RCC_I3C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* I3C1 clock enable */
    __HAL_RCC_I3C1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I3C1 GPIO Configuration
    PB8     ------> I3C1_SCL
    PB9     ------> I3C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_I3C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I3C1 interrupt Init */
    HAL_NVIC_SetPriority(I3C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I3C1_EV_IRQn);
  /* USER CODE BEGIN I3C1_MspInit 1 */

  /* USER CODE END I3C1_MspInit 1 */
  }
}

void HAL_I3C_MspDeInit(I3C_HandleTypeDef* i3cHandle)
{

  if(i3cHandle->Instance==I3C1)
  {
  /* USER CODE BEGIN I3C1_MspDeInit 0 */

  /* USER CODE END I3C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I3C1_CLK_DISABLE();

    /**I3C1 GPIO Configuration
    PB8     ------> I3C1_SCL
    PB9     ------> I3C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* I3C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I3C1_EV_IRQn);
  /* USER CODE BEGIN I3C1_MspDeInit 1 */

  /* USER CODE END I3C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

static int32_t i3c_send_ccc(I3C_CCCTypeDef *ccc_descriptor, I3C_XferTypeDef *aContextBuffers, uint32_t n_frame, uint32_t option)
{
  int32_t ret = 0;
  if (HAL_I3C_AddDescToFrame(&hi3c1, ccc_descriptor, NULL, aContextBuffers, n_frame, option) != HAL_OK)
    ret = -1;
  else if (HAL_I3C_Ctrl_TransmitCCC(&hi3c1, aContextBuffers, I3C_WAIT_TIME) != HAL_OK)
    ret = -1;

  return ret;
}

int32_t i3c_rstdaa(void)
{
  uint32_t aControlBuffer[0xFF];
  I3C_XferTypeDef aContextBuffers;
  uint8_t aTxBuffer[0x0F];

  I3C_CCCTypeDef rstdaa_desc[] = {
    { 0x00, 0x06, { NULL, 0 }, LL_I3C_DIRECTION_WRITE },
  };

  aContextBuffers.CtrlBuf.pBuffer = aControlBuffer;
  aContextBuffers.CtrlBuf.Size = COUNTOF(aControlBuffer);
  aContextBuffers.TxBuf.pBuffer = aTxBuffer;
  aContextBuffers.TxBuf.Size = 4;

  return i3c_send_ccc(rstdaa_desc, &aContextBuffers, COUNTOF(rstdaa_desc), I3C_BROADCAST_WITHOUT_DEFBYTE_RESTART);
}

int32_t i3c_enec(void)
{
  uint32_t aControlBuffer[0xFF];
  I3C_XferTypeDef aContextBuffers;
  uint8_t aTxBuffer[0x0F];
  // supports enint only
  uint8_t event_byte = 0x01;

  // supports broadcast enec only
  I3C_CCCTypeDef enec_desc[] = {
    { 0x00, 0x00, { &event_byte, 1 }, LL_I3C_DIRECTION_WRITE },
  };

  aContextBuffers.CtrlBuf.pBuffer = aControlBuffer;
  aContextBuffers.CtrlBuf.Size = COUNTOF(aControlBuffer);
  aContextBuffers.TxBuf.pBuffer = aTxBuffer;
  aContextBuffers.TxBuf.Size = 4;

  return i3c_send_ccc(enec_desc, &aContextBuffers, COUNTOF(enec_desc), I3C_BROADCAST_WITHOUT_DEFBYTE_RESTART);
}

int32_t i3c_disec(void)
{
  uint32_t aControlBuffer[0xFF];
  I3C_XferTypeDef aContextBuffers;
  uint8_t aTxBuffer[0x0F];
  // supports disint only
  uint8_t event_byte = 0x01;

  I3C_CCCTypeDef disec_desc[] = {
    { 0x00, 0x01, { &event_byte, 1 }, LL_I3C_DIRECTION_WRITE },
  };

  aContextBuffers.CtrlBuf.pBuffer = aControlBuffer;
  aContextBuffers.CtrlBuf.Size = COUNTOF(aControlBuffer);
  aContextBuffers.TxBuf.pBuffer = aTxBuffer;
  aContextBuffers.TxBuf.Size = 4;

  return i3c_send_ccc(disec_desc, &aContextBuffers, COUNTOF(disec_desc), I3C_BROADCAST_WITHOUT_DEFBYTE_RESTART);
}

int32_t i3c_write(uint16_t addr, uint16_t reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = 0;

  I3C_XferTypeDef contextBuffer;
  uint32_t controlBuffer[0xF];
  I3C_PrivateTypeDef privateDescriptor;
  uint8_t data[32] = { 0 };
  data[0] = (uint8_t)reg;
  memcpy(&data[1], pdata, len);

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

  if (HAL_I3C_AddDescToFrame(&hi3c1, NULL, &privateDescriptor, &contextBuffer, contextBuffer.CtrlBuf.Size, I3C_PRIVATE_WITH_ARB_RESTART) != HAL_OK)
    ret = -1;
  else if (HAL_I3C_Ctrl_Transmit(&hi3c1, &contextBuffer, I3C_WAIT_TIME) != HAL_OK)
    ret = -1;

  return ret;
}

int32_t i3c_read(uint16_t addr, uint16_t reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = 0;

  I3C_XferTypeDef contextBuffer;
  uint32_t controlBuffer[0xF];
  I3C_PrivateTypeDef privateDescriptor;
  uint8_t myReg[1] = { (uint8_t)reg };
  uint8_t data[32] = { 0 };

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

  if (HAL_I3C_AddDescToFrame(&hi3c1, NULL, &privateDescriptor, &contextBuffer, contextBuffer.CtrlBuf.Size, I3C_PRIVATE_WITH_ARB_RESTART) != HAL_OK)
    ret = -1;
  else if (HAL_I3C_Ctrl_Transmit(&hi3c1, &contextBuffer, I3C_WAIT_TIME) != HAL_OK)
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

    if (HAL_I3C_AddDescToFrame(&hi3c1, NULL, &privateDescriptor, &contextBuffer, contextBuffer.CtrlBuf.Size, I3C_PRIVATE_WITH_ARB_STOP) != HAL_OK)
      ret = -1;
    else if (HAL_I3C_Ctrl_Receive(&hi3c1, &contextBuffer, I3C_WAIT_TIME) != HAL_OK)
      ret = -1;
    else
      memcpy(pdata, data, len);
  }

  return ret;
}

int32_t i3c_setdasa(uint8_t addr, uint8_t *cccdata, uint16_t len)
{
  uint32_t aControlBuffer[0xFF];
  I3C_XferTypeDef aContextBuffers;
  uint8_t aTxBuffer[0x0F];

  I3C_CCCTypeDef setdasa_desc[] = {
    { addr, 0x87, { cccdata, len }, LL_I3C_DIRECTION_WRITE},
  };

  aContextBuffers.CtrlBuf.pBuffer = aControlBuffer;
  aContextBuffers.CtrlBuf.Size = COUNTOF(aControlBuffer);
  aContextBuffers.TxBuf.pBuffer = aTxBuffer;
  aContextBuffers.TxBuf.Size = 4;

  return i3c_send_ccc(setdasa_desc, &aContextBuffers, COUNTOF(setdasa_desc), I3C_DIRECT_WITHOUT_DEFBYTE_RESTART);
}

int32_t i3c_set_bus_frequency(uint32_t i3c_freq)
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
  else if (HAL_I3C_Ctrl_BusCharacteristicConfig(&hi3c1, &ctrl_bus_conf) != 0)
    return -1;

  return 0;
}

/* USER CODE END 1 */
