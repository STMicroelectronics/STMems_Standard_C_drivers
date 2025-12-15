/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i3c.h
  * @brief   This file contains all the function prototypes for
  *          the i3c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I3C_H__
#define __I3C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

extern I3C_HandleTypeDef hi3c1;

/* USER CODE BEGIN Private defines */

#define I3C_WAIT_TIME 1000
#define I3C_MAX_IBI_PAYLOAD_SIZE 9
#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* USER CODE END Private defines */

void MX_I3C1_Init(void);

/* USER CODE BEGIN Prototypes */

int32_t i3c_rstdaa(void);
int32_t i3c_enec(void);
int32_t i3c_disec(void);
int32_t i3c_write(uint16_t addr, uint16_t reg, uint8_t *pdata, uint16_t len);
int32_t i3c_read(uint16_t addr, uint16_t reg, uint8_t *pdata, uint16_t len);
int32_t i3c_setdasa(uint8_t addr, uint8_t *cccdata, uint16_t len);
int32_t i3c_set_bus_frequency(uint32_t i3c_freq);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I3C_H__ */

