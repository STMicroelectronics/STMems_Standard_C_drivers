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

#ifndef I3C_API_H
#define I3C_API_H

#include <stdint.h>
#include "stm32h5xx_hal.h"
#include "stm32h5xx_util_i3c.h"

int32_t i3c_rstdaa(I3C_HandleTypeDef *handle);
int32_t i3c_set_bus_frequency(I3C_HandleTypeDef *handle, uint32_t i3c_freq);
int32_t i3c_setdasa(I3C_HandleTypeDef *handle, uint8_t addr, uint8_t *cccdata, uint16_t len);
int32_t i3c_write(I3C_HandleTypeDef *handle, uint16_t addr, uint16_t reg, uint8_t *pdata, uint16_t len);
int32_t i3c_read(I3C_HandleTypeDef *handle, uint16_t addr, uint16_t reg, uint8_t *pdata, uint16_t len);

#endif /* I3C_API_H */
