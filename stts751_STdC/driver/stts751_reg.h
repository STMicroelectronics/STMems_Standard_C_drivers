/*
 ******************************************************************************
 * @file    stts751_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          stts751_reg.c driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STTS751_REGS_H
#define STTS751_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup STTS751
  * @{
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct{
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*stmdev_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** @defgroup    Generic address-data structure definition
  * @brief       This structure is useful to load a predefined configuration
  *              of a sensor.
	*              You can create a sensor configuration by your own or using 
	*              Unico / Unicleo tools available on STMicroelectronics
	*              web site.
  *
  * @{
  *
  */

typedef struct {
  uint8_t address;
  uint8_t data;
} ucf_line_t;

/**
  * @}
  *
  */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
  * @}
  *
  */

/** @defgroup STTS751_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format **/
#define STTS751_0xxxx_ADD_7K5  0x91U
#define STTS751_0xxxx_ADD_12K  0x93U
#define STTS751_0xxxx_ADD_20K  0x71U
#define STTS751_0xxxx_ADD_33K  0x73U

#define STTS751_1xxxx_ADD_7K5  0x95U
#define STTS751_1xxxx_ADD_12K  0x97U
#define STTS751_1xxxx_ADD_20K  0x75U
#define STTS751_1xxxx_ADD_33K  0x77U

/** Device Identification **/
/* Product ID */
#define STTS751_ID_0xxxx       0x00U
#define STTS751_ID_1xxxx       0x01U
/* Manufacturer ID */
#define STTS751_ID_MAN         0x53U
/* Revision number */
#define STTS751_REV            0x01U

/**
  * @}
  *
  */

#define STTS751_TEMPERATURE_HIGH            0x00U
#define STTS751_STATUS                      0x01U
typedef struct {
  uint8_t thrm                       : 1;
  uint8_t not_used_01                : 4;
  uint8_t t_low                      : 1;
  uint8_t t_high                     : 1;
  uint8_t busy                       : 1;
} stts751_status_t;

#define STTS751_TEMPERATURE_LOW             0x02U
#define STTS751_CONFIGURATION               0x03U
typedef struct {
  uint8_t not_used_01                : 2;
  uint8_t tres                       : 2;
  uint8_t not_used_02                : 2;
  uint8_t stop                       : 1;
  uint8_t mask1                      : 1;
} stts751_configuration_t;

#define STTS751_CONVERSION_RATE             0x04U
typedef struct {
  uint8_t conv                       : 4;
  uint8_t not_used_01                : 4;
} stts751_conversion_rate_t;

#define STTS751_TEMPERATURE_HIGH_LIMIT_HIGH 0x05U
#define STTS751_TEMPERATURE_HIGH_LIMIT_LOW  0x06U
#define STTS751_TEMPERATURE_LOW_LIMIT_HIGH  0x07U
#define STTS751_TEMPERATURE_LOW_LIMIT_LOW   0x08U
#define STTS751_ONE_SHOT                    0x0FU
#define STTS751_THERM_LIMIT                 0x20U
#define STTS751_THERM_HYSTERESIS            0x21U
#define STTS751_SMBUS_TIMEOUT               0x22U
typedef struct {
  uint8_t not_used_01                : 7;
  uint8_t timeout                    : 1;
} stts751_smbus_timeout_t;

#define STTS751_PRODUCT_ID                  0xFDU
#define STTS751_MANUFACTURER_ID             0xFEU
#define STTS751_REVISION_ID                 0xFFU

/**
  * @defgroup STTS751_Register_Union
  * @brief    This union group all the registers that has a bitfield
  *           description.
  *           This union is useful but not need by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */
typedef union{
  stts751_status_t                       status;
  stts751_configuration_t                configuration;
  stts751_conversion_rate_t              conversion_rate;
  stts751_smbus_timeout_t                smbus_timeout;
  bitwise_t                              bitwise;
  uint8_t                                byte;
} stts751_reg_t;

/**
  * @}
  *
  */

int32_t stts751_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);
int32_t stts751_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                           uint16_t len);

extern float stts751_from_lsb_to_celsius(int16_t lsb);
extern int16_t stts751_from_celsius_to_lsb(float celsius);

typedef enum {
  STTS751_TEMP_ODR_OFF        = 0x80,
  STTS751_TEMP_ODR_ONE_SHOT   = 0x90,
  STTS751_TEMP_ODR_62mHz5     = 0x00,
  STTS751_TEMP_ODR_125mHz     = 0x01,
  STTS751_TEMP_ODR_250mHz     = 0x02,
  STTS751_TEMP_ODR_500mHz     = 0x03,
  STTS751_TEMP_ODR_1Hz        = 0x04,
  STTS751_TEMP_ODR_2Hz        = 0x05,
  STTS751_TEMP_ODR_4Hz        = 0x06,
  STTS751_TEMP_ODR_8Hz        = 0x07,
  STTS751_TEMP_ODR_16Hz       = 0x08, /* 9, 10, or 11-bit resolutions only */
  STTS751_TEMP_ODR_32Hz       = 0x09, /* 9 or 10-bit resolutions only */
} stts751_odr_t;
int32_t stts751_temp_data_rate_set(stmdev_ctx_t *ctx, stts751_odr_t val);
int32_t stts751_temp_data_rate_get(stmdev_ctx_t *ctx, stts751_odr_t *val);

typedef enum {
  STTS751_9bit      = 2,
  STTS751_10bit     = 0,
  STTS751_11bit     = 1,
  STTS751_12bit     = 3,
} stts751_tres_t;
int32_t stts751_resolution_set(stmdev_ctx_t *ctx, stts751_tres_t val);
int32_t stts751_resolution_get(stmdev_ctx_t *ctx, stts751_tres_t *val);

int32_t stts751_status_reg_get(stmdev_ctx_t *ctx, stts751_status_t *val);

int32_t stts751_flag_busy_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t stts751_temperature_raw_get(stmdev_ctx_t *ctx, int16_t *buff);

int32_t stts751_pin_event_route_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t stts751_pin_event_route_get(stmdev_ctx_t *ctx, uint8_t *val);


int32_t stts751_high_temperature_threshold_set(stmdev_ctx_t *ctx,
                                               int16_t buff);
int32_t stts751_high_temperature_threshold_get(stmdev_ctx_t *ctx,
                                               int16_t *buff);

int32_t stts751_low_temperature_threshold_set(stmdev_ctx_t *ctx,
                                              int16_t buff);
int32_t stts751_low_temperature_threshold_get(stmdev_ctx_t *ctx,
                                              int16_t *buff);

int32_t stts751_ota_thermal_limit_set(stmdev_ctx_t *ctx, int8_t val);
int32_t stts751_ota_thermal_limit_get(stmdev_ctx_t *ctx, int8_t *val);

int32_t stts751_ota_thermal_hyst_set(stmdev_ctx_t *ctx, int8_t val);
int32_t stts751_ota_thermal_hyst_get(stmdev_ctx_t *ctx, int8_t *val);

int32_t stts751_smbus_timeout_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t stts751_smbus_timeout_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct {
  uint8_t product_id;
  uint8_t manufacturer_id;
  uint8_t revision_id;
} stts751_id_t;
int32_t stts751_device_id_get(stmdev_ctx_t *ctx, stts751_id_t *buff);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /*STTS751_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
