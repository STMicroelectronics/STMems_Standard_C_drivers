/*
 ******************************************************************************
 * @file    lps33k_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          lps33k_reg.c driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#ifndef LPS33K_REGS_H
#define LPS33K_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup LPS33K
  * @{
  *
  */

/** @defgroup  Endianness definitions
  * @{
  *
  */

#ifndef DRV_BYTE_ORDER
#ifndef __BYTE_ORDER__

#define DRV_LITTLE_ENDIAN 1234
#define DRV_BIG_ENDIAN    4321

/** if _BYTE_ORDER is not defined, choose the endianness of your architecture
  * by uncommenting the define which fits your platform endianness
  */
//#define DRV_BYTE_ORDER    DRV_BIG_ENDIAN
#define DRV_BYTE_ORDER    DRV_LITTLE_ENDIAN

#else /* defined __BYTE_ORDER__ */

#define DRV_LITTLE_ENDIAN  __ORDER_LITTLE_ENDIAN__
#define DRV_BIG_ENDIAN     __ORDER_BIG_ENDIAN__
#define DRV_BYTE_ORDER     __BYTE_ORDER__

#endif /* __BYTE_ORDER__*/
#endif /* DRV_BYTE_ORDER */

/**
  * @}
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7       : 1;
  uint8_t bit6       : 1;
  uint8_t bit5       : 1;
  uint8_t bit4       : 1;
  uint8_t bit3       : 1;
  uint8_t bit2       : 1;
  uint8_t bit1       : 1;
  uint8_t bit0       : 1;
#endif /* DRV_BYTE_ORDER */
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

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, uint8_t *,
                                    uint16_t);
typedef int32_t (*stmdev_read_ptr) (void *, uint8_t, uint8_t *,
                                    uint16_t);

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


/** @defgroup LPS22HB_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format. **/
#define LPS33K_I2C_ADD       0xBBU

/** Device Identification (Who am I) **/
#define LPS33K_ID            0xB1U

/**
  * @}
  *
  */

#define LPS33K_WHO_AM_I       0x0FU
#define LPS33K_CTRL_REG1      0x10U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01      : 1;
  uint8_t bdu              : 1;
  uint8_t lpfp             : 2; /* en_lpfp + lpfp_cfg -> lpfp */
  uint8_t odr              : 3;
  uint8_t not_used_02      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02      : 1;
  uint8_t odr              : 3;
  uint8_t lpfp             : 2; /* en_lpfp + lpfp_cfg -> lpfp */
  uint8_t bdu              : 1;
  uint8_t not_used_01      : 1;
#endif /* DRV_BYTE_ORDER */
} lps33k_ctrl_reg1_t;

#define LPS33K_CTRL_REG2      0x11U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t one_shot         : 1;
  uint8_t not_used_01      : 1;
  uint8_t swreset          : 1;
  uint8_t not_used_02      : 1;
  uint8_t if_add_inc       : 1;
  uint8_t not_used_03      : 2;
  uint8_t boot             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot             : 1;
  uint8_t not_used_03      : 2;
  uint8_t if_add_inc       : 1;
  uint8_t not_used_02      : 1;
  uint8_t swreset          : 1;
  uint8_t not_used_01      : 1;
  uint8_t one_shot         : 1;
#endif /* DRV_BYTE_ORDER */
} lps33k_ctrl_reg2_t;

#define LPS33K_RPDS_L         0x18U
#define LPS33K_RPDS_H         0x19U

#define LPS33K_RES_CONF       0x1AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lc_en            : 1;
  uint8_t not_used_01      : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01      : 7;
  uint8_t lc_en            : 1;
#endif /* DRV_BYTE_ORDER */
} lps33k_res_conf_t;

#define LPS33K_STATUS         0x27U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t p_da             : 1;
  uint8_t t_da             : 1;
  uint8_t not_used_02      : 2;
  uint8_t p_or             : 1;
  uint8_t t_or             : 1;
  uint8_t not_used_01      : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01      : 2;
  uint8_t t_or             : 1;
  uint8_t p_or             : 1;
  uint8_t not_used_02      : 2;
  uint8_t t_da             : 1;
  uint8_t p_da             : 1;
#endif /* DRV_BYTE_ORDER */
} lps33k_status_t;

#define LPS33K_PRESS_OUT_XL   0x28U
#define LPS33K_PRESS_OUT_L    0x29U
#define LPS33K_PRESS_OUT_H    0x2AU
#define LPS33K_TEMP_OUT_L     0x2BU
#define LPS33K_TEMP_OUT_H     0x2CU
#define LPS33K_LPFP_RES       0x33U

/**
  * @defgroup LPS33K_Register_Union
  * @brief    This union group all the registers that has a bit-field
  *           description.
  *           This union is useful but not need by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */

typedef union {
  lps33k_ctrl_reg1_t          ctrl_reg1;
  lps33k_ctrl_reg2_t          ctrl_reg2;
  lps33k_res_conf_t           res_conf;
  lps33k_status_t             status;
  bitwise_t                    bitwise;
  uint8_t                      byte;
} lps33k_reg_t;

/**
  * @}
  *
  */

int32_t lps33k_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data,
                        uint16_t len);
int32_t lps33k_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                         uint8_t *data,
                         uint16_t len);

float_t lps33k_from_lsb_to_hpa(int32_t lsb);
float_t lps33k_from_lsb_to_degc(int16_t lsb);

int32_t lps33k_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33k_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS33K_LPF_ODR_DIV_2  = 0,
  LPS33K_LPF_ODR_DIV_9  = 2,
  LPS33K_LPF_ODR_DIV_20 = 3,
} lps33k_lpfp_t;
int32_t lps33k_low_pass_filter_mode_set(stmdev_ctx_t *ctx,
                                        lps33k_lpfp_t val);
int32_t lps33k_low_pass_filter_mode_get(stmdev_ctx_t *ctx,
                                        lps33k_lpfp_t *val);

typedef enum {
  LPS33K_POWER_DOWN  = 0,
  LPS33K_ODR_1_Hz    = 1,
  LPS33K_ODR_10_Hz   = 2,
  LPS33K_ODR_25_Hz   = 3,
  LPS33K_ODR_50_Hz   = 4,
  LPS33K_ODR_75_Hz   = 5,
} lps33k_odr_t;
int32_t lps33k_data_rate_set(stmdev_ctx_t *ctx, lps33k_odr_t val);
int32_t lps33k_data_rate_get(stmdev_ctx_t *ctx, lps33k_odr_t *val);

int32_t lps33k_one_shoot_trigger_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33k_one_shoot_trigger_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33k_pressure_offset_set(stmdev_ctx_t *ctx, int16_t val);
int32_t lps33k_pressure_offset_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lps33k_press_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33k_temp_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33k_press_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33k_temp_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33k_pressure_raw_get(stmdev_ctx_t *ctx, uint32_t *buff);

int32_t lps33k_temperature_raw_get(stmdev_ctx_t *ctx, int16_t *buff);

int32_t lps33k_low_pass_rst_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps33k_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps33k_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33k_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33k_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33k_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33k_low_power_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33k_low_power_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33k_auto_add_inc_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33k_auto_add_inc_get(stmdev_ctx_t *ctx, uint8_t *val);

/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* LPS33K_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
