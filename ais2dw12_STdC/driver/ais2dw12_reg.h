/*
 ******************************************************************************
 * @file    ais2dw12_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          ais2dw12_reg.c driver.
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
#ifndef AIS2DW12_REGS_H
#define AIS2DW12_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup AIS2DW12
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

/**
  * @}
  *
  */

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

/** @defgroup AIS2DW12_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> 31 if SA0=1 -> 33 **/
#define AIS2DW12_I2C_ADD_L   0x31U
#define AIS2DW12_I2C_ADD_H   0x33U

/** Device Identification (Who am I) **/
#define AIS2DW12_ID            0x44U

/**
  * @}
  *
  */

#define AIS2DW12_OUT_T_L                     0x0DU
#define AIS2DW12_OUT_T_H                     0x0EU
#define AIS2DW12_WHO_AM_I                    0x0FU
#define AIS2DW12_CTRL1                       0x20U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t pw_mode                    : 2;
  uint8_t op_mode                    : 2;
  uint8_t odr                        : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr                        : 4;
  uint8_t op_mode                    : 2;
  uint8_t pw_mode                    : 2;
#endif /* DRV_BYTE_ORDER*/
} ais2dw12_ctrl1_t;

#define AIS2DW12_CTRL2                       0x21U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sim                        : 1;
  uint8_t i2c_disable                : 1;
  uint8_t if_add_inc                 : 1;
  uint8_t bdu                        : 1;
  uint8_t cs_pu_disc                 : 1;
  uint8_t not_used_01                : 1;
  uint8_t soft_reset                 : 1;
  uint8_t boot                       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                       : 1;
  uint8_t soft_reset                 : 1;
  uint8_t not_used_01                : 1;
  uint8_t cs_pu_disc                 : 1;
  uint8_t bdu                        : 1;
  uint8_t if_add_inc                 : 1;
  uint8_t i2c_disable                : 1;
  uint8_t sim                        : 1;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_ctrl2_t;

#define AIS2DW12_CTRL3                       0x22U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
uint8_t slp_mode                   :
  2;  /* slp_mode_sel + slp_mode_1 */
  uint8_t not_used_01                : 1;
  uint8_t h_lactive                  : 1;
  uint8_t lir                        : 1;
  uint8_t pp_od                      : 1;
  uint8_t st                         : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t st                         : 2;
  uint8_t pp_od                      : 1;
  uint8_t lir                        : 1;
  uint8_t h_lactive                  : 1;
  uint8_t not_used_01                : 1;
uint8_t slp_mode                   :
  2;  /* slp_mode_sel + slp_mode_1 */
#endif /* DRV_BYTE_ORDER */
} ais2dw12_ctrl3_t;

#define AIS2DW12_CTRL4_INT1                  0x23U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_drdy                  : 1;
  uint8_t int1_fth                   : 1;
  uint8_t int1_diff5                 : 1;
  uint8_t not_used_01                : 1;
  uint8_t int1_ff                    : 1;
  uint8_t int1_wu                    : 1;
  uint8_t not_used_02                : 1;
  uint8_t int1_6d                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_6d                    : 1;
  uint8_t not_used_02                : 1;
  uint8_t int1_wu                    : 1;
  uint8_t int1_ff                    : 1;
  uint8_t not_used_01                : 1;
  uint8_t int1_diff5                 : 1;
  uint8_t int1_fth                   : 1;
  uint8_t int1_drdy                  : 1;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_ctrl4_int1_t;

#define AIS2DW12_CTRL5_INT2                  0x24U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_drdy                  : 1;
  uint8_t int2_fth                   : 1;
  uint8_t int2_diff5                 : 1;
  uint8_t int2_ovr                   : 1;
  uint8_t int2_drdy_t                : 1;
  uint8_t int2_boot                  : 1;
  uint8_t int2_sleep_chg             : 1;
  uint8_t int2_sleep_state           : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_sleep_state           : 1;
  uint8_t int2_sleep_chg             : 1;
  uint8_t int2_boot                  : 1;
  uint8_t int2_drdy_t                : 1;
  uint8_t int2_ovr                   : 1;
  uint8_t int2_diff5                 : 1;
  uint8_t int2_fth                   : 1;
  uint8_t int2_drdy                  : 1;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_ctrl5_int2_t;

#define AIS2DW12_CTRL6                       0x25U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01                : 3;
  uint8_t fds                        : 1;
  uint8_t fs                         : 2;
  uint8_t bw_filt                    : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bw_filt                    : 2;
  uint8_t fs                         : 2;
  uint8_t fds                        : 1;
  uint8_t not_used_01                : 3;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_ctrl6_t;

#define AIS2DW12_OUT_T                       0x26U
#define AIS2DW12_STATUS                      0x27U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t drdy                       : 1;
  uint8_t ff_ia                      : 1;
  uint8_t _6d_ia                     : 1;
  uint8_t not_used_01                : 2;
  uint8_t sleep_state                : 1;
  uint8_t wu_ia                      : 1;
  uint8_t fifo_ths                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_ths                   : 1;
  uint8_t wu_ia                      : 1;
  uint8_t sleep_state                : 1;
  uint8_t not_used_01                : 2;
  uint8_t _6d_ia                     : 1;
  uint8_t ff_ia                      : 1;
  uint8_t drdy                       : 1;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_status_t;

#define AIS2DW12_OUT_X_L                     0x28U
#define AIS2DW12_OUT_X_H                     0x29U
#define AIS2DW12_OUT_Y_L                     0x2AU
#define AIS2DW12_OUT_Y_H                     0x2BU
#define AIS2DW12_OUT_Z_L                     0x2CU
#define AIS2DW12_OUT_Z_H                     0x2DU
#define AIS2DW12_FIFO_CTRL                   0x2EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fth                        : 5;
  uint8_t fmode                      : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fmode                      : 3;
  uint8_t fth                        : 5;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_fifo_ctrl_t;

#define AIS2DW12_FIFO_SAMPLES                0x2FU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t diff                       : 6;
  uint8_t fifo_ovr                   : 1;
  uint8_t fifo_fth                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_fth                   : 1;
  uint8_t fifo_ovr                   : 1;
  uint8_t diff                       : 6;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_fifo_samples_t;

#define AIS2DW12_SIXD_THS                    0x30U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01                 : 5;
  uint8_t _6d_ths                     : 2;
  uint8_t _4d_en                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t _4d_en                      : 1;
  uint8_t _6d_ths                     : 2;
  uint8_t not_used_01                 : 5;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_sixd_ths_t;

#define AIS2DW12_WAKE_UP_THS                 0x34U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wk_ths                     : 6;
  uint8_t sleep_on                   : 1;
  uint8_t not_used_01                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01                : 1;
  uint8_t sleep_on                   : 1;
  uint8_t wk_ths                     : 6;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_wake_up_ths_t;

#define AIS2DW12_WAKE_UP_DUR                 0x35U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sleep_dur                  : 4;
  uint8_t stationary                 : 1;
  uint8_t wake_dur                   : 2;
  uint8_t ff_dur                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur                     : 1;
  uint8_t wake_dur                   : 2;
  uint8_t stationary                 : 1;
  uint8_t sleep_dur                  : 4;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_wake_up_dur_t;

#define AIS2DW12_FREE_FALL                   0x36U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ths                     : 3;
  uint8_t ff_dur                     : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur                     : 5;
  uint8_t ff_ths                     : 3;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_free_fall_t;

#define AIS2DW12_STATUS_DUP                  0x37U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t drdy                       : 1;
  uint8_t ff_ia                      : 1;
  uint8_t _6d_ia                     : 1;
  uint8_t not_used_01                : 2;
  uint8_t sleep_state_ia             : 1;
  uint8_t drdy_t                     : 1;
  uint8_t ovr                        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ovr                        : 1;
  uint8_t drdy_t                     : 1;
  uint8_t sleep_state_ia             : 1;
  uint8_t not_used_01                : 2;
  uint8_t _6d_ia                     : 1;
  uint8_t ff_ia                      : 1;
  uint8_t drdy                       : 1;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_status_dup_t;

#define AIS2DW12_WAKE_UP_SRC                 0x38U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_wu                       : 1;
  uint8_t y_wu                       : 1;
  uint8_t x_wu                       : 1;
  uint8_t wu_ia                      : 1;
  uint8_t sleep_state_ia             : 1;
  uint8_t ff_ia                      : 1;
  uint8_t not_used_01                : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01                : 2;
  uint8_t ff_ia                      : 1;
  uint8_t sleep_state_ia             : 1;
  uint8_t wu_ia                      : 1;
  uint8_t x_wu                       : 1;
  uint8_t y_wu                       : 1;
  uint8_t z_wu                       : 1;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_wake_up_src_t;

#define AIS2DW12_SIXD_SRC                    0x3AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl                         : 1;
  uint8_t xh                         : 1;
  uint8_t yl                         : 1;
  uint8_t yh                         : 1;
  uint8_t zl                         : 1;
  uint8_t zh                         : 1;
  uint8_t _6d_ia                     : 1;
  uint8_t not_used_01                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01                : 1;
  uint8_t _6d_ia                     : 1;
  uint8_t zh                         : 1;
  uint8_t zl                         : 1;
  uint8_t yh                         : 1;
  uint8_t yl                         : 1;
  uint8_t xh                         : 1;
  uint8_t xl                         : 1;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_sixd_src_t;

#define AIS2DW12_ALL_INT_SRC                 0x3BU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ia                      : 1;
  uint8_t wu_ia                      : 1;
  uint8_t not_used_01                : 2;
  uint8_t _6d_ia                     : 1;
  uint8_t sleep_change_ia            : 1;
  uint8_t not_used_02                : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02                : 2;
  uint8_t sleep_change_ia            : 1;
  uint8_t _6d_ia                     : 1;
  uint8_t not_used_01                : 2;
  uint8_t wu_ia                      : 1;
  uint8_t ff_ia                      : 1;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_all_int_src_t;

#define AIS2DW12_X_OFS_USR                   0x3CU
#define AIS2DW12_Y_OFS_USR                   0x3DU
#define AIS2DW12_Z_OFS_USR                   0x3EU
#define AIS2DW12_CTRL7                       0x3FU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lpass_on6d                 : 1;
  uint8_t hp_ref_mode                : 1;
  uint8_t usr_off_w                  : 1;
  uint8_t usr_off_on_wu              : 1;
  uint8_t usr_off_on_out             : 1;
  uint8_t interrupts_enable          : 1;
  uint8_t int2_on_int1               : 1;
  uint8_t drdy_pulsed                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t drdy_pulsed                : 1;
  uint8_t int2_on_int1               : 1;
  uint8_t interrupts_enable          : 1;
  uint8_t usr_off_on_out             : 1;
  uint8_t usr_off_on_wu              : 1;
  uint8_t usr_off_w                  : 1;
  uint8_t hp_ref_mode                : 1;
  uint8_t lpass_on6d                 : 1;
#endif /* DRV_BYTE_ORDER */
} ais2dw12_ctrl7_t;

/**
  * @defgroup AIS2DW12_Register_Union
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
typedef union {
  ais2dw12_ctrl1_t                   ctrl1;
  ais2dw12_ctrl2_t                   ctrl2;
  ais2dw12_ctrl3_t                   ctrl3;
  ais2dw12_ctrl4_int1_t              ctrl4_int1;
  ais2dw12_ctrl5_int2_t              ctrl5_int2;
  ais2dw12_ctrl6_t                   ctrl6;
  ais2dw12_status_t                  status;
  ais2dw12_fifo_ctrl_t               fifo_ctrl;
  ais2dw12_fifo_samples_t            fifo_samples;
  ais2dw12_sixd_ths_t                sixd_ths;
  ais2dw12_wake_up_ths_t             wake_up_ths;
  ais2dw12_wake_up_dur_t             wake_up_dur;
  ais2dw12_free_fall_t               free_fall;
  ais2dw12_status_dup_t              status_dup;
  ais2dw12_wake_up_src_t             wake_up_src;
  ais2dw12_sixd_src_t                sixd_src;
  ais2dw12_all_int_src_t             all_int_src;
  ais2dw12_ctrl7_t                   ctrl7;
  bitwise_t                          bitwise;
  uint8_t                            byte;
} ais2dw12_reg_t;

/**
  * @}
  *
  */

int32_t ais2dw12_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
                          uint8_t *data,
                          uint16_t len);
int32_t ais2dw12_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                           uint8_t *data,
                           uint16_t len);

float_t ais2dw12_from_fs2_to_mg(int16_t lsb);
float_t ais2dw12_from_fs4_to_mg(int16_t lsb);
float_t ais2dw12_from_fs2_12bit_to_mg(int16_t lsb);
float_t ais2dw12_from_fs4_12bit_to_mg(int16_t lsb);

float_t ais2dw12_from_lsb_to_celsius(int16_t lsb);

typedef enum {
  AIS2DW12_PWR_MD_4                           = 0x03,
  AIS2DW12_PWR_MD_3                           = 0x02,
  AIS2DW12_PWR_MD_2                           = 0x01,
  AIS2DW12_PWR_MD_12bit                       = 0x00,
  AIS2DW12_SINGLE_PWR_MD_4                    = 0x0B,
  AIS2DW12_SINGLE_PWR_MD_3                    = 0x0A,
  AIS2DW12_SINGLE_PWR_MD_2                    = 0x09,
  AIS2DW12_SINGLE_PWR_MD_12bit                = 0x08,
} ais2dw12_mode_t;
int32_t ais2dw12_power_mode_set(stmdev_ctx_t *ctx,
                                ais2dw12_mode_t val);
int32_t ais2dw12_power_mode_get(stmdev_ctx_t *ctx,
                                ais2dw12_mode_t *val);

typedef enum {
  AIS2DW12_XL_ODR_OFF            = 0x00,
  AIS2DW12_XL_ODR_1Hz6           = 0x01,
  AIS2DW12_XL_ODR_12Hz5          = 0x02,
  AIS2DW12_XL_ODR_25Hz           = 0x03,
  AIS2DW12_XL_ODR_50Hz           = 0x04,
  AIS2DW12_XL_ODR_100Hz          = 0x05,
  AIS2DW12_XL_SET_SW_TRIG        = 0x12,  /* Use this only in SINGLE mode */
  AIS2DW12_XL_SET_PIN_TRIG       = 0x22,  /* Use this only in SINGLE mode */
} ais2dw12_odr_t;
int32_t ais2dw12_data_rate_set(stmdev_ctx_t *ctx, ais2dw12_odr_t val);
int32_t ais2dw12_data_rate_get(stmdev_ctx_t *ctx,
                               ais2dw12_odr_t *val);

int32_t ais2dw12_block_data_update_set(stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t ais2dw12_block_data_update_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

typedef enum {
  AIS2DW12_2g     = 0,
  AIS2DW12_4g     = 1,
} ais2dw12_fs_t;
int32_t ais2dw12_full_scale_set(stmdev_ctx_t *ctx, ais2dw12_fs_t val);
int32_t ais2dw12_full_scale_get(stmdev_ctx_t *ctx,
                                ais2dw12_fs_t *val);

int32_t ais2dw12_status_reg_get(stmdev_ctx_t *ctx,
                                ais2dw12_status_t *val);

int32_t ais2dw12_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct {
  ais2dw12_status_dup_t   status_dup;
  ais2dw12_wake_up_src_t  wake_up_src;
  ais2dw12_sixd_src_t     sixd_src;
  ais2dw12_all_int_src_t  all_int_src;
} ais2dw12_all_sources_t;
int32_t ais2dw12_all_sources_get(stmdev_ctx_t *ctx,
                                 ais2dw12_all_sources_t *val);

int32_t ais2dw12_usr_offset_x_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t ais2dw12_usr_offset_x_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t ais2dw12_usr_offset_y_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t ais2dw12_usr_offset_y_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t ais2dw12_usr_offset_z_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t ais2dw12_usr_offset_z_get(stmdev_ctx_t *ctx, uint8_t *buff);

typedef enum {
  AIS2DW12_LSb_977ug    = 0,
  AIS2DW12_LSb_15mg6    = 1,
} ais2dw12_usr_off_w_t;
int32_t ais2dw12_offset_weight_set(stmdev_ctx_t *ctx,
                                   ais2dw12_usr_off_w_t val);
int32_t ais2dw12_offset_weight_get(stmdev_ctx_t *ctx,
                                   ais2dw12_usr_off_w_t *val);

int32_t ais2dw12_temperature_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t ais2dw12_acceleration_raw_get(stmdev_ctx_t *ctx,
                                      int16_t *val);

int32_t ais2dw12_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t ais2dw12_auto_increment_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ais2dw12_auto_increment_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ais2dw12_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ais2dw12_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ais2dw12_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ais2dw12_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  AIS2DW12_XL_ST_DISABLE      = 0,
  AIS2DW12_XL_ST_POSITIVE     = 1,
  AIS2DW12_XL_ST_NEGATIVE     = 2,
} ais2dw12_st_t;
int32_t ais2dw12_self_test_set(stmdev_ctx_t *ctx, ais2dw12_st_t val);
int32_t ais2dw12_self_test_get(stmdev_ctx_t *ctx, ais2dw12_st_t *val);

typedef enum {
  AIS2DW12_DRDY_LATCHED   = 0,
  AIS2DW12_DRDY_PULSED    = 1,
} ais2dw12_drdy_pulsed_t;
int32_t ais2dw12_data_ready_mode_set(stmdev_ctx_t *ctx,
                                     ais2dw12_drdy_pulsed_t val);
int32_t ais2dw12_data_ready_mode_get(stmdev_ctx_t *ctx,
                                     ais2dw12_drdy_pulsed_t *val);

typedef enum {
  AIS2DW12_LPF_ON_OUT         = 0x00,
  AIS2DW12_USER_OFFSET_ON_OUT  = 0x01,
  AIS2DW12_HIGH_PASS_ON_OUT    = 0x10,
} ais2dw12_fds_t;
int32_t ais2dw12_filter_path_set(stmdev_ctx_t *ctx,
                                 ais2dw12_fds_t val);
int32_t ais2dw12_filter_path_get(stmdev_ctx_t *ctx,
                                 ais2dw12_fds_t *val);

typedef enum {
  AIS2DW12_ODR_DIV_2     = 0,
  AIS2DW12_ODR_DIV_4     = 1,
  AIS2DW12_ODR_DIV_10    = 2,
  AIS2DW12_ODR_DIV_20    = 3,
} ais2dw12_bw_filt_t;
int32_t ais2dw12_filter_bandwidth_set(stmdev_ctx_t *ctx,
                                      ais2dw12_bw_filt_t val);
int32_t ais2dw12_filter_bandwidth_get(stmdev_ctx_t *ctx,
                                      ais2dw12_bw_filt_t *val);

int32_t ais2dw12_reference_mode_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ais2dw12_reference_mode_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  AIS2DW12_SPI_4_WIRE   = 0,
  AIS2DW12_SPI_3_WIRE   = 1,
} ais2dw12_sim_t;
int32_t ais2dw12_spi_mode_set(stmdev_ctx_t *ctx, ais2dw12_sim_t val);
int32_t ais2dw12_spi_mode_get(stmdev_ctx_t *ctx, ais2dw12_sim_t *val);

typedef enum {
  AIS2DW12_I2C_ENABLE    = 0,
  AIS2DW12_I2C_DISABLE   = 1,
} ais2dw12_i2c_disable_t;
int32_t ais2dw12_i2c_interface_set(stmdev_ctx_t *ctx,
                                   ais2dw12_i2c_disable_t val);
int32_t ais2dw12_i2c_interface_get(stmdev_ctx_t *ctx,
                                   ais2dw12_i2c_disable_t *val);

typedef enum {
  AIS2DW12_PULL_UP_CONNECT     = 0,
  AIS2DW12_PULL_UP_DISCONNECT  = 1,
} ais2dw12_cs_pu_disc_t;
int32_t ais2dw12_cs_mode_set(stmdev_ctx_t *ctx,
                             ais2dw12_cs_pu_disc_t val);
int32_t ais2dw12_cs_mode_get(stmdev_ctx_t *ctx,
                             ais2dw12_cs_pu_disc_t *val);

typedef enum {
  AIS2DW12_ACTIVE_HIGH  = 0,
  AIS2DW12_ACTIVE_LOW   = 1,
} ais2dw12_h_lactive_t;
int32_t ais2dw12_pin_polarity_set(stmdev_ctx_t *ctx,
                                  ais2dw12_h_lactive_t val);
int32_t ais2dw12_pin_polarity_get(stmdev_ctx_t *ctx,
                                  ais2dw12_h_lactive_t *val);

typedef enum {
  AIS2DW12_INT_PULSED   = 0,
  AIS2DW12_INT_LATCHED  = 1,
} ais2dw12_lir_t;
int32_t ais2dw12_int_notification_set(stmdev_ctx_t *ctx,
                                      ais2dw12_lir_t val);
int32_t ais2dw12_int_notification_get(stmdev_ctx_t *ctx,
                                      ais2dw12_lir_t *val);

typedef enum {
  AIS2DW12_PUSH_PULL   = 0,
  AIS2DW12_OPEN_DRAIN  = 1,
} ais2dw12_pp_od_t;
int32_t ais2dw12_pin_mode_set(stmdev_ctx_t *ctx,
                              ais2dw12_pp_od_t val);
int32_t ais2dw12_pin_mode_get(stmdev_ctx_t *ctx,
                              ais2dw12_pp_od_t *val);

int32_t ais2dw12_pin_int1_route_set(stmdev_ctx_t *ctx,
                                    ais2dw12_ctrl4_int1_t *val);
int32_t ais2dw12_pin_int1_route_get(stmdev_ctx_t *ctx,
                                    ais2dw12_ctrl4_int1_t *val);

int32_t ais2dw12_pin_int2_route_set(stmdev_ctx_t *ctx,
                                    ais2dw12_ctrl5_int2_t *val);
int32_t ais2dw12_pin_int2_route_get(stmdev_ctx_t *ctx,
                                    ais2dw12_ctrl5_int2_t *val);

int32_t ais2dw12_all_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ais2dw12_all_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ais2dw12_wkup_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ais2dw12_wkup_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ais2dw12_wkup_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ais2dw12_wkup_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  AIS2DW12_HP_FEED           = 0,
  AIS2DW12_USER_OFFSET_FEED  = 1,
} ais2dw12_usr_off_on_wu_t;
int32_t ais2dw12_wkup_feed_data_set(stmdev_ctx_t *ctx,
                                    ais2dw12_usr_off_on_wu_t val);
int32_t ais2dw12_wkup_feed_data_get(stmdev_ctx_t *ctx,
                                    ais2dw12_usr_off_on_wu_t *val);

typedef enum {
  AIS2DW12_NO_DETECTION        = 0,
  AIS2DW12_DETECT_ACT_INACT    = 1,
  AIS2DW12_DETECT_STAT_MOTION  = 3,
} ais2dw12_sleep_on_t;
int32_t ais2dw12_act_mode_set(stmdev_ctx_t *ctx,
                              ais2dw12_sleep_on_t val);
int32_t ais2dw12_act_mode_get(stmdev_ctx_t *ctx,
                              ais2dw12_sleep_on_t *val);

int32_t ais2dw12_act_sleep_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ais2dw12_act_sleep_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ais2dw12_6d_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ais2dw12_6d_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ais2dw12_4d_mode_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ais2dw12_4d_mode_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ais2dw12_6d_src_get(stmdev_ctx_t *ctx,
                            ais2dw12_sixd_src_t *val);

typedef enum {
  AIS2DW12_ODR_DIV_2_FEED   = 0,
  AIS2DW12_LPF2_FEED        = 1,
} ais2dw12_lpass_on6d_t;
int32_t ais2dw12_6d_feed_data_set(stmdev_ctx_t *ctx,
                                  ais2dw12_lpass_on6d_t val);
int32_t ais2dw12_6d_feed_data_get(stmdev_ctx_t *ctx,
                                  ais2dw12_lpass_on6d_t *val);

int32_t ais2dw12_ff_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ais2dw12_ff_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  AIS2DW12_FF_TSH_5LSb_FS2g  = 0,
  AIS2DW12_FF_TSH_7LSb_FS2g  = 1,
  AIS2DW12_FF_TSH_8LSb_FS2g  = 2,
  AIS2DW12_FF_TSH_10LSb_FS2g = 3,
  AIS2DW12_FF_TSH_11LSb_FS2g = 4,
  AIS2DW12_FF_TSH_13LSb_FS2g = 5,
  AIS2DW12_FF_TSH_15LSb_FS2g = 6,
  AIS2DW12_FF_TSH_16LSb_FS2g = 7,
} ais2dw12_ff_ths_t;
int32_t ais2dw12_ff_threshold_set(stmdev_ctx_t *ctx,
                                  ais2dw12_ff_ths_t val);
int32_t ais2dw12_ff_threshold_get(stmdev_ctx_t *ctx,
                                  ais2dw12_ff_ths_t *val);

int32_t ais2dw12_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ais2dw12_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  AIS2DW12_BYPASS_MODE             = 0,
  AIS2DW12_FIFO_MODE               = 1,
  AIS2DW12_STREAM_TO_FIFO_MODE     = 3,
  AIS2DW12_BYPASS_TO_STREAM_MODE   = 4,
  AIS2DW12_STREAM_MODE             = 6,
} ais2dw12_fmode_t;
int32_t ais2dw12_fifo_mode_set(stmdev_ctx_t *ctx,
                               ais2dw12_fmode_t val);
int32_t ais2dw12_fifo_mode_get(stmdev_ctx_t *ctx,
                               ais2dw12_fmode_t *val);

int32_t ais2dw12_fifo_data_level_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ais2dw12_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ais2dw12_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /*AIS2DW12_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
