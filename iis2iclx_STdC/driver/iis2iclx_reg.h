/*
 ******************************************************************************
 * @file    iis2iclx_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          iis2iclx_reg.c driver.
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
#ifndef IIS2ICLX_REGS_H
#define IIS2ICLX_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup IIS2ICLX
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

/** @defgroup IIS2ICLX Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> D5 if SA0=1 -> D7 **/
#define IIS2ICLX_I2C_ADD_L                    0xD5U
#define IIS2ICLX_I2C_ADD_H                    0xD7U

/** Device Identification (Who am I) **/
#define IIS2ICLX_ID                           0x6BU

/**
  * @}
  *
  */

#define IIS2ICLX_FUNC_CFG_ACCESS              0x01U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 6;
uint8_t reg_access               :
  2; /* shub_reg_access + func_cfg_access */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
uint8_t reg_access               :
  2; /* shub_reg_access + func_cfg_access */
  uint8_t not_used_01              : 6;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_func_cfg_access_t;

#define IIS2ICLX_PIN_CTRL                     0x02U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 6;
  uint8_t sdo_pu_en                : 1;
  uint8_t not_used_02              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 1;
  uint8_t sdo_pu_en                : 1;
  uint8_t not_used_01              : 6;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_pin_ctrl_t;

#define IIS2ICLX_FIFO_CTRL1                   0x07U
typedef struct {
  uint8_t wtm                      : 8;
} iis2iclx_fifo_ctrl1_t;

#define IIS2ICLX_FIFO_CTRL2                   0x08U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wtm                      : 1;
  uint8_t not_used_01              : 3;
  uint8_t odrchg_en                : 1;
  uint8_t not_used_02              : 2;
  uint8_t stop_on_wtm              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t stop_on_wtm              : 1;
  uint8_t not_used_02              : 2;
  uint8_t odrchg_en                : 1;
  uint8_t not_used_01              : 3;
  uint8_t wtm                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fifo_ctrl2_t;

#define IIS2ICLX_FIFO_CTRL3                   0x09U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bdr_xl                   : 4;
  uint8_t not_used_01              : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 4;
  uint8_t bdr_xl                   : 4;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fifo_ctrl3_t;

#define IIS2ICLX_FIFO_CTRL4                   0x0AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fifo_mode                : 3;
  uint8_t not_used_01              : 1;
  uint8_t odr_t_batch              : 2;
  uint8_t odr_ts_batch             : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_ts_batch             : 2;
  uint8_t odr_t_batch              : 2;
  uint8_t not_used_01              : 1;
  uint8_t fifo_mode                : 3;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fifo_ctrl4_t;

#define IIS2ICLX_COUNTER_BDR_REG1             0x0BU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t cnt_bdr_th               : 1;
  uint8_t not_used_01              : 5;
  uint8_t rst_counter_bdr          : 1;
  uint8_t dataready_pulsed         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t dataready_pulsed         : 1;
  uint8_t rst_counter_bdr          : 1;
  uint8_t not_used_01              : 5;
  uint8_t cnt_bdr_th               : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_counter_bdr_reg1_t;

#define IIS2ICLX_COUNTER_BDR_REG2             0x0CU
typedef struct {
  uint8_t cnt_bdr_th               : 8;
} iis2iclx_counter_bdr_reg2_t;

#define IIS2ICLX_INT1_CTRL                    0x0DU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_drdy_xl             : 1;
  uint8_t not_used_01              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fifo_th             : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_fifo_full           : 1;
  uint8_t int1_cnt_bdr             : 1;
  uint8_t den_drdy_flag            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_drdy_flag            : 1;
  uint8_t int1_cnt_bdr             : 1;
  uint8_t int1_fifo_full           : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_fifo_th             : 1;
  uint8_t int1_boot                : 1;
  uint8_t not_used_01              : 1;
  uint8_t int1_drdy_xl             : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_int1_ctrl_t;

#define IIS2ICLX_INT2_CTRL                    0x0EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_drdy_xl             : 1;
  uint8_t not_used_01              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fifo_th             : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_fifo_full           : 1;
  uint8_t int2_cnt_bdr             : 1;
  uint8_t not_used_02              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 1;
  uint8_t int2_cnt_bdr             : 1;
  uint8_t int2_fifo_full           : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_fifo_th             : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t not_used_01              : 1;
  uint8_t int2_drdy_xl             : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_int2_ctrl_t;

#define IIS2ICLX_WHO_AM_I                     0x0FU
#define IIS2ICLX_CTRL1_XL                     0x10U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t lpf2_xl_en               : 1;
  uint8_t fs_xl                    : 2;
  uint8_t odr_xl                   : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_xl                   : 4;
  uint8_t fs_xl                    : 2;
  uint8_t lpf2_xl_en               : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_ctrl1_xl_t;

#define IIS2ICLX_CTRL3_C                      0x12U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sw_reset                 : 1;
  uint8_t not_used_01              : 1;
  uint8_t if_inc                   : 1;
  uint8_t sim                      : 1;
  uint8_t pp_od                    : 1;
  uint8_t h_lactive                : 1;
  uint8_t bdu                      : 1;
  uint8_t boot                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                     : 1;
  uint8_t bdu                      : 1;
  uint8_t h_lactive                : 1;
  uint8_t pp_od                    : 1;
  uint8_t sim                      : 1;
  uint8_t if_inc                   : 1;
  uint8_t not_used_01              : 1;
  uint8_t sw_reset                 : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_ctrl3_c_t;

#define IIS2ICLX_CTRL4_C                      0x13U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t i2c_disable              : 1;
  uint8_t drdy_mask                : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_on_int1             : 1;
  uint8_t not_used_03              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_03              : 2;
  uint8_t int2_on_int1             : 1;
  uint8_t not_used_02              : 1;
  uint8_t drdy_mask                : 1;
  uint8_t i2c_disable              : 1;
  uint8_t not_used_01              : 2;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_ctrl4_c_t;

#define IIS2ICLX_CTRL5_C                      0x14U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st_xl                    : 2;
  uint8_t not_used_01              : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 6;
  uint8_t st_xl                    : 2;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_ctrl5_c_t;

#define IIS2ICLX_CTRL6_C                      0x15U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 3;
  uint8_t usr_off_w                : 1;
  uint8_t not_used_02              : 1;
uint8_t den_mode                 :
  3;   /* trig_en + lvl1_en + lvl2_en */
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
uint8_t den_mode                 :
  3;   /* trig_en + lvl1_en + lvl2_en */
  uint8_t not_used_02              : 1;
  uint8_t usr_off_w                : 1;
  uint8_t not_used_01              : 3;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_ctrl6_c_t;

#define IIS2ICLX_CTRL7_XL                     0x16U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t usr_off_on_out           : 1;
  uint8_t not_used_02              : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 6;
  uint8_t usr_off_on_out           : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_ctrl7_xl_t;

#define IIS2ICLX_CTRL8_XL                     0x17U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t hp_slope_xl_en           : 1;
  uint8_t fastsettl_mode_xl        : 1;
  uint8_t hp_ref_mode_xl           : 1;
  uint8_t hpcf_xl                  : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t hpcf_xl                  : 3;
  uint8_t hp_ref_mode_xl           : 1;
  uint8_t fastsettl_mode_xl        : 1;
  uint8_t hp_slope_xl_en           : 1;
  uint8_t not_used_01              : 2;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_ctrl8_xl_t;

#define IIS2ICLX_CTRL9_XL                     0x18U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t device_conf              : 1;
  uint8_t den_lh                   : 1;
  uint8_t den_en                   : 3;
  uint8_t den_y                    : 1;
  uint8_t den_x                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_x                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_en                   : 3;
  uint8_t den_lh                   : 1;
  uint8_t device_conf              : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_ctrl9_xl_t;

#define IIS2ICLX_CTRL10_C                     0x19U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t timestamp_en             : 1;
  uint8_t not_used_02              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 2;
  uint8_t timestamp_en             : 1;
  uint8_t not_used_01              : 5;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_ctrl10_c_t;

#define IIS2ICLX_ALL_INT_SRC                  0x1AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t wu_ia                    : 1;
  uint8_t single_tap               : 1;
  uint8_t double_tap               : 1;
  uint8_t not_used_02              : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t not_used_03              : 1;
  uint8_t timestamp_endcount       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t timestamp_endcount       : 1;
  uint8_t not_used_03              : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t not_used_02              : 1;
  uint8_t double_tap               : 1;
  uint8_t single_tap               : 1;
  uint8_t wu_ia                    : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_all_int_src_t;

#define IIS2ICLX_WAKE_UP_SRC                  0x1BU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t y_wu                     : 1;
  uint8_t x_wu                     : 1;
  uint8_t wu_ia                    : 1;
  uint8_t sleep_state              : 1;
  uint8_t not_used_02              : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t not_used_03              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_03              : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t not_used_02              : 1;
  uint8_t sleep_state              : 1;
  uint8_t wu_ia                    : 1;
  uint8_t x_wu                     : 1;
  uint8_t y_wu                     : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_wake_up_src_t;

#define IIS2ICLX_TAP_SRC                      0x1CU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t y_tap                    : 1;
  uint8_t x_tap                    : 1;
  uint8_t tap_sign                 : 1;
  uint8_t double_tap               : 1;
  uint8_t single_tap               : 1;
  uint8_t tap_ia                   : 1;
  uint8_t not_used_02              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 1;
  uint8_t tap_ia                   : 1;
  uint8_t single_tap               : 1;
  uint8_t double_tap               : 1;
  uint8_t tap_sign                 : 1;
  uint8_t x_tap                    : 1;
  uint8_t y_tap                    : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_tap_src_t;

#define IIS2ICLX_DEN_SRC                      0x1DU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 7;
  uint8_t den_drdy                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t den_drdy                 : 1;
  uint8_t not_used_01              : 7;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_den_src_t;

#define IIS2ICLX_STATUS_REG                   0x1EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                     : 1;
  uint8_t not_used_01              : 1;
  uint8_t tda                      : 1;
  uint8_t not_used_02              : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 5;
  uint8_t tda                      : 1;
  uint8_t not_used_01              : 1;
  uint8_t xlda                     : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_status_reg_t;

#define IIS2ICLX_OUT_TEMP_L                   0x20U
#define IIS2ICLX_OUT_TEMP_H                   0x21U
#define IIS2ICLX_OUTX_L_A                     0x28U
#define IIS2ICLX_OUTX_H_A                     0x29U
#define IIS2ICLX_OUTY_L_A                     0x2AU
#define IIS2ICLX_OUTY_H_A                     0x2BU
#define IIS2ICLX_EMB_FUNC_STATUS_MAINPAGE     0x35U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01             : 7;
  uint8_t is_fsm_lc               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm_lc               : 1;
  uint8_t not_used_01             : 7;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_emb_func_status_mainpage_t;

#define IIS2ICLX_FSM_STATUS_A_MAINPAGE        0x36U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_fsm1                 : 1;
  uint8_t is_fsm2                 : 1;
  uint8_t is_fsm3                 : 1;
  uint8_t is_fsm4                 : 1;
  uint8_t is_fsm5                 : 1;
  uint8_t is_fsm6                 : 1;
  uint8_t is_fsm7                 : 1;
  uint8_t is_fsm8                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm8                 : 1;
  uint8_t is_fsm7                 : 1;
  uint8_t is_fsm6                 : 1;
  uint8_t is_fsm5                 : 1;
  uint8_t is_fsm4                 : 1;
  uint8_t is_fsm3                 : 1;
  uint8_t is_fsm2                 : 1;
  uint8_t is_fsm1                 : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_status_a_mainpage_t;

#define IIS2ICLX_FSM_STATUS_B_MAINPAGE        0x37U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_fsm9                 : 1;
  uint8_t is_fsm10                : 1;
  uint8_t is_fsm11                : 1;
  uint8_t is_fsm12                : 1;
  uint8_t is_fsm13                : 1;
  uint8_t is_fsm14                : 1;
  uint8_t is_fsm15                : 1;
  uint8_t is_fsm16                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm16                : 1;
  uint8_t is_fsm15                : 1;
  uint8_t is_fsm14                : 1;
  uint8_t is_fsm13                : 1;
  uint8_t is_fsm12                : 1;
  uint8_t is_fsm11                : 1;
  uint8_t is_fsm10                : 1;
  uint8_t is_fsm09                : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_status_b_mainpage_t;

#define IIS2ICLX_MLC_STATUS_MAINPAGE          0x38U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_mlc1                 : 1;
  uint8_t is_mlc2                 : 1;
  uint8_t is_mlc3                 : 1;
  uint8_t is_mlc4                 : 1;
  uint8_t is_mlc5                 : 1;
  uint8_t is_mlc6                 : 1;
  uint8_t is_mlc7                 : 1;
  uint8_t is_mlc8                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_mlc8                 : 1;
  uint8_t is_mlc7                 : 1;
  uint8_t is_mlc6                 : 1;
  uint8_t is_mlc5                 : 1;
  uint8_t is_mlc4                 : 1;
  uint8_t is_mlc3                 : 1;
  uint8_t is_mlc2                 : 1;
  uint8_t is_mlc1                 : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_mlc_status_mainpage_t;

#define IIS2ICLX_STATUS_MASTER_MAINPAGE       0x39U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sens_hub_endop          : 1;
  uint8_t not_used_01             : 2;
  uint8_t slave0_nack             : 1;
  uint8_t slave1_nack             : 1;
  uint8_t slave2_nack             : 1;
  uint8_t slave3_nack             : 1;
  uint8_t wr_once_done            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t wr_once_done            : 1;
  uint8_t slave3_nack             : 1;
  uint8_t slave2_nack             : 1;
  uint8_t slave1_nack             : 1;
  uint8_t slave0_nack             : 1;
  uint8_t not_used_01             : 2;
  uint8_t sens_hub_endop          : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_status_master_mainpage_t;

#define IIS2ICLX_FIFO_STATUS1                 0x3AU
typedef struct {
  uint8_t diff_fifo                : 8;
} iis2iclx_fifo_status1_t;

#define IIS2ICLX_FIFO_STATUS2                 0x3BU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t diff_fifo                : 2;
  uint8_t not_used_01              : 1;
  uint8_t over_run_latched         : 1;
  uint8_t counter_bdr_ia           : 1;
  uint8_t fifo_full_ia             : 1;
  uint8_t fifo_ovr_ia              : 1;
  uint8_t fifo_wtm_ia              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_wtm_ia              : 1;
  uint8_t fifo_ovr_ia              : 1;
  uint8_t fifo_full_ia             : 1;
  uint8_t counter_bdr_ia           : 1;
  uint8_t over_run_latched         : 1;
  uint8_t not_used_01              : 1;
  uint8_t diff_fifo                : 2;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fifo_status2_t;

#define IIS2ICLX_TIMESTAMP0                   0x40U
#define IIS2ICLX_TIMESTAMP1                   0x41U
#define IIS2ICLX_TIMESTAMP2                   0x42U
#define IIS2ICLX_TIMESTAMP3                   0x43U
#define IIS2ICLX_TAP_CFG0                     0x56U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lir                      : 1;
  uint8_t not_used_01              : 1;
  uint8_t tap_y_en                 : 1;
  uint8_t tap_x_en                 : 1;
  uint8_t slope_fds                : 1;
  uint8_t sleep_status_on_int      : 1;
  uint8_t int_clr_on_read          : 1;
  uint8_t not_used_02              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 1;
  uint8_t int_clr_on_read          : 1;
  uint8_t sleep_status_on_int      : 1;
  uint8_t slope_fds                : 1;
  uint8_t tap_x_en                 : 1;
  uint8_t tap_y_en                 : 1;
  uint8_t not_used_01              : 1;
  uint8_t lir                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_tap_cfg0_t;

#define IIS2ICLX_TAP_CFG1                     0x57U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_ths_x                : 5;
  uint8_t tap_priority             : 1;
  uint8_t not_used_01              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t tap_priority             : 1;
  uint8_t tap_ths_x                : 5;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_tap_cfg1_t;

#define IIS2ICLX_TAP_CFG2                     0x58U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_ths_y                : 5;
  uint8_t not_used_01              : 2;
  uint8_t interrupts_enable        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t interrupts_enable        : 1;
  uint8_t not_used_01              : 2;
  uint8_t tap_ths_y                : 5;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_tap_cfg2_t;

#define IIS2ICLX_INT_DUR2                     0x5AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t shock                    : 2;
  uint8_t quiet                    : 2;
  uint8_t dur                      : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t dur                      : 4;
  uint8_t quiet                    : 2;
  uint8_t shock                    : 2;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_int_dur2_t;

#define IIS2ICLX_WAKE_UP_THS                  0x5BU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wk_ths                   : 6;
  uint8_t usr_off_on_wu            : 1;
  uint8_t single_double_tap        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t single_double_tap        : 1;
  uint8_t usr_off_on_wu            : 1;
  uint8_t wk_ths                   : 6;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_wake_up_ths_t;

#define IIS2ICLX_WAKE_UP_DUR                  0x5CU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sleep_dur                : 4;
  uint8_t wake_ths_w               : 1;
  uint8_t wake_dur                 : 2;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t wake_dur                 : 2;
  uint8_t wake_ths_w               : 1;
  uint8_t sleep_dur                : 4;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_wake_up_dur_t;

#define IIS2ICLX_MD1_CFG                      0x5EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_shub                : 1;
  uint8_t int1_emb_func            : 1;
  uint8_t not_used_01              : 1;
  uint8_t int1_double_tap          : 1;
  uint8_t not_used_02              : 1;
  uint8_t int1_wu                  : 1;
  uint8_t int1_single_tap          : 1;
  uint8_t int1_sleep_change        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_sleep_change        : 1;
  uint8_t int1_single_tap          : 1;
  uint8_t int1_wu                  : 1;
  uint8_t not_used_02              : 1;
  uint8_t int1_double_tap          : 1;
  uint8_t not_used_01              : 1;
  uint8_t int1_emb_func            : 1;
  uint8_t int1_shub                : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_md1_cfg_t;

#define IIS2ICLX_MD2_CFG                      0x5FU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_timestamp           : 1;
  uint8_t int2_emb_func            : 1;
  uint8_t not_used_01              : 1;
  uint8_t int2_double_tap          : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_wu                  : 1;
  uint8_t int2_single_tap          : 1;
  uint8_t int2_sleep_change        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_sleep_change        : 1;
  uint8_t int2_single_tap          : 1;
  uint8_t int2_wu                  : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_double_tap          : 1;
  uint8_t not_used_01              : 1;
  uint8_t int2_emb_func            : 1;
  uint8_t int2_timestamp           : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_md2_cfg_t;

#define IIS2ICLX_INTERNAL_FREQ_FINE           0x63U
typedef struct {
  uint8_t freq_fine                : 8;
} iis2iclx_internal_freq_fine_t;

#define IIS2ICLX_X_OFS_USR                    0x73U
#define IIS2ICLX_Y_OFS_USR                    0x74U
#define IIS2ICLX_FIFO_DATA_OUT_TAG            0x78U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tag_parity               : 1;
  uint8_t tag_cnt                  : 2;
  uint8_t tag_sensor               : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t tag_sensor               : 5;
  uint8_t tag_cnt                  : 2;
  uint8_t tag_parity               : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fifo_data_out_tag_t;

#define IIS2ICLX_FIFO_DATA_OUT_X_L            0x79U
#define IIS2ICLX_FIFO_DATA_OUT_X_H            0x7AU
#define IIS2ICLX_FIFO_DATA_OUT_Y_L            0x7BU
#define IIS2ICLX_FIFO_DATA_OUT_Y_H            0x7CU
#define IIS2ICLX_FIFO_DATA_OUT_Z_L            0x7DU
#define IIS2ICLX_FIFO_DATA_OUT_Z_H            0x7EU
#define IIS2ICLX_PAGE_SEL                     0x02U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 4;
  uint8_t page_sel                 : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t page_sel                 : 4;
  uint8_t not_used_01              : 4;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_page_sel_t;

#define IIS2ICLX_EMB_FUNC_EN_B                0x05U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_en                   : 1;
  uint8_t not_used_01              : 3;
  uint8_t mlc_en                   : 1;
  uint8_t not_used_02              : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 3;
  uint8_t mlc_en                   : 1;
  uint8_t not_used_01              : 3;
  uint8_t fsm_en                   : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_emb_func_en_b_t;

#define IIS2ICLX_PAGE_ADDRESS                 0x08U
typedef struct {
  uint8_t page_addr                : 8;
} iis2iclx_page_address_t;

#define IIS2ICLX_PAGE_VALUE                   0x09U
typedef struct {
  uint8_t page_value               : 8;
} iis2iclx_page_value_t;

#define IIS2ICLX_EMB_FUNC_INT1                0x0AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 7;
  uint8_t int1_fsm_lc              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_fsm_lc              : 1;
  uint8_t not_used_01              : 7;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_emb_func_int1_t;

#define IIS2ICLX_FSM_INT1_A                   0x0BU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_fsm1                : 1;
  uint8_t int1_fsm2                : 1;
  uint8_t int1_fsm3                : 1;
  uint8_t int1_fsm4                : 1;
  uint8_t int1_fsm5                : 1;
  uint8_t int1_fsm6                : 1;
  uint8_t int1_fsm7                : 1;
  uint8_t int1_fsm8                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_fsm8                : 1;
  uint8_t int1_fsm7                : 1;
  uint8_t int1_fsm6                : 1;
  uint8_t int1_fsm5                : 1;
  uint8_t int1_fsm4                : 1;
  uint8_t int1_fsm3                : 1;
  uint8_t int1_fsm2                : 1;
  uint8_t int1_fsm1                : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_int1_a_t;

#define IIS2ICLX_FSM_INT1_B                   0x0CU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_fsm9                : 1;
  uint8_t int1_fsm10               : 1;
  uint8_t int1_fsm11               : 1;
  uint8_t int1_fsm12               : 1;
  uint8_t int1_fsm13               : 1;
  uint8_t int1_fsm14               : 1;
  uint8_t int1_fsm15               : 1;
  uint8_t int1_fsm16               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_fsm16               : 1;
  uint8_t int1_fsm15               : 1;
  uint8_t int1_fsm14               : 1;
  uint8_t int1_fsm13               : 1;
  uint8_t int1_fsm12               : 1;
  uint8_t int1_fsm11               : 1;
  uint8_t int1_fsm10               : 1;
  uint8_t int1_fsm9                : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_int1_b_t;

#define IIS2ICLX_MLC_INT1                     0x0DU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_mlc1                : 1;
  uint8_t int1_mlc2                : 1;
  uint8_t int1_mlc3                : 1;
  uint8_t int1_mlc4                : 1;
  uint8_t int1_mlc5                : 1;
  uint8_t int1_mlc6                : 1;
  uint8_t int1_mlc7                : 1;
  uint8_t int1_mlc8                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_mlc8                : 1;
  uint8_t int1_mlc7                : 1;
  uint8_t int1_mlc6                : 1;
  uint8_t int1_mlc5                : 1;
  uint8_t int1_mlc4                : 1;
  uint8_t int1_mlc3                : 1;
  uint8_t int1_mlc2                : 1;
  uint8_t int1_mlc1                : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_mlc_int1_t;

#define IIS2ICLX_EMB_FUNC_INT2                0x0EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 7;
  uint8_t int2_fsm_lc              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_fsm_lc              : 1;
  uint8_t not_used_01              : 7;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_emb_func_int2_t;

#define IIS2ICLX_FSM_INT2_A                   0x0FU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_fsm1                : 1;
  uint8_t int2_fsm2                : 1;
  uint8_t int2_fsm3                : 1;
  uint8_t int2_fsm4                : 1;
  uint8_t int2_fsm5                : 1;
  uint8_t int2_fsm6                : 1;
  uint8_t int2_fsm7                : 1;
  uint8_t int2_fsm8                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_fsm8                : 1;
  uint8_t int2_fsm7                : 1;
  uint8_t int2_fsm6                : 1;
  uint8_t int2_fsm5                : 1;
  uint8_t int2_fsm4                : 1;
  uint8_t int2_fsm3                : 1;
  uint8_t int2_fsm2                : 1;
  uint8_t int2_fsm1                : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_int2_a_t;

#define IIS2ICLX_FSM_INT2_B                   0x10U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_fsm9                : 1;
  uint8_t int2_fsm10               : 1;
  uint8_t int2_fsm11               : 1;
  uint8_t int2_fsm12               : 1;
  uint8_t int2_fsm13               : 1;
  uint8_t int2_fsm14               : 1;
  uint8_t int2_fsm15               : 1;
  uint8_t int2_fsm16               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_fsm16               : 1;
  uint8_t int2_fsm15               : 1;
  uint8_t int2_fsm14               : 1;
  uint8_t int2_fsm13               : 1;
  uint8_t int2_fsm12               : 1;
  uint8_t int2_fsm11               : 1;
  uint8_t int2_fsm10               : 1;
  uint8_t int2_fsm9                : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_int2_b_t;

#define IIS2ICLX_MLC_INT2                     0x11U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_mlc1             : 1;
  uint8_t int2_mlc2             : 1;
  uint8_t int2_mlc3             : 1;
  uint8_t int2_mlc4             : 1;
  uint8_t int2_mlc5             : 1;
  uint8_t int2_mlc6             : 1;
  uint8_t int2_mlc7             : 1;
  uint8_t int2_mlc8             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_mlc8             : 1;
  uint8_t int2_mlc7             : 1;
  uint8_t int2_mlc6             : 1;
  uint8_t int2_mlc5             : 1;
  uint8_t int2_mlc4             : 1;
  uint8_t int2_mlc3             : 1;
  uint8_t int2_mlc2             : 1;
  uint8_t int2_mlc1             : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_mlc_int2_t;

#define IIS2ICLX_EMB_FUNC_STATUS              0x12U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 7;
  uint8_t is_fsm_lc                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm_lc                : 1;
  uint8_t not_used_01              : 7;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_emb_func_status_t;

#define IIS2ICLX_FSM_STATUS_A                 0x13U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_fsm1                  : 1;
  uint8_t is_fsm2                  : 1;
  uint8_t is_fsm3                  : 1;
  uint8_t is_fsm4                  : 1;
  uint8_t is_fsm5                  : 1;
  uint8_t is_fsm6                  : 1;
  uint8_t is_fsm7                  : 1;
  uint8_t is_fsm8                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm8                  : 1;
  uint8_t is_fsm7                  : 1;
  uint8_t is_fsm6                  : 1;
  uint8_t is_fsm5                  : 1;
  uint8_t is_fsm4                  : 1;
  uint8_t is_fsm3                  : 1;
  uint8_t is_fsm2                  : 1;
  uint8_t is_fsm1                  : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_status_a_t;

#define IIS2ICLX_FSM_STATUS_B                 0x14U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_fsm9                  : 1;
  uint8_t is_fsm10                 : 1;
  uint8_t is_fsm11                 : 1;
  uint8_t is_fsm12                 : 1;
  uint8_t is_fsm13                 : 1;
  uint8_t is_fsm14                 : 1;
  uint8_t is_fsm15                 : 1;
  uint8_t is_fsm16                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_fsm16                 : 1;
  uint8_t is_fsm15                 : 1;
  uint8_t is_fsm14                 : 1;
  uint8_t is_fsm13                 : 1;
  uint8_t is_fsm12                 : 1;
  uint8_t is_fsm11                 : 1;
  uint8_t is_fsm10                 : 1;
  uint8_t is_fsm9                  : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_status_b_t;

#define IIS2ICLX_MLC_STATUS                   0x15U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t is_mlc1            : 1;
  uint8_t is_mlc2            : 1;
  uint8_t is_mlc3            : 1;
  uint8_t is_mlc4            : 1;
  uint8_t is_mlc5            : 1;
  uint8_t is_mlc6            : 1;
  uint8_t is_mlc7            : 1;
  uint8_t is_mlc8            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t is_mlc8            : 1;
  uint8_t is_mlc7            : 1;
  uint8_t is_mlc6            : 1;
  uint8_t is_mlc5            : 1;
  uint8_t is_mlc4            : 1;
  uint8_t is_mlc3            : 1;
  uint8_t is_mlc2            : 1;
  uint8_t is_mlc1            : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_mlc_status_t;

#define IIS2ICLX_PAGE_RW                      0x17U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 5;
  uint8_t page_rw                  : 2;  /* page_write + page_read */
  uint8_t emb_func_lir             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t emb_func_lir             : 1;
  uint8_t page_rw                  : 2;  /* page_write + page_read */
  uint8_t not_used_01              : 5;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_page_rw_t;

#define IIS2ICLX_FSM_ENABLE_A                 0x46U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm1_en                  : 1;
  uint8_t fsm2_en                  : 1;
  uint8_t fsm3_en                  : 1;
  uint8_t fsm4_en                  : 1;
  uint8_t fsm5_en                  : 1;
  uint8_t fsm6_en                  : 1;
  uint8_t fsm7_en                  : 1;
  uint8_t fsm8_en                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm8_en                  : 1;
  uint8_t fsm7_en                  : 1;
  uint8_t fsm6_en                  : 1;
  uint8_t fsm5_en                  : 1;
  uint8_t fsm4_en                  : 1;
  uint8_t fsm3_en                  : 1;
  uint8_t fsm2_en                  : 1;
  uint8_t fsm1_en                  : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_enable_a_t;

#define IIS2ICLX_FSM_ENABLE_B                 0x47U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm9_en                  : 1;
  uint8_t fsm10_en                 : 1;
  uint8_t fsm11_en                 : 1;
  uint8_t fsm12_en                 : 1;
  uint8_t fsm13_en                 : 1;
  uint8_t fsm14_en                 : 1;
  uint8_t fsm15_en                 : 1;
  uint8_t fsm16_en                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fsm16_en                  : 1;
  uint8_t fsm15_en                 : 1;
  uint8_t fsm14_en                 : 1;
  uint8_t fsm13_en                 : 1;
  uint8_t fsm12_en                 : 1;
  uint8_t fsm11_en                 : 1;
  uint8_t fsm10_en                 : 1;
  uint8_t fsm9_en                  : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_enable_b_t;

#define IIS2ICLX_FSM_LONG_COUNTER_L           0x48U
#define IIS2ICLX_FSM_LONG_COUNTER_H           0x49U
#define IIS2ICLX_FSM_LONG_COUNTER_CLEAR       0x4AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
uint8_t fsm_lc_clr               :
  2;  /* fsm_lc_cleared + fsm_lc_clear */
  uint8_t not_used_01              : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 6;
uint8_t fsm_lc_clr               :
  2;  /* fsm_lc_cleared + fsm_lc_clear */
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_long_counter_clear_t;

#define IIS2ICLX_FSM_OUTS1                    0x4CU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs1_t;

#define IIS2ICLX_FSM_OUTS2                    0x4DU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs2_t;

#define IIS2ICLX_FSM_OUTS3                    0x4EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs3_t;

#define IIS2ICLX_FSM_OUTS4                    0x4FU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs4_t;

#define IIS2ICLX_FSM_OUTS5                    0x50U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs5_t;

#define IIS2ICLX_FSM_OUTS6                    0x51U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs6_t;

#define IIS2ICLX_FSM_OUTS7                    0x52U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs7_t;

#define IIS2ICLX_FSM_OUTS8                    0x53U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs8_t;

#define IIS2ICLX_FSM_OUTS9                    0x54U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs9_t;

#define IIS2ICLX_FSM_OUTS10                   0x55U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs10_t;

#define IIS2ICLX_FSM_OUTS11                   0x56U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs11_t;

#define IIS2ICLX_FSM_OUTS12                   0x57U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs12_t;

#define IIS2ICLX_FSM_OUTS13                   0x58U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs13_t;

#define IIS2ICLX_FSM_OUTS14                   0x59U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs14_t;

#define IIS2ICLX_FSM_OUTS15                   0x5AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs15_t;

#define IIS2ICLX_FSM_OUTS16                   0x5BU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t n_v                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_x                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_x                      : 1;
  uint8_t n_x                      : 1;
  uint8_t p_y                      : 1;
  uint8_t n_y                      : 1;
  uint8_t p_z                      : 1;
  uint8_t n_z                      : 1;
  uint8_t p_v                      : 1;
  uint8_t n_v                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_fsm_outs16_t;

#define IIS2ICLX_EMB_FUNC_ODR_CFG_B           0x5FU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 3;
  uint8_t fsm_odr                  : 2;
  uint8_t not_used_02              : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 3;
  uint8_t fsm_odr                  : 2;
  uint8_t not_used_01              : 3;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_emb_func_odr_cfg_b_t;

#define IIS2ICLX_EMB_FUNC_ODR_CFG_C           0x60U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01             : 4;
  uint8_t mlc_odr                 : 2;
  uint8_t not_used_02             : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02             : 2;
  uint8_t mlc_odr                 : 2;
  uint8_t not_used_01             : 4;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_emb_func_odr_cfg_c_t;

#define IIS2ICLX_EMB_FUNC_INIT_B              0x67U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fsm_init                 : 1;
  uint8_t not_used_01              : 3;
  uint8_t mlc_init                 : 1;
  uint8_t not_used_02              : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 3;
  uint8_t mlc_init                 : 1;
  uint8_t not_used_01              : 3;
  uint8_t fsm_init                 : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_emb_func_init_b_t;

#define IIS2ICLX_MLC0_SRC                     0x70U
#define IIS2ICLX_MLC1_SRC                     0x71U
#define IIS2ICLX_MLC2_SRC                     0x72U
#define IIS2ICLX_MLC3_SRC                     0x73U
#define IIS2ICLX_MLC4_SRC                     0x74U
#define IIS2ICLX_MLC5_SRC                     0x75U
#define IIS2ICLX_MLC6_SRC                     0x76U
#define IIS2ICLX_MLC7_SRC                     0x77U

#define IIS2ICLX_FSM_LC_TIMEOUT_L             0x17AU
#define IIS2ICLX_FSM_LC_TIMEOUT_H             0x17BU
#define IIS2ICLX_FSM_PROGRAMS                 0x17CU
#define IIS2ICLX_FSM_START_ADD_L              0x17EU
#define IIS2ICLX_FSM_START_ADD_H              0x17FU
#define IIS2ICLX_SENSOR_HUB_1                 0x02U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_1_t;

#define IIS2ICLX_SENSOR_HUB_2                 0x03U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_2_t;

#define IIS2ICLX_SENSOR_HUB_3                 0x04U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_3_t;

#define IIS2ICLX_SENSOR_HUB_4                 0x05U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_4_t;

#define IIS2ICLX_SENSOR_HUB_5                 0x06U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_5_t;

#define IIS2ICLX_SENSOR_HUB_6                 0x07U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_6_t;

#define IIS2ICLX_SENSOR_HUB_7                 0x08U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_7_t;

#define IIS2ICLX_SENSOR_HUB_8                 0x09U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_8_t;

#define IIS2ICLX_SENSOR_HUB_9                 0x0AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_9_t;

#define IIS2ICLX_SENSOR_HUB_10                0x0BU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_10_t;

#define IIS2ICLX_SENSOR_HUB_11                0x0CU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_11_t;

#define IIS2ICLX_SENSOR_HUB_12                0x0DU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_12_t;

#define IIS2ICLX_SENSOR_HUB_13                0x0EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_13_t;

#define IIS2ICLX_SENSOR_HUB_14                0x0FU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_14_t;

#define IIS2ICLX_SENSOR_HUB_15                0x10U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_15_t;

#define IIS2ICLX_SENSOR_HUB_16                0x11U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_16_t;

#define IIS2ICLX_SENSOR_HUB_17                0x12U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_17_t;

#define IIS2ICLX_SENSOR_HUB_18                0x13U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit7                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                    : 1;
  uint8_t bit6                    : 1;
  uint8_t bit5                    : 1;
  uint8_t bit4                    : 1;
  uint8_t bit3                    : 1;
  uint8_t bit2                    : 1;
  uint8_t bit1                    : 1;
  uint8_t bit0                    : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_sensor_hub_18_t;

#define IIS2ICLX_MASTER_CONFIG                0x14U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t aux_sens_on              : 2;
  uint8_t master_on                : 1;
  uint8_t shub_pu_en               : 1;
  uint8_t pass_through_mode        : 1;
  uint8_t start_config             : 1;
  uint8_t write_once               : 1;
  uint8_t rst_master_regs          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t rst_master_regs          : 1;
  uint8_t write_once               : 1;
  uint8_t start_config             : 1;
  uint8_t pass_through_mode        : 1;
  uint8_t shub_pu_en               : 1;
  uint8_t master_on                : 1;
  uint8_t aux_sens_on              : 2;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_master_config_t;

#define IIS2ICLX_SLV0_ADD                     0x15U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t rw_0                     : 1;
  uint8_t slave0                   : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave0                   : 7;
  uint8_t rw_0                     : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_slv0_add_t;

#define IIS2ICLX_SLV0_SUBADD                  0x16U
typedef struct {
  uint8_t slave0_reg               : 8;
} iis2iclx_slv0_subadd_t;

#define IIS2ICLX_SLV0_CONFIG                  0x17U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave0_numop             : 3;
  uint8_t batch_ext_sens_0_en      : 1;
  uint8_t not_used_01              : 2;
  uint8_t shub_odr                 : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t shub_odr                 : 2;
  uint8_t not_used_01              : 2;
  uint8_t batch_ext_sens_0_en      : 1;
  uint8_t slave0_numop             : 3;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_slv0_config_t;

#define IIS2ICLX_SLV1_ADD                     0x18U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t r_1                      : 1;
  uint8_t slave1_add               : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave1_add               : 7;
  uint8_t r_1                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_slv1_add_t;

#define IIS2ICLX_SLV1_SUBADD                  0x19U
typedef struct {
  uint8_t slave1_reg               : 8;
} iis2iclx_slv1_subadd_t;

#define IIS2ICLX_SLV1_CONFIG                  0x1AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave1_numop             : 3;
  uint8_t batch_ext_sens_1_en      : 1;
  uint8_t not_used_01              : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 4;
  uint8_t batch_ext_sens_1_en      : 1;
  uint8_t slave1_numop             : 3;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_slv1_config_t;

#define IIS2ICLX_SLV2_ADD                     0x1BU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t r_2                      : 1;
  uint8_t slave2_add               : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave2_add               : 7;
  uint8_t r_2                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_slv2_add_t;

#define IIS2ICLX_SLV2_SUBADD                  0x1CU
typedef struct {
  uint8_t slave2_reg               : 8;
} iis2iclx_slv2_subadd_t;

#define IIS2ICLX_SLV2_CONFIG                  0x1DU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave2_numop             : 3;
  uint8_t batch_ext_sens_2_en      : 1;
  uint8_t not_used_01              : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 4;
  uint8_t batch_ext_sens_2_en      : 1;
  uint8_t slave2_numop             : 3;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_slv2_config_t;

#define IIS2ICLX_SLV3_ADD                     0x1EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t r_3                      : 1;
  uint8_t slave3_add               : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t slave3_add               : 7;
  uint8_t r_3                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_slv3_add_t;

#define IIS2ICLX_SLV3_SUBADD                  0x1FU
typedef struct {
  uint8_t slave3_reg               : 8;
} iis2iclx_slv3_subadd_t;

#define IIS2ICLX_SLV3_CONFIG                  0x20U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t slave3_numop             : 3;
  uint8_t batch_ext_sens_3_en      : 1;
  uint8_t not_used_01              : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 4;
  uint8_t batch_ext_sens_3_en      : 1;
  uint8_t slave3_numop             : 3;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_slv3_config_t;

#define IIS2ICLX_DATAWRITE_SLV0  0x21U
typedef struct {
  uint8_t slave0_dataw             : 8;
} iis2iclx_datawrite_slv0_t;

#define IIS2ICLX_STATUS_MASTER                0x22U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sens_hub_endop           : 1;
  uint8_t not_used_01              : 2;
  uint8_t slave0_nack              : 1;
  uint8_t slave1_nack              : 1;
  uint8_t slave2_nack              : 1;
  uint8_t slave3_nack              : 1;
  uint8_t wr_once_done             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t wr_once_done             : 1;
  uint8_t slave3_nack              : 1;
  uint8_t slave2_nack              : 1;
  uint8_t slave1_nack              : 1;
  uint8_t slave0_nack              : 1;
  uint8_t not_used_01              : 2;
  uint8_t sens_hub_endop           : 1;
#endif /* DRV_BYTE_ORDER */
} iis2iclx_status_master_t;

/**
  * @defgroup IIS2ICLX_Register_Union
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
  iis2iclx_func_cfg_access_t               func_cfg_access;
  iis2iclx_pin_ctrl_t                      pin_ctrl;
  iis2iclx_fifo_ctrl1_t                    fifo_ctrl1;
  iis2iclx_fifo_ctrl2_t                    fifo_ctrl2;
  iis2iclx_fifo_ctrl3_t                    fifo_ctrl3;
  iis2iclx_fifo_ctrl4_t                    fifo_ctrl4;
  iis2iclx_counter_bdr_reg1_t              counter_bdr_reg1;
  iis2iclx_counter_bdr_reg2_t              counter_bdr_reg2;
  iis2iclx_int1_ctrl_t                     int1_ctrl;
  iis2iclx_int2_ctrl_t                     int2_ctrl;
  iis2iclx_ctrl1_xl_t                      ctrl1_xl;
  iis2iclx_ctrl3_c_t                       ctrl3_c;
  iis2iclx_ctrl4_c_t                       ctrl4_c;
  iis2iclx_ctrl5_c_t                       ctrl5_c;
  iis2iclx_ctrl6_c_t                       ctrl6_c;
  iis2iclx_ctrl7_xl_t                      ctrl7_xl;
  iis2iclx_ctrl8_xl_t                      ctrl8_xl;
  iis2iclx_ctrl9_xl_t                      ctrl9_xl;
  iis2iclx_ctrl10_c_t                      ctrl10_c;
  iis2iclx_all_int_src_t                   all_int_src;
  iis2iclx_wake_up_src_t                   wake_up_src;
  iis2iclx_tap_src_t                       tap_src;
  iis2iclx_den_src_t                       den_src;
  iis2iclx_status_reg_t                    status_reg;
  iis2iclx_fifo_status1_t                  fifo_status1;
  iis2iclx_fifo_status2_t                  fifo_status2;
  iis2iclx_tap_cfg0_t                      tap_cfg0;
  iis2iclx_tap_cfg1_t                      tap_cfg1;
  iis2iclx_tap_cfg2_t                      tap_cfg2;
  iis2iclx_int_dur2_t                      int_dur2;
  iis2iclx_wake_up_ths_t                   wake_up_ths;
  iis2iclx_wake_up_dur_t                   wake_up_dur;
  iis2iclx_md1_cfg_t                       md1_cfg;
  iis2iclx_md2_cfg_t                       md2_cfg;
  iis2iclx_internal_freq_fine_t            internal_freq_fine;
  iis2iclx_fifo_data_out_tag_t             fifo_data_out_tag;
  iis2iclx_page_sel_t                      page_sel;
  iis2iclx_emb_func_en_b_t                 emb_func_en_b;
  iis2iclx_page_address_t                  page_address;
  iis2iclx_page_value_t                    page_value;
  iis2iclx_emb_func_int1_t                 emb_func_int1;
  iis2iclx_fsm_int1_a_t                    fsm_int1_a;
  iis2iclx_fsm_int1_b_t                    fsm_int1_b;
  iis2iclx_mlc_int1_t                      mlc_int1;
  iis2iclx_emb_func_int2_t                 emb_func_int2;
  iis2iclx_fsm_int2_a_t                    fsm_int2_a;
  iis2iclx_fsm_int2_b_t                    fsm_int2_b;
  iis2iclx_mlc_int2_t                      mlc_int2;
  iis2iclx_emb_func_status_t               emb_func_status;
  iis2iclx_fsm_status_a_t                  fsm_status_a;
  iis2iclx_fsm_status_b_t                  fsm_status_b;
  iis2iclx_mlc_status_mainpage_t           mlc_status_mainpage;
  iis2iclx_emb_func_odr_cfg_c_t            emb_func_odr_cfg_c;
  iis2iclx_page_rw_t                       page_rw;
  iis2iclx_fsm_enable_a_t                  fsm_enable_a;
  iis2iclx_fsm_enable_b_t                  fsm_enable_b;
  iis2iclx_fsm_long_counter_clear_t        fsm_long_counter_clear;
  iis2iclx_fsm_outs1_t                     fsm_outs1;
  iis2iclx_fsm_outs2_t                     fsm_outs2;
  iis2iclx_fsm_outs3_t                     fsm_outs3;
  iis2iclx_fsm_outs4_t                     fsm_outs4;
  iis2iclx_fsm_outs5_t                     fsm_outs5;
  iis2iclx_fsm_outs6_t                     fsm_outs6;
  iis2iclx_fsm_outs7_t                     fsm_outs7;
  iis2iclx_fsm_outs8_t                     fsm_outs8;
  iis2iclx_fsm_outs9_t                     fsm_outs9;
  iis2iclx_fsm_outs10_t                    fsm_outs10;
  iis2iclx_fsm_outs11_t                    fsm_outs11;
  iis2iclx_fsm_outs12_t                    fsm_outs12;
  iis2iclx_fsm_outs13_t                    fsm_outs13;
  iis2iclx_fsm_outs14_t                    fsm_outs14;
  iis2iclx_fsm_outs15_t                    fsm_outs15;
  iis2iclx_fsm_outs16_t                    fsm_outs16;
  iis2iclx_emb_func_odr_cfg_b_t            emb_func_odr_cfg_b;
  iis2iclx_emb_func_init_b_t               emb_func_init_b;
  iis2iclx_sensor_hub_1_t                  sensor_hub_1;
  iis2iclx_sensor_hub_2_t                  sensor_hub_2;
  iis2iclx_sensor_hub_3_t                  sensor_hub_3;
  iis2iclx_sensor_hub_4_t                  sensor_hub_4;
  iis2iclx_sensor_hub_5_t                  sensor_hub_5;
  iis2iclx_sensor_hub_6_t                  sensor_hub_6;
  iis2iclx_sensor_hub_7_t                  sensor_hub_7;
  iis2iclx_sensor_hub_8_t                  sensor_hub_8;
  iis2iclx_sensor_hub_9_t                  sensor_hub_9;
  iis2iclx_sensor_hub_10_t                 sensor_hub_10;
  iis2iclx_sensor_hub_11_t                 sensor_hub_11;
  iis2iclx_sensor_hub_12_t                 sensor_hub_12;
  iis2iclx_sensor_hub_13_t                 sensor_hub_13;
  iis2iclx_sensor_hub_14_t                 sensor_hub_14;
  iis2iclx_sensor_hub_15_t                 sensor_hub_15;
  iis2iclx_sensor_hub_16_t                 sensor_hub_16;
  iis2iclx_sensor_hub_17_t                 sensor_hub_17;
  iis2iclx_sensor_hub_18_t                 sensor_hub_18;
  iis2iclx_master_config_t                 master_config;
  iis2iclx_slv0_add_t                      slv0_add;
  iis2iclx_slv0_subadd_t                   slv0_subadd;
  iis2iclx_slv0_config_t                   slv0_config;
  iis2iclx_slv1_add_t                      slv1_add;
  iis2iclx_slv1_subadd_t                   slv1_subadd;
  iis2iclx_slv1_config_t                   slv1_config;
  iis2iclx_slv2_add_t                      slv2_add;
  iis2iclx_slv2_subadd_t                   slv2_subadd;
  iis2iclx_slv2_config_t                   slv2_config;
  iis2iclx_slv3_add_t                      slv3_add;
  iis2iclx_slv3_subadd_t                   slv3_subadd;
  iis2iclx_slv3_config_t                   slv3_config;
  iis2iclx_datawrite_slv0_t                datawrite_slv0;
  iis2iclx_status_master_t                 status_master;
  bitwise_t                               bitwise;
  uint8_t                                 byte;
} iis2iclx_reg_t;

/**
  * @}
  *
  */

int32_t iis2iclx_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
                          uint8_t *data,
                          uint16_t len);
int32_t iis2iclx_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                           uint8_t *data,
                           uint16_t len);

extern float_t iis2iclx_from_fs500mg_to_mg(int16_t lsb);
extern float_t iis2iclx_from_fs1g_to_mg(int16_t lsb);
extern float_t iis2iclx_from_fs2g_to_mg(int16_t lsb);
extern float_t iis2iclx_from_fs3g_to_mg(int16_t lsb);
extern float_t iis2iclx_from_lsb_to_celsius(int16_t lsb);
extern float_t iis2iclx_from_lsb_to_nsec(int32_t lsb);

typedef enum {
  IIS2ICLX_500mg   = 0,
  IIS2ICLX_3g      = 1,
  IIS2ICLX_1g      = 2,
  IIS2ICLX_2g      = 3,
} iis2iclx_fs_xl_t;
int32_t iis2iclx_xl_full_scale_set(stmdev_ctx_t *ctx,
                                   iis2iclx_fs_xl_t val);
int32_t iis2iclx_xl_full_scale_get(stmdev_ctx_t *ctx,
                                   iis2iclx_fs_xl_t *val);

typedef enum {
  IIS2ICLX_XL_ODR_OFF    = 0,
  IIS2ICLX_XL_ODR_12Hz5  = 1,
  IIS2ICLX_XL_ODR_26Hz   = 2,
  IIS2ICLX_XL_ODR_52Hz   = 3,
  IIS2ICLX_XL_ODR_104Hz  = 4,
  IIS2ICLX_XL_ODR_208Hz  = 5,
  IIS2ICLX_XL_ODR_416Hz  = 6,
  IIS2ICLX_XL_ODR_833Hz  = 7,
} iis2iclx_odr_xl_t;
int32_t iis2iclx_xl_data_rate_set(stmdev_ctx_t *ctx,
                                  iis2iclx_odr_xl_t val);
int32_t iis2iclx_xl_data_rate_get(stmdev_ctx_t *ctx,
                                  iis2iclx_odr_xl_t *val);

int32_t iis2iclx_block_data_update_set(stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t iis2iclx_block_data_update_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

typedef enum {
  IIS2ICLX_LSb_1mg  = 0,
  IIS2ICLX_LSb_16mg = 1,
} iis2iclx_usr_off_w_t;
int32_t iis2iclx_xl_offset_weight_set(stmdev_ctx_t *ctx,
                                      iis2iclx_usr_off_w_t val);
int32_t iis2iclx_xl_offset_weight_get(stmdev_ctx_t *ctx,
                                      iis2iclx_usr_off_w_t *val);

typedef enum {
  IIS2ICLX_HIGH_PERFORMANCE_MD  = 0,
  IIS2ICLX_LOW_NORMAL_POWER_MD  = 1,
} iis2iclx_xl_hm_mode_t;
int32_t iis2iclx_xl_power_mode_set(stmdev_ctx_t *ctx,
                                   iis2iclx_xl_hm_mode_t val);
int32_t iis2iclx_xl_power_mode_get(stmdev_ctx_t *ctx,
                                   iis2iclx_xl_hm_mode_t *val);

typedef enum {
  IIS2ICLX_GY_HIGH_PERFORMANCE  = 0,
  IIS2ICLX_GY_NORMAL            = 1,
} iis2iclx_g_hm_mode_t;
int32_t iis2iclx_gy_power_mode_set(stmdev_ctx_t *ctx,
                                   iis2iclx_g_hm_mode_t val);
int32_t iis2iclx_gy_power_mode_get(stmdev_ctx_t *ctx,
                                   iis2iclx_g_hm_mode_t *val);

typedef struct {
  iis2iclx_all_int_src_t       all_int_src;
  iis2iclx_wake_up_src_t       wake_up_src;
  iis2iclx_tap_src_t           tap_src;
  iis2iclx_den_src_t           den_src;
  iis2iclx_status_reg_t        status_reg;
  iis2iclx_emb_func_status_t   emb_func_status;
  iis2iclx_fsm_status_a_t      fsm_status_a;
  iis2iclx_fsm_status_b_t      fsm_status_b;
} iis2iclx_all_sources_t;
int32_t iis2iclx_all_sources_get(stmdev_ctx_t *ctx,
                                 iis2iclx_all_sources_t *val);

int32_t iis2iclx_status_reg_get(stmdev_ctx_t *ctx,
                                iis2iclx_status_reg_t *val);

int32_t iis2iclx_xl_flag_data_ready_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t iis2iclx_temp_flag_data_ready_get(stmdev_ctx_t *ctx,
                                          uint8_t *val);

int32_t iis2iclx_xl_usr_offset_x_set(stmdev_ctx_t *ctx,
                                     uint8_t *buff);
int32_t iis2iclx_xl_usr_offset_x_get(stmdev_ctx_t *ctx,
                                     uint8_t *buff);

int32_t iis2iclx_xl_usr_offset_y_set(stmdev_ctx_t *ctx,
                                     uint8_t *buff);
int32_t iis2iclx_xl_usr_offset_y_get(stmdev_ctx_t *ctx,
                                     uint8_t *buff);

int32_t iis2iclx_xl_usr_offset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_xl_usr_offset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_timestamp_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_timestamp_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_timestamp_raw_get(stmdev_ctx_t *ctx, int32_t *val);

int32_t iis2iclx_temperature_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t iis2iclx_acceleration_raw_get(stmdev_ctx_t *ctx,
                                      int16_t *val);

int32_t iis2iclx_fifo_out_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t iis2iclx_device_conf_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_device_conf_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_odr_cal_reg_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_odr_cal_reg_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS2ICLX_USER_BANK           = 0,
  IIS2ICLX_SENSOR_HUB_BANK     = 1,
  IIS2ICLX_EMBEDDED_FUNC_BANK  = 2,
} iis2iclx_reg_access_t;
int32_t iis2iclx_mem_bank_set(stmdev_ctx_t *ctx,
                              iis2iclx_reg_access_t val);
int32_t iis2iclx_mem_bank_get(stmdev_ctx_t *ctx,
                              iis2iclx_reg_access_t *val);

int32_t iis2iclx_ln_pg_write_byte(stmdev_ctx_t *ctx, uint16_t address,
                                  uint8_t *val);
int32_t iis2iclx_ln_pg_write(stmdev_ctx_t *ctx, uint16_t address,
                             uint8_t *buf, uint8_t len);
int32_t iis2iclx_ln_pg_read_byte(stmdev_ctx_t *ctx, uint16_t add,
                                 uint8_t *val);
int32_t iis2iclx_ln_pg_read(stmdev_ctx_t *ctx, uint16_t address,
                            uint8_t *val);

typedef enum {
  IIS2ICLX_DRDY_LATCHED = 0,
  IIS2ICLX_DRDY_PULSED  = 1,
} iis2iclx_dataready_pulsed_t;
int32_t iis2iclx_data_ready_mode_set(stmdev_ctx_t *ctx,
                                     iis2iclx_dataready_pulsed_t val);
int32_t iis2iclx_data_ready_mode_get(stmdev_ctx_t *ctx,
                                     iis2iclx_dataready_pulsed_t *val);

int32_t iis2iclx_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t iis2iclx_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_auto_increment_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_auto_increment_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS2ICLX_XL_ST_DISABLE  = 0,
  IIS2ICLX_XL_ST_POSITIVE = 1,
  IIS2ICLX_XL_ST_NEGATIVE = 2,
} iis2iclx_st_xl_t;
int32_t iis2iclx_xl_self_test_set(stmdev_ctx_t *ctx,
                                  iis2iclx_st_xl_t val);
int32_t iis2iclx_xl_self_test_get(stmdev_ctx_t *ctx,
                                  iis2iclx_st_xl_t *val);

int32_t iis2iclx_xl_filter_lp2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_xl_filter_lp2_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_filter_settling_mask_set(stmdev_ctx_t *ctx,
                                          uint8_t val);
int32_t iis2iclx_filter_settling_mask_get(stmdev_ctx_t *ctx,
                                          uint8_t *val);

typedef enum {
  IIS2ICLX_HP_PATH_DISABLE_ON_OUT    = 0x00,
  IIS2ICLX_SLOPE_ODR_DIV_4           = 0x10,
  IIS2ICLX_HP_ODR_DIV_10             = 0x11,
  IIS2ICLX_HP_ODR_DIV_20             = 0x12,
  IIS2ICLX_HP_ODR_DIV_45             = 0x13,
  IIS2ICLX_HP_ODR_DIV_100            = 0x14,
  IIS2ICLX_HP_ODR_DIV_200            = 0x15,
  IIS2ICLX_HP_ODR_DIV_400            = 0x16,
  IIS2ICLX_HP_ODR_DIV_800            = 0x17,
  IIS2ICLX_HP_REF_MD_ODR_DIV_10      = 0x31,
  IIS2ICLX_HP_REF_MD_ODR_DIV_20      = 0x32,
  IIS2ICLX_HP_REF_MD_ODR_DIV_45      = 0x33,
  IIS2ICLX_HP_REF_MD_ODR_DIV_100     = 0x34,
  IIS2ICLX_HP_REF_MD_ODR_DIV_200     = 0x35,
  IIS2ICLX_HP_REF_MD_ODR_DIV_400     = 0x36,
  IIS2ICLX_HP_REF_MD_ODR_DIV_800     = 0x37,
  IIS2ICLX_LP_ODR_DIV_10             = 0x01,
  IIS2ICLX_LP_ODR_DIV_20             = 0x02,
  IIS2ICLX_LP_ODR_DIV_45             = 0x03,
  IIS2ICLX_LP_ODR_DIV_100            = 0x04,
  IIS2ICLX_LP_ODR_DIV_200            = 0x05,
  IIS2ICLX_LP_ODR_DIV_400            = 0x06,
  IIS2ICLX_LP_ODR_DIV_800            = 0x07,
} iis2iclx_hp_slope_xl_en_t;
int32_t iis2iclx_xl_hp_path_on_out_set(stmdev_ctx_t *ctx,
                                       iis2iclx_hp_slope_xl_en_t val);
int32_t iis2iclx_xl_hp_path_on_out_get(stmdev_ctx_t *ctx,
                                       iis2iclx_hp_slope_xl_en_t *val);

int32_t iis2iclx_xl_fast_settling_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_xl_fast_settling_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum {
  IIS2ICLX_USE_SLOPE = 0,
  IIS2ICLX_USE_HPF   = 1,
} iis2iclx_slope_fds_t;
int32_t iis2iclx_xl_hp_path_internal_set(stmdev_ctx_t *ctx,
                                         iis2iclx_slope_fds_t val);
int32_t iis2iclx_xl_hp_path_internal_get(stmdev_ctx_t *ctx,
                                         iis2iclx_slope_fds_t *val);

typedef enum {
  IIS2ICLX_PULL_UP_DISC       = 0,
  IIS2ICLX_PULL_UP_CONNECT    = 1,
} iis2iclx_sdo_pu_en_t;
int32_t iis2iclx_sdo_sa0_mode_set(stmdev_ctx_t *ctx,
                                  iis2iclx_sdo_pu_en_t val);
int32_t iis2iclx_sdo_sa0_mode_get(stmdev_ctx_t *ctx,
                                  iis2iclx_sdo_pu_en_t *val);

typedef enum {
  IIS2ICLX_SPI_4_WIRE = 0,
  IIS2ICLX_SPI_3_WIRE = 1,
} iis2iclx_sim_t;
int32_t iis2iclx_spi_mode_set(stmdev_ctx_t *ctx, iis2iclx_sim_t val);
int32_t iis2iclx_spi_mode_get(stmdev_ctx_t *ctx, iis2iclx_sim_t *val);

typedef enum {
  IIS2ICLX_I2C_ENABLE  = 0,
  IIS2ICLX_I2C_DISABLE = 1,
} iis2iclx_i2c_disable_t;
int32_t iis2iclx_i2c_interface_set(stmdev_ctx_t *ctx,
                                   iis2iclx_i2c_disable_t val);
int32_t iis2iclx_i2c_interface_get(stmdev_ctx_t *ctx,
                                   iis2iclx_i2c_disable_t *val);

typedef struct {
  iis2iclx_int1_ctrl_t          int1_ctrl;
  iis2iclx_md1_cfg_t            md1_cfg;
  iis2iclx_emb_func_int1_t      emb_func_int1;
  iis2iclx_fsm_int1_a_t         fsm_int1_a;
  iis2iclx_fsm_int1_b_t         fsm_int1_b;
  iis2iclx_mlc_int1_t           mlc_int1;
} iis2iclx_pin_int1_route_t;
int32_t iis2iclx_pin_int1_route_set(stmdev_ctx_t *ctx,
                                    iis2iclx_pin_int1_route_t *val);
int32_t iis2iclx_pin_int1_route_get(stmdev_ctx_t *ctx,
                                    iis2iclx_pin_int1_route_t *val);

typedef struct {
  iis2iclx_int2_ctrl_t          int2_ctrl;
  iis2iclx_md2_cfg_t            md2_cfg;
  iis2iclx_emb_func_int2_t      emb_func_int2;
  iis2iclx_fsm_int2_a_t         fsm_int2_a;
  iis2iclx_fsm_int2_b_t         fsm_int2_b;
  iis2iclx_mlc_int2_t           mlc_int2;
} iis2iclx_pin_int2_route_t;
int32_t iis2iclx_pin_int2_route_set(stmdev_ctx_t *ctx,
                                    iis2iclx_pin_int2_route_t *val);
int32_t iis2iclx_pin_int2_route_get(stmdev_ctx_t *ctx,
                                    iis2iclx_pin_int2_route_t *val);

typedef enum {
  IIS2ICLX_PUSH_PULL   = 0,
  IIS2ICLX_OPEN_DRAIN  = 1,
} iis2iclx_pp_od_t;
int32_t iis2iclx_pin_mode_set(stmdev_ctx_t *ctx,
                              iis2iclx_pp_od_t val);
int32_t iis2iclx_pin_mode_get(stmdev_ctx_t *ctx,
                              iis2iclx_pp_od_t *val);

typedef enum {
  IIS2ICLX_ACTIVE_HIGH = 0,
  IIS2ICLX_ACTIVE_LOW  = 1,
} iis2iclx_h_lactive_t;
int32_t iis2iclx_pin_polarity_set(stmdev_ctx_t *ctx,
                                  iis2iclx_h_lactive_t val);
int32_t iis2iclx_pin_polarity_get(stmdev_ctx_t *ctx,
                                  iis2iclx_h_lactive_t *val);

int32_t iis2iclx_all_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_all_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS2ICLX_ALL_INT_PULSED            = 0,
  IIS2ICLX_BASE_LATCHED_EMB_PULSED   = 1,
  IIS2ICLX_BASE_PULSED_EMB_LATCHED   = 2,
  IIS2ICLX_ALL_INT_LATCHED           = 3,
} iis2iclx_lir_t;
int32_t iis2iclx_int_notification_set(stmdev_ctx_t *ctx,
                                      iis2iclx_lir_t val);
int32_t iis2iclx_int_notification_get(stmdev_ctx_t *ctx,
                                      iis2iclx_lir_t *val);

typedef enum {
  IIS2ICLX_LSb_FS_DIV_64       = 0,
  IIS2ICLX_LSb_FS_DIV_256      = 1,
} iis2iclx_wake_ths_w_t;
int32_t iis2iclx_wkup_ths_weight_set(stmdev_ctx_t *ctx,
                                     iis2iclx_wake_ths_w_t val);
int32_t iis2iclx_wkup_ths_weight_get(stmdev_ctx_t *ctx,
                                     iis2iclx_wake_ths_w_t *val);

int32_t iis2iclx_wkup_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_wkup_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_xl_usr_offset_on_wkup_set(stmdev_ctx_t *ctx,
                                           uint8_t val);
int32_t iis2iclx_xl_usr_offset_on_wkup_get(stmdev_ctx_t *ctx,
                                           uint8_t *val);

int32_t iis2iclx_wkup_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_wkup_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS2ICLX_DRIVE_SLEEP_CHG_EVENT = 0,
  IIS2ICLX_DRIVE_SLEEP_STATUS    = 1,
} iis2iclx_sleep_status_on_int_t;
int32_t iis2iclx_act_pin_notification_set(stmdev_ctx_t *ctx,
                                          iis2iclx_sleep_status_on_int_t val);
int32_t iis2iclx_act_pin_notification_get(stmdev_ctx_t *ctx,
                                          iis2iclx_sleep_status_on_int_t *val);

int32_t iis2iclx_act_sleep_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_act_sleep_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_tap_detection_on_y_set(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t iis2iclx_tap_detection_on_y_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t iis2iclx_tap_detection_on_x_set(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t iis2iclx_tap_detection_on_x_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t iis2iclx_tap_threshold_x_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_tap_threshold_x_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS2ICLX_XY = 0,
  IIS2ICLX_YX = 1,
} iis2iclx_tap_priority_t;
int32_t iis2iclx_tap_axis_priority_set(stmdev_ctx_t *ctx,
                                       iis2iclx_tap_priority_t val);
int32_t iis2iclx_tap_axis_priority_get(stmdev_ctx_t *ctx,
                                       iis2iclx_tap_priority_t *val);

int32_t iis2iclx_tap_threshold_y_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_tap_threshold_y_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_tap_shock_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_tap_shock_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_tap_quiet_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_tap_quiet_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_tap_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_tap_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS2ICLX_ONLY_SINGLE        = 0,
  IIS2ICLX_BOTH_SINGLE_DOUBLE = 1,
} iis2iclx_single_double_tap_t;
int32_t iis2iclx_tap_mode_set(stmdev_ctx_t *ctx,
                              iis2iclx_single_double_tap_t val);
int32_t iis2iclx_tap_mode_get(stmdev_ctx_t *ctx,
                              iis2iclx_single_double_tap_t *val);

int32_t iis2iclx_fifo_watermark_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t iis2iclx_fifo_watermark_get(stmdev_ctx_t *ctx, uint16_t *val);

int32_t iis2iclx_fifo_virtual_sens_odr_chg_set(stmdev_ctx_t *ctx,
                                               uint8_t val);
int32_t iis2iclx_fifo_virtual_sens_odr_chg_get(stmdev_ctx_t *ctx,
                                               uint8_t *val);

int32_t iis2iclx_fifo_stop_on_wtm_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_fifo_stop_on_wtm_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum {
  IIS2ICLX_XL_NOT_BATCHED        =  0,
  IIS2ICLX_XL_BATCHED_AT_1Hz6    = 11,
  IIS2ICLX_XL_BATCHED_AT_12Hz5   =  1,
  IIS2ICLX_XL_BATCHED_AT_26Hz    =  2,
  IIS2ICLX_XL_BATCHED_AT_52Hz    =  3,
  IIS2ICLX_XL_BATCHED_AT_104Hz   =  4,
  IIS2ICLX_XL_BATCHED_AT_208Hz   =  5,
  IIS2ICLX_XL_BATCHED_AT_417Hz   =  6,
  IIS2ICLX_XL_BATCHED_AT_833Hz   =  7,
} iis2iclx_bdr_xl_t;
int32_t iis2iclx_fifo_xl_batch_set(stmdev_ctx_t *ctx,
                                   iis2iclx_bdr_xl_t val);
int32_t iis2iclx_fifo_xl_batch_get(stmdev_ctx_t *ctx,
                                   iis2iclx_bdr_xl_t *val);

typedef enum {
  IIS2ICLX_BYPASS_MODE             = 0,
  IIS2ICLX_FIFO_MODE               = 1,
  IIS2ICLX_STREAM_TO_FIFO_MODE     = 3,
  IIS2ICLX_BYPASS_TO_STREAM_MODE   = 4,
  IIS2ICLX_STREAM_MODE             = 6,
  IIS2ICLX_BYPASS_TO_FIFO_MODE     = 7,
} iis2iclx_fifo_mode_t;
int32_t iis2iclx_fifo_mode_set(stmdev_ctx_t *ctx,
                               iis2iclx_fifo_mode_t val);
int32_t iis2iclx_fifo_mode_get(stmdev_ctx_t *ctx,
                               iis2iclx_fifo_mode_t *val);

typedef enum {
  IIS2ICLX_TEMP_NOT_BATCHED        = 0,
  IIS2ICLX_TEMP_BATCHED_AT_1Hz6    = 1,
  IIS2ICLX_TEMP_BATCHED_AT_12Hz5   = 2,
  IIS2ICLX_TEMP_BATCHED_AT_52Hz    = 3,
} iis2iclx_odr_t_batch_t;
int32_t iis2iclx_fifo_temp_batch_set(stmdev_ctx_t *ctx,
                                     iis2iclx_odr_t_batch_t val);
int32_t iis2iclx_fifo_temp_batch_get(stmdev_ctx_t *ctx,
                                     iis2iclx_odr_t_batch_t *val);

typedef enum {
  IIS2ICLX_NO_DECIMATION = 0,
  IIS2ICLX_DEC_1         = 1,
  IIS2ICLX_DEC_8         = 2,
  IIS2ICLX_DEC_32        = 3,
} iis2iclx_odr_ts_batch_t;
int32_t iis2iclx_fifo_timestamp_decimation_set(stmdev_ctx_t *ctx,
                                               iis2iclx_odr_ts_batch_t val);
int32_t iis2iclx_fifo_timestamp_decimation_get(stmdev_ctx_t *ctx,
                                               iis2iclx_odr_ts_batch_t *val);

int32_t iis2iclx_rst_batch_counter_set(stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t iis2iclx_rst_batch_counter_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

int32_t iis2iclx_batch_counter_threshold_set(stmdev_ctx_t *ctx,
                                             uint16_t val);
int32_t iis2iclx_batch_counter_threshold_get(stmdev_ctx_t *ctx,
                                             uint16_t *val);

int32_t iis2iclx_fifo_data_level_get(stmdev_ctx_t *ctx,
                                     uint16_t *val);

int32_t iis2iclx_fifo_status_get(stmdev_ctx_t *ctx,
                                 iis2iclx_fifo_status2_t *val);

int32_t iis2iclx_fifo_full_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS2ICLX_XL_NC_TAG    = 2,
  IIS2ICLX_TEMPERATURE_TAG,
  IIS2ICLX_TIMESTAMP_TAG,
  IIS2ICLX_CFG_CHANGE_TAG,
  IIS2ICLX_SENSORHUB_SLAVE0_TAG,
  IIS2ICLX_SENSORHUB_SLAVE1_TAG,
  IIS2ICLX_SENSORHUB_SLAVE2_TAG,
  IIS2ICLX_SENSORHUB_SLAVE3_TAG,
  IIS2ICLX_SENSORHUB_NACK_TAG  = 0x19,
} iis2iclx_fifo_tag_t;
int32_t iis2iclx_fifo_sensor_tag_get(stmdev_ctx_t *ctx,
                                     iis2iclx_fifo_tag_t *val);

int32_t iis2iclx_sh_batch_slave_0_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_sh_batch_slave_0_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t iis2iclx_sh_batch_slave_1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_sh_batch_slave_1_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t iis2iclx_sh_batch_slave_2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_sh_batch_slave_2_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t iis2iclx_sh_batch_slave_3_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_sh_batch_slave_3_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum {
  IIS2ICLX_DEN_DISABLE    = 0x00,
  IIS2ICLX_LEVEL_FIFO     = 0x76,
  IIS2ICLX_LEVEL_LETCHED  = 0x73,
  IIS2ICLX_LEVEL_TRIGGER  = 0x72,
  IIS2ICLX_EDGE_TRIGGER   = 0x74,
} iis2iclx_den_mode_t;
int32_t iis2iclx_den_mode_set(stmdev_ctx_t *ctx,
                              iis2iclx_den_mode_t val);
int32_t iis2iclx_den_mode_get(stmdev_ctx_t *ctx,
                              iis2iclx_den_mode_t *val);

typedef enum {
  IIS2ICLX_DEN_ACT_LOW  = 0,
  IIS2ICLX_DEN_ACT_HIGH = 1,
} iis2iclx_den_lh_t;
int32_t iis2iclx_den_polarity_set(stmdev_ctx_t *ctx,
                                  iis2iclx_den_lh_t val);
int32_t iis2iclx_den_polarity_get(stmdev_ctx_t *ctx,
                                  iis2iclx_den_lh_t *val);

int32_t iis2iclx_den_mark_axis_x_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_den_mark_axis_x_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_den_mark_axis_y_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_den_mark_axis_y_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_long_cnt_flag_data_ready_get(stmdev_ctx_t *ctx,
                                              uint8_t *val);

int32_t iis2iclx_emb_fsm_en_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_emb_fsm_en_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct {
  iis2iclx_fsm_enable_a_t          fsm_enable_a;
  iis2iclx_fsm_enable_b_t          fsm_enable_b;
} iis2iclx_emb_fsm_enable_t;
int32_t iis2iclx_fsm_enable_set(stmdev_ctx_t *ctx,
                                iis2iclx_emb_fsm_enable_t *val);
int32_t iis2iclx_fsm_enable_get(stmdev_ctx_t *ctx,
                                iis2iclx_emb_fsm_enable_t *val);

int32_t iis2iclx_long_cnt_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t iis2iclx_long_cnt_get(stmdev_ctx_t *ctx, uint16_t *val);

typedef enum {
  IIS2ICLX_LC_NORMAL     = 0,
  IIS2ICLX_LC_CLEAR      = 1,
  IIS2ICLX_LC_CLEAR_DONE = 2,
} iis2iclx_fsm_lc_clr_t;
int32_t iis2iclx_long_clr_set(stmdev_ctx_t *ctx,
                              iis2iclx_fsm_lc_clr_t val);
int32_t iis2iclx_long_clr_get(stmdev_ctx_t *ctx,
                              iis2iclx_fsm_lc_clr_t *val);

typedef struct {
  iis2iclx_fsm_outs1_t    fsm_outs1;
  iis2iclx_fsm_outs2_t    fsm_outs2;
  iis2iclx_fsm_outs3_t    fsm_outs3;
  iis2iclx_fsm_outs4_t    fsm_outs4;
  iis2iclx_fsm_outs5_t    fsm_outs5;
  iis2iclx_fsm_outs6_t    fsm_outs6;
  iis2iclx_fsm_outs7_t    fsm_outs7;
  iis2iclx_fsm_outs8_t    fsm_outs8;
  iis2iclx_fsm_outs9_t    fsm_outs9;
  iis2iclx_fsm_outs10_t    fsm_outs10;
  iis2iclx_fsm_outs11_t    fsm_outs11;
  iis2iclx_fsm_outs12_t    fsm_outs12;
  iis2iclx_fsm_outs13_t    fsm_outs13;
  iis2iclx_fsm_outs14_t    fsm_outs14;
  iis2iclx_fsm_outs15_t    fsm_outs15;
  iis2iclx_fsm_outs16_t    fsm_outs16;
} iis2iclx_fsm_out_t;
int32_t iis2iclx_fsm_out_get(stmdev_ctx_t *ctx,
                             iis2iclx_fsm_out_t *val);

typedef enum {
  IIS2ICLX_ODR_FSM_12Hz5 = 0,
  IIS2ICLX_ODR_FSM_26Hz  = 1,
  IIS2ICLX_ODR_FSM_52Hz  = 2,
  IIS2ICLX_ODR_FSM_104Hz = 3,
} iis2iclx_fsm_odr_t;
int32_t iis2iclx_fsm_data_rate_set(stmdev_ctx_t *ctx,
                                   iis2iclx_fsm_odr_t val);
int32_t iis2iclx_fsm_data_rate_get(stmdev_ctx_t *ctx,
                                   iis2iclx_fsm_odr_t *val);

int32_t iis2iclx_fsm_init_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_fsm_init_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_long_cnt_int_value_set(stmdev_ctx_t *ctx,
                                        uint16_t val);
int32_t iis2iclx_long_cnt_int_value_get(stmdev_ctx_t *ctx,
                                        uint16_t *val);

int32_t iis2iclx_fsm_number_of_programs_set(stmdev_ctx_t *ctx,
                                            uint8_t *buff);
int32_t iis2iclx_fsm_number_of_programs_get(stmdev_ctx_t *ctx,
                                            uint8_t *buff);

int32_t iis2iclx_fsm_start_address_set(stmdev_ctx_t *ctx,
                                       uint16_t val);
int32_t iis2iclx_fsm_start_address_get(stmdev_ctx_t *ctx,
                                       uint16_t *val);

int32_t iis2iclx_mlc_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_mlc_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis2iclx_mlc_status_get(stmdev_ctx_t *ctx,
                                iis2iclx_mlc_status_mainpage_t *val);

typedef enum {
  IIS2ICLX_ODR_PRGS_12Hz5 = 0,
  IIS2ICLX_ODR_PRGS_26Hz  = 1,
  IIS2ICLX_ODR_PRGS_52Hz  = 2,
  IIS2ICLX_ODR_PRGS_104Hz = 3,
} iis2iclx_mlc_odr_t;
int32_t iis2iclx_mlc_data_rate_set(stmdev_ctx_t *ctx,
                                   iis2iclx_mlc_odr_t val);
int32_t iis2iclx_mlc_data_rate_get(stmdev_ctx_t *ctx,
                                   iis2iclx_mlc_odr_t *val);

int32_t iis2iclx_mlc_out_get(stmdev_ctx_t *ctx, uint8_t *buff);

typedef struct {
  iis2iclx_sensor_hub_1_t   sh_byte_1;
  iis2iclx_sensor_hub_2_t   sh_byte_2;
  iis2iclx_sensor_hub_3_t   sh_byte_3;
  iis2iclx_sensor_hub_4_t   sh_byte_4;
  iis2iclx_sensor_hub_5_t   sh_byte_5;
  iis2iclx_sensor_hub_6_t   sh_byte_6;
  iis2iclx_sensor_hub_7_t   sh_byte_7;
  iis2iclx_sensor_hub_8_t   sh_byte_8;
  iis2iclx_sensor_hub_9_t   sh_byte_9;
  iis2iclx_sensor_hub_10_t  sh_byte_10;
  iis2iclx_sensor_hub_11_t  sh_byte_11;
  iis2iclx_sensor_hub_12_t  sh_byte_12;
  iis2iclx_sensor_hub_13_t  sh_byte_13;
  iis2iclx_sensor_hub_14_t  sh_byte_14;
  iis2iclx_sensor_hub_15_t  sh_byte_15;
  iis2iclx_sensor_hub_16_t  sh_byte_16;
  iis2iclx_sensor_hub_17_t  sh_byte_17;
  iis2iclx_sensor_hub_18_t  sh_byte_18;
} iis2iclx_emb_sh_read_t;
int32_t iis2iclx_sh_read_data_raw_get(stmdev_ctx_t *ctx,
                                      iis2iclx_emb_sh_read_t *val);

typedef enum {
  IIS2ICLX_SLV_0       = 0,
  IIS2ICLX_SLV_0_1     = 1,
  IIS2ICLX_SLV_0_1_2   = 2,
  IIS2ICLX_SLV_0_1_2_3 = 3,
} iis2iclx_aux_sens_on_t;
int32_t iis2iclx_sh_slave_connected_set(stmdev_ctx_t *ctx,
                                        iis2iclx_aux_sens_on_t val);
int32_t iis2iclx_sh_slave_connected_get(stmdev_ctx_t *ctx,
                                        iis2iclx_aux_sens_on_t *val);

int32_t iis2iclx_sh_master_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_sh_master_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS2ICLX_EXT_PULL_UP      = 0,
  IIS2ICLX_INTERNAL_PULL_UP = 1,
} iis2iclx_shub_pu_en_t;
int32_t iis2iclx_sh_pin_mode_set(stmdev_ctx_t *ctx,
                                 iis2iclx_shub_pu_en_t val);
int32_t iis2iclx_sh_pin_mode_get(stmdev_ctx_t *ctx,
                                 iis2iclx_shub_pu_en_t *val);

int32_t iis2iclx_sh_pass_through_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis2iclx_sh_pass_through_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS2ICLX_EXT_ON_INT2_PIN = 1,
  IIS2ICLX_XL_GY_DRDY      = 0,
} iis2iclx_start_config_t;
int32_t iis2iclx_sh_syncro_mode_set(stmdev_ctx_t *ctx,
                                    iis2iclx_start_config_t val);
int32_t iis2iclx_sh_syncro_mode_get(stmdev_ctx_t *ctx,
                                    iis2iclx_start_config_t *val);

typedef enum {
  IIS2ICLX_EACH_SH_CYCLE    = 0,
  IIS2ICLX_ONLY_FIRST_CYCLE = 1,
} iis2iclx_write_once_t;
int32_t iis2iclx_sh_write_mode_set(stmdev_ctx_t *ctx,
                                   iis2iclx_write_once_t val);
int32_t iis2iclx_sh_write_mode_get(stmdev_ctx_t *ctx,
                                   iis2iclx_write_once_t *val);

int32_t iis2iclx_sh_reset_set(stmdev_ctx_t *ctx);
int32_t iis2iclx_sh_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS2ICLX_SH_ODR_104Hz = 0,
  IIS2ICLX_SH_ODR_52Hz  = 1,
  IIS2ICLX_SH_ODR_26Hz  = 2,
  IIS2ICLX_SH_ODR_13Hz  = 3,
} iis2iclx_shub_odr_t;
int32_t iis2iclx_sh_data_rate_set(stmdev_ctx_t *ctx,
                                  iis2iclx_shub_odr_t val);
int32_t iis2iclx_sh_data_rate_get(stmdev_ctx_t *ctx,
                                  iis2iclx_shub_odr_t *val);

typedef struct {
  uint8_t   slv0_add;
  uint8_t   slv0_subadd;
  uint8_t   slv0_data;
} iis2iclx_sh_cfg_write_t;
int32_t iis2iclx_sh_cfg_write(stmdev_ctx_t *ctx,
                              iis2iclx_sh_cfg_write_t *val);

typedef struct {
  uint8_t   slv_add;
  uint8_t   slv_subadd;
  uint8_t   slv_len;
} iis2iclx_sh_cfg_read_t;
int32_t iis2iclx_sh_slv0_cfg_read(stmdev_ctx_t *ctx,
                                  iis2iclx_sh_cfg_read_t *val);
int32_t iis2iclx_sh_slv1_cfg_read(stmdev_ctx_t *ctx,
                                  iis2iclx_sh_cfg_read_t *val);
int32_t iis2iclx_sh_slv2_cfg_read(stmdev_ctx_t *ctx,
                                  iis2iclx_sh_cfg_read_t *val);
int32_t iis2iclx_sh_slv3_cfg_read(stmdev_ctx_t *ctx,
                                  iis2iclx_sh_cfg_read_t *val);

int32_t iis2iclx_sh_status_get(stmdev_ctx_t *ctx,
                               iis2iclx_status_master_t *val);

typedef enum {
  IIS2ICLX_SEL_BY_HW   = 0x00, /* bus mode select by HW (SPI 3W disable) */
  IIS2ICLX_SPI_4W      = 0x01, /* Only SPI: SDO / SDI separated pins */
  IIS2ICLX_SPI_3W      = 0x03, /* Only SPI: SDO / SDI share the same pin */
} iis2iclx_bus_mode_t;
int32_t iis2iclx_bus_mode_set(stmdev_ctx_t *ctx,
                              iis2iclx_bus_mode_t val);
int32_t iis2iclx_bus_mode_get(stmdev_ctx_t *ctx,
                              iis2iclx_bus_mode_t *val);

/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* IIS2ICLX_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
