/*
 ******************************************************************************
 * @file    ism303dac_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          ism303dac_reg.c driver.
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
#ifndef ISM303DAC_REGS_H
#define ISM303DAC_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup ISM303DAC
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

/** @defgroup ism303dac_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format **/
#define ISM303DAC_I2C_ADD_XL       0x3BU
#define ISM303DAC_I2C_ADD_MG       0x3DU

/** Device Identification (Who am I) **/
#define ISM303DAC_ID_XL            0x43U
#define ISM303DAC_ID_MG            0x40U

/**
  * @}
  *
  */

#define ISM303DAC_MODULE_8BIT_A           0x0CU
#define ISM303DAC_WHO_AM_I_A              0x0FU
#define ISM303DAC_CTRL1_A                 0x20U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bdu                 : 1;
  uint8_t hf_odr              : 1;
  uint8_t fs                  : 2;
  uint8_t odr                 : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr                 : 4;
  uint8_t fs                  : 2;
  uint8_t hf_odr              : 1;
  uint8_t bdu                 : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_ctrl1_a_t;

#define ISM303DAC_CTRL2_A                 0x21U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sim                 : 1;
  uint8_t i2c_disable         : 1;
  uint8_t if_add_inc          : 1;
  uint8_t fds_slope           : 1;
  uint8_t not_used_01         : 2;
  uint8_t soft_reset          : 1;
  uint8_t boot                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                : 1;
  uint8_t soft_reset          : 1;
  uint8_t not_used_01         : 2;
  uint8_t fds_slope           : 1;
  uint8_t if_add_inc          : 1;
  uint8_t i2c_disable         : 1;
  uint8_t sim                 : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_ctrl2_a_t;

#define ISM303DAC_CTRL3_A                 0x22U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t pp_od               : 1;
  uint8_t h_lactive           : 1;
  uint8_t lir                 : 1;
  uint8_t tap_z_en            : 1;
  uint8_t tap_y_en            : 1;
  uint8_t tap_x_en            : 1;
  uint8_t st                  : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t st                  : 2;
  uint8_t tap_x_en            : 1;
  uint8_t tap_y_en            : 1;
  uint8_t tap_z_en            : 1;
  uint8_t lir                 : 1;
  uint8_t h_lactive           : 1;
  uint8_t pp_od               : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_ctrl3_a_t;

#define ISM303DAC_CTRL4_A                 0x23U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_drdy           : 1;
  uint8_t int1_fth            : 1;
  uint8_t int1_6d             : 1;
  uint8_t int1_tap            : 1;
  uint8_t int1_ff             : 1;
  uint8_t int1_wu             : 1;
  uint8_t int1_s_tap          : 1;
  uint8_t not_used_01         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01         : 1;
  uint8_t int1_s_tap          : 1;
  uint8_t int1_wu             : 1;
  uint8_t int1_ff             : 1;
  uint8_t int1_tap            : 1;
  uint8_t int1_6d             : 1;
  uint8_t int1_fth            : 1;
  uint8_t int1_drdy           : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_ctrl4_a_t;

#define ISM303DAC_CTRL5_A                 0x24U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_drdy           : 1;
  uint8_t int2_fth            : 1;
  uint8_t not_used_01         : 3;
  uint8_t int2_on_int1        : 1;
  uint8_t int2_boot           : 1;
  uint8_t drdy_pulsed         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t drdy_pulsed         : 1;
  uint8_t int2_boot           : 1;
  uint8_t int2_on_int1        : 1;
  uint8_t not_used_01         : 3;
  uint8_t int2_fth            : 1;
  uint8_t int2_drdy           : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_ctrl5_a_t;

#define ISM303DAC_FIFO_CTRL_A             0x25U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t if_cs_pu_dis        : 1;
  uint8_t not_used_01         : 2;
  uint8_t module_to_fifo      : 1;
  uint8_t not_used_02         : 1;
  uint8_t fmode               : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fmode               : 3;
  uint8_t not_used_02         : 1;
  uint8_t module_to_fifo      : 1;
  uint8_t not_used_01         : 2;
  uint8_t if_cs_pu_dis        : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_fifo_ctrl_a_t;

#define ISM303DAC_OUT_T_A                 0x26U
#define ISM303DAC_STATUS_A                0x27U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t drdy                : 1;
  uint8_t ff_ia               : 1;
  uint8_t _6d_ia              : 1;
  uint8_t single_tap          : 1;
  uint8_t double_tap          : 1;
  uint8_t sleep_state         : 1;
  uint8_t wu_ia               : 1;
  uint8_t fifo_ths            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_ths            : 1;
  uint8_t wu_ia               : 1;
  uint8_t sleep_state         : 1;
  uint8_t double_tap          : 1;
  uint8_t single_tap          : 1;
  uint8_t _6d_ia              : 1;
  uint8_t ff_ia               : 1;
  uint8_t drdy                : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_status_a_t;

#define ISM303DAC_OUT_X_L_A               0x28U
#define ISM303DAC_OUT_X_H_A               0x29U
#define ISM303DAC_OUT_Y_L_A               0x2AU
#define ISM303DAC_OUT_Y_H_A               0x2BU
#define ISM303DAC_OUT_Z_L_A               0x2CU
#define ISM303DAC_OUT_Z_H_A               0x2DU
#define ISM303DAC_FIFO_THS_A              0x2EU
#define ISM303DAC_FIFO_SRC_A              0x2FU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01         : 5;
  uint8_t diff                : 1;
  uint8_t fifo_ovr            : 1;
  uint8_t fth                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fth                 : 1;
  uint8_t fifo_ovr            : 1;
  uint8_t diff                : 1;
  uint8_t not_used_01         : 5;
#endif /* DRV_BYTE_ORDER */
} ism303dac_fifo_src_a_t;

#define ISM303DAC_FIFO_SAMPLES_A          0x30U
typedef struct {
  uint8_t diff                : 8;
} ism303dac_fifo_samples_a_t;

#define ISM303DAC_TAP_6D_THS_A            0x31U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tap_ths             : 5;
  uint8_t _6d_ths             : 2;
  uint8_t _4d_en              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t _4d_en              : 1;
  uint8_t _6d_ths             : 2;
  uint8_t tap_ths             : 5;
#endif /* DRV_BYTE_ORDER */
} ism303dac_tap_6d_ths_a_t;

#define ISM303DAC_INT_DUR_A               0x32U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t shock               : 2;
  uint8_t quiet               : 2;
  uint8_t lat                 : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t lat                 : 4;
  uint8_t quiet               : 2;
  uint8_t shock               : 2;
#endif /* DRV_BYTE_ORDER */

} ism303dac_int_dur_a_t;

#define ISM303DAC_WAKE_UP_THS_A           0x33U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wu_ths              : 6;
  uint8_t sleep_on            : 1;
  uint8_t single_double_tap   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t single_double_tap   : 1;
  uint8_t sleep_on            : 1;
  uint8_t wu_ths              : 6;
#endif /* DRV_BYTE_ORDER */

} ism303dac_wake_up_ths_a_t;

#define ISM303DAC_WAKE_UP_DUR_A           0x34U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sleep_dur           : 4;
  uint8_t int1_fss7           : 1;
  uint8_t wu_dur              : 2;
  uint8_t ff_dur              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur              : 1;
  uint8_t wu_dur              : 2;
  uint8_t int1_fss7           : 1;
  uint8_t sleep_dur           : 4;
#endif /* DRV_BYTE_ORDER */

} ism303dac_wake_up_dur_a_t;

#define ISM303DAC_FREE_FALL_A             0x35U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ff_ths              : 3;
  uint8_t ff_dur              : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ff_dur              : 5;
  uint8_t ff_ths              : 3;
#endif /* DRV_BYTE_ORDER */
} ism303dac_free_fall_a_t;

#define ISM303DAC_STATUS_DUP_A            0x36U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t drdy                : 1;
  uint8_t ff_ia               : 1;
  uint8_t _6d_ia              : 1;
  uint8_t single_tap          : 1;
  uint8_t double_tap          : 1;
  uint8_t sleep_state         : 1;
  uint8_t wu_ia               : 1;
  uint8_t ovr                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ovr                 : 1;
  uint8_t wu_ia               : 1;
  uint8_t sleep_state         : 1;
  uint8_t double_tap          : 1;
  uint8_t single_tap          : 1;
  uint8_t _6d_ia              : 1;
  uint8_t ff_ia               : 1;
  uint8_t drdy                : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_status_dup_a_t;

#define ISM303DAC_WAKE_UP_SRC_A           0x37U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_wu                : 1;
  uint8_t y_wu                : 1;
  uint8_t x_wu                : 1;
  uint8_t wu_ia               : 1;
  uint8_t sleep_state_ia      : 1;
  uint8_t ff_ia               : 1;
  uint8_t not_used_01         : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01         : 2;
  uint8_t ff_ia               : 1;
  uint8_t sleep_state_ia      : 1;
  uint8_t wu_ia               : 1;
  uint8_t x_wu                : 1;
  uint8_t y_wu                : 1;
  uint8_t z_wu                : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_wake_up_src_a_t;

#define ISM303DAC_TAP_SRC_A               0x38U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t z_tap               : 1;
  uint8_t y_tap               : 1;
  uint8_t x_tap               : 1;
  uint8_t tap_sign            : 1;
  uint8_t double_tap          : 1;
  uint8_t single_tap          : 1;
  uint8_t tap_ia              : 1;
  uint8_t not_used_01         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01         : 1;
  uint8_t tap_ia              : 1;
  uint8_t single_tap          : 1;
  uint8_t double_tap          : 1;
  uint8_t tap_sign            : 1;
  uint8_t x_tap               : 1;
  uint8_t y_tap               : 1;
  uint8_t z_tap               : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_tap_src_a_t;

#define ISM303DAC_6D_SRC_A                0x39U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl                  : 1;
  uint8_t xh                  : 1;
  uint8_t yl                  : 1;
  uint8_t yh                  : 1;
  uint8_t zl                  : 1;
  uint8_t zh                  : 1;
  uint8_t _6d_ia              : 1;
  uint8_t not_used_01         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01         : 1;
  uint8_t _6d_ia              : 1;
  uint8_t zh                  : 1;
  uint8_t zl                  : 1;
  uint8_t yh                  : 1;
  uint8_t yl                  : 1;
  uint8_t xh                  : 1;
  uint8_t xl                  : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_6d_src_a_t;

#define ISM303DAC_FUNC_SRC_A              0x3EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01         : 1;
  uint8_t module_ready        : 1;
  uint8_t not_used_02         : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02         : 6;
  uint8_t module_ready        : 1;
  uint8_t not_used_01         : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_func_src_a_t;

#define ISM303DAC_FUNC_CTRL_A             0x3FU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01         : 5;
  uint8_t module_on           : 1;
  uint8_t not_used_02         : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02         : 2;
  uint8_t module_on           : 1;
  uint8_t not_used_01         : 5;
#endif /* DRV_BYTE_ORDER */

} ism303dac_func_ctrl_a_t;

#define ISM303DAC_OFFSET_X_REG_L_M          0x45U
#define ISM303DAC_OFFSET_X_REG_H_M          0x46U
#define ISM303DAC_OFFSET_Y_REG_L_M          0x47U
#define ISM303DAC_OFFSET_Y_REG_H_M          0x48U
#define ISM303DAC_OFFSET_Z_REG_L_M          0x49U
#define ISM303DAC_OFFSET_Z_REG_H_M          0x4AU
#define ISM303DAC_WHO_AM_I_M                0x4FU
#define ISM303DAC_CFG_REG_A_M               0x60U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t md                     : 2;
  uint8_t odr                    : 2;
  uint8_t lp                     : 1;
  uint8_t soft_rst               : 1;
  uint8_t reboot                 : 1;
  uint8_t comp_temp_en           : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t comp_temp_en           : 1;
  uint8_t reboot                 : 1;
  uint8_t soft_rst               : 1;
  uint8_t lp                     : 1;
  uint8_t odr                    : 2;
  uint8_t md                     : 2;
#endif /* DRV_BYTE_ORDER */

} ism303dac_cfg_reg_a_m_t;

#define ISM303DAC_CFG_REG_B_M               0x61U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lpf                    : 1;
  uint8_t set_rst                : 2; /* off_canc + set_freq */
  uint8_t int_on_dataoff         : 1;
  uint8_t off_canc_one_shot      : 1;
  uint8_t not_used_01            : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01            : 3;
  uint8_t off_canc_one_shot      : 1;
  uint8_t int_on_dataoff         : 1;
  uint8_t set_rst                : 2; /* off_canc + set_freq */
  uint8_t lpf                    : 1;
#endif /* DRV_BYTE_ORDER */

} ism303dac_cfg_reg_b_m_t;

#define ISM303DAC_CFG_REG_C_M               0x62U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int_mag                : 1;
  uint8_t self_test              : 1;
  uint8_t not_used_01            : 1;
  uint8_t ble                    : 1;
  uint8_t bdu                    : 1;
  uint8_t i2c_dis                : 1;
  uint8_t int_mag_pin            : 1;
  uint8_t not_used_02            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02            : 1;
  uint8_t int_mag_pin            : 1;
  uint8_t i2c_dis                : 1;
  uint8_t bdu                    : 1;
  uint8_t ble                    : 1;
  uint8_t not_used_01            : 1;
  uint8_t self_test              : 1;
  uint8_t int_mag                : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_cfg_reg_c_m_t;

#define ISM303DAC_INT_CRTL_REG_M            0x63U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ien                    : 1;
  uint8_t iel                    : 1;
  uint8_t iea                    : 1;
  uint8_t not_used_01            : 2;
  uint8_t zien                   : 1;
  uint8_t yien                   : 1;
  uint8_t xien                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t xien                   : 1;
  uint8_t yien                   : 1;
  uint8_t zien                   : 1;
  uint8_t not_used_01            : 2;
  uint8_t iea                    : 1;
  uint8_t iel                    : 1;
  uint8_t ien                    : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_int_crtl_reg_m_t;

#define ISM303DAC_INT_SOURCE_REG_M          0x64U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t _int                   : 1;
  uint8_t mroi                   : 1;
  uint8_t n_th_s_z               : 1;
  uint8_t n_th_s_y               : 1;
  uint8_t n_th_s_x               : 1;
  uint8_t p_th_s_z               : 1;
  uint8_t p_th_s_y               : 1;
  uint8_t p_th_s_x               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t p_th_s_x               : 1;
  uint8_t p_th_s_y               : 1;
  uint8_t p_th_s_z               : 1;
  uint8_t n_th_s_x               : 1;
  uint8_t n_th_s_y               : 1;
  uint8_t n_th_s_z               : 1;
  uint8_t mroi                   : 1;
  uint8_t _int                   : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_int_source_reg_m_t;

#define ISM303DAC_INT_THS_L_REG_M           0x65U
#define ISM303DAC_INT_THS_H_REG_M           0x66U
#define ISM303DAC_STATUS_REG_M              0x67U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xda                    : 1;
  uint8_t yda                    : 1;
  uint8_t zda                    : 1;
  uint8_t zyxda                  : 1;
  uint8_t _xor                   : 1;
  uint8_t yor                    : 1;
  uint8_t zor                    : 1;
  uint8_t zyxor                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t zyxor                  : 1;
  uint8_t zor                    : 1;
  uint8_t yor                    : 1;
  uint8_t _xor                   : 1;
  uint8_t zyxda                  : 1;
  uint8_t zda                    : 1;
  uint8_t yda                    : 1;
  uint8_t xda                    : 1;
#endif /* DRV_BYTE_ORDER */
} ism303dac_status_reg_m_t;

#define ISM303DAC_OUTX_L_REG_M              0x68U
#define ISM303DAC_OUTX_H_REG_M              0x69U
#define ISM303DAC_OUTY_L_REG_M              0x6AU
#define ISM303DAC_OUTY_H_REG_M              0x6BU
#define ISM303DAC_OUTZ_L_REG_M              0x6CU
#define ISM303DAC_OUTZ_H_REG_M              0x6DU

/**
  * @defgroup ISM303DAC_Register_Union
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
  ism303dac_ctrl1_a_t                      ctrl1_a;
  ism303dac_ctrl2_a_t                      ctrl2_a;
  ism303dac_ctrl3_a_t                      ctrl3_a;
  ism303dac_ctrl4_a_t                      ctrl4_a;
  ism303dac_ctrl5_a_t                      ctrl5_a;
  ism303dac_fifo_ctrl_a_t                  fifo_ctrl_a;
  ism303dac_status_a_t                     status_a;
  ism303dac_fifo_src_a_t                   fifo_src_a;
  ism303dac_fifo_samples_a_t               fifo_samples_a;
  ism303dac_tap_6d_ths_a_t                 tap_6d_ths_a;
  ism303dac_int_dur_a_t                    int_dur_a;
  ism303dac_wake_up_ths_a_t                wake_up_ths_a;
  ism303dac_wake_up_dur_a_t                wake_up_dur_a;
  ism303dac_free_fall_a_t                  free_fall_a;
  ism303dac_status_dup_a_t                 status_dup_a;
  ism303dac_wake_up_src_a_t                wake_up_src_a;
  ism303dac_tap_src_a_t                    tap_src_a;
  ism303dac_6d_src_a_t                     _6d_src_a;
  ism303dac_func_src_a_t                   func_src_a;
  ism303dac_func_ctrl_a_t                  func_ctrl_a;
  ism303dac_cfg_reg_a_m_t                  cfg_reg_a_m;
  ism303dac_cfg_reg_b_m_t                  cfg_reg_b_m;
  ism303dac_cfg_reg_c_m_t                  cfg_reg_c_m;
  ism303dac_int_crtl_reg_m_t               int_crtl_reg_m;
  ism303dac_int_source_reg_m_t             int_source_reg_m;
  ism303dac_status_reg_m_t                 status_reg_m;
  bitwise_t                                bitwise;
  uint8_t                                  byte;
} ism303dac_reg_t;

/**
  * @}
  *
  */

int32_t ism303dac_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
                           uint8_t *data,
                           uint16_t len);
int32_t ism303dac_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                            uint8_t *data,
                            uint16_t len);

float_t ism303dac_from_fs2g_to_mg(int16_t lsb);
float_t ism303dac_from_fs4g_to_mg(int16_t lsb);
float_t ism303dac_from_fs8g_to_mg(int16_t lsb);
float_t ism303dac_from_fs16g_to_mg(int16_t lsb);

float_t ism303dac_from_lsb_to_mG(int16_t lsb);

float_t ism303dac_from_lsb_to_celsius(int16_t lsb);

typedef struct {
  ism303dac_fifo_src_a_t       fifo_src_a;
  ism303dac_status_dup_a_t     status_dup_a;
  ism303dac_wake_up_src_a_t    wake_up_src_a;
  ism303dac_tap_src_a_t        tap_src_a;
  ism303dac_6d_src_a_t         _6d_src_a;
  ism303dac_func_src_a_t       func_src_a;
} ism303dac_xl_all_sources_t;
int32_t ism303dac_xl_all_sources_get(stmdev_ctx_t *ctx,
                                     ism303dac_xl_all_sources_t *val);

int32_t ism303dac_xl_block_data_update_set(stmdev_ctx_t *ctx,
                                           uint8_t val);
int32_t ism303dac_xl_block_data_update_get(stmdev_ctx_t *ctx,
                                           uint8_t *val);

int32_t ism303dac_mg_block_data_update_set(stmdev_ctx_t *ctx,
                                           uint8_t val);
int32_t ism303dac_mg_block_data_update_get(stmdev_ctx_t *ctx,
                                           uint8_t *val);

typedef enum {
  ISM303DAC_MG_LSB_AT_LOW_ADD  = 0,
  ISM303DAC_MG_MSB_AT_LOW_ADD  = 1,
} ism303dac_mg_ble_t;
int32_t ism303dac_mg_data_format_set(stmdev_ctx_t *ctx,
                                     ism303dac_mg_ble_t val);
int32_t ism303dac_mg_data_format_get(stmdev_ctx_t *ctx,
                                     ism303dac_mg_ble_t *val);

typedef enum {
  ISM303DAC_XL_2g  = 0,
  ISM303DAC_XL_16g = 1,
  ISM303DAC_XL_4g  = 2,
  ISM303DAC_XL_8g  = 3,
} ism303dac_xl_fs_t;
int32_t ism303dac_xl_full_scale_set(stmdev_ctx_t *ctx,
                                    ism303dac_xl_fs_t val);
int32_t ism303dac_xl_full_scale_get(stmdev_ctx_t *ctx,
                                    ism303dac_xl_fs_t *val);

typedef enum {
  ISM303DAC_XL_ODR_OFF         = 0x00,
  ISM303DAC_XL_ODR_1Hz_LP      = 0x08,
  ISM303DAC_XL_ODR_12Hz5_LP    = 0x09,
  ISM303DAC_XL_ODR_25Hz_LP     = 0x0A,
  ISM303DAC_XL_ODR_50Hz_LP     = 0x0B,
  ISM303DAC_XL_ODR_100Hz_LP    = 0x0C,
  ISM303DAC_XL_ODR_200Hz_LP    = 0x0D,
  ISM303DAC_XL_ODR_400Hz_LP    = 0x0E,
  ISM303DAC_XL_ODR_800Hz_LP    = 0x0F,
  ISM303DAC_XL_ODR_12Hz5_HR    = 0x01,
  ISM303DAC_XL_ODR_25Hz_HR     = 0x02,
  ISM303DAC_XL_ODR_50Hz_HR     = 0x03,
  ISM303DAC_XL_ODR_100Hz_HR    = 0x04,
  ISM303DAC_XL_ODR_200Hz_HR    = 0x05,
  ISM303DAC_XL_ODR_400Hz_HR    = 0x06,
  ISM303DAC_XL_ODR_800Hz_HR    = 0x07,
  ISM303DAC_XL_ODR_1k6Hz_HF    = 0x15,
  ISM303DAC_XL_ODR_3k2Hz_HF    = 0x16,
  ISM303DAC_XL_ODR_6k4Hz_HF    = 0x17,
} ism303dac_xl_odr_t;
int32_t ism303dac_xl_data_rate_set(stmdev_ctx_t *ctx,
                                   ism303dac_xl_odr_t val);
int32_t ism303dac_xl_data_rate_get(stmdev_ctx_t *ctx,
                                   ism303dac_xl_odr_t *val);

int32_t ism303dac_xl_status_reg_get(stmdev_ctx_t *ctx,
                                    ism303dac_status_a_t *val);

int32_t ism303dac_mg_status_get(stmdev_ctx_t *ctx,
                                ism303dac_status_reg_m_t *val);

int32_t ism303dac_xl_flag_data_ready_get(stmdev_ctx_t *ctx,
                                         uint8_t *val);

int32_t ism303dac_mg_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);
int32_t ism303dac_mg_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ism303dac_mg_user_offset_set(stmdev_ctx_t *ctx,
                                     uint16_t *val);
int32_t ism303dac_mg_user_offset_get(stmdev_ctx_t *ctx,
                                     uint16_t *val);

typedef enum {
  ISM303DAC_MG_CONTINUOUS_MODE  = 0,
  ISM303DAC_MG_SINGLE_TRIGGER   = 1,
  ISM303DAC_MG_POWER_DOWN       = 2,
} ism303dac_mg_md_t;
int32_t ism303dac_mg_operating_mode_set(stmdev_ctx_t *ctx,
                                        ism303dac_mg_md_t val);
int32_t ism303dac_mg_operating_mode_get(stmdev_ctx_t *ctx,
                                        ism303dac_mg_md_t *val);

typedef enum {
  ISM303DAC_MG_ODR_10Hz   = 0,
  ISM303DAC_MG_ODR_20Hz   = 1,
  ISM303DAC_MG_ODR_50Hz   = 2,
  ISM303DAC_MG_ODR_100Hz  = 3,
} ism303dac_mg_odr_t;
int32_t ism303dac_mg_data_rate_set(stmdev_ctx_t *ctx,
                                   ism303dac_mg_odr_t val);
int32_t ism303dac_mg_data_rate_get(stmdev_ctx_t *ctx,
                                   ism303dac_mg_odr_t *val);

typedef enum {
  ISM303DAC_MG_HIGH_RESOLUTION  = 0,
  ISM303DAC_MG_LOW_POWER        = 1,
} ism303dac_mg_lp_t;
int32_t ism303dac_mg_power_mode_set(stmdev_ctx_t *ctx,
                                    ism303dac_mg_lp_t val);
int32_t ism303dac_mg_power_mode_get(stmdev_ctx_t *ctx,
                                    ism303dac_mg_lp_t *val);

int32_t ism303dac_mg_offset_temp_comp_set(stmdev_ctx_t *ctx,
                                          uint8_t val);
int32_t ism303dac_mg_offset_temp_comp_get(stmdev_ctx_t *ctx,
                                          uint8_t *val);

typedef enum {
  ISM303DAC_MG_SET_SENS_ODR_DIV_63        = 0,
  ISM303DAC_MG_SENS_OFF_CANC_EVERY_ODR    = 1,
  ISM303DAC_MG_SET_SENS_ONLY_AT_POWER_ON  = 2,
} ism303dac_mg_set_rst_t;
int32_t ism303dac_mg_set_rst_mode_set(stmdev_ctx_t *ctx,
                                      ism303dac_mg_set_rst_t val);
int32_t ism303dac_mg_set_rst_mode_get(stmdev_ctx_t *ctx,
                                      ism303dac_mg_set_rst_t *val);

int32_t ism303dac_mg_set_rst_sensor_single_set(stmdev_ctx_t *ctx,
                                               uint8_t val);
int32_t ism303dac_mg_set_rst_sensor_single_get(stmdev_ctx_t *ctx,
                                               uint8_t *val);

int32_t ism303dac_acceleration_module_raw_get(stmdev_ctx_t *ctx,
                                              uint8_t *buff);

int32_t ism303dac_magnetic_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t ism303dac_xl_temperature_raw_get(stmdev_ctx_t *ctx,
                                         uint8_t *buff);

int32_t ism303dac_acceleration_raw_get(stmdev_ctx_t *ctx,
                                       int16_t *val);

int32_t ism303dac_xl_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t ism303dac_mg_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t ism303dac_xl_auto_increment_set(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t ism303dac_xl_auto_increment_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t ism303dac_xl_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_xl_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ism303dac_mg_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_mg_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ism303dac_xl_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_xl_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ism303dac_mg_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_mg_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  ISM303DAC_XL_ST_DISABLE     = 0,
  ISM303DAC_XL_ST_POSITIVE    = 1,
  ISM303DAC_XL_ST_NEGATIVE    = 2,
} ism303dac_xl_st_t;
int32_t ism303dac_xl_self_test_set(stmdev_ctx_t *ctx,
                                   ism303dac_xl_st_t val);
int32_t ism303dac_xl_self_test_get(stmdev_ctx_t *ctx,
                                   ism303dac_xl_st_t *val);

int32_t ism303dac_mg_self_test_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_mg_self_test_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  ISM303DAC_XL_DRDY_LATCHED   = 0,
  ISM303DAC_XL_DRDY_PULSED    = 1,
} ism303dac_xl_drdy_pulsed_t;
int32_t ism303dac_xl_data_ready_mode_set(stmdev_ctx_t *ctx,
                                         ism303dac_xl_drdy_pulsed_t val);
int32_t ism303dac_xl_data_ready_mode_get(stmdev_ctx_t *ctx,
                                         ism303dac_xl_drdy_pulsed_t *val);

typedef enum {
  ISM303DAC_XL_HP_INTERNAL_ONLY  = 0,
  ISM303DAC_XL_HP_ON_OUTPUTS     = 1,
} ism303dac_xl_fds_slope_t;
int32_t ism303dac_xl_hp_path_set(stmdev_ctx_t *ctx,
                                 ism303dac_xl_fds_slope_t val);
int32_t ism303dac_xl_hp_path_get(stmdev_ctx_t *ctx,
                                 ism303dac_xl_fds_slope_t *val);

typedef enum {
  ISM303DAC_MG_ODR_DIV_2  = 0,
  ISM303DAC_MG_ODR_DIV_4  = 1,
} ism303dac_mg_lpf_t;
int32_t ism303dac_mg_low_pass_bandwidth_set(stmdev_ctx_t *ctx,
                                            ism303dac_mg_lpf_t val);
int32_t ism303dac_mg_low_pass_bandwidth_get(stmdev_ctx_t *ctx,
                                            ism303dac_mg_lpf_t *val);

typedef enum {
  ISM303DAC_XL_SPI_4_WIRE   = 0,
  ISM303DAC_XL_SPI_3_WIRE   = 1,
} ism303dac_xl_sim_t;
int32_t ism303dac_xl_spi_mode_set(stmdev_ctx_t *ctx,
                                  ism303dac_xl_sim_t val);
int32_t ism303dac_xl_spi_mode_get(stmdev_ctx_t *ctx,
                                  ism303dac_xl_sim_t *val);

typedef enum {
  ISM303DAC_XL_I2C_ENABLE   = 0,
  ISM303DAC_XL_I2C_DISABLE  = 1,
} ism303dac_xl_i2c_disable_t;
int32_t ism303dac_xl_i2c_interface_set(stmdev_ctx_t *ctx,
                                       ism303dac_xl_i2c_disable_t val);
int32_t ism303dac_xl_i2c_interface_get(stmdev_ctx_t *ctx,
                                       ism303dac_xl_i2c_disable_t *val);

typedef enum {
  ISM303DAC_MG_I2C_ENABLE   = 0,
  ISM303DAC_MG_I2C_DISABLE  = 1,
} ism303dac_mg_i2c_dis_t;
int32_t ism303dac_mg_i2c_interface_set(stmdev_ctx_t *ctx,
                                       ism303dac_mg_i2c_dis_t val);
int32_t ism303dac_mg_i2c_interface_get(stmdev_ctx_t *ctx,
                                       ism303dac_mg_i2c_dis_t *val);

typedef enum {
  ISM303DAC_XL_PULL_UP_CONNECTED     = 0,
  ISM303DAC_XL_PULL_UP_DISCONNECTED  = 1,
} ism303dac_xl_if_cs_pu_dis_t;
int32_t ism303dac_xl_cs_mode_set(stmdev_ctx_t *ctx,
                                 ism303dac_xl_if_cs_pu_dis_t val);
int32_t ism303dac_xl_cs_mode_get(stmdev_ctx_t *ctx,
                                 ism303dac_xl_if_cs_pu_dis_t *val);

typedef enum {
  ISM303DAC_XL_PUSH_PULL   = 0,
  ISM303DAC_XL_OPEN_DRAIN  = 1,
} ism303dac_xl_pp_od_t;
int32_t ism303dac_xl_pin_mode_set(stmdev_ctx_t *ctx,
                                  ism303dac_xl_pp_od_t val);
int32_t ism303dac_xl_pin_mode_get(stmdev_ctx_t *ctx,
                                  ism303dac_xl_pp_od_t *val);

typedef enum {
  ISM303DAC_XL_ACTIVE_HIGH  = 0,
  ISM303DAC_XL_ACTIVE_LOW   = 1,
} ism303dac_xl_h_lactive_t;
int32_t ism303dac_xl_pin_polarity_set(stmdev_ctx_t *ctx,
                                      ism303dac_xl_h_lactive_t val);
int32_t ism303dac_xl_pin_polarity_get(stmdev_ctx_t *ctx,
                                      ism303dac_xl_h_lactive_t *val);

typedef enum {
  ISM303DAC_XL_INT_PULSED   = 0,
  ISM303DAC_XL_INT_LATCHED  = 1,
} ism303dac_xl_lir_t;
int32_t ism303dac_xl_int_notification_set(stmdev_ctx_t *ctx,
                                          ism303dac_xl_lir_t val);
int32_t ism303dac_xl_int_notification_get(stmdev_ctx_t *ctx,
                                          ism303dac_xl_lir_t *val);

typedef struct {
  uint8_t int1_drdy               : 1;
  uint8_t int1_fth                : 1;
  uint8_t int1_6d                 : 1;
  uint8_t int1_tap                : 1;
  uint8_t int1_ff                 : 1;
  uint8_t int1_wu                 : 1;
  uint8_t int1_s_tap              : 1;
  uint8_t int1_fss7               : 1;
} ism303dac_xl_pin_int1_route_t;
int32_t ism303dac_xl_pin_int1_route_set(stmdev_ctx_t *ctx,
                                        ism303dac_xl_pin_int1_route_t val);
int32_t ism303dac_xl_pin_int1_route_get(stmdev_ctx_t *ctx,
                                        ism303dac_xl_pin_int1_route_t *val);

typedef struct {
  uint8_t int2_boot               : 1;
  uint8_t int2_fth                : 1;
  uint8_t int2_drdy               : 1;
} ism303dac_xl_pin_int2_route_t;
int32_t ism303dac_xl_pin_int2_route_set(stmdev_ctx_t *ctx,
                                        ism303dac_xl_pin_int2_route_t val);
int32_t ism303dac_xl_pin_int2_route_get(stmdev_ctx_t *ctx,
                                        ism303dac_xl_pin_int2_route_t *val);

int32_t ism303dac_xl_all_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_xl_all_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ism303dac_mg_drdy_on_pin_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_mg_drdy_on_pin_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ism303dac_mg_int_on_pin_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_mg_int_on_pin_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ism303dac_mg_int_gen_conf_set(stmdev_ctx_t *ctx,
                                      ism303dac_int_crtl_reg_m_t *val);
int32_t ism303dac_mg_int_gen_conf_get(stmdev_ctx_t *ctx,
                                      ism303dac_int_crtl_reg_m_t *val);

int32_t ism303dac_mg_int_gen_source_get(stmdev_ctx_t *ctx,
                                        ism303dac_int_source_reg_m_t *val);

int32_t ism303dac_mg_int_gen_treshold_set(stmdev_ctx_t *ctx,
                                          uint16_t val);
int32_t ism303dac_mg_int_gen_treshold_get(stmdev_ctx_t *ctx,
                                          uint16_t *val);

typedef enum {
  ISM303DAC_MG_CHECK_BEFORE  = 0,
  ISM303DAC_MG_CHECK_AFTER   = 1,
} ism303dac_mg_int_on_dataoff_t;
int32_t ism303dac_mg_offset_int_conf_set(stmdev_ctx_t *ctx,
                                         ism303dac_mg_int_on_dataoff_t val);
int32_t ism303dac_mg_offset_int_conf_get(stmdev_ctx_t *ctx,
                                         ism303dac_mg_int_on_dataoff_t *val);

int32_t ism303dac_xl_wkup_threshold_set(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t ism303dac_xl_wkup_threshold_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t ism303dac_xl_wkup_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_xl_wkup_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ism303dac_xl_sleep_mode_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_xl_sleep_mode_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ism303dac_xl_act_sleep_dur_set(stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t ism303dac_xl_act_sleep_dur_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

int32_t ism303dac_xl_tap_detection_on_z_set(stmdev_ctx_t *ctx,
                                            uint8_t val);
int32_t ism303dac_xl_tap_detection_on_z_get(stmdev_ctx_t *ctx,
                                            uint8_t *val);

int32_t ism303dac_xl_tap_detection_on_y_set(stmdev_ctx_t *ctx,
                                            uint8_t val);
int32_t ism303dac_xl_tap_detection_on_y_get(stmdev_ctx_t *ctx,
                                            uint8_t *val);

int32_t ism303dac_xl_tap_detection_on_x_set(stmdev_ctx_t *ctx,
                                            uint8_t val);
int32_t ism303dac_xl_tap_detection_on_x_get(stmdev_ctx_t *ctx,
                                            uint8_t *val);

int32_t ism303dac_xl_tap_threshold_set(stmdev_ctx_t *ctx,
                                       uint8_t val);
int32_t ism303dac_xl_tap_threshold_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

int32_t ism303dac_xl_tap_shock_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_xl_tap_shock_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ism303dac_xl_tap_quiet_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_xl_tap_quiet_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ism303dac_xl_tap_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_xl_tap_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  ISM303DAC_XL_ONLY_SINGLE  = 0,
  ISM303DAC_XL_ONLY_DOUBLE  = 1,
} ism303dac_xl_single_double_tap_t;
int32_t ism303dac_xl_tap_mode_set(stmdev_ctx_t *ctx,
                                  ism303dac_xl_single_double_tap_t val);
int32_t ism303dac_xl_tap_mode_get(stmdev_ctx_t *ctx,
                                  ism303dac_xl_single_double_tap_t *val);

int32_t ism303dac_xl_tap_src_get(stmdev_ctx_t *ctx,
                                 ism303dac_tap_src_a_t *val);

typedef enum {
  ISM303DAC_XL_DEG_80   = 0,
  ISM303DAC_XL_DEG_70   = 1,
  ISM303DAC_XL_DEG_60   = 2,
  ISM303DAC_XL_DEG_50   = 3,
} ism303dac_xl_6d_ths_t;
int32_t ism303dac_xl_6d_threshold_set(stmdev_ctx_t *ctx,
                                      ism303dac_xl_6d_ths_t val);
int32_t ism303dac_xl_6d_threshold_get(stmdev_ctx_t *ctx,
                                      ism303dac_xl_6d_ths_t *val);

int32_t ism303dac_xl_4d_mode_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_xl_4d_mode_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ism303dac_xl_6d_src_get(stmdev_ctx_t *ctx,
                                ism303dac_6d_src_a_t *val);

int32_t ism303dac_xl_ff_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_xl_ff_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t ism303dac_xl_ff_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_xl_ff_threshold_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t ism303dac_xl_fifo_xl_module_batch_set(stmdev_ctx_t *ctx,
                                              uint8_t val);
int32_t ism303dac_xl_fifo_xl_module_batch_get(stmdev_ctx_t *ctx,
                                              uint8_t *val);

typedef enum {
  ISM303DAC_XL_BYPASS_MODE            = 0,
  ISM303DAC_XL_FIFO_MODE              = 1,
  ISM303DAC_XL_STREAM_TO_FIFO_MODE    = 3,
  ISM303DAC_XL_BYPASS_TO_STREAM_MODE  = 4,
  ISM303DAC_XL_STREAM_MODE            = 6,
} ism303dac_xl_fmode_t;
int32_t ism303dac_xl_fifo_mode_set(stmdev_ctx_t *ctx,
                                   ism303dac_xl_fmode_t val);
int32_t ism303dac_xl_fifo_mode_get(stmdev_ctx_t *ctx,
                                   ism303dac_xl_fmode_t *val);

int32_t ism303dac_xl_fifo_watermark_set(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t ism303dac_xl_fifo_watermark_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t ism303dac_xl_fifo_full_flag_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t ism303dac_xl_fifo_ovr_flag_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

int32_t ism303dac_xl_fifo_wtm_flag_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

int32_t ism303dac_xl_fifo_data_level_get(stmdev_ctx_t *ctx,
                                         uint16_t *val);

int32_t ism303dac_xl_fifo_src_get(stmdev_ctx_t *ctx,
                                  ism303dac_fifo_src_a_t *val);

int32_t ism303dac_xl_module_sens_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t ism303dac_xl_module_sens_get(stmdev_ctx_t *ctx, uint8_t *val);

/**
  * @}
  *
  */

/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* ISM303DAC_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
