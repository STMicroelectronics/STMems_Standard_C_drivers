/*
 ******************************************************************************
 * @file    lsm9ds1_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          lsm9ds1_reg.c driver.
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
#ifndef LSM9DS1_REGS_H
#define LSM9DS1_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup LSM9DS1
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

/** @defgroup LSM9DS1_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> 0xD5 if SA0=1 -> 0xD7 **/
#define LSM9DS1_IMU_I2C_ADD_L      0xD5U
#define LSM9DS1_IMU_I2C_ADD_H      0xD7U

/** I2C Device Address 8 bit format  if SA0=0 -> 0x3D if SA0=1 -> 0x39 **/
#define LSM9DS1_MAG_I2C_ADD_L      0x39U
#define LSM9DS1_MAG_I2C_ADD_H      0x3DU

/** Device Identification (Who am I) **/
#define LSM9DS1_IMU_ID             0x68U

/** Device Identification (Who am I) **/
#define LSM9DS1_MAG_ID             0x3DU

/**
  * @}
  *
  */

#define LSM9DS1_ACT_THS            0x04U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t act_ths                  : 7;
  uint8_t sleep_on_inact_en        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sleep_on_inact_en        : 1;
  uint8_t act_ths                  : 7;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_act_ths_t;

#define LSM9DS1_ACT_DUR            0x05U
#define LSM9DS1_INT_GEN_CFG_XL     0x06U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlie_xl                  : 1;
  uint8_t xhie_xl                  : 1;
  uint8_t ylie_xl                  : 1;
  uint8_t yhie_xl                  : 1;
  uint8_t zlie_xl                  : 1;
  uint8_t zhie_xl                  : 1;
  uint8_t _6d                      : 1;
  uint8_t aoi_xl                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t aoi_xl                   : 1;
  uint8_t _6d                      : 1;
  uint8_t zhie_xl                  : 1;
  uint8_t zlie_xl                  : 1;
  uint8_t yhie_xl                  : 1;
  uint8_t ylie_xl                  : 1;
  uint8_t xhie_xl                  : 1;
  uint8_t xlie_xl                  : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int_gen_cfg_xl_t;

#define LSM9DS1_INT_GEN_THS_X_XL   0x07U
#define LSM9DS1_INT_GEN_THS_Y_XL   0x08U
#define LSM9DS1_INT_GEN_THS_Z_XL   0x09U
#define LSM9DS1_INT_GEN_DUR_XL     0x0AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t dur_xl                   : 7;
  uint8_t wait_xl                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t wait_xl                  : 1;
  uint8_t dur_xl                   : 7;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int_gen_dur_xl_t;

#define LSM9DS1_REFERENCE_G        0x0BU
#define LSM9DS1_INT1_CTRL          0x0CU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_drdy_xl             : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fth                 : 1;
  uint8_t int1_ovr                 : 1;
  uint8_t int1_fss5                : 1;
  uint8_t int1_ig_xl               : 1;
  uint8_t int1_ig_g                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_ig_g                : 1;
  uint8_t int1_ig_xl               : 1;
  uint8_t int1_fss5                : 1;
  uint8_t int1_ovr                 : 1;
  uint8_t int1_fth                 : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_drdy_xl             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int1_ctrl_t;

#define LSM9DS1_INT2_CTRL          0x0DU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fth                 : 1;
  uint8_t int2_ovr                 : 1;
  uint8_t int2_fss5                : 1;
  uint8_t not_used_01              : 1;
  uint8_t int2_inact               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_inact               : 1;
  uint8_t not_used_01              : 1;
  uint8_t int2_fss5                : 1;
  uint8_t int2_ovr                 : 1;
  uint8_t int2_fth                 : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_xl             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int2_ctrl_t;

#define LSM9DS1_WHO_AM_I           0x0FU
#define LSM9DS1_CTRL_REG1_G        0x10U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bw_g                     : 2;
  uint8_t not_used_01              : 1;
  uint8_t fs_g                     : 2;
  uint8_t odr_g                    : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_g                    : 3;
  uint8_t fs_g                     : 2;
  uint8_t not_used_01              : 1;
  uint8_t bw_g                     : 2;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg1_g_t;

#define LSM9DS1_CTRL_REG2_G        0x11U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t out_sel                  : 2;
  uint8_t int_sel                  : 2;
  uint8_t not_used_01              : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 4;
  uint8_t int_sel                  : 2;
  uint8_t out_sel                  : 2;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg2_g_t;

#define LSM9DS1_CTRL_REG3_G        0x12U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t hpcf_g                   : 4;
  uint8_t not_used_01              : 2;
  uint8_t hp_en                    : 1;
  uint8_t lp_mode                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t lp_mode                  : 1;
  uint8_t hp_en                    : 1;
  uint8_t not_used_01              : 2;
  uint8_t hpcf_g                   : 4;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg3_g_t;

#define LSM9DS1_ORIENT_CFG_G       0x13U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t orient                   : 3;
  uint8_t signz_g                  : 1;
  uint8_t signy_g                  : 1;
  uint8_t signx_g                  : 1;
  uint8_t not_used_01              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t signx_g                  : 1;
  uint8_t signy_g                  : 1;
  uint8_t signz_g                  : 1;
  uint8_t orient                   : 3;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_orient_cfg_g_t;

#define LSM9DS1_INT_GEN_SRC_G      0x14U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl_g                     : 1;
  uint8_t xh_g                     : 1;
  uint8_t yl_g                     : 1;
  uint8_t yh_g                     : 1;
  uint8_t zl_g                     : 1;
  uint8_t zh_g                     : 1;
  uint8_t ia_g                     : 1;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t ia_g                     : 1;
  uint8_t zh_g                     : 1;
  uint8_t zl_g                     : 1;
  uint8_t yh_g                     : 1;
  uint8_t yl_g                     : 1;
  uint8_t xh_g                     : 1;
  uint8_t xl_g                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int_gen_src_g_t;

#define LSM9DS1_OUT_TEMP_L         0x15U
#define LSM9DS1_OUT_TEMP_H         0x16U
#define LSM9DS1_STATUS_REG         0x17U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                     : 1;
  uint8_t gda                      : 1;
  uint8_t tda                      : 1;
  uint8_t boot_status              : 1;
  uint8_t inact                    : 1;
  uint8_t ig_g                     : 1;
  uint8_t ig_xl                    : 1;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t ig_xl                    : 1;
  uint8_t ig_g                     : 1;
  uint8_t inact                    : 1;
  uint8_t boot_status              : 1;
  uint8_t tda                      : 1;
  uint8_t gda                      : 1;
  uint8_t xlda                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_status_reg_t;

#define LSM9DS1_OUT_X_L_G          0x18U
#define LSM9DS1_OUT_X_H_G          0x19U
#define LSM9DS1_OUT_Y_L_G          0x1AU
#define LSM9DS1_OUT_Y_H_G          0x1BU
#define LSM9DS1_OUT_Z_L_G          0x1CU
#define LSM9DS1_OUT_Z_H_G          0x1DU
#define LSM9DS1_CTRL_REG4          0x1EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t _4d_xl1                  : 1;
  uint8_t lir_xl1                  : 1;
  uint8_t not_used_01              : 1;
  uint8_t xen_g                    : 1;
  uint8_t yen_g                    : 1;
  uint8_t zen_g                    : 1;
  uint8_t not_used_02              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 2;
  uint8_t zen_g                    : 1;
  uint8_t yen_g                    : 1;
  uint8_t xen_g                    : 1;
  uint8_t not_used_01              : 1;
  uint8_t lir_xl1                  : 1;
  uint8_t _4d_xl1                  : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg4_t;

#define LSM9DS1_CTRL_REG5_XL       0x1FU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 3;
  uint8_t xen_xl                   : 1;
  uint8_t yen_xl                   : 1;
  uint8_t zen_xl                   : 1;
  uint8_t dec                      : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t dec                      : 2;
  uint8_t zen_xl                   : 1;
  uint8_t yen_xl                   : 1;
  uint8_t xen_xl                   : 1;
  uint8_t not_used_01              : 3;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg5_xl_t;

#define LSM9DS1_CTRL_REG6_XL       0x20U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bw_xl                    : 2;
  uint8_t bw_scal_odr              : 1;
  uint8_t fs_xl                    : 2;
  uint8_t odr_xl                   : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_xl                   : 3;
  uint8_t fs_xl                    : 2;
  uint8_t bw_scal_odr              : 1;
  uint8_t bw_xl                    : 2;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg6_xl_t;

#define LSM9DS1_CTRL_REG7_XL       0x21U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t hpis1                    : 1;
  uint8_t not_used_01              : 1;
  uint8_t fds                      : 1;
  uint8_t not_used_02              : 2;
  uint8_t dcf                      : 2;
  uint8_t hr                       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t hr                       : 1;
  uint8_t dcf                      : 2;
  uint8_t not_used_02              : 2;
  uint8_t fds                      : 1;
  uint8_t not_used_01              : 1;
  uint8_t hpis1                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg7_xl_t;

#define LSM9DS1_CTRL_REG8          0x22U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sw_reset                 : 1;
  uint8_t ble                      : 1;
  uint8_t if_add_inc               : 1;
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
  uint8_t if_add_inc               : 1;
  uint8_t ble                      : 1;
  uint8_t sw_reset                 : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg8_t;

#define LSM9DS1_CTRL_REG9          0x23U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t stop_on_fth              : 1;
  uint8_t fifo_en                  : 1;
  uint8_t i2c_disable              : 1;
  uint8_t drdy_mask_bit            : 1;
  uint8_t fifo_temp_en             : 1;
  uint8_t not_used_01              : 1;
  uint8_t sleep_g                  : 1;
  uint8_t not_used_02              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 1;
  uint8_t sleep_g                  : 1;
  uint8_t not_used_01              : 1;
  uint8_t fifo_temp_en             : 1;
  uint8_t drdy_mask_bit            : 1;
  uint8_t i2c_disable              : 1;
  uint8_t fifo_en                  : 1;
  uint8_t stop_on_fth              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg9_t;

#define LSM9DS1_CTRL_REG10         0x24U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st_xl                    : 1;
  uint8_t not_used_01              : 1;
  uint8_t st_g                     : 1;
  uint8_t not_used_02              : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 5;
  uint8_t st_g                     : 1;
  uint8_t not_used_01              : 1;
  uint8_t st_xl                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg10_t;

#define LSM9DS1_INT_GEN_SRC_XL     0x26U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl_xl                    : 1;
  uint8_t xh_xl                    : 1;
  uint8_t yl_xl                    : 1;
  uint8_t yh_xl                    : 1;
  uint8_t zl_xl                    : 1;
  uint8_t zh_xl                    : 1;
  uint8_t ia_xl                    : 1;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t ia_xl                    : 1;
  uint8_t zh_xl                    : 1;
  uint8_t zl_xl                    : 1;
  uint8_t yh_xl                    : 1;
  uint8_t yl_xl                    : 1;
  uint8_t xh_xl                    : 1;
  uint8_t xl_xl                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int_gen_src_xl_t;

#define LSM9DS1_OUT_X_L_XL         0x28U
#define LSM9DS1_OUT_X_H_XL         0x29U
#define LSM9DS1_OUT_Y_L_XL         0x2AU
#define LSM9DS1_OUT_Y_H_XL         0x2BU
#define LSM9DS1_OUT_Z_L_XL         0x2CU
#define LSM9DS1_OUT_Z_H_XL         0x2DU
#define LSM9DS1_FIFO_CTRL          0x2EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fth                      : 5;
  uint8_t fmode                    : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fmode                    : 3;
  uint8_t fth                      : 5;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_fifo_ctrl_t;

#define LSM9DS1_FIFO_SRC           0x2FU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fss                      : 6;
  uint8_t ovrn                     : 1;
  uint8_t fth                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fth                      : 1;
  uint8_t ovrn                     : 1;
  uint8_t fss                      : 6;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_fifo_src_t;

#define LSM9DS1_INT_GEN_CFG_G      0x30U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlie_g                   : 1;
  uint8_t xhie_g                   : 1;
  uint8_t ylie_g                   : 1;
  uint8_t yhie_g                   : 1;
  uint8_t zlie_g                   : 1;
  uint8_t zhie_g                   : 1;
  uint8_t lir_g                    : 1;
  uint8_t aoi_g                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t aoi_g                    : 1;
  uint8_t lir_g                    : 1;
  uint8_t zhie_g                   : 1;
  uint8_t zlie_g                   : 1;
  uint8_t yhie_g                   : 1;
  uint8_t ylie_g                   : 1;
  uint8_t xhie_g                   : 1;
  uint8_t xlie_g                   : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int_gen_cfg_g_t;

#define LSM9DS1_INT_GEN_THS_XH_G   0x31U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ths_g_x                  : 7;
  uint8_t dcrm_g                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t dcrm_g                   : 1;
  uint8_t ths_g_x                  : 7;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int_gen_ths_xh_g_t;

#define LSM9DS1_INT_GEN_THS_XL_G   0x32U
typedef struct {
  uint8_t ths_g_x                  : 8;
} lsm9ds1_int_gen_ths_xl_g_t;
#define LSM9DS1_INT_GEN_THS_YH_G   0x33U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ths_g_y                  : 7;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t ths_g_y                  : 7;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int_gen_ths_yh_g_t;

#define LSM9DS1_INT_GEN_THS_YL_G   0x34U
typedef struct {
  uint8_t ths_g_y                  : 8;
} lsm9ds1_int_gen_ths_yl_g_t;
#define LSM9DS1_INT_GEN_THS_ZH_G   0x35U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ths_g_z                  : 7;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t ths_g_z                  : 7;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int_gen_ths_zh_g_t;

#define LSM9DS1_INT_GEN_THS_ZL_G   0x36U
typedef struct {
  uint8_t ths_g_z                  : 8;
} lsm9ds1_int_gen_ths_zl_g_t;
#define LSM9DS1_INT_GEN_DUR_G      0x37U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t dur_g                    : 7;
  uint8_t wait_g                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t wait_g                   : 1;
  uint8_t dur_g                    : 7;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int_gen_dur_g_t;

#define LSM9DS1_OFFSET_X_REG_L_M   0x05U
#define LSM9DS1_OFFSET_X_REG_H_M   0x06U
#define LSM9DS1_OFFSET_Y_REG_L_M   0x07U
#define LSM9DS1_OFFSET_Y_REG_H_M   0x08U
#define LSM9DS1_OFFSET_Z_REG_L_M   0x09U
#define LSM9DS1_OFFSET_Z_REG_H_M   0x0AU

#define LSM9DS1_WHO_AM_I_M         0x0FU
#define LSM9DS1_CTRL_REG1_M        0x20U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st                       : 1;
  uint8_t fast_odr                 : 1;
  uint8_t _do                      : 3;
  uint8_t om                       : 2;
  uint8_t temp_comp                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t temp_comp                : 1;
  uint8_t om                       : 2;
  uint8_t _do                      : 3;
  uint8_t fast_odr                 : 1;
  uint8_t st                       : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg1_m_t;

#define LSM9DS1_CTRL_REG2_M        0x21U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 2;
  uint8_t soft_rst                 : 1;
  uint8_t reboot                   : 1;
  uint8_t not_used_02              : 1;
  uint8_t fs                       : 2;
  uint8_t not_used_03              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_03              : 1;
  uint8_t fs                       : 2;
  uint8_t not_used_02              : 1;
  uint8_t reboot                   : 1;
  uint8_t soft_rst                 : 1;
  uint8_t not_used_01              : 2;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg2_m_t;

#define LSM9DS1_CTRL_REG3_M        0x22U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t md                       : 2;
  uint8_t sim                      : 1;
  uint8_t not_used_01              : 2;
  uint8_t lp                       : 1;
  uint8_t not_used_02              : 1;
  uint8_t i2c_disable              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t i2c_disable              : 1;
  uint8_t not_used_02              : 1;
  uint8_t lp                       : 1;
  uint8_t not_used_01              : 2;
  uint8_t sim                      : 1;
  uint8_t md                       : 2;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg3_m_t;

#define LSM9DS1_CTRL_REG4_M        0x23U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t ble                      : 1;
  uint8_t omz                      : 2;
  uint8_t not_used_02              : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_02              : 4;
  uint8_t omz                      : 2;
  uint8_t ble                      : 1;
  uint8_t not_used_01              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg4_m_t;

#define LSM9DS1_CTRL_REG5_M        0x24U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01              : 6;
  uint8_t bdu                      : 1;
  uint8_t fast_read                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fast_read                : 1;
  uint8_t bdu                      : 1;
  uint8_t not_used_01              : 6;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_ctrl_reg5_m_t;

#define LSM9DS1_STATUS_REG_M       0x27U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xda                      : 1;
  uint8_t yda                      : 1;
  uint8_t zda                      : 1;
  uint8_t zyxda                    : 1;
  uint8_t _xor                     : 1;
  uint8_t yor                      : 1;
  uint8_t zor                      : 1;
  uint8_t zyxor                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t zyxor                    : 1;
  uint8_t zor                      : 1;
  uint8_t yor                      : 1;
  uint8_t _xor                     : 1;
  uint8_t zyxda                    : 1;
  uint8_t zda                      : 1;
  uint8_t yda                      : 1;
  uint8_t xda                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_status_reg_m_t;

#define LSM9DS1_OUT_X_L_M          0x28U
#define LSM9DS1_OUT_X_H_M          0x29U
#define LSM9DS1_OUT_Y_L_M          0x2AU
#define LSM9DS1_OUT_Y_H_M          0x2BU
#define LSM9DS1_OUT_Z_L_M          0x2CU
#define LSM9DS1_OUT_Z_H_M          0x2DU
#define LSM9DS1_INT_CFG_M          0x30U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ien                      : 1;
  uint8_t iel                      : 1;
  uint8_t iea                      : 1;
  uint8_t not_used_01              : 2;
  uint8_t zien                     : 1;
  uint8_t yien                     : 1;
  uint8_t xien                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t xien                     : 1;
  uint8_t yien                     : 1;
  uint8_t zien                     : 1;
  uint8_t not_used_01              : 2;
  uint8_t iea                      : 1;
  uint8_t iel                      : 1;
  uint8_t ien                      : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int_cfg_m_t;

#define LSM9DS1_INT_SRC_M          0x31U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t _int                     : 1;
  uint8_t mroi                     : 1;
  uint8_t nth_z                    : 1;
  uint8_t nth_y                    : 1;
  uint8_t nth_x                    : 1;
  uint8_t pth_z                    : 1;
  uint8_t pth_y                    : 1;
  uint8_t pth_x                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t pth_x                    : 1;
  uint8_t pth_y                    : 1;
  uint8_t pth_z                    : 1;
  uint8_t nth_x                    : 1;
  uint8_t nth_y                    : 1;
  uint8_t nth_z                    : 1;
  uint8_t mroi                     : 1;
  uint8_t _int                     : 1;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int_src_m_t;

#define LSM9DS1_INT_THS_L_M        0x32U
typedef struct {
  uint8_t ths                      : 8;
} lsm9ds1_int_ths_l_m_t;

#define LSM9DS1_INT_THS_H_M        0x33U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ths                      : 7;
  uint8_t not_used_01              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01              : 1;
  uint8_t ths                      : 7;
#endif /* DRV_BYTE_ORDER */
} lsm9ds1_int_ths_h_m_t;

/**
  * @defgroup LSM9DS1_Register_Union
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
  lsm9ds1_act_ths_t                act_ths;
  lsm9ds1_int_gen_cfg_xl_t         int_gen_cfg_xl;
  lsm9ds1_int_gen_dur_xl_t         int_gen_dur_xl;
  lsm9ds1_int1_ctrl_t              int1_ctrl;
  lsm9ds1_int2_ctrl_t              int2_ctrl;
  lsm9ds1_ctrl_reg1_g_t            ctrl_reg1_g;
  lsm9ds1_ctrl_reg2_g_t            ctrl_reg2_g;
  lsm9ds1_ctrl_reg3_g_t            ctrl_reg3_g;
  lsm9ds1_orient_cfg_g_t           orient_cfg_g;
  lsm9ds1_int_gen_src_g_t          int_gen_src_g;
  lsm9ds1_status_reg_t             status_reg;
  lsm9ds1_ctrl_reg4_t              ctrl_reg4;
  lsm9ds1_ctrl_reg5_xl_t           ctrl_reg5_xl;
  lsm9ds1_ctrl_reg6_xl_t           ctrl_reg6_xl;
  lsm9ds1_ctrl_reg7_xl_t           ctrl_reg7_xl;
  lsm9ds1_ctrl_reg8_t              ctrl_reg8;
  lsm9ds1_ctrl_reg9_t              ctrl_reg9;
  lsm9ds1_ctrl_reg10_t             ctrl_reg10;
  lsm9ds1_int_gen_src_xl_t         int_gen_src_xl;
  lsm9ds1_fifo_ctrl_t              fifo_ctrl;
  lsm9ds1_fifo_src_t               fifo_src;
  lsm9ds1_int_gen_cfg_g_t          int_gen_cfg_g;
  lsm9ds1_int_gen_ths_xh_g_t       int_gen_ths_xh_g;
  lsm9ds1_int_gen_ths_xl_g_t       int_gen_ths_xl_g;
  lsm9ds1_int_gen_ths_yh_g_t       int_gen_ths_yh_g;
  lsm9ds1_int_gen_ths_yl_g_t       int_gen_ths_yl_g;
  lsm9ds1_int_gen_ths_zh_g_t       int_gen_ths_zh_g;
  lsm9ds1_int_gen_ths_zl_g_t       int_gen_ths_zl_g;
  lsm9ds1_int_gen_dur_g_t          int_gen_dur_g;
  lsm9ds1_ctrl_reg1_m_t            ctrl_reg1_m;
  lsm9ds1_ctrl_reg2_m_t            ctrl_reg2_m;
  lsm9ds1_ctrl_reg3_m_t            ctrl_reg3_m;
  lsm9ds1_ctrl_reg4_m_t            ctrl_reg4_m;
  lsm9ds1_ctrl_reg5_m_t            ctrl_reg5_m;
  lsm9ds1_status_reg_m_t           status_reg_m;
  lsm9ds1_int_cfg_m_t              int_cfg_m;
  lsm9ds1_int_src_m_t              int_src_m;
  lsm9ds1_int_ths_l_m_t            int_ths_l_m;
  lsm9ds1_int_ths_h_m_t            int_ths_h_m;
  bitwise_t                        bitwise;
  uint8_t                          byte;
} lsm9ds1_reg_t;

/**
  * @}
  *
  */

int32_t lsm9ds1_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
                         uint8_t *data,
                         uint16_t len);
int32_t lsm9ds1_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                          uint8_t *data,
                          uint16_t len);

float_t lsm9ds1_from_fs2g_to_mg(int16_t lsb);
float_t lsm9ds1_from_fs4g_to_mg(int16_t lsb);
float_t lsm9ds1_from_fs8g_to_mg(int16_t lsb);
float_t lsm9ds1_from_fs16g_to_mg(int16_t lsb);

float_t lsm9ds1_from_fs245dps_to_mdps(int16_t lsb);
float_t lsm9ds1_from_fs500dps_to_mdps(int16_t lsb);
float_t lsm9ds1_from_fs2000dps_to_mdps(int16_t lsb);

float_t lsm9ds1_from_fs4gauss_to_mG(int16_t lsb);
float_t lsm9ds1_from_fs8gauss_to_mG(int16_t lsb);
float_t lsm9ds1_from_fs12gauss_to_mG(int16_t lsb);
float_t lsm9ds1_from_fs16gauss_to_mG(int16_t lsb);

float_t lsm9ds1_from_lsb_to_celsius(int16_t lsb);

typedef enum {
  LSM9DS1_245dps = 0,
  LSM9DS1_500dps = 1,
  LSM9DS1_2000dps = 3,
} lsm9ds1_gy_fs_t;
int32_t lsm9ds1_gy_full_scale_set(stmdev_ctx_t *ctx,
                                  lsm9ds1_gy_fs_t val);
int32_t lsm9ds1_gy_full_scale_get(stmdev_ctx_t *ctx,
                                  lsm9ds1_gy_fs_t *val);

typedef enum {
  LSM9DS1_IMU_OFF              = 0x00,
  LSM9DS1_GY_OFF_XL_10Hz       = 0x10,
  LSM9DS1_GY_OFF_XL_50Hz       = 0x20,
  LSM9DS1_GY_OFF_XL_119Hz      = 0x30,
  LSM9DS1_GY_OFF_XL_238Hz      = 0x40,
  LSM9DS1_GY_OFF_XL_476Hz      = 0x50,
  LSM9DS1_GY_OFF_XL_952Hz      = 0x60,
  LSM9DS1_XL_OFF_GY_14Hz9      = 0x01,
  LSM9DS1_XL_OFF_GY_59Hz5      = 0x02,
  LSM9DS1_XL_OFF_GY_119Hz      = 0x03,
  LSM9DS1_XL_OFF_GY_238Hz      = 0x04,
  LSM9DS1_XL_OFF_GY_476Hz      = 0x05,
  LSM9DS1_XL_OFF_GY_952Hz      = 0x06,
  LSM9DS1_IMU_14Hz9            = 0x11,
  LSM9DS1_IMU_59Hz5            = 0x22,
  LSM9DS1_IMU_119Hz            = 0x33,
  LSM9DS1_IMU_238Hz            = 0x44,
  LSM9DS1_IMU_476Hz            = 0x55,
  LSM9DS1_IMU_952Hz            = 0x66,
  LSM9DS1_XL_OFF_GY_14Hz9_LP   = 0x81,
  LSM9DS1_XL_OFF_GY_59Hz5_LP   = 0x82,
  LSM9DS1_XL_OFF_GY_119Hz_LP   = 0x83,
  LSM9DS1_IMU_14Hz9_LP         = 0x91,
  LSM9DS1_IMU_59Hz5_LP         = 0xA2,
  LSM9DS1_IMU_119Hz_LP         = 0xB3,
} lsm9ds1_imu_odr_t;
int32_t lsm9ds1_imu_data_rate_set(stmdev_ctx_t *ctx,
                                  lsm9ds1_imu_odr_t val);
int32_t lsm9ds1_imu_data_rate_get(stmdev_ctx_t *ctx,
                                  lsm9ds1_imu_odr_t *val);

typedef struct {
  uint8_t orient              : 3;
  uint8_t signz_g             : 1; /*(0: positive; 1: negative)*/
  uint8_t signy_g             : 1; /*(0: positive; 1: negative)*/
  uint8_t signx_g             : 1; /*(0: positive; 1: negative)*/
} lsm9ds1_gy_orient_t;
int32_t lsm9ds1_gy_orient_set(stmdev_ctx_t *ctx,
                              lsm9ds1_gy_orient_t val);
int32_t lsm9ds1_gy_orient_get(stmdev_ctx_t *ctx,
                              lsm9ds1_gy_orient_t *val);

int32_t lsm9ds1_xl_flag_data_ready_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

int32_t lsm9ds1_gy_flag_data_ready_get(stmdev_ctx_t *ctx,
                                       uint8_t *val);

int32_t lsm9ds1_temp_flag_data_ready_get(stmdev_ctx_t *ctx,
                                         uint8_t *val);

typedef struct {
  uint8_t xen_g             : 1;
  uint8_t yen_g             : 1;
  uint8_t zen_g             : 1;
} lsm9ds1_gy_axis_t;
int32_t lsm9ds1_gy_axis_set(stmdev_ctx_t *ctx, lsm9ds1_gy_axis_t val);
int32_t lsm9ds1_gy_axis_get(stmdev_ctx_t *ctx,
                            lsm9ds1_gy_axis_t *val);

typedef struct {
  uint8_t xen_xl             : 1;
  uint8_t yen_xl             : 1;
  uint8_t zen_xl             : 1;
} lsm9ds1_xl_axis_t;
int32_t lsm9ds1_xl_axis_set(stmdev_ctx_t *ctx, lsm9ds1_xl_axis_t val);
int32_t lsm9ds1_xl_axis_get(stmdev_ctx_t *ctx,
                            lsm9ds1_xl_axis_t *val);

typedef enum {
  LSM9DS1_NO_DECIMATION     = 0,
  LSM9DS1_EVERY_2_SAMPLES   = 1,
  LSM9DS1_EVERY_4_SAMPLES   = 2,
  LSM9DS1_EVERY_8_SAMPLES   = 3,
} lsm9ds1_dec_t;
int32_t lsm9ds1_xl_decimation_set(stmdev_ctx_t *ctx,
                                  lsm9ds1_dec_t val);
int32_t lsm9ds1_xl_decimation_get(stmdev_ctx_t *ctx,
                                  lsm9ds1_dec_t *val);

typedef enum {
  LSM9DS1_2g     = 0,
  LSM9DS1_16g    = 1,
  LSM9DS1_4g     = 2,
  LSM9DS1_8g     = 3,
} lsm9ds1_xl_fs_t;
int32_t lsm9ds1_xl_full_scale_set(stmdev_ctx_t *ctx,
                                  lsm9ds1_xl_fs_t val);
int32_t lsm9ds1_xl_full_scale_get(stmdev_ctx_t *ctx,
                                  lsm9ds1_xl_fs_t *val);

int32_t lsm9ds1_block_data_update_set(stmdev_ctx_t *ctx_mag,
                                      stmdev_ctx_t *ctx_imu, uint8_t val);
int32_t lsm9ds1_block_data_update_get(stmdev_ctx_t *ctx_mag,
                                      stmdev_ctx_t *ctx_imu, uint8_t *val);

int32_t lsm9ds1_mag_offset_set(stmdev_ctx_t *ctx, int16_t *val);
int32_t lsm9ds1_mag_offset_get(stmdev_ctx_t *ctx, int16_t *val);

typedef enum {
  LSM9DS1_MAG_POWER_DOWN    = 0xC0,
  LSM9DS1_MAG_LP_0Hz625     = 0x00,
  LSM9DS1_MAG_LP_1Hz25      = 0x01,
  LSM9DS1_MAG_LP_2Hz5       = 0x02,
  LSM9DS1_MAG_LP_5Hz        = 0x03,
  LSM9DS1_MAG_LP_10Hz       = 0x04,
  LSM9DS1_MAG_LP_20Hz       = 0x05,
  LSM9DS1_MAG_LP_40Hz       = 0x06,
  LSM9DS1_MAG_LP_80Hz       = 0x07,
  LSM9DS1_MAG_MP_0Hz625     = 0x10,
  LSM9DS1_MAG_MP_1Hz25      = 0x11,
  LSM9DS1_MAG_MP_2Hz5       = 0x12,
  LSM9DS1_MAG_MP_5Hz        = 0x13,
  LSM9DS1_MAG_MP_10Hz       = 0x14,
  LSM9DS1_MAG_MP_20Hz       = 0x15,
  LSM9DS1_MAG_MP_40Hz       = 0x16,
  LSM9DS1_MAG_MP_80Hz       = 0x17,
  LSM9DS1_MAG_HP_0Hz625     = 0x20,
  LSM9DS1_MAG_HP_1Hz25      = 0x21,
  LSM9DS1_MAG_HP_2Hz5       = 0x22,
  LSM9DS1_MAG_HP_5Hz        = 0x23,
  LSM9DS1_MAG_HP_10Hz       = 0x24,
  LSM9DS1_MAG_HP_20Hz       = 0x25,
  LSM9DS1_MAG_HP_40Hz       = 0x26,
  LSM9DS1_MAG_HP_80Hz       = 0x27,
  LSM9DS1_MAG_UHP_0Hz625    = 0x30,
  LSM9DS1_MAG_UHP_1Hz25     = 0x31,
  LSM9DS1_MAG_UHP_2Hz5      = 0x32,
  LSM9DS1_MAG_UHP_5Hz       = 0x33,
  LSM9DS1_MAG_UHP_10Hz      = 0x34,
  LSM9DS1_MAG_UHP_20Hz      = 0x35,
  LSM9DS1_MAG_UHP_40Hz      = 0x36,
  LSM9DS1_MAG_UHP_80Hz      = 0x37,
  LSM9DS1_MAG_UHP_155Hz     = 0x38,
  LSM9DS1_MAG_HP_300Hz      = 0x28,
  LSM9DS1_MAG_MP_560Hz      = 0x18,
  LSM9DS1_MAG_LP_1000Hz     = 0x08,
  LSM9DS1_MAG_ONE_SHOT      = 0x70,
} lsm9ds1_mag_data_rate_t;
int32_t lsm9ds1_mag_data_rate_set(stmdev_ctx_t *ctx,
                                  lsm9ds1_mag_data_rate_t val);
int32_t lsm9ds1_mag_data_rate_get(stmdev_ctx_t *ctx,
                                  lsm9ds1_mag_data_rate_t *val);

typedef enum {
  LSM9DS1_4Ga    = 0,
  LSM9DS1_8Ga    = 1,
  LSM9DS1_12Ga   = 2,
  LSM9DS1_16Ga   = 3,
} lsm9ds1_mag_fs_t;
int32_t lsm9ds1_mag_full_scale_set(stmdev_ctx_t *ctx,
                                   lsm9ds1_mag_fs_t val);
int32_t lsm9ds1_mag_full_scale_get(stmdev_ctx_t *ctx,
                                   lsm9ds1_mag_fs_t *val);

int32_t lsm9ds1_mag_flag_data_ready_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

int32_t lsm9ds1_temperature_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lsm9ds1_angular_rate_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lsm9ds1_acceleration_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lsm9ds1_magnetic_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lsm9ds1_magnetic_overflow_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef struct {
  uint8_t imu;
  uint8_t mag;
} lsm9ds1_id_t;
int32_t lsm9ds1_dev_id_get(stmdev_ctx_t *ctx_mag,
                           stmdev_ctx_t *ctx_imu,
                           lsm9ds1_id_t *buff);

typedef struct {
  lsm9ds1_status_reg_m_t status_mag;
  lsm9ds1_status_reg_t   status_imu;
} lsm9ds1_status_t;
int32_t lsm9ds1_dev_status_get(stmdev_ctx_t *ctx_mag,
                               stmdev_ctx_t *ctx_imu,
                               lsm9ds1_status_t *val);

int32_t lsm9ds1_dev_reset_set(stmdev_ctx_t *ctx_mag,
                              stmdev_ctx_t *ctx_imu,
                              uint8_t val);
int32_t lsm9ds1_dev_reset_get(stmdev_ctx_t *ctx_mag,
                              stmdev_ctx_t *ctx_imu,
                              uint8_t *val);

typedef enum {
  LSM9DS1_LSB_LOW_ADDRESS = 0,
  LSM9DS1_MSB_LOW_ADDRESS = 1,
} lsm9ds1_ble_t;
int32_t lsm9ds1_dev_data_format_set(stmdev_ctx_t *ctx_mag,
                                    stmdev_ctx_t *ctx_imu,
                                    lsm9ds1_ble_t val);
int32_t lsm9ds1_dev_data_format_get(stmdev_ctx_t *ctx_mag,
                                    stmdev_ctx_t *ctx_imu,
                                    lsm9ds1_ble_t *val);

int32_t lsm9ds1_dev_boot_set(stmdev_ctx_t *ctx_mag,
                             stmdev_ctx_t *ctx_imu,
                             uint8_t val);
int32_t lsm9ds1_dev_boot_get(stmdev_ctx_t *ctx_mag,
                             stmdev_ctx_t *ctx_imu,
                             uint8_t *val);

int32_t lsm9ds1_gy_filter_reference_set(stmdev_ctx_t *ctx,
                                        uint8_t *buff);
int32_t lsm9ds1_gy_filter_reference_get(stmdev_ctx_t *ctx,
                                        uint8_t *buff);

typedef enum {
  LSM9DS1_LP_STRONG        = 0,
  LSM9DS1_LP_MEDIUM        = 1,
  LSM9DS1_LP_LIGHT         = 2,
  LSM9DS1_LP_ULTRA_LIGHT   = 3,
} lsm9ds1_gy_lp_bw_t;
int32_t lsm9ds1_gy_filter_lp_bandwidth_set(stmdev_ctx_t *ctx,
                                           lsm9ds1_gy_lp_bw_t val);
int32_t lsm9ds1_gy_filter_lp_bandwidth_get(stmdev_ctx_t *ctx,
                                           lsm9ds1_gy_lp_bw_t *val);

typedef enum {
  LSM9DS1_LPF1_OUT              = 0x00,
  LSM9DS1_LPF1_HPF_OUT          = 0x01,
  LSM9DS1_LPF1_LPF2_OUT         = 0x02,
  LSM9DS1_LPF1_HPF_LPF2_OUT     = 0x12,
} lsm9ds1_gy_out_path_t;
int32_t lsm9ds1_gy_filter_out_path_set(stmdev_ctx_t *ctx,
                                       lsm9ds1_gy_out_path_t val);
int32_t lsm9ds1_gy_filter_out_path_get(stmdev_ctx_t *ctx,
                                       lsm9ds1_gy_out_path_t *val);

typedef enum {
  LSM9DS1_LPF1_INT              = 0x00,
  LSM9DS1_LPF1_HPF_INT          = 0x01,
  LSM9DS1_LPF1_LPF2_INT         = 0x02,
  LSM9DS1_LPF1_HPF_LPF2_INT     = 0x12,
} lsm9ds1_gy_int_path_t;
int32_t lsm9ds1_gy_filter_int_path_set(stmdev_ctx_t *ctx,
                                       lsm9ds1_gy_int_path_t val);
int32_t lsm9ds1_gy_filter_int_path_get(stmdev_ctx_t *ctx,
                                       lsm9ds1_gy_int_path_t *val);

typedef enum {
  LSM9DS1_HP_EXTREME       = 0,
  LSM9DS1_HP_ULTRA_STRONG  = 1,
  LSM9DS1_HP_STRONG        = 2,
  LSM9DS1_HP_ULTRA_HIGH    = 3,
  LSM9DS1_HP_HIGH          = 4,
  LSM9DS1_HP_MEDIUM        = 5,
  LSM9DS1_HP_LOW           = 6,
  LSM9DS1_HP_ULTRA_LOW     = 7,
  LSM9DS1_HP_LIGHT         = 8,
  LSM9DS1_HP_ULTRA_LIGHT   = 9,
} lsm9ds1_gy_hp_bw_t;
int32_t lsm9ds1_gy_filter_hp_bandwidth_set(stmdev_ctx_t *ctx,
                                           lsm9ds1_gy_hp_bw_t val);
int32_t lsm9ds1_gy_filter_hp_bandwidth_get(stmdev_ctx_t *ctx,
                                           lsm9ds1_gy_hp_bw_t *val);

typedef enum {
  LSM9DS1_AUTO      = 0x00,
  LSM9DS1_408Hz     = 0x10,
  LSM9DS1_211Hz     = 0x11,
  LSM9DS1_105Hz     = 0x12,
  LSM9DS1_50Hz      = 0x13,
} lsm9ds1_xl_aa_bw_t;
int32_t lsm9ds1_xl_filter_aalias_bandwidth_set(stmdev_ctx_t *ctx,
                                               lsm9ds1_xl_aa_bw_t val);
int32_t lsm9ds1_xl_filter_aalias_bandwidth_get(stmdev_ctx_t *ctx,
                                               lsm9ds1_xl_aa_bw_t *val);
typedef enum {
  LSM9DS1_HP_DIS  = 0,
  LSM9DS1_HP_EN   = 1,
} lsm9ds1_xl_hp_path_t;
int32_t lsm9ds1_xl_filter_int_path_set(stmdev_ctx_t *ctx,
                                       lsm9ds1_xl_hp_path_t val);
int32_t lsm9ds1_xl_filter_int_path_get(stmdev_ctx_t *ctx,
                                       lsm9ds1_xl_hp_path_t *val);

typedef enum {
  LSM9DS1_LP_OUT     = 0,
  LSM9DS1_HP_OUT     = 1,
} lsm9ds1_xl_out_path_t;
int32_t lsm9ds1_xl_filter_out_path_set(stmdev_ctx_t *ctx,
                                       lsm9ds1_xl_out_path_t val);
int32_t lsm9ds1_xl_filter_out_path_get(stmdev_ctx_t *ctx,
                                       lsm9ds1_xl_out_path_t *val);

typedef enum {
  LSM9DS1_LP_DISABLE     = 0x00,
  LSM9DS1_LP_ODR_DIV_50     = 0x10,
  LSM9DS1_LP_ODR_DIV_100    = 0x11,
  LSM9DS1_LP_ODR_DIV_9      = 0x12,
  LSM9DS1_LP_ODR_DIV_400    = 0x13,
} lsm9ds1_xl_lp_bw_t;
int32_t lsm9ds1_xl_filter_lp_bandwidth_set(stmdev_ctx_t *ctx,
                                           lsm9ds1_xl_lp_bw_t val);
int32_t lsm9ds1_xl_filter_lp_bandwidth_get(stmdev_ctx_t *ctx,
                                           lsm9ds1_xl_lp_bw_t *val);

typedef enum {
  LSM9DS1_HP_ODR_DIV_50   = 0,
  LSM9DS1_HP_ODR_DIV_100  = 1,
  LSM9DS1_HP_ODR_DIV_9    = 2,
  LSM9DS1_HP_ODR_DIV_400  = 3,
} lsm9ds1_xl_hp_bw_t;
int32_t lsm9ds1_xl_filter_hp_bandwidth_set(stmdev_ctx_t *ctx,
                                           lsm9ds1_xl_hp_bw_t val);
int32_t lsm9ds1_xl_filter_hp_bandwidth_get(stmdev_ctx_t *ctx,
                                           lsm9ds1_xl_hp_bw_t *val);

int32_t lsm9ds1_filter_settling_mask_set(stmdev_ctx_t *ctx,
                                         uint8_t val);
int32_t lsm9ds1_filter_settling_mask_get(stmdev_ctx_t *ctx,
                                         uint8_t *val);

int32_t lsm9ds1_auto_increment_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm9ds1_auto_increment_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM9DS1_SPI_4_WIRE = 0,
  LSM9DS1_SPI_3_WIRE = 1,
} lsm9ds1_sim_t;
int32_t lsm9ds1_spi_mode_set(stmdev_ctx_t *ctx_mag,
                             stmdev_ctx_t *ctx_imu,
                             lsm9ds1_sim_t val);
int32_t lsm9ds1_spi_mode_get(stmdev_ctx_t *ctx_mag,
                             stmdev_ctx_t *ctx_imu,
                             lsm9ds1_sim_t *val);

typedef enum {
  LSM9DS1_I2C_ENABLE = 0,
  LSM9DS1_I2C_DISABLE = 1,
} lsm9ds1_i2c_dis_t;
int32_t lsm9ds1_i2c_interface_set(stmdev_ctx_t *ctx_mag,
                                  stmdev_ctx_t *ctx_imu,
                                  lsm9ds1_i2c_dis_t val);
int32_t lsm9ds1_i2c_interface_get(stmdev_ctx_t *ctx_mag,
                                  stmdev_ctx_t *ctx_imu,
                                  lsm9ds1_i2c_dis_t *val);

typedef enum {
  LSM9DS1_LOGIC_OR    = 0,
  LSM9DS1_LOGIC_AND   = 1,
} lsm9ds1_pin_logic_t;
int32_t lsm9ds1_pin_logic_set(stmdev_ctx_t *ctx,
                              lsm9ds1_pin_logic_t val);
int32_t lsm9ds1_pin_logic_get(stmdev_ctx_t *ctx,
                              lsm9ds1_pin_logic_t *val);

typedef struct {
  uint8_t int1_drdy_xl      : 1;
  uint8_t int1_drdy_g       : 1;
  uint8_t int1_boot         : 1;
  uint8_t int1_fth          : 1;
  uint8_t int1_ovr          : 1;
  uint8_t int1_fss5         : 1;
  uint8_t int1_ig_xl        : 1;
  uint8_t int1_ig_g         : 1;
} lsm9ds1_pin_int1_route_t;
int32_t lsm9ds1_pin_int1_route_set(stmdev_ctx_t *ctx,
                                   lsm9ds1_pin_int1_route_t val);
int32_t lsm9ds1_pin_int1_route_get(stmdev_ctx_t *ctx,
                                   lsm9ds1_pin_int1_route_t *val);
typedef struct {
  uint8_t int2_drdy_xl       : 1;
  uint8_t int2_drdy_g        : 1;
  uint8_t int2_drdy_temp     : 1;
  uint8_t int2_fth           : 1;
  uint8_t int2_ovr           : 1;
  uint8_t int2_fss5          : 1;
  uint8_t int2_inact         : 1;
} lsm9ds1_pin_int2_route_t;
int32_t lsm9ds1_pin_int2_route_set(stmdev_ctx_t *ctx,
                                   lsm9ds1_pin_int2_route_t val);
int32_t lsm9ds1_pin_int2_route_get(stmdev_ctx_t *ctx,
                                   lsm9ds1_pin_int2_route_t *val);

typedef enum {
  LSM9DS1_INT_PULSED  = 0,
  LSM9DS1_INT_LATCHED = 1,
} lsm9ds1_lir_t;
int32_t lsm9ds1_pin_notification_set(stmdev_ctx_t *ctx_mag,
                                     stmdev_ctx_t *ctx_imu,
                                     lsm9ds1_lir_t val);
int32_t lsm9ds1_pin_notification_get(stmdev_ctx_t *ctx_mag,
                                     stmdev_ctx_t *ctx_imu,
                                     lsm9ds1_lir_t *val);

typedef enum {
  LSM9DS1_PUSH_PULL   = 0,
  LSM9DS1_OPEN_DRAIN  = 1,
} lsm9ds1_pp_od_t;
int32_t lsm9ds1_pin_mode_set(stmdev_ctx_t *ctx, lsm9ds1_pp_od_t val);
int32_t lsm9ds1_pin_mode_get(stmdev_ctx_t *ctx, lsm9ds1_pp_od_t *val);

typedef struct {
  uint8_t ien             : 1;
} lsm9ds1_pin_m_route_t;
int32_t lsm9ds1_pin_int_m_route_set(stmdev_ctx_t *ctx,
                                    lsm9ds1_pin_m_route_t val);
int32_t lsm9ds1_pin_int_m_route_get(stmdev_ctx_t *ctx,
                                    lsm9ds1_pin_m_route_t *val);
typedef enum {
  LSM9DS1_ACTIVE_LOW    = 0,
  LSM9DS1_ACTIVE_HIGH   = 1,
} lsm9ds1_polarity_t;
int32_t lsm9ds1_pin_polarity_set(stmdev_ctx_t *ctx_mag,
                                 stmdev_ctx_t *ctx_imu,
                                 lsm9ds1_polarity_t val);
int32_t lsm9ds1_pin_polarity_get(stmdev_ctx_t *ctx_mag,
                                 stmdev_ctx_t *ctx_imu,
                                 lsm9ds1_polarity_t *val);

typedef struct {
  uint8_t xlie_xl             : 1;
  uint8_t xhie_xl             : 1;
  uint8_t ylie_xl             : 1;
  uint8_t yhie_xl             : 1;
  uint8_t zlie_xl             : 1;
  uint8_t zhie_xl             : 1;
} lsm9ds1_xl_trshld_en_t;
int32_t lsm9ds1_xl_trshld_axis_set(stmdev_ctx_t *ctx,
                                   lsm9ds1_xl_trshld_en_t val);
int32_t lsm9ds1_xl_trshld_axis_get(stmdev_ctx_t *ctx,
                                   lsm9ds1_xl_trshld_en_t *val);

int32_t lsm9ds1_xl_trshld_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lsm9ds1_xl_trshld_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lsm9ds1_xl_trshld_min_sample_set(stmdev_ctx_t *ctx,
                                         uint8_t val);
int32_t lsm9ds1_xl_trshld_min_sample_get(stmdev_ctx_t *ctx,
                                         uint8_t *val);

typedef struct {
  uint8_t xl_g             : 1;
  uint8_t xh_g             : 1;
  uint8_t yl_g             : 1;
  uint8_t yh_g             : 1;
  uint8_t zl_g             : 1;
  uint8_t zh_g             : 1;
  uint8_t ia_g             : 1;
} lsm9ds1_gy_trshld_src_t;
int32_t lsm9ds1_gy_trshld_src_get(stmdev_ctx_t *ctx,
                                  lsm9ds1_gy_trshld_src_t *val);

typedef struct {
  uint8_t xl_xl             : 1;
  uint8_t xh_xl             : 1;
  uint8_t yl_xl             : 1;
  uint8_t yh_xl             : 1;
  uint8_t zl_xl             : 1;
  uint8_t zh_xl             : 1;
  uint8_t ia_xl             : 1;
} lsm9ds1_xl_trshld_src_t;
int32_t lsm9ds1_xl_trshld_src_get(stmdev_ctx_t *ctx,
                                  lsm9ds1_xl_trshld_src_t *val);

typedef struct {
  uint8_t xlie_g             : 1;
  uint8_t xhie_g             : 1;
  uint8_t ylie_g             : 1;
  uint8_t yhie_g             : 1;
  uint8_t zlie_g             : 1;
  uint8_t zhie_g             : 1;
} lsm9ds1_gy_trshld_en_t;
int32_t lsm9ds1_gy_trshld_axis_set(stmdev_ctx_t *ctx,
                                   lsm9ds1_gy_trshld_en_t val);
int32_t lsm9ds1_gy_trshld_axis_get(stmdev_ctx_t *ctx,
                                   lsm9ds1_gy_trshld_en_t *val);

typedef enum {
  LSM9DS1_RESET_MODE = 0,
  LSM9DS1_DECREMENT_MODE = 1,
} lsm9ds1_dcrm_g_t;
int32_t lsm9ds1_gy_trshld_mode_set(stmdev_ctx_t *ctx,
                                   lsm9ds1_dcrm_g_t val);
int32_t lsm9ds1_gy_trshld_mode_get(stmdev_ctx_t *ctx,
                                   lsm9ds1_dcrm_g_t *val);

int32_t lsm9ds1_gy_trshld_x_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t lsm9ds1_gy_trshld_x_get(stmdev_ctx_t *ctx, uint16_t *val);

int32_t lsm9ds1_gy_trshld_y_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t lsm9ds1_gy_trshld_y_get(stmdev_ctx_t *ctx, uint16_t *val);

int32_t lsm9ds1_gy_trshld_z_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t lsm9ds1_gy_trshld_z_get(stmdev_ctx_t *ctx, uint16_t *val);

int32_t lsm9ds1_gy_trshld_min_sample_set(stmdev_ctx_t *ctx,
                                         uint8_t val);
int32_t lsm9ds1_gy_trshld_min_sample_get(stmdev_ctx_t *ctx,
                                         uint8_t *val);

typedef struct {
  uint8_t zien             : 1;
  uint8_t yien             : 1;
  uint8_t xien             : 1;
} lsm9ds1_mag_trshld_axis_t;
int32_t lsm9ds1_mag_trshld_axis_set(stmdev_ctx_t *ctx,
                                    lsm9ds1_mag_trshld_axis_t val);
int32_t lsm9ds1_mag_trshld_axis_get(stmdev_ctx_t *ctx,
                                    lsm9ds1_mag_trshld_axis_t *val);
typedef struct {
  uint8_t _int              : 1;
  uint8_t nth_z             : 1;
  uint8_t nth_y             : 1;
  uint8_t nth_x             : 1;
  uint8_t pth_z             : 1;
  uint8_t pth_y             : 1;
  uint8_t pth_x             : 1;
} lsm9ds1_mag_trshld_src_t;
int32_t lsm9ds1_mag_trshld_src_get(stmdev_ctx_t *ctx,
                                   lsm9ds1_mag_trshld_src_t *val);

int32_t lsm9ds1_mag_trshld_set(stmdev_ctx_t *ctx, uint8_t *val);
int32_t lsm9ds1_mag_trshld_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm9ds1_act_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm9ds1_act_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM9DS1_GYRO_POWER_DOWN = 0,
  LSM9DS1_GYRO_SLEEP = 1,
} lsm9ds1_act_mode_t;
int32_t lsm9ds1_act_mode_set(stmdev_ctx_t *ctx,
                             lsm9ds1_act_mode_t val);
int32_t lsm9ds1_act_mode_get(stmdev_ctx_t *ctx,
                             lsm9ds1_act_mode_t *val);

int32_t lsm9ds1_act_duration_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lsm9ds1_act_duration_get(stmdev_ctx_t *ctx, uint8_t *buff);

typedef enum {
  LSM9DS1_ACTIVITY    = 0,
  LSM9DS1_INACTIVITY  = 1,
} lsm9ds1_inact_t;
int32_t lsm9ds1_act_src_get(stmdev_ctx_t *ctx, lsm9ds1_inact_t *val);

typedef enum {
  LSM9DS1_POS_MOVE_RECO_DISABLE   = 0x00,
  LSM9DS1_6D_MOVE_RECO            = 0x01,
  LSM9DS1_4D_MOVE_RECO            = 0x05,
  LSM9DS1_6D_POS_RECO             = 0x03,
  LSM9DS1_4D_POS_RECO             = 0x07,
} lsm9ds1_6d_mode_t;
int32_t lsm9ds1_6d_mode_set(stmdev_ctx_t *ctx, lsm9ds1_6d_mode_t val);
int32_t lsm9ds1_6d_mode_get(stmdev_ctx_t *ctx,
                            lsm9ds1_6d_mode_t *val);

int32_t lsm9ds1_6d_threshold_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lsm9ds1_6d_threshold_get(stmdev_ctx_t *ctx, uint8_t *buff);

typedef struct {
  uint8_t xl_xl             : 1;
  uint8_t xh_xl             : 1;
  uint8_t yl_xl             : 1;
  uint8_t yh_xl             : 1;
  uint8_t zl_xl             : 1;
  uint8_t zh_xl             : 1;
  uint8_t ia_xl             : 1;
} lsm9ds1_6d_src_t;
int32_t lsm9ds1_6d_src_get(stmdev_ctx_t *ctx, lsm9ds1_6d_src_t *val);

int32_t lsm9ds1_fifo_stop_on_wtm_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm9ds1_fifo_stop_on_wtm_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM9DS1_FIFO_OFF                = 0x00,
  LSM9DS1_BYPASS_MODE             = 0x10,
  LSM9DS1_FIFO_MODE               = 0x11,
  LSM9DS1_STREAM_TO_FIFO_MODE     = 0x13,
  LSM9DS1_BYPASS_TO_STREAM_MODE   = 0x14,
  LSM9DS1_STREAM_MODE             = 0x16,
} lsm9ds1_fifo_md_t;
int32_t lsm9ds1_fifo_mode_set(stmdev_ctx_t *ctx,
                              lsm9ds1_fifo_md_t val);
int32_t lsm9ds1_fifo_mode_get(stmdev_ctx_t *ctx,
                              lsm9ds1_fifo_md_t *val);

int32_t lsm9ds1_fifo_temp_batch_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm9ds1_fifo_temp_batch_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm9ds1_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm9ds1_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm9ds1_fifo_full_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm9ds1_fifo_data_level_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm9ds1_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm9ds1_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm9ds1_xl_self_test_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm9ds1_xl_self_test_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm9ds1_gy_self_test_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm9ds1_gy_self_test_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lsm9ds1_mag_self_test_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lsm9ds1_mag_self_test_get(stmdev_ctx_t *ctx, uint8_t *val);
/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* LSM9DS1_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
