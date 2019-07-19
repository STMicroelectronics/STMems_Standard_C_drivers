/*
 ******************************************************************************
 * @file    l3gd20h_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          l3gd20h_reg.c driver.
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
#ifndef L3GD20H_REGS_H
#define L3GD20H_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup L3GD20H
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

/** @defgroup L3GD20H_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> 0xD5 if SA0=1 -> 0xD7 **/
#define L3GD20H_I2C_ADD_L     0xD5U
#define L3GD20H_I2C_ADD_H     0xD7U

/** Device Identification (Who am I) **/
#define L3GD20H_ID            0xD7U

/**
  * @}
  *
  */

#define L3GD20H_WHO_AM_I                0x0FU
#define L3GD20H_CTRL1                   0x20U
typedef struct {
  uint8_t xen                 : 1;
  uint8_t yen                 : 1;
  uint8_t zen                 : 1;
  uint8_t pd                  : 1;
  uint8_t bw                  : 2;
  uint8_t dr                  : 2;
} l3gd20h_ctrl1_t;

#define L3GD20H_CTRL2                   0x21U
typedef struct {
  uint8_t hpcf                : 4;
  uint8_t hpm                 : 2;
  uint8_t lvlen               : 1;
  uint8_t extren              : 1;
} l3gd20h_ctrl2_t;

#define L3GD20H_CTRL3                   0x22U
typedef struct {
  uint8_t int2_empty          : 1;
  uint8_t int2_orun           : 1;
  uint8_t int2_fth            : 1;
  uint8_t int2_drdy           : 1;
  uint8_t pp_od               : 1;
  uint8_t h_lactive           : 1;
  uint8_t int1_boot           : 1;
  uint8_t int1_ig             : 1;
} l3gd20h_ctrl3_t;

#define L3GD20H_CTRL4                   0x23U
typedef struct {
  uint8_t sim                 : 1;
  uint8_t st                  : 2;
  uint8_t impen               : 1;
  uint8_t fs                  : 2;
  uint8_t ble                 : 1;
  uint8_t bdu                 : 1;
} l3gd20h_ctrl4_t;

#define L3GD20H_CTRL5                   0x24U
typedef struct {
  uint8_t out_sel             : 2;
  uint8_t ig_sel              : 2;
  uint8_t hpen                : 1;
  uint8_t stoponfth           : 1;
  uint8_t fifo_en             : 1;
  uint8_t boot                : 1;
} l3gd20h_ctrl5_t;

#define L3GD20H_REFERENCE               0x25U
#define L3GD20H_OUT_TEMP                0x26U
#define L3GD20H_STATUS                  0x27U
typedef struct {
  uint8_t xda                 : 1;
  uint8_t yda                 : 1;
  uint8_t zda                 : 1;
  uint8_t zyxda               : 1;
  uint8_t _xor                : 1;
  uint8_t yor                 : 1;
  uint8_t zor                 : 1;
  uint8_t zyxor               : 1;
} l3gd20h_status_t;

#define L3GD20H_OUT_X_L                 0x28U
#define L3GD20H_OUT_X_H                 0x29U
#define L3GD20H_OUT_Y_L                 0x2AU
#define L3GD20H_OUT_Y_H                 0x2BU
#define L3GD20H_OUT_Z_L                 0x2CU
#define L3GD20H_OUT_Z_H                 0x2DU
#define L3GD20H_FIFO_CTRL               0x2EU
typedef struct {
  uint8_t fth                 : 5;
  uint8_t fm                  : 3;
} l3gd20h_fifo_ctrl_t;

#define L3GD20H_FIFO_SRC                0x2FU
typedef struct {
  uint8_t fss                 : 5;
  uint8_t empty               : 1;
  uint8_t ovrn                : 1;
  uint8_t fth                 : 1;
} l3gd20h_fifo_src_t;

#define L3GD20H_IG_CFG                  0x30U
typedef struct {
  uint8_t xlie                : 1;
  uint8_t xhie                : 1;
  uint8_t ylie                : 1;
  uint8_t yhie                : 1;
  uint8_t zlie                : 1;
  uint8_t zhie                : 1;
  uint8_t lir                 : 1;
  uint8_t and_or              : 1;
} l3gd20h_ig_cfg_t;

#define L3GD20H_IG_SRC                  0x31U
typedef struct {
  uint8_t xl                  : 1;
  uint8_t xh                  : 1;
  uint8_t yl                  : 1;
  uint8_t yh                  : 1;
  uint8_t zl                  : 1;
  uint8_t zh                  : 1;
  uint8_t ia                  : 1;
  uint8_t not_used_01         : 1;
} l3gd20h_ig_src_t;

#define L3GD20H_IG_THS_XH               0x32U
typedef struct {
  uint8_t thsx                : 7;
  uint8_t dcrm                : 1;
} l3gd20h_ig_ths_xh_t;

#define L3GD20H_IG_THS_XL               0x33U
typedef struct {
  uint8_t thsx                : 8;
} l3gd20h_ig_ths_xl_t;

#define L3GD20H_IG_THS_YH               0x34U
typedef struct {
  uint8_t thsy                : 7;
  uint8_t not_used_01         : 1;
} l3gd20h_ig_ths_yh_t;

#define L3GD20H_IG_THS_YL               0x35U
typedef struct {
  uint8_t thsy                : 8;
} l3gd20h_ig_ths_yl_t;

#define L3GD20H_IG_THS_ZH               0x36U
typedef struct {
  uint8_t thsz                : 7;
  uint8_t not_used_01         : 1;
} l3gd20h_ig_ths_zh_t;

#define L3GD20H_IG_THS_ZL               0x37U
typedef struct {
  uint8_t thsz                : 8;
} l3gd20h_ig_ths_zl_t;

#define L3GD20H_IG_DURATION             0x38U
typedef struct {
  uint8_t d                   : 7;
  uint8_t wait                : 1;
} l3gd20h_ig_duration_t;

#define L3GD20H_LOW_ODR                 0x39U
typedef struct {
  uint8_t low_odr             : 1;
  uint8_t not_used_01         : 1;
  uint8_t sw_res              : 1;
  uint8_t i2c_dis             : 1;
  uint8_t not_used_02         : 1;
  uint8_t drdy_hl             : 1;
  uint8_t not_used_03         : 2;
} l3gd20h_low_odr_t;

/**
  * @defgroup L3GD20H_Register_Union
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
typedef union{
  l3gd20h_ctrl1_t             ctrl1;
  l3gd20h_ctrl2_t             ctrl2;
  l3gd20h_ctrl3_t             ctrl3;
  l3gd20h_ctrl4_t             ctrl4;
  l3gd20h_ctrl5_t             ctrl5;
  l3gd20h_status_t            status;
  l3gd20h_fifo_ctrl_t         fifo_ctrl;
  l3gd20h_fifo_src_t          fifo_src;
  l3gd20h_ig_cfg_t            ig_cfg;
  l3gd20h_ig_src_t            ig_src;
  l3gd20h_ig_ths_xh_t         ig_ths_xh;
  l3gd20h_ig_ths_xl_t         ig_ths_xl;
  l3gd20h_ig_ths_yh_t         ig_ths_yh;
  l3gd20h_ig_ths_yl_t         ig_ths_yl;
  l3gd20h_ig_ths_zh_t         ig_ths_zh;
  l3gd20h_ig_ths_zl_t         ig_ths_zl;
  l3gd20h_ig_duration_t       ig_duration;
  l3gd20h_low_odr_t           low_odr;
  bitwise_t                   bitwise;
  uint8_t                     byte;
} l3gd20h_reg_t;

/**
  * @}
  *
  */

int32_t l3gd20h_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);
int32_t l3gd20h_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);

extern float_t l3gd20h_from_fs245_to_mdps(int16_t lsb);
extern float_t l3gd20h_from_fs500_to_mdps(int16_t lsb);
extern float_t l3gd20h_from_fs2000_to_mdps(int16_t lsb);

extern float_t l3gd20h_from_lsb_to_celsius(int16_t lsb);

typedef struct {
  uint8_t xen             : 1;
  uint8_t yen             : 1;
  uint8_t zen             : 1;
} l3gd20h_gy_axis_t;
int32_t l3gd20h_gy_axis_set(stmdev_ctx_t *ctx, l3gd20h_gy_axis_t val);
int32_t l3gd20h_gy_axis_get(stmdev_ctx_t *ctx, l3gd20h_gy_axis_t *val);

typedef enum {
  L3GD20H_POWER_DOWN    = 0x00,
  L3GD20H_12Hz5         = 0x90,
  L3GD20H_25Hz          = 0x91,
  L3GD20H_50Hz          = 0x92,
  L3GD20H_100Hz         = 0x80,
  L3GD20H_200Hz         = 0x81,
  L3GD20H_400Hz         = 0x82,
  L3GD20H_800Hz         = 0x83,
} l3gd20h_gy_data_rate_t;
int32_t l3gd20h_gy_data_rate_set(stmdev_ctx_t *ctx, 
                                 l3gd20h_gy_data_rate_t val);
int32_t l3gd20h_gy_data_rate_get(stmdev_ctx_t *ctx,
                                 l3gd20h_gy_data_rate_t *val);


typedef enum {
  L3GD20H_245dps    = 0x00,
  L3GD20H_500dps    = 0x01,
  L3GD20H_2000dps   = 0x02,
} l3gd20h_gy_fs_t;
int32_t l3gd20h_gy_full_scale_set(stmdev_ctx_t *ctx, l3gd20h_gy_fs_t val);
int32_t l3gd20h_gy_full_scale_get(stmdev_ctx_t *ctx, l3gd20h_gy_fs_t *val);

int32_t l3gd20h_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t l3gd20h_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t l3gd20h_gy_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t l3gd20h_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t l3gd20h_angular_rate_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t l3gd20h_dev_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

typedef enum {
  L3GD20H_LSB_LOW_ADDRESS = 0,
  L3GD20H_MSB_LOW_ADDRESS = 1,
} l3gd20h_ble_t;
int32_t l3gd20h_dev_data_format_set(stmdev_ctx_t *ctx, l3gd20h_ble_t val);
int32_t l3gd20h_dev_data_format_get(stmdev_ctx_t *ctx, l3gd20h_ble_t *val);

int32_t l3gd20h_dev_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t l3gd20h_dev_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct {
  uint8_t xda               : 1;
  uint8_t yda               : 1;
  uint8_t zda               : 1;
  uint8_t zyxda             : 1;
  uint8_t _xor              : 1;
  uint8_t yor               : 1;
  uint8_t zor               : 1;
  uint8_t zyxor             : 1;
} l3gd20h_status_reg_t;
int32_t l3gd20h_dev_status_get(stmdev_ctx_t *ctx,
                               l3gd20h_status_reg_t *val);

int32_t l3gd20h_dev_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t l3gd20h_dev_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  L3GD20H_16Hz6_USE_ODR_50Hz      = 0xA0,
  L3GD20H_12Hz5_USE_ODR_100Hz     = 0x00,
  L3GD20H_25Hz_USE_ODR_100Hz      = 0x01,
  L3GD20H_12Hz5_USE_ODR_200Hz     = 0x10,
  L3GD20H_70Hz_USE_ODR_200Hz      = 0x13,
  L3GD20H_20Hz_USE_ODR_400Hz      = 0x20,
  L3GD20H_25Hz_USE_ODR_400Hz      = 0x21,
  L3GD20H_50Hz_USE_ODR_400Hz      = 0x22,
  L3GD20H_110Hz_USE_ODR_400Hz     = 0x23,
  L3GD20H_30Hz_USE_ODR_800Hz      = 0x30,
  L3GD20H_35Hz_USE_ODR_800Hz      = 0x31,
  L3GD20H_100Hz_USE_ODR_800Hz     = 0x33,
} l3gd20h_lpbw_t;
int32_t l3gd20h_gy_filter_lp_bandwidth_set(stmdev_ctx_t *ctx,
                                           l3gd20h_lpbw_t val);
int32_t l3gd20h_gy_filter_lp_bandwidth_get(stmdev_ctx_t *ctx,
                                           l3gd20h_lpbw_t *val);

typedef enum {
  L3GD20H_NORMAL_MODE_LIGHT      = 0x00,
  L3GD20H_NORMAL_MODE_NORMAL     = 0x01,
  L3GD20H_NORMAL_MODE_STRONG     = 0x02,
  L3GD20H_NORMAL_MODE_EXTREME    = 0x03,
  L3GD20H_USE_REFERENCE_LIGHT    = 0x10,
  L3GD20H_USE_REFERENCE_NORMAL   = 0x11,
  L3GD20H_USE_REFERENCE_STRONG   = 0x12,
  L3GD20H_USE_REFERENCE_EXTREME  = 0x13,
  L3GD20H_AUTORESET_LIGHT        = 0x30,
  L3GD20H_AUTORESET_NORMAL       = 0x31,
  L3GD20H_AUTORESET_STRONG       = 0x32,
  L3GD20H_AUTORESET_EXTREME      = 0x33,
} l3gd20h_gy_hp_bw_t;
int32_t l3gd20h_gy_filter_hp_bandwidth_set(stmdev_ctx_t *ctx,
                                           l3gd20h_gy_hp_bw_t val);
int32_t l3gd20h_gy_filter_hp_bandwidth_get(stmdev_ctx_t *ctx,
                                           l3gd20h_gy_hp_bw_t *val);

typedef enum {
  L3GD20H_LPF1_OUT           = 0x00,
  L3GD20H_LPF1_HPF_OUT       = 0x11,
  L3GD20H_LPF1_LPF2_OUT      = 0x02,
  L3GD20H_LPF1_HPF_LPF2_OUT  = 0x12,
} l3gd20h_gy_out_path_t;
int32_t l3gd20h_gy_filter_out_path_set(stmdev_ctx_t *ctx,
                                       l3gd20h_gy_out_path_t val);
int32_t l3gd20h_gy_filter_out_path_get(stmdev_ctx_t *ctx,
                                       l3gd20h_gy_out_path_t *val);
typedef enum {
  L3GD20H_LPF1_INT           = 0x00,
  L3GD20H_LPF1_HPF_INT       = 0x11,
  L3GD20H_LPF1_LPF2_INT      = 0x02,
  L3GD20H_LPF1_HPF_LPF2_INT  = 0x12,
} l3gd20h_gy_int_path_t;
int32_t l3gd20h_gy_filter_int_path_set(stmdev_ctx_t *ctx,
                                       l3gd20h_gy_int_path_t val);
int32_t l3gd20h_gy_filter_int_path_get(stmdev_ctx_t *ctx,
                                       l3gd20h_gy_int_path_t *val);

int32_t l3gd20h_gy_filter_reference_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t l3gd20h_gy_filter_reference_get(stmdev_ctx_t *ctx, uint8_t *buff);

typedef enum {
  L3GD20H_SPI_4_WIRE = 0,
  L3GD20H_SPI_3_WIRE = 1,
} l3gd20h_sim_t;
int32_t l3gd20h_spi_mode_set(stmdev_ctx_t *ctx, l3gd20h_sim_t val);
int32_t l3gd20h_spi_mode_get(stmdev_ctx_t *ctx, l3gd20h_sim_t *val);

typedef enum {
  L3GD20H_I2C_ENABLE  = 0,
  L3GD20H_I2C_DISABLE = 1,
} l3gd20h_i2c_dis_t;
int32_t l3gd20h_i2c_interface_set(stmdev_ctx_t *ctx,
                                  l3gd20h_i2c_dis_t val);
int32_t l3gd20h_i2c_interface_get(stmdev_ctx_t *ctx,
                                  l3gd20h_i2c_dis_t *val);

typedef struct {
  uint8_t int2_empty            : 1;
  uint8_t int2_orun             : 1;
  uint8_t int2_fth              : 1;
  uint8_t int2_drdy             : 1;
} l3gd20h_pin_int2_rt_t;
int32_t l3gd20h_pin_int2_route_set(stmdev_ctx_t *ctx,
                                   l3gd20h_pin_int2_rt_t val);
int32_t l3gd20h_pin_int2_route_get(stmdev_ctx_t *ctx,
                                   l3gd20h_pin_int2_rt_t *val);

typedef enum {
  L3GD20H_PUSH_PULL  = 0,
  L3GD20H_OPEN_DRAIN = 1,
} l3gd20h_pp_od_t;
int32_t l3gd20h_pin_mode_set(stmdev_ctx_t *ctx, l3gd20h_pp_od_t val);
int32_t l3gd20h_pin_mode_get(stmdev_ctx_t *ctx, l3gd20h_pp_od_t *val);

typedef enum {
  L3GD20H_ACTIVE_HIGH = 0,
  L3GD20H_ACTIVE_LOW  = 1,
} l3gd20h_pin_pol_t;
int32_t l3gd20h_pin_polarity_set(stmdev_ctx_t *ctx, l3gd20h_pin_pol_t val);
int32_t l3gd20h_pin_polarity_get(stmdev_ctx_t *ctx, l3gd20h_pin_pol_t *val);

typedef struct {
  uint8_t int1_boot           : 1;
  uint8_t int1_ig             : 1;
} l3gd20h_pin_int1_rt_t;
int32_t l3gd20h_pin_int1_route_set(stmdev_ctx_t *ctx,
                                   l3gd20h_pin_int1_rt_t val);
int32_t l3gd20h_pin_int1_route_get(stmdev_ctx_t *ctx,
                                   l3gd20h_pin_int1_rt_t *val);

typedef enum {
  L3GD20H_INT_PULSED  = 0,
  L3GD20H_INT_LATCHED = 1,
} l3gd20h_lir_t;
int32_t l3gd20h_pin_notification_set(stmdev_ctx_t *ctx, l3gd20h_lir_t val);
int32_t l3gd20h_pin_notification_get(stmdev_ctx_t *ctx, l3gd20h_lir_t *val);
typedef enum {
  L3GD20H_LOGIC_OR  = 0,
  L3GD20H_LOGIC_AND = 1,
} l3gd20h_pin_logic_t;
int32_t l3gd20h_pin_logic_set(stmdev_ctx_t *ctx, l3gd20h_pin_logic_t val);
int32_t l3gd20h_pin_logic_get(stmdev_ctx_t *ctx, l3gd20h_pin_logic_t *val);

typedef struct {
  uint8_t xlie             : 1;
  uint8_t xhie             : 1;
  uint8_t ylie             : 1;
  uint8_t yhie             : 1;
  uint8_t zlie             : 1;
  uint8_t zhie             : 1;
} l3gd20h_gy_trshld_en_t;
int32_t l3gd20h_gy_trshld_axis_set(stmdev_ctx_t *ctx,
                                   l3gd20h_gy_trshld_en_t val);
int32_t l3gd20h_gy_trshld_axis_get(stmdev_ctx_t *ctx,
                                   l3gd20h_gy_trshld_en_t *val);

typedef struct {
  uint8_t xl             : 1;
  uint8_t xh             : 1;
  uint8_t yl             : 1;
  uint8_t yh             : 1;
  uint8_t zl             : 1;
  uint8_t zh             : 1;
  uint8_t ia             : 1;
} l3gd20h_gy_trshld_src_t;
int32_t l3gd20h_gy_trshld_src_get(stmdev_ctx_t *ctx,
                                  l3gd20h_gy_trshld_src_t *val);

int32_t l3gd20h_gy_trshld_x_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t l3gd20h_gy_trshld_x_get(stmdev_ctx_t *ctx, uint16_t *val);

typedef enum {
  L3GD20H_RESET_MODE       = 0x00,
  L3GD20H_DECREMENT_MODE   = 0x01,
} l3gd20h_dcrm_g_t;
int32_t l3gd20h_gy_trshld_mode_set(stmdev_ctx_t *ctx,
                                   l3gd20h_dcrm_g_t val);
int32_t l3gd20h_gy_trshld_mode_get(stmdev_ctx_t *ctx,
                                   l3gd20h_dcrm_g_t *val);

int32_t l3gd20h_gy_trshld_y_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t l3gd20h_gy_trshld_y_get(stmdev_ctx_t *ctx, uint16_t *val);

int32_t l3gd20h_gy_trshld_z_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t l3gd20h_gy_trshld_z_get(stmdev_ctx_t *ctx, uint16_t *val);

int32_t l3gd20h_gy_trshld_min_sample_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t l3gd20h_gy_trshld_min_sample_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t l3gd20h_fifo_stop_on_wtm_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t l3gd20h_fifo_stop_on_wtm_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  L3GD20H_BYPASS_MODE              = 0x00,
  L3GD20H_FIFO_MODE                = 0x11,
  L3GD20H_STREAM_MODE              = 0x12,
  L3GD20H_STREAM_TO_FIFO_MODE      = 0x13,
  L3GD20H_BYPASS_TO_STREAM_MODE    = 0x14,
  L3GD20H_DYNAMIC_STREAM_MODE      = 0x16,
  L3GD20H_BYPASS_TO_FIFO_MODE      = 0x17,
} l3gd20h_fifo_m_t;
int32_t l3gd20h_fifo_mode_set(stmdev_ctx_t *ctx, l3gd20h_fifo_m_t val);
int32_t l3gd20h_fifo_mode_get(stmdev_ctx_t *ctx, l3gd20h_fifo_m_t *val);

int32_t l3gd20h_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t l3gd20h_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct {
  uint8_t fss             : 1;
  uint8_t empty           : 1;
  uint8_t ovrn            : 1;
  uint8_t fth             : 1;
} l3gd20h_fifo_srs_t;
int32_t l3gd20h_fifo_src_get(stmdev_ctx_t *ctx, l3gd20h_fifo_srs_t *val);

int32_t l3gd20h_fifo_data_level_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t l3gd20h_fifo_full_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t l3gd20h_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  L3GD20H_DEN_DISABLE           = 0x00,
  L3GD20H_DEN_ON_LEVEL_TRIGGER  = 0x04,
  L3GD20H_DEN_ON_EDGE_TRIGGER   = 0x02,
  L3GD20H_DEN_IMPULSE_TRIGGER   = 0x05,
} l3gd20h_den_md_t;
int32_t l3gd20h_den_mode_set(stmdev_ctx_t *ctx, l3gd20h_den_md_t val);
int32_t l3gd20h_den_mode_get(stmdev_ctx_t *ctx, l3gd20h_den_md_t *val);

typedef enum {
  L3GD20H_ST_DISABLE    = 0x00,
  L3GD20H_ST_POSITIVE   = 0x01,
  L3GD20H_ST_NEGATIVE   = 0x03,
} l3gd20h_st_t;
int32_t l3gd20h_gy_self_test_set(stmdev_ctx_t *ctx, l3gd20h_st_t val);
int32_t l3gd20h_gy_self_test_get(stmdev_ctx_t *ctx, l3gd20h_st_t *val);

/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* L3GD20H_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
