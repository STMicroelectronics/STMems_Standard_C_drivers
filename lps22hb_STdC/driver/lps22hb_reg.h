/*
 ******************************************************************************
 * @file    lps22hb_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          lps22hb_reg.c driver.
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
#ifndef LPS22HB_REGS_H
#define LPS22HB_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup LPS22HB
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

typedef struct{
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
#endif /* DRV_BIG_ENDIAN */
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


/** @defgroup LSM9DS1_Infos
  * @{
  *
  */

  /** I2C Device Address 8 bit format: if SA0=0 -> 0xB9 if SA0=1 -> 0xBB **/
#define LPS22HB_I2C_ADD_H   0xBBU
#define LPS22HB_I2C_ADD_L   0xB9U

/** Device Identification (Who am I) **/
#define LPS22HB_ID            0xB1U

/**
  * @}
  *
  */

#define LPS22HB_INTERRUPT_CFG  0x0BU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t pe               : 2; /* ple + phe -> pe */
  uint8_t lir              : 1;
  uint8_t diff_en          : 1;
  uint8_t reset_az         : 1;
  uint8_t autozero         : 1;
  uint8_t reset_arp        : 1;
  uint8_t autorifp         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t autorifp         : 1;
  uint8_t reset_arp        : 1;
  uint8_t autozero         : 1;
  uint8_t reset_az         : 1;
  uint8_t diff_en          : 1;
  uint8_t lir              : 1;
  uint8_t pe               : 2; /* ple + phe -> pe */
#endif /* DRV_BIG_ENDIAN */
} lps22hb_interrupt_cfg_t;

#define LPS22HB_THS_P_L        0x0CU
#define LPS22HB_THS_P_H        0x0DU
#define LPS22HB_WHO_AM_I       0x0FU
#define LPS22HB_CTRL_REG1      0x10U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sim              : 1;
  uint8_t bdu              : 1;
  uint8_t lpfp             : 2; /* en_lpfp + lpfp_cfg -> lpfp */
  uint8_t odr              : 3;
  uint8_t not_used_01      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01      : 1;
  uint8_t odr              : 3;
  uint8_t lpfp             : 2; /* en_lpfp + lpfp_cfg -> lpfp */
  uint8_t bdu              : 1;
  uint8_t sim              : 1;
#endif /* DRV_BIG_ENDIAN */
} lps22hb_ctrl_reg1_t;

#define LPS22HB_CTRL_REG2      0x11U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t one_shot         : 1;
  uint8_t not_used_01      : 1;
  uint8_t swreset          : 1;
  uint8_t i2c_dis          : 1;
  uint8_t if_add_inc       : 1;
  uint8_t stop_on_fth      : 1;
  uint8_t fifo_en          : 1;
  uint8_t boot             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot             : 1;
  uint8_t fifo_en          : 1;
  uint8_t stop_on_fth      : 1;
  uint8_t if_add_inc       : 1;
  uint8_t i2c_dis          : 1;
  uint8_t swreset          : 1;
  uint8_t not_used_01      : 1;
  uint8_t one_shot         : 1;
#endif /* DRV_BIG_ENDIAN */
} lps22hb_ctrl_reg2_t;

#define LPS22HB_CTRL_REG3      0x12U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int_s            : 2;
  uint8_t drdy             : 1;
  uint8_t f_ovr            : 1;
  uint8_t f_fth            : 1;
  uint8_t f_fss5           : 1;
  uint8_t pp_od            : 1;
  uint8_t int_h_l          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int_h_l          : 1;
  uint8_t pp_od            : 1;
  uint8_t f_fss5           : 1;
  uint8_t f_fth            : 1;
  uint8_t f_ovr            : 1;
  uint8_t drdy             : 1;
  uint8_t int_s            : 2;
#endif /* DRV_BIG_ENDIAN */

} lps22hb_ctrl_reg3_t;


#define LPS22HB_FIFO_CTRL      0x14U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wtm              : 5;
  uint8_t f_mode           : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t f_mode           : 3;
  uint8_t wtm              : 5;
#endif /* DRV_BIG_ENDIAN */

} lps22hb_fifo_ctrl_t;

#define LPS22HB_REF_P_XL       0x15U
#define LPS22HB_REF_P_L        0x16U
#define LPS22HB_REF_P_H        0x17U
#define LPS22HB_RPDS_L         0x18U
#define LPS22HB_RPDS_H         0x19U

#define LPS22HB_RES_CONF       0x1AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lc_en            : 1;
  uint8_t not_used_01      : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01      : 7;
  uint8_t lc_en            : 1;
#endif /* DRV_BIG_ENDIAN */

} lps22hb_res_conf_t;

#define LPS22HB_INT_SOURCE     0x25U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ph               : 1;
  uint8_t pl               : 1;
  uint8_t ia               : 1;
  uint8_t not_used_01      : 4;
  uint8_t boot_status      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot_status      : 1;
  uint8_t not_used_01      : 4;
  uint8_t ia               : 1;
  uint8_t pl               : 1;
  uint8_t ph               : 1;
#endif /* DRV_BIG_ENDIAN */
} lps22hb_int_source_t;

#define LPS22HB_FIFO_STATUS    0x26U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fss              : 6;
  uint8_t ovr              : 1;
  uint8_t fth_fifo         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fth_fifo         : 1;
  uint8_t ovr              : 1;
  uint8_t fss              : 6;
#endif /* DRV_BIG_ENDIAN */
} lps22hb_fifo_status_t;

#define LPS22HB_STATUS         0x27U
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
#endif /* DRV_BIG_ENDIAN */
} lps22hb_status_t;

#define LPS22HB_PRESS_OUT_XL   0x28U
#define LPS22HB_PRESS_OUT_L    0x29U
#define LPS22HB_PRESS_OUT_H    0x2AU
#define LPS22HB_TEMP_OUT_L     0x2BU
#define LPS22HB_TEMP_OUT_H     0x2CU
#define LPS22HB_LPFP_RES       0x33U

/**
  * @defgroup LPS22HB_Register_Union
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
  lps22hb_interrupt_cfg_t      interrupt_cfg;
  lps22hb_ctrl_reg1_t          ctrl_reg1;
  lps22hb_ctrl_reg2_t          ctrl_reg2;
  lps22hb_ctrl_reg3_t          ctrl_reg3;
  lps22hb_fifo_ctrl_t          fifo_ctrl;
  lps22hb_res_conf_t           res_conf;
  lps22hb_int_source_t         int_source;
  lps22hb_fifo_status_t        fifo_status;
  lps22hb_status_t             status;
  bitwise_t                    bitwise;
  uint8_t                      byte;
} lps22hb_reg_t;

/**
  * @}
  *
  */

int32_t lps22hb_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);
int32_t lps22hb_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);

extern float_t lps22hb_from_lsb_to_hpa(int32_t lsb);
extern float_t lps22hb_from_lsb_to_degc(int16_t lsb);

int32_t lps22hb_autozero_rst_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_autozero_rst_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_autozero_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_autozero_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_pressure_snap_rst_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_pressure_snap_rst_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_pressure_snap_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_pressure_snap_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS22HB_LPF_ODR_DIV_2  = 0,
  LPS22HB_LPF_ODR_DIV_9  = 2,
  LPS22HB_LPF_ODR_DIV_20 = 3,
} lps22hb_lpfp_t;
int32_t lps22hb_low_pass_filter_mode_set(stmdev_ctx_t *ctx,
                                         lps22hb_lpfp_t val);
int32_t lps22hb_low_pass_filter_mode_get(stmdev_ctx_t *ctx,
                                         lps22hb_lpfp_t *val);

typedef enum {
  LPS22HB_POWER_DOWN  = 0,
  LPS22HB_ODR_1_Hz    = 1,
  LPS22HB_ODR_10_Hz   = 2,
  LPS22HB_ODR_25_Hz   = 3,
  LPS22HB_ODR_50_Hz   = 4,
  LPS22HB_ODR_75_Hz   = 5,
} lps22hb_odr_t;
int32_t lps22hb_data_rate_set(stmdev_ctx_t *ctx, lps22hb_odr_t val);
int32_t lps22hb_data_rate_get(stmdev_ctx_t *ctx, lps22hb_odr_t *val);

int32_t lps22hb_one_shoot_trigger_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_one_shoot_trigger_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_pressure_ref_set(stmdev_ctx_t *ctx, int32_t val);
int32_t lps22hb_pressure_ref_get(stmdev_ctx_t *ctx, int32_t *val);

int32_t lps22hb_pressure_offset_set(stmdev_ctx_t *ctx, int16_t val);
int32_t lps22hb_pressure_offset_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lps22hb_press_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_temp_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_press_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_temp_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_pressure_raw_get(stmdev_ctx_t *ctx, uint32_t *buff);

int32_t lps22hb_temperature_raw_get(stmdev_ctx_t *ctx, int16_t *buff);

int32_t lps22hb_low_pass_rst_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_low_power_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_low_power_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_boot_status_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct{
  lps22hb_fifo_status_t  fifo_status;
  lps22hb_status_t       status;
} lps22hb_dev_stat_t;
int32_t lps22hb_dev_status_get(stmdev_ctx_t *ctx, lps22hb_dev_stat_t *val);

typedef enum {
  LPS22HB_NO_THRESHOLD = 0,
  LPS22HB_POSITIVE     = 1,
  LPS22HB_NEGATIVE     = 2,
  LPS22HB_BOTH         = 3,
} lps22hb_pe_t;
int32_t lps22hb_sign_of_int_threshold_set(stmdev_ctx_t *ctx,
                                          lps22hb_pe_t val);
int32_t lps22hb_sign_of_int_threshold_get(stmdev_ctx_t *ctx,
                                          lps22hb_pe_t *val);

typedef enum {
  LPS22HB_INT_PULSED  = 0,
  LPS22HB_INT_LATCHED = 1,
} lps22hb_lir_t;
int32_t lps22hb_int_notification_mode_set(stmdev_ctx_t *ctx,
                                          lps22hb_lir_t val);
int32_t lps22hb_int_notification_mode_get(stmdev_ctx_t *ctx,
                                          lps22hb_lir_t *val);

int32_t lps22hb_int_generation_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_int_generation_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_int_threshold_set(stmdev_ctx_t *ctx, uint16_t val);
int32_t lps22hb_int_threshold_get(stmdev_ctx_t *ctx, uint16_t *val);

typedef enum {
  LPS22HB_DRDY_OR_FIFO_FLAGS = 0,
  LPS22HB_HIGH_PRES_INT      = 1,
  LPS22HB_LOW_PRES_INT       = 2,
  LPS22HB_EVERY_PRES_INT     = 3,
} lps22hb_int_s_t;
int32_t lps22hb_int_pin_mode_set(stmdev_ctx_t *ctx, lps22hb_int_s_t val);
int32_t lps22hb_int_pin_mode_get(stmdev_ctx_t *ctx, lps22hb_int_s_t *val);

int32_t lps22hb_drdy_on_int_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_drdy_on_int_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_ovr_on_int_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_ovr_on_int_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_threshold_on_int_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_threshold_on_int_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_full_on_int_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_full_on_int_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS22HB_PUSH_PULL  = 0,
  LPS22HB_OPEN_DRAIN = 1,
} lps22hb_pp_od_t;
int32_t lps22hb_pin_mode_set(stmdev_ctx_t *ctx, lps22hb_pp_od_t val);
int32_t lps22hb_pin_mode_get(stmdev_ctx_t *ctx, lps22hb_pp_od_t *val);

typedef enum {
  LPS22HB_ACTIVE_HIGH = 0,
  LPS22HB_ACTIVE_LOW = 1,
} lps22hb_int_h_l_t;
int32_t lps22hb_int_polarity_set(stmdev_ctx_t *ctx, lps22hb_int_h_l_t val);
int32_t lps22hb_int_polarity_get(stmdev_ctx_t *ctx, lps22hb_int_h_l_t *val);

int32_t lps22hb_int_source_get(stmdev_ctx_t *ctx, lps22hb_int_source_t *val);

int32_t lps22hb_int_on_press_high_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_int_on_press_low_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_interrupt_event_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_stop_on_fifo_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_stop_on_fifo_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS22HB_BYPASS_MODE           = 0,
  LPS22HB_FIFO_MODE             = 1,
  LPS22HB_STREAM_MODE           = 2,
  LPS22HB_STREAM_TO_FIFO_MODE   = 3,
  LPS22HB_BYPASS_TO_STREAM_MODE = 4,
  LPS22HB_DYNAMIC_STREAM_MODE   = 6,
  LPS22HB_BYPASS_TO_FIFO_MODE   = 7,
} lps22hb_f_mode_t;
int32_t lps22hb_fifo_mode_set(stmdev_ctx_t *ctx, lps22hb_f_mode_t val);
int32_t lps22hb_fifo_mode_get(stmdev_ctx_t *ctx, lps22hb_f_mode_t *val);

int32_t lps22hb_fifo_data_level_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_fth_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS22HB_SPI_4_WIRE = 0,
  LPS22HB_SPI_3_WIRE = 1,
} lps22hb_sim_t;
int32_t lps22hb_spi_mode_set(stmdev_ctx_t *ctx, lps22hb_sim_t val);
int32_t lps22hb_spi_mode_get(stmdev_ctx_t *ctx, lps22hb_sim_t *val);

typedef enum {
  LPS22HB_I2C_ENABLE = 0,
  LPS22HB_I2C_DISABLE = 1,
} lps22hb_i2c_dis_t;
int32_t lps22hb_i2c_interface_set(stmdev_ctx_t *ctx, lps22hb_i2c_dis_t val);
int32_t lps22hb_i2c_interface_get(stmdev_ctx_t *ctx, lps22hb_i2c_dis_t *val);

int32_t lps22hb_auto_add_inc_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hb_auto_add_inc_get(stmdev_ctx_t *ctx, uint8_t *val);

/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* LPS22HB_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
