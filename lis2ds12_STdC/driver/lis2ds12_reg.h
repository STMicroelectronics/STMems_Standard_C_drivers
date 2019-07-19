/*
 ******************************************************************************
 * @file    lis2ds12_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          lis2ds12_reg.c driver.
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
#ifndef LIS2DS12_REGS_H
#define LIS2DS12_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup LIS2DS12
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

/** @defgroup LIS2DS12_Infos
  * @{
  *
  */

  /** I2C Device Address 8 bit format  if SA0=0 -> 0x3D if SA0=1 -> 0x3B **/
#define LIS2DS12_I2C_ADD_L     0x3DU
#define LIS2DS12_I2C_ADD_H     0x3BU

/** Device Identification (Who am I) **/
#define LIS2DS12_ID            0x43U

/**
  * @}
  *
  */

#define LIS2DS12_SENSORHUB1_REG        0x06U
typedef struct {
  uint8_t bit0                : 1;
  uint8_t bit1                : 1;
  uint8_t bit2                : 1;
  uint8_t bit3                : 1;
  uint8_t bit4                : 1;
  uint8_t bit5                : 1;
  uint8_t bit6                : 1;
  uint8_t bit7                : 1;
} lis2ds12_sensorhub1_reg_t;

#define LIS2DS12_SENSORHUB2_REG        0x07U
typedef struct {
  uint8_t bit0                : 1;
  uint8_t bit1                : 1;
  uint8_t bit2                : 1;
  uint8_t bit3                : 1;
  uint8_t bit4                : 1;
  uint8_t bit5                : 1;
  uint8_t bit6                : 1;
  uint8_t bit7                : 1;
} lis2ds12_sensorhub2_reg_t;

#define LIS2DS12_SENSORHUB3_REG        0x08U
typedef struct {
  uint8_t bit0                : 1;
  uint8_t bit1                : 1;
  uint8_t bit2                : 1;
  uint8_t bit3                : 1;
  uint8_t bit4                : 1;
  uint8_t bit5                : 1;
  uint8_t bit6                : 1;
  uint8_t bit7                : 1;
} lis2ds12_sensorhub3_reg_t;

#define LIS2DS12_SENSORHUB4_REG        0x09U
typedef struct {
  uint8_t bit0                : 1;
  uint8_t bit1                : 1;
  uint8_t bit2                : 1;
  uint8_t bit3                : 1;
  uint8_t bit4                : 1;
  uint8_t bit5                : 1;
  uint8_t bit6                : 1;
  uint8_t bit7                : 1;
} lis2ds12_sensorhub4_reg_t;

#define LIS2DS12_SENSORHUB5_REG        0x0AU
typedef struct {
  uint8_t bit0                : 1;
  uint8_t bit1                : 1;
  uint8_t bit2                : 1;
  uint8_t bit3                : 1;
  uint8_t bit4                : 1;
  uint8_t bit5                : 1;
  uint8_t bit6                : 1;
  uint8_t bit7                : 1;
} lis2ds12_sensorhub5_reg_t;

#define LIS2DS12_SENSORHUB6_REG        0x0BU
typedef struct {
  uint8_t bit0                : 1;
  uint8_t bit1                : 1;
  uint8_t bit2                : 1;
  uint8_t bit3                : 1;
  uint8_t bit4                : 1;
  uint8_t bit5                : 1;
  uint8_t bit6                : 1;
  uint8_t bit7                : 1;
} lis2ds12_sensorhub6_reg_t;

#define LIS2DS12_MODULE_8BIT           0x0CU
#define LIS2DS12_WHO_AM_I              0x0FU
#define LIS2DS12_CTRL1                 0x20U
typedef struct {
  uint8_t bdu                 : 1;
  uint8_t hf_odr              : 1;
  uint8_t fs                  : 2;
  uint8_t odr                 : 4;
} lis2ds12_ctrl1_t;

#define LIS2DS12_CTRL2                 0x21U
typedef struct {
  uint8_t sim                 : 1;
  uint8_t i2c_disable         : 1;
  uint8_t if_add_inc          : 1;
  uint8_t fds_slope           : 1;
  uint8_t func_cfg_en         : 1;
  uint8_t not_used_01         : 1;
  uint8_t soft_reset          : 1;
  uint8_t boot                : 1;
} lis2ds12_ctrl2_t;

#define LIS2DS12_CTRL3                 0x22U
typedef struct {
  uint8_t pp_od               : 1;
  uint8_t h_lactive           : 1;
  uint8_t lir                 : 1;
  uint8_t tap_z_en            : 1;
  uint8_t tap_y_en            : 1;
  uint8_t tap_x_en            : 1;
  uint8_t st                  : 2;
} lis2ds12_ctrl3_t;

#define LIS2DS12_CTRL4                 0x23U
typedef struct {
  uint8_t int1_drdy           : 1;
  uint8_t int1_fth            : 1;
  uint8_t int1_6d             : 1;
  uint8_t int1_tap            : 1;
  uint8_t int1_ff             : 1;
  uint8_t int1_wu             : 1;
  uint8_t int1_s_tap          : 1;
  uint8_t int1_master_drdy    : 1;
} lis2ds12_ctrl4_t;

#define LIS2DS12_CTRL5                 0x24U
typedef struct {
  uint8_t int2_drdy           : 1;
  uint8_t int2_fth            : 1;
  uint8_t int2_step_det       : 1;
  uint8_t int2_sig_mot        : 1;
  uint8_t int2_tilt           : 1;
  uint8_t int2_on_int1        : 1;
  uint8_t int2_boot           : 1;
  uint8_t drdy_pulsed         : 1;
} lis2ds12_ctrl5_t;

#define LIS2DS12_FIFO_CTRL             0x25U
typedef struct {
  uint8_t if_cs_pu_dis        : 1;
  uint8_t not_used_01         : 2;
  uint8_t module_to_fifo      : 1;
  uint8_t int2_step_count_ov  : 1;
  uint8_t fmode               : 3;
} lis2ds12_fifo_ctrl_t;

#define LIS2DS12_OUT_T                 0x26U
#define LIS2DS12_STATUS                0x27U
typedef struct {
  uint8_t drdy                : 1;
  uint8_t ff_ia               : 1;
  uint8_t _6d_ia              : 1;
  uint8_t single_tap          : 1;
  uint8_t double_tap          : 1;
  uint8_t sleep_state         : 1;
  uint8_t wu_ia               : 1;
  uint8_t fifo_ths            : 1;
} lis2ds12_status_t;

#define LIS2DS12_OUT_X_L               0x28U
#define LIS2DS12_OUT_X_H               0x29U
#define LIS2DS12_OUT_Y_L               0x2AU
#define LIS2DS12_OUT_Y_H               0x2BU
#define LIS2DS12_OUT_Z_L               0x2CU
#define LIS2DS12_OUT_Z_H               0x2DU
#define LIS2DS12_FIFO_THS              0x2EU
typedef struct {
  uint8_t fth                 : 8;
} lis2ds12_fifo_ths_t;

#define LIS2DS12_FIFO_SRC              0x2FU
typedef struct {
  uint8_t not_used_01         : 5;
  uint8_t diff                : 1;
  uint8_t fifo_ovr            : 1;
  uint8_t fth                 : 1;
} lis2ds12_fifo_src_t;

#define LIS2DS12_FIFO_SAMPLES          0x30U
#define LIS2DS12_TAP_6D_THS            0x31U
typedef struct {
  uint8_t tap_ths             : 5;
  uint8_t _6d_ths             : 2;
  uint8_t _4d_en              : 1;
} lis2ds12_tap_6d_ths_t;

#define LIS2DS12_INT_DUR               0x32U
typedef struct {
  uint8_t shock               : 2;
  uint8_t quiet               : 2;
  uint8_t lat                 : 4;
} lis2ds12_int_dur_t;

#define LIS2DS12_WAKE_UP_THS           0x33U
typedef struct {
  uint8_t wu_ths              : 6;
  uint8_t sleep_on            : 1;
  uint8_t single_double_tap   : 1;
} lis2ds12_wake_up_ths_t;

#define LIS2DS12_WAKE_UP_DUR           0x34U
typedef struct {
  uint8_t sleep_dur           : 4;
  uint8_t int1_fss7           : 1;
  uint8_t wu_dur              : 2;
  uint8_t ff_dur              : 1;
} lis2ds12_wake_up_dur_t;

#define LIS2DS12_FREE_FALL             0x35U
typedef struct {
  uint8_t ff_ths              : 3;
  uint8_t ff_dur              : 5;
} lis2ds12_free_fall_t;

#define LIS2DS12_STATUS_DUP            0x36U
typedef struct {
  uint8_t drdy                : 1;
  uint8_t ff_ia               : 1;
  uint8_t _6d_ia              : 1;
  uint8_t single_tap          : 1;
  uint8_t double_tap          : 1;
  uint8_t sleep_state         : 1;
  uint8_t wu_ia               : 1;
  uint8_t ovr                 : 1;
} lis2ds12_status_dup_t;

#define LIS2DS12_WAKE_UP_SRC           0x37U
typedef struct {
  uint8_t z_wu                : 1;
  uint8_t y_wu                : 1;
  uint8_t x_wu                : 1;
  uint8_t wu_ia               : 1;
  uint8_t sleep_state_ia      : 1;
  uint8_t ff_ia               : 1;
  uint8_t not_used_01         : 2;
} lis2ds12_wake_up_src_t;

#define LIS2DS12_TAP_SRC               0x38U
typedef struct {
  uint8_t z_tap               : 1;
  uint8_t y_tap               : 1;
  uint8_t x_tap               : 1;
  uint8_t tap_sign            : 1;
  uint8_t double_tap          : 1;
  uint8_t single_tap          : 1;
  uint8_t tap_ia              : 1;
  uint8_t not_used_01         : 1;
} lis2ds12_tap_src_t;

#define LIS2DS12_6D_SRC                0x39U
typedef struct {
  uint8_t xl                  : 1;
  uint8_t xh                  : 1;
  uint8_t yl                  : 1;
  uint8_t yh                  : 1;
  uint8_t zl                  : 1;
  uint8_t zh                  : 1;
  uint8_t _6d_ia              : 1;
  uint8_t not_used_01         : 1;
} lis2ds12_6d_src_t;

#define LIS2DS12_STEP_COUNTER_MINTHS   0x3AU
typedef struct {
  uint8_t sc_mths             : 6;
  uint8_t pedo4g              : 1;
  uint8_t rst_nstep           : 1;
} lis2ds12_step_counter_minths_t;

#define LIS2DS12_STEP_COUNTER_L        0x3BU
#define LIS2DS12_STEP_COUNTER_H        0x3CU
#define LIS2DS12_FUNC_CK_GATE          0x3DU
typedef struct {
  uint8_t ck_gate_func        : 1;
  uint8_t step_detect         : 1;
  uint8_t rst_pedo            : 1;
  uint8_t rst_sign_mot        : 1;
  uint8_t sig_mot_detect      : 1;
  uint8_t fs_src              : 2;
  uint8_t tilt_int            : 1;
} lis2ds12_func_ck_gate_t;

#define LIS2DS12_FUNC_SRC              0x3EU
typedef struct {
  uint8_t sensorhub_end_op    : 1;
  uint8_t module_ready        : 1;
  uint8_t rst_tilt            : 1;
  uint8_t not_used_01         : 5;
} lis2ds12_func_src_t;

#define LIS2DS12_FUNC_CTRL             0x3FU
typedef struct {
  uint8_t step_cnt_on         : 1;
  uint8_t sign_mot_on         : 1;
  uint8_t master_on           : 1;
  uint8_t tud_en              : 1;
  uint8_t tilt_on             : 1;
  uint8_t module_on           : 1;
  uint8_t not_used_01         : 2;
} lis2ds12_func_ctrl_t;

#define LIS2DS12_PEDO_DEB_REG          0x2BU
typedef struct {
  uint8_t deb_step            : 3;
  uint8_t deb_time            : 5;
} lis2ds12_pedo_deb_reg_t;

#define LIS2DS12_SLV0_ADD              0x30U
typedef struct {
  uint8_t rw_0                : 1;
  uint8_t slave0_add          : 7;
} lis2ds12_slv0_add_t;

#define LIS2DS12_SLV0_SUBADD           0x31U
typedef struct {
  uint8_t slave0_reg          : 8;
} lis2ds12_slv0_subadd_t;

#define LIS2DS12_SLV0_CONFIG           0x32U
typedef struct {
  uint8_t slave0_numop        : 3;
  uint8_t not_used_01         : 5;
} lis2ds12_slv0_config_t;

#define LIS2DS12_DATAWRITE_SLV0        0x33U
typedef struct {
  uint8_t slave_dataw         : 8;
} lis2ds12_datawrite_slv0_t;

#define LIS2DS12_SM_THS                0x34U
typedef struct {
  uint8_t sm_ths              : 8;
} lis2ds12_sm_ths_t;

#define LIS2DS12_STEP_COUNT_DELTA      0x3AU
typedef struct {
  uint8_t step_count_d        : 8;
} lis2ds12_step_count_delta_t;

#define LIS2DS12_CTRL2_ADV             0x3FU
typedef struct {
  uint8_t sim                 : 1;
  uint8_t i2c_disable         : 1;
  uint8_t if_add_inc          : 1;
  uint8_t fds_slope           : 1;
  uint8_t func_cfg_en         : 1;
  uint8_t not_used_01         : 1;
  uint8_t soft_reset          : 1;
  uint8_t boot                : 1;
} lis2ds12_ctrl2_adv_t;

/**
  * @defgroup LIS2DS12_Register_Union
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
  lis2ds12_sensorhub1_reg_t            sensorhub1_reg;
  lis2ds12_sensorhub2_reg_t            sensorhub2_reg;
  lis2ds12_sensorhub3_reg_t            sensorhub3_reg;
  lis2ds12_sensorhub4_reg_t            sensorhub4_reg;
  lis2ds12_sensorhub5_reg_t            sensorhub5_reg;
  lis2ds12_sensorhub6_reg_t            sensorhub6_reg;
  lis2ds12_ctrl1_t                     ctrl1;
  lis2ds12_ctrl2_t                     ctrl2;
  lis2ds12_ctrl3_t                     ctrl3;
  lis2ds12_ctrl4_t                     ctrl4;
  lis2ds12_ctrl5_t                     ctrl5;
  lis2ds12_fifo_ctrl_t                 fifo_ctrl;
  lis2ds12_status_t                    status;
  lis2ds12_fifo_src_t                  fifo_src;
  lis2ds12_tap_6d_ths_t                tap_6d_ths;
  lis2ds12_int_dur_t                   int_dur;
  lis2ds12_wake_up_ths_t               wake_up_ths;
  lis2ds12_wake_up_dur_t               wake_up_dur;
  lis2ds12_free_fall_t                 free_fall;
  lis2ds12_status_dup_t                status_dup;
  lis2ds12_wake_up_src_t               wake_up_src;
  lis2ds12_tap_src_t                   tap_src;
  lis2ds12_6d_src_t                    _6d_src;
  lis2ds12_step_counter_minths_t       step_counter_minths;
  lis2ds12_func_ck_gate_t              func_ck_gate;
  lis2ds12_func_src_t                  func_src;
  lis2ds12_func_ctrl_t                 func_ctrl;
  lis2ds12_pedo_deb_reg_t              pedo_deb_reg;
  lis2ds12_slv0_add_t                  slv0_add;
  lis2ds12_slv0_subadd_t               slv0_subadd;
  lis2ds12_slv0_config_t               slv0_config;
  lis2ds12_datawrite_slv0_t            datawrite_slv0;
  lis2ds12_sm_ths_t                    sm_ths;
  lis2ds12_step_count_delta_t          step_count_delta;
  lis2ds12_ctrl2_adv_t                 ctrl2_adv;
  bitwise_t                            bitwise;
  uint8_t                              byte;
} lis2ds12_reg_t;

/**
  * @}
  *
  */

int32_t lis2ds12_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);
int32_t lis2ds12_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                           uint16_t len);

extern float_t lis2ds12_from_fs2g_to_mg(int16_t lsb);
extern float_t lis2ds12_from_fs4g_to_mg(int16_t lsb);
extern float_t lis2ds12_from_fs8g_to_mg(int16_t lsb);
extern float_t lis2ds12_from_fs16g_to_mg(int16_t lsb);

extern float_t lis2ds12_from_lsb_to_celsius(int16_t lsb);

typedef struct {
  lis2ds12_fifo_src_t       fifo_src;
  lis2ds12_status_dup_t     status_dup;
  lis2ds12_wake_up_src_t    wake_up_src;
  lis2ds12_tap_src_t        tap_src;
  lis2ds12_6d_src_t         _6d_src;
  lis2ds12_func_ck_gate_t   func_ck_gate;
  lis2ds12_func_src_t       func_src;
  } lis2ds12_all_sources_t;
int32_t lis2ds12_all_sources_get(stmdev_ctx_t *ctx,
                                 lis2ds12_all_sources_t *val);

int32_t lis2ds12_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LIS2DS12_2g = 0,
  LIS2DS12_16g = 1,
  LIS2DS12_4g = 2,
  LIS2DS12_8g = 3,
} lis2ds12_fs_t;
int32_t lis2ds12_xl_full_scale_set(stmdev_ctx_t *ctx, lis2ds12_fs_t val);
int32_t lis2ds12_xl_full_scale_get(stmdev_ctx_t *ctx, lis2ds12_fs_t *val);

typedef enum {
  LIS2DS12_XL_ODR_OFF         = 0x00,
  LIS2DS12_XL_ODR_1Hz_LP      = 0x08,
  LIS2DS12_XL_ODR_12Hz5_LP    = 0x09,
  LIS2DS12_XL_ODR_25Hz_LP     = 0x0A,
  LIS2DS12_XL_ODR_50Hz_LP     = 0x0B,
  LIS2DS12_XL_ODR_100Hz_LP    = 0x0C,
  LIS2DS12_XL_ODR_200Hz_LP    = 0x0D,
  LIS2DS12_XL_ODR_400Hz_LP    = 0x0E,
  LIS2DS12_XL_ODR_800Hz_LP    = 0x0F,
  LIS2DS12_XL_ODR_12Hz5_HR    = 0x01,
  LIS2DS12_XL_ODR_25Hz_HR     = 0x02,
  LIS2DS12_XL_ODR_50Hz_HR     = 0x03,
  LIS2DS12_XL_ODR_100Hz_HR    = 0x04,
  LIS2DS12_XL_ODR_200Hz_HR    = 0x05,
  LIS2DS12_XL_ODR_400Hz_HR    = 0x06,
  LIS2DS12_XL_ODR_800Hz_HR    = 0x07,
  LIS2DS12_XL_ODR_1k6Hz_HF    = 0x15,
  LIS2DS12_XL_ODR_3k2Hz_HF    = 0x16,
  LIS2DS12_XL_ODR_6k4Hz_HF    = 0x17,
} lis2ds12_odr_t;
int32_t lis2ds12_xl_data_rate_set(stmdev_ctx_t *ctx, lis2ds12_odr_t val);
int32_t lis2ds12_xl_data_rate_get(stmdev_ctx_t *ctx, lis2ds12_odr_t *val);

int32_t lis2ds12_status_reg_get(stmdev_ctx_t *ctx, lis2ds12_status_t *val);

int32_t lis2ds12_xl_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_acceleration_module_raw_get(stmdev_ctx_t *ctx,
                                             uint8_t *buff);

int32_t lis2ds12_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lis2ds12_acceleration_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lis2ds12_number_of_steps_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lis2ds12_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lis2ds12_auto_increment_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_auto_increment_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LIS2DS12_USER_BANK   = 0,
  LIS2DS12_ADV_BANK    = 1,
} lis2ds12_func_cfg_en_t;
int32_t lis2ds12_mem_bank_set(stmdev_ctx_t *ctx,
                              lis2ds12_func_cfg_en_t val);

int32_t lis2ds12_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LIS2DS12_XL_ST_DISABLE     = 0,
  LIS2DS12_XL_ST_POSITIVE    = 1,
  LIS2DS12_XL_ST_NEGATIVE    = 2,
} lis2ds12_st_t;
int32_t lis2ds12_xl_self_test_set(stmdev_ctx_t *ctx, lis2ds12_st_t val);
int32_t lis2ds12_xl_self_test_get(stmdev_ctx_t *ctx, lis2ds12_st_t *val);

typedef enum {
  LIS2DS12_DRDY_LATCHED   = 0,
  LIS2DS12_DRDY_PULSED    = 1,
} lis2ds12_drdy_pulsed_t;
int32_t lis2ds12_data_ready_mode_set(stmdev_ctx_t *ctx,
                                     lis2ds12_drdy_pulsed_t val);
int32_t lis2ds12_data_ready_mode_get(stmdev_ctx_t *ctx,
                                     lis2ds12_drdy_pulsed_t *val);

typedef enum {
  LIS2DS12_HP_INTERNAL_ONLY  = 0,
  LIS2DS12_HP_ON_OUTPUTS     = 1,
} lis2ds12_fds_slope_t;
int32_t lis2ds12_xl_hp_path_set(stmdev_ctx_t *ctx,
                                lis2ds12_fds_slope_t val);
int32_t lis2ds12_xl_hp_path_get(stmdev_ctx_t *ctx,
                                lis2ds12_fds_slope_t *val);

typedef enum {
  LIS2DS12_SPI_4_WIRE   = 0,
  LIS2DS12_SPI_3_WIRE   = 1,
} lis2ds12_sim_t;
int32_t lis2ds12_spi_mode_set(stmdev_ctx_t *ctx, lis2ds12_sim_t val);
int32_t lis2ds12_spi_mode_get(stmdev_ctx_t *ctx, lis2ds12_sim_t *val);

typedef enum {
  LIS2DS12_I2C_ENABLE   = 0,
  LIS2DS12_I2C_DISABLE  = 1,
} lis2ds12_i2c_disable_t;
int32_t lis2ds12_i2c_interface_set(stmdev_ctx_t *ctx,
                                   lis2ds12_i2c_disable_t val);
int32_t lis2ds12_i2c_interface_get(stmdev_ctx_t *ctx,
                                   lis2ds12_i2c_disable_t *val);

typedef enum {
  LIS2DS12_PULL_UP_CONNECTED     = 0,
  LIS2DS12_PULL_UP_DISCONNECTED  = 1,
} lis2ds12_if_cs_pu_dis_t;
int32_t lis2ds12_cs_mode_set(stmdev_ctx_t *ctx,
                             lis2ds12_if_cs_pu_dis_t val);
int32_t lis2ds12_cs_mode_get(stmdev_ctx_t *ctx,
                             lis2ds12_if_cs_pu_dis_t *val);

typedef enum {
  LIS2DS12_PUSH_PULL   = 0,
  LIS2DS12_OPEN_DRAIN  = 1,
} lis2ds12_pp_od_t;
int32_t lis2ds12_pin_mode_set(stmdev_ctx_t *ctx, lis2ds12_pp_od_t val);
int32_t lis2ds12_pin_mode_get(stmdev_ctx_t *ctx, lis2ds12_pp_od_t *val);

typedef enum {
  LIS2DS12_ACTIVE_HIGH  = 0,
  LIS2DS12_ACTIVE_LOW   = 1,
} lis2ds12_h_lactive_t;
int32_t lis2ds12_pin_polarity_set(stmdev_ctx_t *ctx,
                                  lis2ds12_h_lactive_t val);
int32_t lis2ds12_pin_polarity_get(stmdev_ctx_t *ctx,
                                  lis2ds12_h_lactive_t *val);

typedef enum {
  LIS2DS12_INT_PULSED   = 0,
  LIS2DS12_INT_LATCHED  = 1,
} lis2ds12_lir_t;
int32_t lis2ds12_int_notification_set(stmdev_ctx_t *ctx,
                                      lis2ds12_lir_t val);
int32_t lis2ds12_int_notification_get(stmdev_ctx_t *ctx,
                                      lis2ds12_lir_t *val);

typedef struct{
  uint8_t int1_drdy               : 1;
  uint8_t int1_fth                : 1;
  uint8_t int1_6d                 : 1;
  uint8_t int1_tap                : 1;
  uint8_t int1_ff                 : 1;
  uint8_t int1_wu                 : 1;
  uint8_t int1_s_tap              : 1;
  uint8_t int1_master_drdy        : 1;
  uint8_t int1_fss7               : 1;
} lis2ds12_pin_int1_route_t;
int32_t lis2ds12_pin_int1_route_set(stmdev_ctx_t *ctx,
                                    lis2ds12_pin_int1_route_t val);
int32_t lis2ds12_pin_int1_route_get(stmdev_ctx_t *ctx,
                                    lis2ds12_pin_int1_route_t *val);

typedef struct{
  uint8_t int2_boot               : 1;
  uint8_t int2_tilt               : 1;
  uint8_t int2_sig_mot            : 1;
  uint8_t int2_step_det           : 1;
  uint8_t int2_fth                : 1;
  uint8_t int2_drdy               : 1;
} lis2ds12_pin_int2_route_t;
int32_t lis2ds12_pin_int2_route_set(stmdev_ctx_t *ctx,
                                    lis2ds12_pin_int2_route_t val);
int32_t lis2ds12_pin_int2_route_get(stmdev_ctx_t *ctx,
                                    lis2ds12_pin_int2_route_t *val);

int32_t lis2ds12_all_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_all_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_wkup_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_wkup_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_wkup_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_wkup_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_sleep_mode_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_sleep_mode_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_act_sleep_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_act_sleep_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_tap_detection_on_z_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_tap_detection_on_z_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_tap_detection_on_y_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_tap_detection_on_y_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_tap_detection_on_x_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_tap_detection_on_x_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_tap_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_tap_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_tap_shock_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_tap_shock_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_tap_quiet_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_tap_quiet_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_tap_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_tap_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LIS2DS12_ONLY_SINGLE  = 0,
  LIS2DS12_ONLY_DOUBLE  = 1,
} lis2ds12_single_double_tap_t;
int32_t lis2ds12_tap_mode_set(stmdev_ctx_t *ctx,
                              lis2ds12_single_double_tap_t val);
int32_t lis2ds12_tap_mode_get(stmdev_ctx_t *ctx,
                              lis2ds12_single_double_tap_t *val);

int32_t lis2ds12_tap_src_get(stmdev_ctx_t *ctx, lis2ds12_tap_src_t *val);

typedef enum {
  LIS2DS12_DEG_80   = 0,
  LIS2DS12_DEG_70   = 1,
  LIS2DS12_DEG_60   = 2,
  LIS2DS12_DEG_50   = 3,
} lis2ds12_6d_ths_t;
int32_t lis2ds12_6d_threshold_set(stmdev_ctx_t *ctx, lis2ds12_6d_ths_t val);
int32_t lis2ds12_6d_threshold_get(stmdev_ctx_t *ctx, lis2ds12_6d_ths_t *val);

int32_t lis2ds12_4d_mode_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_4d_mode_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_6d_src_get(stmdev_ctx_t *ctx, lis2ds12_6d_src_t *val);

int32_t lis2ds12_ff_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_ff_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_ff_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_ff_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_fifo_xl_module_batch_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_fifo_xl_module_batch_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LIS2DS12_BYPASS_MODE            = 0,
  LIS2DS12_FIFO_MODE              = 1,
  LIS2DS12_STREAM_TO_FIFO_MODE    = 3,
  LIS2DS12_BYPASS_TO_STREAM_MODE  = 4,
  LIS2DS12_STREAM_MODE            = 6,
} lis2ds12_fmode_t;
int32_t lis2ds12_fifo_mode_set(stmdev_ctx_t *ctx, lis2ds12_fmode_t val);
int32_t lis2ds12_fifo_mode_get(stmdev_ctx_t *ctx, lis2ds12_fmode_t *val);

int32_t lis2ds12_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_fifo_full_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_fifo_data_level_get(stmdev_ctx_t *ctx, uint16_t *val);

int32_t lis2ds12_fifo_src_get(stmdev_ctx_t *ctx, lis2ds12_fifo_src_t *val);

int32_t lis2ds12_pedo_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_pedo_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LIS2DS12_PEDO_AT_2g  = 0,
  LIS2DS12_PEDO_AT_4g  = 1,
} lis2ds12_pedo4g_t;
int32_t lis2ds12_pedo_full_scale_set(stmdev_ctx_t *ctx,
                                     lis2ds12_pedo4g_t val);
int32_t lis2ds12_pedo_full_scale_get(stmdev_ctx_t *ctx,
                                     lis2ds12_pedo4g_t *val);

int32_t lis2ds12_pedo_step_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_pedo_step_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_pedo_step_detect_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_pedo_sens_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_pedo_sens_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_pedo_debounce_steps_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_pedo_debounce_steps_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_pedo_timeout_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_pedo_timeout_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_pedo_steps_period_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lis2ds12_pedo_steps_period_get(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lis2ds12_motion_data_ready_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_motion_sens_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_motion_sens_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_motion_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_motion_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_tilt_data_ready_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_tilt_sens_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_tilt_sens_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_module_sens_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_module_sens_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct {
    lis2ds12_sensorhub1_reg_t  sensorhub1_reg;
    lis2ds12_sensorhub2_reg_t  sensorhub2_reg;
    lis2ds12_sensorhub3_reg_t  sensorhub3_reg;
    lis2ds12_sensorhub4_reg_t  sensorhub4_reg;
    lis2ds12_sensorhub5_reg_t  sensorhub5_reg;
    lis2ds12_sensorhub6_reg_t  sensorhub6_reg;
} lis2ds12_sh_read_data_raw_t;
int32_t lis2ds12_sh_read_data_raw_get(stmdev_ctx_t *ctx,
                                      lis2ds12_sh_read_data_raw_t *val);

int32_t lis2ds12_sh_master_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_sh_master_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LIS2DS12_EXT_PULL_UP        = 0,
  LIS2DS12_INTERNAL_PULL_UP   = 1,
} lis2ds12_tud_en_t;
int32_t lis2ds12_sh_pin_mode_set(stmdev_ctx_t *ctx, lis2ds12_tud_en_t val);
int32_t lis2ds12_sh_pin_mode_get(stmdev_ctx_t *ctx, lis2ds12_tud_en_t *val);

typedef struct{
  uint8_t   slv_add;
  uint8_t   slv_subadd;
  uint8_t   slv_data;
} lis2ds12_sh_cfg_write_t;
int32_t lis2ds12_sh_cfg_write(stmdev_ctx_t *ctx,
                              lis2ds12_sh_cfg_write_t *val);

typedef struct{
  uint8_t   slv_add;
  uint8_t   slv_subadd;
  uint8_t   slv_len;
} lis2ds12_sh_cfg_read_t;
int32_t lis2ds12_sh_slv_cfg_read(stmdev_ctx_t *ctx,
                                 lis2ds12_sh_cfg_read_t *val);

int32_t lis2ds12_sh_slv0_cfg_read_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2ds12_sh_slv0_cfg_read_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2ds12_sh_end_op_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /*__LIS2DS12_DRIVER__H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
