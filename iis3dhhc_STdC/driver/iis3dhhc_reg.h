/*
 ******************************************************************************
 * @file    iis3dhhc_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          iis3dhhc_reg.c driver.
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
#ifndef IIS3DHHC_REGS_H
#define IIS3DHHC_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup IIS3DHHC
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

/** @defgroup IIS3DHHC_Infos
  * @{
  *
  */

/** Device Identification (Who am I) **/
#define IIS3DHHC_ID            0x11U

/**
  * @}
  *
  */

#define IIS3DHHC_WHO_AM_I      0x0FU
typedef struct {
  uint8_t not_used_01      : 7;
  uint8_t asic_id          : 1;
} iis3dhhc_id_reg_t;

#define IIS3DHHC_CTRL_REG1     0x20U
typedef struct {
  uint8_t bdu              : 1;
  uint8_t drdy_pulse       : 1;
  uint8_t sw_reset         : 1;
  uint8_t boot             : 1;
  uint8_t not_used_01      : 2;
  uint8_t if_add_inc       : 1;
  uint8_t norm_mod_en      : 1;
} iis3dhhc_ctrl_reg1_t;

#define IIS3DHHC_INT1_CTRL     0x21U
typedef struct {
  uint8_t not_used_01      : 2;
  uint8_t int1_ext         : 1;
  uint8_t int1_fth         : 1;
  uint8_t int1_fss5        : 1;
  uint8_t int1_ovr         : 1;
  uint8_t int1_boot        : 1;
  uint8_t int1_drdy        : 1;
} iis3dhhc_int1_ctrl_t;

#define IIS3DHHC_INT2_CTRL     0x22U
typedef struct {
  uint8_t not_used_01      : 3;
  uint8_t int2_fth         : 1;
  uint8_t int2_fss5        : 1;
  uint8_t int2_ovr         : 1;
  uint8_t int2_boot        : 1;
  uint8_t int2_drdy        : 1;
} iis3dhhc_int2_ctrl_t;

#define IIS3DHHC_CTRL_REG4     0x23U
typedef struct {
  uint8_t off_tcomp_en     : 1;
  uint8_t fifo_en          : 1;
  uint8_t pp_od            : 2;
  uint8_t st               : 2;
  uint8_t dsp              : 2;  /* dsp_lp_type + dsp_bw_sel */
} iis3dhhc_ctrl_reg4_t;

#define IIS3DHHC_CTRL_REG5     0x24U
typedef struct {
  uint8_t fifo_spi_hs_on   : 1;
  uint8_t not_used_01      : 7;
} iis3dhhc_ctrl_reg5_t;

#define IIS3DHHC_OUT_TEMP_L    0x25U
#define IIS3DHHC_OUT_TEMP_H    0x26U
#define IIS3DHHC_STATUS        0x27U
typedef struct {
  uint8_t xda              : 1;
  uint8_t yda              : 1;
  uint8_t zda              : 1;
  uint8_t zyxda            : 1;
  uint8_t _xor             : 1;
  uint8_t yor              : 1;
  uint8_t zor              : 1;
  uint8_t zyxor            : 1;
} iis3dhhc_status_t;

#define IIS3DHHC_OUT_X_L_XL    0x28U
#define IIS3DHHC_OUT_X_H_XL    0x29U
#define IIS3DHHC_OUT_Y_L_XL    0x2AU
#define IIS3DHHC_OUT_Y_H_XL    0x2BU
#define IIS3DHHC_OUT_Z_L_XL    0x2CU
#define IIS3DHHC_OUT_Z_H_XL    0x2DU
#define IIS3DHHC_FIFO_CTRL     0x2EU
typedef struct {
  uint8_t fth              : 5;
  uint8_t fmode            : 3;
} iis3dhhc_fifo_ctrl_t;

#define IIS3DHHC_FIFO_SRC      0x2FU
typedef struct {
  uint8_t fss             : 6;
  uint8_t ovrn            : 1;
  uint8_t fth             : 1;
} iis3dhhc_fifo_src_t;

/**
  * @defgroup IIS3DHHC_Register_Union
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
  iis3dhhc_id_reg_t       id_reg;
  iis3dhhc_ctrl_reg1_t    ctrl_reg1;
  iis3dhhc_int1_ctrl_t    int1_ctrl;
  iis3dhhc_int2_ctrl_t    int2_ctrl;
  iis3dhhc_ctrl_reg4_t    ctrl_reg4;
  iis3dhhc_ctrl_reg5_t    ctrl_reg5;
  iis3dhhc_status_t       status;
  iis3dhhc_fifo_ctrl_t    fifo_ctrl;
  iis3dhhc_fifo_src_t     fifo_src;
  bitwise_t               bitwise;
  uint8_t                 byte;
} iis3dhhc_reg_t;

/**
  * @}
  *
  */

int32_t iis3dhhc_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);
int32_t iis3dhhc_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                           uint16_t len);

extern float_t iis3dhhc_from_lsb_to_mg(int16_t lsb);
extern float_t iis3dhhc_from_lsb_to_celsius(int16_t lsb);

int32_t iis3dhhc_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS3DHHC_POWER_DOWN  = 0,
  IIS3DHHC_1kHz1       = 1,
} iis3dhhc_norm_mod_en_t;
int32_t iis3dhhc_data_rate_set(stmdev_ctx_t *ctx,
                               iis3dhhc_norm_mod_en_t val);
int32_t iis3dhhc_data_rate_get(stmdev_ctx_t *ctx,
                               iis3dhhc_norm_mod_en_t *val);

int32_t iis3dhhc_offset_temp_comp_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_offset_temp_comp_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t iis3dhhc_acceleration_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t iis3dhhc_xl_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_xl_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t iis3dhhc_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS3DHHC_ST_DISABLE   = 0,
  IIS3DHHC_ST_POSITIVE  = 1,
  IIS3DHHC_ST_NEGATIVE  = 2,
} iis3dhhc_st_t;
int32_t iis3dhhc_self_test_set(stmdev_ctx_t *ctx, iis3dhhc_st_t val);
int32_t iis3dhhc_self_test_get(stmdev_ctx_t *ctx, iis3dhhc_st_t *val);

typedef enum {
  IIS3DHHC_LINEAR_PHASE_440Hz      = 0,
  IIS3DHHC_LINEAR_PHASE_235Hz      = 1,
  IIS3DHHC_NO_LINEAR_PHASE_440Hz   = 2,
  IIS3DHHC_NO_LINEAR_PHASE_235Hz   = 3,
} iis3dhhc_dsp_t;
int32_t iis3dhhc_filter_config_set(stmdev_ctx_t *ctx, iis3dhhc_dsp_t val);
int32_t iis3dhhc_filter_config_get(stmdev_ctx_t *ctx, iis3dhhc_dsp_t *val);

int32_t iis3dhhc_status_get(stmdev_ctx_t *ctx, iis3dhhc_status_t *val);

typedef enum {
  IIS3DHHC_LATCHED  = 0,
  IIS3DHHC_PULSED   = 1,
} iis3dhhc_drdy_pulse_t;
int32_t iis3dhhc_drdy_notification_mode_set(stmdev_ctx_t *ctx,
                                            iis3dhhc_drdy_pulse_t val);
int32_t iis3dhhc_drdy_notification_mode_get(stmdev_ctx_t *ctx,
                                            iis3dhhc_drdy_pulse_t *val);

typedef enum {
  IIS3DHHC_PIN_AS_INTERRUPT   = 0,
  IIS3DHHC_PIN_AS_TRIGGER     = 1,
} iis3dhhc_int1_ext_t;
int32_t iis3dhhc_int1_mode_set(stmdev_ctx_t *ctx, iis3dhhc_int1_ext_t val);
int32_t iis3dhhc_int1_mode_get(stmdev_ctx_t *ctx, iis3dhhc_int1_ext_t *val);

int32_t iis3dhhc_fifo_threshold_on_int1_set(stmdev_ctx_t *ctx,
                                            uint8_t val);
int32_t iis3dhhc_fifo_threshold_on_int1_get(stmdev_ctx_t *ctx,
                                            uint8_t *val);

int32_t iis3dhhc_fifo_full_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_fifo_full_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_fifo_ovr_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_fifo_ovr_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_boot_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_boot_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_drdy_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_drdy_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_fifo_threshold_on_int2_set(stmdev_ctx_t *ctx,
                                            uint8_t val);
int32_t iis3dhhc_fifo_threshold_on_int2_get(stmdev_ctx_t *ctx,
                                            uint8_t *val);

int32_t iis3dhhc_fifo_full_on_int2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_fifo_full_on_int2_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_fifo_ovr_on_int2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_fifo_ovr_on_int2_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_boot_on_int2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_boot_on_int2_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_drdy_on_int2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_drdy_on_int2_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS3DHHC_ALL_PUSH_PULL    = 0,
  IIS3DHHC_INT1_OD_INT2_PP  = 1,
  IIS3DHHC_INT1_PP_INT2_OD  = 2,
  IIS3DHHC_ALL_OPEN_DRAIN   = 3,
} iis3dhhc_pp_od_t;
int32_t iis3dhhc_pin_mode_set(stmdev_ctx_t *ctx, iis3dhhc_pp_od_t val);
int32_t iis3dhhc_pin_mode_get(stmdev_ctx_t *ctx, iis3dhhc_pp_od_t *val);

int32_t iis3dhhc_fifo_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_fifo_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_fifo_block_spi_hs_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_fifo_block_spi_hs_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  IIS3DHHC_BYPASS_MODE             = 0,
  IIS3DHHC_FIFO_MODE               = 1,
  IIS3DHHC_STREAM_TO_FIFO_MODE     = 3,
  IIS3DHHC_BYPASS_TO_STREAM_MODE   = 4,
  IIS3DHHC_DYNAMIC_STREAM_MODE     = 6,
} iis3dhhc_fmode_t;
int32_t iis3dhhc_fifo_mode_set(stmdev_ctx_t *ctx, iis3dhhc_fmode_t val);
int32_t iis3dhhc_fifo_mode_get(stmdev_ctx_t *ctx, iis3dhhc_fmode_t *val);

int32_t iis3dhhc_fifo_status_get(stmdev_ctx_t *ctx,
                                 iis3dhhc_fifo_src_t *val);

int32_t iis3dhhc_fifo_full_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_fifo_fth_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dhhc_auto_add_inc_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dhhc_auto_add_inc_get(stmdev_ctx_t *ctx, uint8_t *val);

/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* IIS3DHHC_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
