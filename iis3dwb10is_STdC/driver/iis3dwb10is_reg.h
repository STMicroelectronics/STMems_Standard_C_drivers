/*
 ******************************************************************************
 * @file    iis3dwb10is_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          iis3dwb10is_reg.c driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#ifndef IIS3DWB10IS_REGS_H
#define IIS3DWB10IS_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup IIS3DWB10IS
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
  uint8_t bit0                         : 1;
  uint8_t bit1                         : 1;
  uint8_t bit2                         : 1;
  uint8_t bit3                         : 1;
  uint8_t bit4                         : 1;
  uint8_t bit5                         : 1;
  uint8_t bit6                         : 1;
  uint8_t bit7                         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7                         : 1;
  uint8_t bit6                         : 1;
  uint8_t bit5                         : 1;
  uint8_t bit4                         : 1;
  uint8_t bit3                         : 1;
  uint8_t bit2                         : 1;
  uint8_t bit1                         : 1;
  uint8_t bit0                         : 1;
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

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef void (*stmdev_mdelay_ptr)(uint32_t millisec);

typedef struct
{
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Component optional fields **/
  stmdev_mdelay_ptr   mdelay;
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

#ifndef MEMS_UCF_EXT_SHARED_TYPES
#define MEMS_UCF_EXT_SHARED_TYPES

/*
 * UCF extended format supports the following commands:
 * - MEMS_UCF_OP_READ: read the register at the location specified by the
 *   "address" field ("data" field is ignored
 * - MEMS_UCF_OP_WRITE: write the value specified by the "data" field at the
 *   location specified by the "address" field
 * - MEMS_UCF_OP_DELAY: wait the number of milliseconds specified by the "data"
 *   field ("address" field is ignored)
 * - MEMS_UCF_OP_POLL_SET: poll the register at the location specified by the
 *   "address" field until all the bits identified by the mask specified by the
 *   "data" field are set to 1
 * - MEMS_UCF_OP_POLL_RESET: poll the register at the location specified by the
 *   "address" field until all the bits identified by the mask specified by the
 *   "data" field are reset to 0
 */

#define MEMS_UCF_OP_READ       0
#define MEMS_UCF_OP_WRITE      1
#define MEMS_UCF_OP_DELAY      2
#define MEMS_UCF_OP_POLL_SET   3
#define MEMS_UCF_OP_POLL_RESET 4

typedef struct {
  uint8_t op;
  uint8_t address;
  uint8_t data;
} ucf_line_ext_t;

#endif /* MEMS_UCF_EXT_SHARED_TYPES */

/** @defgroup IIS3DWB10IS_Infos
  * @{
  *
  */


/** Device Identification (Who am I) **/
#define IIS3DWB10IS_ID                           0x50U

#define IIS3DWB10IS_RAM_ACCESS                   0x01U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0                    : 6;
  uint8_t ispu_ram_access_if           : 1;
  uint8_t page_sel                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t page_sel                     : 1;
  uint8_t ispu_ram_access_if           : 1;
  uint8_t not_used0                    : 6;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_ram_access_t;

#define IIS3DWB10IS_PAD_CTRL                     0x02U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t io_pad_strength              : 2;
  uint8_t pd_dis_int                   : 2;
  uint8_t not_used1                    : 3;
  uint8_t ibhr_por_en                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ibhr_por_en                  : 1;
  uint8_t not_used1                    : 3;
  uint8_t pd_dis_int                   : 2;
  uint8_t io_pad_strength              : 2;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_pad_ctrl_t;

#define IIS3DWB10IS_IF_CFG                       0x3U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t i3c_disable                  : 1;
  uint8_t not_used0                    : 2;
  uint8_t int_pp_od                    : 1;
  uint8_t int_active_level             : 1;
  uint8_t not_used1                    : 2;
  uint8_t sda_pu_en                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sda_pu_en                    : 1;
  uint8_t not_used1                    : 2;
  uint8_t int_active_level             : 1;
  uint8_t int_pp_od                    : 1;
  uint8_t not_used0                    : 2;
  uint8_t i3c_disable                  : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_if_cfg_t;

#define IIS3DWB10IS_FIFO_CTRL1                   0x05U
typedef struct
{
  uint8_t wtm                          : 8;
} iis3dwb10is_fifo_ctrl1_t;

#define IIS3DWB10IS_FIFO_CTRL2                   0x06U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t wtm                          : 4;
  uint8_t dec_ts_batch                 : 2;
  uint8_t fifo_trigger_cfg             : 1;
  uint8_t fifo_read_from_ispu          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_read_from_ispu          : 1;
  uint8_t fifo_trigger_cfg             : 1;
  uint8_t dec_ts_batch                 : 2;
  uint8_t wtm                          : 4;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_fifo_ctrl2_t;

#define IIS3DWB10IS_FIFO_CTRL3                   0x07U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fifo_mode                    : 3;
  uint8_t xl_batch                     : 1;
  uint8_t t_batch                      : 1;
  uint8_t ispu_batch                   : 1;
  uint8_t qvar_batch                   : 1;
  uint8_t stop_on_wtm                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t stop_on_wtm                  : 1;
  uint8_t qvar_batch                   : 1;
  uint8_t ispu_batch                   : 1;
  uint8_t t_batch                      : 1;
  uint8_t xl_batch                     : 1;
  uint8_t fifo_mode                    : 3;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_fifo_ctrl3_t;

#define IIS3DWB10IS_PLL_CTRL1                    0x0AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ref_div                      : 3;
  uint8_t not_used0                    : 4;
  uint8_t osc_ext_sel                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t osc_ext_sel                  : 1;
  uint8_t not_used0                    : 4;
  uint8_t ref_div                      : 3;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_pll_ctrl1_t;

#define IIS3DWB10IS_PLL_CTRL2                    0x0BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t pll_div                      : 6;
  uint8_t not_used0                    : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 2;
  uint8_t pll_div                      : 6;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_pll_ctrl2_t;

#define IIS3DWB10IS_INT_CTRL0                    0x0CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_drdy_temp               : 1;
  uint8_t int1_drdy_temp               : 1;
  uint8_t not_used0                    : 2;
  uint8_t int1_boot                    : 1;
  uint8_t int2_sleep_ispu              : 1;
  uint8_t int2_on_int1                 : 1;
  uint8_t pulsed_dataready             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t pulsed_dataready             : 1;
  uint8_t int2_on_int1                 : 1;
  uint8_t int2_sleep_ispu              : 1;
  uint8_t int1_boot                    : 1;
  uint8_t not_used0                    : 2;
  uint8_t int1_drdy_temp               : 1;
  uint8_t int2_drdy_temp               : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_int0_ctrl_t;

#define IIS3DWB10IS_INT_CTRL1                    0x0DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_drdy_xl                 : 1;
  uint8_t int1_drdy_qvar               : 1;
  uint8_t int1_sleepcnt                : 1;
  uint8_t int1_ext_trig                : 1;
  uint8_t int1_fifo_th                 : 1;
  uint8_t int1_fifo_ovr                : 1;
  uint8_t int1_fifo_full               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_fifo_full               : 1;
  uint8_t int1_fifo_ovr                : 1;
  uint8_t int1_fifo_th                 : 1;
  uint8_t int1_ext_trig                : 1;
  uint8_t int1_sleepcnt                : 1;
  uint8_t int1_drdy_qvar               : 1;
  uint8_t int1_drdy_xl                 : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_int1_ctrl_t;

#define IIS3DWB10IS_WHO_AM_I                     0x0FU

#define IIS3DWB10IS_CTRL1                        0x10U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t odr_xl                       : 4;
  uint8_t burst_cfg                    : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t burst_cfg                    : 4;
  uint8_t odr_xl                       : 4;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_ctrl1_t;

#define IIS3DWB10IS_CTRL2                        0x11U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0                    : 5;
  uint8_t fs                           : 2;
  uint8_t not_used1                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1                    : 1;
  uint8_t fs                           : 2;
  uint8_t not_used0                    : 5;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_ctrl2_t;

#define IIS3DWB10IS_CTRL3                        0x12U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sw_reset                     : 1;
  uint8_t not_used0                    : 1;
  uint8_t if_inc                       : 1;
  uint8_t ispu_en                      : 1;
  uint8_t fifo_en                      : 1;
  uint8_t burst_force_trg              : 1;
  uint8_t bdu                          : 1;
  uint8_t boot                         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                         : 1;
  uint8_t bdu                          : 1;
  uint8_t burst_force_trg              : 1;
  uint8_t fifo_en                      : 1;
  uint8_t ispu_en                      : 1;
  uint8_t if_inc                       : 1;
  uint8_t not_used0                    : 1;
  uint8_t sw_reset                     : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_ctrl3_t;

#define IIS3DWB10IS_CTRL4                        0x13U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0                    : 2;
  uint8_t x_axis_enable_us             : 1;
  uint8_t y_axis_enable_us             : 1;
  uint8_t z_axis_enable_us             : 1;
  uint8_t qvar_enable                  : 1;
  uint8_t rounding                     : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t rounding                     : 2;
  uint8_t qvar_enable                  : 1;
  uint8_t z_axis_enable_us             : 1;
  uint8_t y_axis_enable_us             : 1;
  uint8_t x_axis_enable_us             : 1;
  uint8_t not_used0                    : 2;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_ctrl4_t;

/**
  * @defgroup IIS3DWB10IS_Register_Union
  * @brief    This union group all the registers having a bit-field
  *           description.
  *           This union is useful but it's not needed by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  */
typedef union
{
  iis3dwb10is_ram_access_t             ram_access;
  iis3dwb10is_pad_ctrl_t               pad_ctrl;
  iis3dwb10is_if_cfg_t                 if_cfg;
  iis3dwb10is_fifo_ctrl1_t             fifo_ctrl1;
  iis3dwb10is_fifo_ctrl2_t             fifo_ctrl2;
  iis3dwb10is_fifo_ctrl3_t             fifo_ctrl3;
  iis3dwb10is_pll_ctrl1_t              pll_ctrl1;
  iis3dwb10is_pll_ctrl2_t              pll_ctrl2;
  iis3dwb10is_int0_ctrl_t              int0_ctrl;
  iis3dwb10is_int1_ctrl_t              int1_ctrl;
  iis3dwb10is_ctrl1_t                  ctrl1;
  iis3dwb10is_ctrl2_t                  ctrl2;
  iis3dwb10is_ctrl3_t                  ctrl3;
  iis3dwb10is_ctrl4_t                  ctrl4;
  bitwise_t                            bitwise;
  uint8_t                              byte;
} iis3dwb10is_reg_t;

/**
  * @}
  *
  */

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

/*
 * These are the basic platform dependent I/O routines to read
 * and write device registers connected on a standard bus.
 * The driver keeps offering a default implementation based on function
 * pointers to read/write routines for backward compatibility.
 * The __weak directive allows the final application to overwrite
 * them with a custom implementation.
 */

int32_t iis3dwb10is_read_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                             uint8_t *data,
                             uint16_t len);
int32_t iis3dwb10is_write_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                              uint8_t *data,
                              uint16_t len);

int32_t iis3dwb10is_device_id_get(const stmdev_ctx_t *ctx, uint8_t *id);

typedef enum
{
  IIS3DWB10IS_MAIN_MEM_BANK =                    0x0,
  IIS3DWB10IS_ISPU_MEM_BANK =                    0x3,
} iis3dwb10is_mem_bank_t;
int32_t iis3dwb10is_mem_bank_set(const stmdev_ctx_t *ctx, iis3dwb10is_mem_bank_t val);
int32_t iis3dwb10is_mem_bank_get(const stmdev_ctx_t *ctx, iis3dwb10is_mem_bank_t *val);

typedef struct
{
  uint8_t sw_rst : 1; /* Reset register to default */
  uint8_t boot   : 1; /* reloads memory content */
} iis3dwb10is_reset_t;
int32_t iis3dwb10is_reset_set(const stmdev_ctx_t *ctx, iis3dwb10is_reset_t val);
int32_t iis3dwb10is_reset_get(const stmdev_ctx_t *ctx, iis3dwb10is_reset_t *val);

typedef struct
{
  enum
  {
    IIS3DWB10IS_CONTINUOS_MODE     = 0x0, /* continuos mode */
    IIS3DWB10IS_TON_UI_TOFF_ISPU   = 0x1, /* Trig-ON: UI, Trig-OFF: ISPU */
    IIS3DWB10IS_TON_UI_TOFF_FIFO   = 0x2, /* Trig-ON: UI, Trig-OFF: FIFO */
    IIS3DWB10IS_TON_STC_TOFF_UI    = 0x4, /* Trig-ON: Sleep Timer Counter, Trig-OFF: UI */
    IIS3DWB10IS_TON_STC_TOFF_ISPU  = 0x5, /* Trig-ON: Sleep Timer Counter, Trig-OFF: ISPU */
    IIS3DWB10IS_TON_STC_TOFF_FIFO  = 0x6, /* Trig-ON: Sleep Timer Counter, Trig-OFF: FIFO */
    IIS3DWB10IS_TON_EXT_TOFF_UI    = 0x8, /* Trig-ON: External, Trigger off: UI */
    IIS3DWB10IS_TON_EXT_TOFF_ISPU  = 0x9, /* Trig-ON: External, Trigger off: ISPU */
    IIS3DWB10IS_TON_EXT_TOFF_FIFO  = 0xA, /* Trig-ON: External, Trig-OFF: FIFO */
  } burst;

  enum
  {
    IIS3DWB10IS_ODR_IDLE       = 0x0,
    IIS3DWB10IS_ODR_2KHz5      = 0x2,
    IIS3DWB10IS_ODR_5KHz       = 0x3,
    IIS3DWB10IS_ODR_10KHz      = 0x4,
    IIS3DWB10IS_ODR_20KHz      = 0x5,
    IIS3DWB10IS_ODR_40KHz      = 0x6,
  } odr;
} iis3dwb10is_data_rate_t;
int32_t iis3dwb10is_xl_data_rate_set(const stmdev_ctx_t *ctx, iis3dwb10is_data_rate_t val);
int32_t iis3dwb10is_xl_data_rate_get(const stmdev_ctx_t *ctx, iis3dwb10is_data_rate_t *val);

typedef enum
{
  IIS3DWB_50g   = 0,
  IIS3DWB_100g  = 1,
  IIS3DWB_200g  = 2,
} iis3dwb10is_fs_xl_t;
int32_t iis3dwb10is_xl_full_scale_set(const stmdev_ctx_t *ctx, iis3dwb10is_fs_xl_t val);
int32_t iis3dwb10is_xl_full_scale_get(const stmdev_ctx_t *ctx, iis3dwb10is_fs_xl_t *val);

typedef struct
{
  enum
  {
    IIS3DWB10IS_PAD_STRENGTH_LOWER        = 0x0,
    IIS3DWB10IS_PAD_STRENGTH_INTERMEDIATE = 0x1,
    IIS3DWB10IS_PAD_STRENGTH_HIGHEST      = 0x3,
  } strength;

  enum
  {
    IIS3DWB10IS_PD_INT1_ON_INT2_ON     = 0x0,
    IIS3DWB10IS_PD_INT1_OFF_INT2_ON    = 0x1,
    IIS3DWB10IS_PD_INT1_ON_INT2_OFF    = 0x2,
    IIS3DWB10IS_PD_INT1_OFF_INT2_OFF   = 0x3,
  } pd_dis;

  enum
  {
    IIS3DWB10IS_PUSH_PULL   = 0,
    IIS3DWB10IS_OPEN_DRAIN  = 1,
  } pp_od;

  uint8_t int_active_level : 1;
} iis3dwb10is_int_pin_t;
int32_t iis3dwb10is_interrupt_pin_mode_set(const stmdev_ctx_t *ctx, iis3dwb10is_int_pin_t val);
int32_t iis3dwb10is_interrupt_pin_mode_get(const stmdev_ctx_t *ctx, iis3dwb10is_int_pin_t *val);

int32_t iis3dwb10is_fifo_watermark_set(const stmdev_ctx_t *ctx, uint16_t val);
int32_t iis3dwb10is_fifo_watermark_get(const stmdev_ctx_t *ctx, uint16_t *val);

int32_t iis3dwb10is_fifo_stop_on_wtm_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dwb10is_fifo_stop_on_wtm_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  IIS3DWB10IS_BYPASS_MODE             = 0x0,
  IIS3DWB10IS_FIFO_MODE               = 0x1,
  IIS3DWB10IS_STREAM_MODE             = 0x2,
  IIS3DWB10IS_STREAM_TO_FIFO_MODE     = 0x3,
  IIS3DWB10IS_BYPASS_TO_STREAM_MODE   = 0x4,
  IIS3DWB10IS_BYPASS_TO_FIFO_MODE     = 0x7,
  IIS3DWB10IS_FIFO_OFF                = 0x8,
} iis3dwb10is_fifo_mode_t;
int32_t iis3dwb10is_fifo_mode_set(const stmdev_ctx_t *ctx, iis3dwb10is_fifo_mode_t val);
int32_t iis3dwb10is_fifo_mode_get(const stmdev_ctx_t *ctx, iis3dwb10is_fifo_mode_t *val);

typedef enum
{
  IIS3DWB10IS_TMSTMP_NOT_BATCHED = 0x0,
  IIS3DWB10IS_TMSTMP_DEC_1       = 0x1,
  IIS3DWB10IS_TMSTMP_DEC_8       = 0x2,
  IIS3DWB10IS_TMSTMP_DEC_32      = 0x3,
} iis3dwb10is_fifo_timestamp_batch_t;
int32_t iis3dwb10is_fifo_timestamp_batch_set(const stmdev_ctx_t *ctx,
                                             iis3dwb10is_fifo_timestamp_batch_t val);
int32_t iis3dwb10is_fifo_timestamp_batch_get(const stmdev_ctx_t *ctx,
                                             iis3dwb10is_fifo_timestamp_batch_t *val);

typedef struct
{
  uint8_t batch_xl   : 1;
  uint8_t batch_temp : 1;
  uint8_t batch_qvar : 1;
  uint8_t batch_ispu : 1;
} iis3dwb10is_fifo_sensor_batch_t;
int32_t iis3dwb10is_fifo_batch_set(const stmdev_ctx_t *ctx, iis3dwb10is_fifo_sensor_batch_t val);
int32_t iis3dwb10is_fifo_batch_get(const stmdev_ctx_t *ctx, iis3dwb10is_fifo_sensor_batch_t *val);

typedef struct
{
  enum
  {
    IIS3DWB10IS_FIFO_TRIGGER_ISPU   = 0,
    IIS3DWB10IS_FIFO_TRIGGER_INT2   = 1,
  } trigger;
  uint8_t read_from_ispu : 1;
} iis3dwb10is_fifo_ispu_ctrl_batch_t;
int32_t iis3dwb10is_fifo_ispu_ctrl_set(const stmdev_ctx_t *ctx,
                                       iis3dwb10is_fifo_ispu_ctrl_batch_t val);
int32_t iis3dwb10is_fifo_ispu_ctrl_get(const stmdev_ctx_t *ctx,
                                       iis3dwb10is_fifo_ispu_ctrl_batch_t *val);

typedef struct
{
  enum
  {
    IIS3DWB10IS_PLL_NO_DIVIDER        = 0x0,
    IIS3DWB10IS_PLL_DIV_2             = 0x1,
    IIS3DWB10IS_PLL_DIV_4             = 0x2,
    IIS3DWB10IS_PLL_DIV_8             = 0x3,
    IIS3DWB10IS_PLL_DIV_16            = 0x4,
    IIS3DWB10IS_PLL_DIV_32            = 0x5,
    IIS3DWB10IS_PLL_DIV_64            = 0x6,
    IIS3DWB10IS_PLL_DIV_128           = 0x7,
  } ref_div;

  enum
  {
    IIS3DWB10IS_PLL_INTERNAL_CLOCK    = 0x0,
    IIS3DWB10IS_PLL_EXTERNAL_CLOCK    = 0x1,
  } osc_ext_sel;

  uint8_t pll_div : 6;
} iis3dwb10is_pll_ctrl_t;
int32_t iis3dwb10is_pll_ctrl_set(const stmdev_ctx_t *ctx, iis3dwb10is_pll_ctrl_t val);
int32_t iis3dwb10is_pll_ctrl_get(const stmdev_ctx_t *ctx, iis3dwb10is_pll_ctrl_t *val);

typedef struct
{
  uint8_t int1_drdy_xl         : 1;
  uint8_t int1_drdy_qvar       : 1;
  uint8_t int1_sleep_cnt       : 1;
  uint8_t int1_ext_trig        : 1;
  uint8_t int1_fifo_th         : 1;
  uint8_t int1_fifo_ovr        : 1;
  uint8_t int1_fifo_full       : 1;
  uint8_t int1_drdy_temp       : 1;
  uint8_t int1_boot            : 1;
  uint8_t int2_drdy_temp       : 1;
  uint8_t int2_sleep_ispu      : 1;
  uint8_t int2_on_int1         : 1;
} iis3dwb10is_pin_int_route_t;
int32_t iis3dwb10is_pin_int_route_set(const stmdev_ctx_t *ctx,
                                      iis3dwb10is_pin_int_route_t val);
int32_t iis3dwb10is_pin_int_route_get(const stmdev_ctx_t *ctx,
                                      iis3dwb10is_pin_int_route_t *val);
/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* IIS3DWB10IS_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
