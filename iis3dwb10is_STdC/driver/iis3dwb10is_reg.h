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

typedef struct
{
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

typedef struct
{
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

typedef struct
{
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
  uint8_t sw_por                       : 1;
  uint8_t not_used0                    : 5;
  uint8_t ispu_ram_access_if           : 1;
  uint8_t page_sel                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t page_sel                     : 1;
  uint8_t ispu_ram_access_if           : 1;
  uint8_t not_used0                    : 5;
  uint8_t sw_por                       : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_ram_access_t;

#define IIS3DWB10IS_PIN_CTRL                     0x02U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t io_pin_strength              : 2;
  uint8_t pd_dis_int                   : 2;
  uint8_t not_used1                    : 3;
  uint8_t ibhr_por_en                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ibhr_por_en                  : 1;
  uint8_t not_used1                    : 3;
  uint8_t pd_dis_int                   : 2;
  uint8_t io_pin_strength              : 2;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_pin_ctrl_t;

#define IIS3DWB10IS_IF_CFG                       0x3U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t i3c_disable                  : 1;
  uint8_t not_used0                    : 1;
  uint8_t jtag_en                      : 1;
  uint8_t int_pp_od                    : 1;
  uint8_t int_active_level             : 1;
  uint8_t not_used1                    : 2;
  uint8_t sda_pu_en                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sda_pu_en                    : 1;
  uint8_t not_used1                    : 2;
  uint8_t int_active_level             : 1;
  uint8_t int_pp_od                    : 1;
  uint8_t jtag_en                      : 1;
  uint8_t not_used0                    : 1;
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

#define IIS3DWB10IS_COUNTER_ODR_H                0x08U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t counter_odr              : 4;
  uint8_t rst_counter_odr_rd_diff  : 1;
  uint8_t trigger_counter_odr      : 1;
  uint8_t no_used                  : 1;
  uint8_t rst_counter_odr          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t rst_counter_odr          : 1;
  uint8_t no_used                  : 1;
  uint8_t trigger_counter_odr      : 1;
  uint8_t rst_counter_odr_rd_diff  : 1;
  uint8_t counter_odr              : 4;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_counter_odr_h_t;

#define IIS3DWB10IS_COUNTER_ODR_L                0x09U
typedef struct
{
  uint8_t counter_odr              : 8;
} iis3dwb10is_counter_odr_l_t;

#define IIS3DWB10IS_PLL_CTRL_1                   0x0AU
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
} iis3dwb10is_pll_ctrl_1_t;

#define IIS3DWB10IS_PLL_CTRL_2                   0x0BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t pll_div                      : 6;
  uint8_t not_used0                    : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 2;
  uint8_t pll_div                      : 6;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_pll_ctrl_2_t;

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
} iis3dwb10is_int_ctrl0_t;

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
  uint8_t not_used0                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 1;
  uint8_t int1_fifo_full               : 1;
  uint8_t int1_fifo_ovr                : 1;
  uint8_t int1_fifo_th                 : 1;
  uint8_t int1_ext_trig                : 1;
  uint8_t int1_sleepcnt                : 1;
  uint8_t int1_drdy_qvar               : 1;
  uint8_t int1_drdy_xl                 : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_int_ctrl1_t;

#define IIS3DWB10IS_INT_CTRL2                    0x0EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_drdy_xl                 : 1;
  uint8_t int2_drdy_qvar               : 1;
  uint8_t int2_sleepcnt                : 1;
  uint8_t not_used0                    : 1;
  uint8_t int2_fifo_th                 : 1;
  uint8_t int2_fifo_ovr                : 1;
  uint8_t int2_fifo_full               : 1;
  uint8_t not_used1                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1                    : 1;
  uint8_t int2_fifo_full               : 1;
  uint8_t int2_fifo_ovr                : 1;
  uint8_t int2_fifo_th                 : 1;
  uint8_t not_used0                    : 1;
  uint8_t int2_sleepcnt                : 1;
  uint8_t int2_drdy_qvar               : 1;
  uint8_t int2_drdy_xl                 : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_int_ctrl2_t;

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
  uint8_t int2_in_lh                   : 1;
  uint8_t not_used0                    : 1;
  uint8_t x_axis_enable_us             : 1;
  uint8_t y_axis_enable_us             : 1;
  uint8_t z_axis_enable_us             : 1;
  uint8_t qvar_en                      : 1;
  uint8_t rounding                     : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t rounding                     : 2;
  uint8_t qvar_en                      : 1;
  uint8_t z_axis_enable_us             : 1;
  uint8_t y_axis_enable_us             : 1;
  uint8_t x_axis_enable_us             : 1;
  uint8_t not_used0                    : 1;
  uint8_t int2_in_lh                   : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_ctrl4_t;

#define IIS3DWB10IS_I3C_CTRL                     0x14U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int_en_i3c                   : 1;
  uint8_t bus_act_sel                  : 1;
  uint8_t not_used0                    : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 6;
  uint8_t bus_act_sel                  : 1;
  uint8_t int_en_i3c                   : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_i3c_ctrl_t;

#define IIS3DWB10IS_SPI_CTRL                     0x15U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sim                          : 1;
  uint8_t not_used0                    : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 7;
  uint8_t sim                          : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_spi_ctrl_t;

#define IIS3DWB10IS_QVAR_CTRL                    0x16U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t qvar_c_zin                   : 2;
  uint8_t not_used0                    : 1;
  uint8_t qvar_switch                  : 1;
  uint8_t qvar2_en                     : 1;
  uint8_t qvar1_en                     : 1;
  uint8_t qvar_hpf                     : 1;
  uint8_t qvar_lpf                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t qvar_lpf                     : 1;
  uint8_t qvar_hpf                     : 1;
  uint8_t qvar1_en                     : 1;
  uint8_t qvar2_en                     : 1;
  uint8_t qvar_switch                  : 1;
  uint8_t not_used0                    : 1;
  uint8_t qvar_c_zin                   : 2;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_qvar_ctrl_t;

#define IIS3DWB10IS_ST_CTRL                      0x17U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st_sign                      : 1;
  uint8_t st_en                        : 2;
  uint8_t not_used0                    : 1;
  uint8_t lpf1_cfg                     : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t lpf1_cfg                     : 4;
  uint8_t not_used0                    : 1;
  uint8_t st_en                        : 2;
  uint8_t st_sign                      : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_st_ctrl_t;

#define IIS3DWB10IS_ISPU_CTRL0                   0x18U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ispu_bdu                     : 1;
  uint8_t not_used0                    : 1;
  uint8_t loprio_user_trig             : 1;
  uint8_t reg_access_confirm           : 1;
  uint8_t ispu_rate                    : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ispu_rate                    : 4;
  uint8_t reg_access_confirm           : 1;
  uint8_t loprio_user_trig             : 1;
  uint8_t not_used0                    : 1;
  uint8_t ispu_bdu                     : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_ispu_ctrl0_t;

#define IIS3DWB10IS_ISPU_CTRL1                   0x19U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0                    : 4;
  uint8_t sw_reset_ispu                : 1;
  uint8_t timestamp_en                 : 1;
  uint8_t not_used1                    : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1                    : 2;
  uint8_t timestamp_en                 : 1;
  uint8_t sw_reset_ispu                : 1;
  uint8_t not_used0                    : 4;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_ispu_ctrl1_t;

#define IIS3DWB10IS_STATUS_A_SL                  0x1AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int_status_a_sl              : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int_status_a_sl              : 8;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_int_status_a_sl_t;

#define IIS3DWB10IS_STATUS_B_SL                  0x1BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int_status_b_sl              : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int_status_b_sl              : 8;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_int_status_b_sl_t;

#define IIS3DWB10IS_STATUS_REG                   0x1EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                         : 1;
  uint8_t qvarda                       : 1;
  uint8_t tda                          : 1;
  uint8_t sleepcnt_ia                  : 1;
  uint8_t ext_trig_ia                  : 1;
  uint8_t not_used0                    : 1;
  uint8_t ispu_ia                      : 1;
  uint8_t timestamp_endcount           : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t timestamp_endcount           : 1;
  uint8_t ispu_ia                      : 1;
  uint8_t not_used0                    : 1;
  uint8_t ext_trig_ia                  : 1;
  uint8_t sleepcnt_ia                  : 1;
  uint8_t tda                          : 1;
  uint8_t qvarda                       : 1;
  uint8_t xlda                         : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_status_reg_t;

#define IIS3DWB10IS_DEVICE_STATUS                0x21U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0                    : 6;
  uint8_t ispu_ctrl_access             : 1;
  uint8_t ispu_core_sleep              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ispu_core_sleep              : 1;
  uint8_t ispu_ctrl_access             : 1;
  uint8_t not_used0                    : 6;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_device_status_t;

#define IIS3DWB10IS_OUT_TEMP_L                   0x22U
#define IIS3DWB10IS_OUT_TEMP_H                   0x23U
#define IIS3DWB10IS_OUTX_L_A                     0x24U
#define IIS3DWB10IS_OUTX_M_A                     0x25U
#define IIS3DWB10IS_OUTX_H_A                     0x26U
#define IIS3DWB10IS_OUTX_HH_A                    0x27U
#define IIS3DWB10IS_OUTY_L_A                     0x28U
#define IIS3DWB10IS_OUTY_M_A                     0x29U
#define IIS3DWB10IS_OUTY_H_A                     0x2AU
#define IIS3DWB10IS_OUTY_HH_A                    0x2BU
#define IIS3DWB10IS_OUTZ_L_A                     0x2CU
#define IIS3DWB10IS_OUTZ_M_A                     0x2DU
#define IIS3DWB10IS_OUTZ_H_A                     0x2EU
#define IIS3DWB10IS_OUTZ_HH_A                    0x2FU
#define IIS3DWB10IS_TIMESTAMP0                   0x30U
#define IIS3DWB10IS_TIMESTAMP1                   0x31U
#define IIS3DWB10IS_TIMESTAMP2                   0x32U
#define IIS3DWB10IS_TIMESTAMP3                   0x33U
#define IIS3DWB10IS_TIMESTAMP4                   0x34U
#define IIS3DWB10IS_OUT_QVAR_L                   0x36U
#define IIS3DWB10IS_OUT_QVAR_H                   0x37U

#define IIS3DWB10IS_SLEEPCNT_CFG                 0x38U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tick_sel                     : 1;
  uint8_t enable_sleepcnt              : 1;
  uint8_t pl_sleepcnt                  : 1;
  uint8_t rst_sleepcnt                 : 1;
  uint8_t not_used0                    : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 4;
  uint8_t rst_sleepcnt                 : 1;
  uint8_t pl_sleepcnt                  : 1;
  uint8_t enable_sleepcnt              : 1;
  uint8_t tick_sel                     : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_sleepcnt_cfg_t;

#define IIS3DWB10IS_ISPU_LOPRIO_EN               0x39U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t loprio_fifo_en               : 1;
  uint8_t loprio_sleepcnt_en           : 1;
  uint8_t loprio_qvar_en               : 1;
  uint8_t loprio_ext_trg_en            : 1;
  uint8_t loprio_user_trg_en           : 1;
  uint8_t loprio_temp_en               : 1;
  uint8_t not_used0                    : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 2;
  uint8_t loprio_temp_en               : 1;
  uint8_t loprio_user_trg_en           : 1;
  uint8_t loprio_ext_trg_en            : 1;
  uint8_t loprio_qvar_en               : 1;
  uint8_t loprio_sleepcnt_en           : 1;
  uint8_t loprio_fifo_en               : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_ispu_loprio_en_t;

#define IIS3DWB10IS_SLEEPCNT_PL_L                0x3CU
#define IIS3DWB10IS_SLEEPCNT_PL_H                0x3DU
#define IIS3DWB10IS_SLEEPCNT_TH_L                0x3EU
#define IIS3DWB10IS_SLEEPCNT_TH_H                0x3FU

#define IIS3DWB10IS_FIFO_DATA_OUT_TAG            0x40U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tag_sensor                   : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t tag_sensor                   : 8;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_fifo_data_out_tag_t;

#define IIS3DWB10IS_FIFO_DATA_OUT_D0             0x41U
#define IIS3DWB10IS_FIFO_DATA_OUT_D1             0x42U
#define IIS3DWB10IS_FIFO_DATA_OUT_D2             0x43U
#define IIS3DWB10IS_FIFO_DATA_OUT_D3             0x44U
#define IIS3DWB10IS_FIFO_DATA_OUT_D4             0x45U
#define IIS3DWB10IS_FIFO_DATA_OUT_D5             0x46U
#define IIS3DWB10IS_FIFO_DATA_OUT_D6             0x47U
#define IIS3DWB10IS_FIFO_DATA_OUT_D7             0x48U
#define IIS3DWB10IS_FIFO_DATA_OUT_D8             0x49U

#define IIS3DWB10IS_FIFO_STATUS1                 0x4CU
typedef struct
{
  uint8_t diff_fifo                    : 8;
} iis3dwb10is_fifo_status1_t;

#define IIS3DWB10IS_FIFO_STATUS2                 0x4DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t diff_fifo                    : 4;
  uint8_t fifo_full_ia                 : 1;
  uint8_t not_used_01                  : 1;
  uint8_t fifo_ovr_ia                  : 1;
  uint8_t fifo_wtm_ia                  : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fifo_wtm_ia                  : 1;
  uint8_t fifo_ovr_ia                  : 1;
  uint8_t not_used_01                  : 1;
  uint8_t fifo_full_ia                 : 1;
  uint8_t diff_fifo                    : 4;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_fifo_status2_t;

#define IIS3DWB10IS_INTERNAL_FREQ_FINE           0x50U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t internal_freq_fine           : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t internal_freq_fine           : 8;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_internal_freq_fine_t;

#define IIS3DWB10IS_ISPU_DUMMYCFG1               0x52U
#define IIS3DWB10IS_ISPU_DUMMYCFG2               0x53U
#define IIS3DWB10IS_ISPU_DUMMYCFG3               0x54U
#define IIS3DWB10IS_ISPU_DUMMYCFG4               0x55U
#define IIS3DWB10IS_ISPU_DUMMYCFG5               0x56U
#define IIS3DWB10IS_ISPU_DUMMYCFG6               0x57U
#define IIS3DWB10IS_ISPU_DUMMYCFG7               0x58U
#define IIS3DWB10IS_ISPU_DUMMYCFG8               0x59U
#define IIS3DWB10IS_ISPU_DUMMYCFG9               0x5AU
#define IIS3DWB10IS_ISPU_DUMMYCFG10              0x5BU
#define IIS3DWB10IS_ISPU_DUMMYCFG11              0x5CU
#define IIS3DWB10IS_ISPU_DUMMYCFG12              0x5DU
#define IIS3DWB10IS_ISPU_DUMMYCFG13              0x5EU
#define IIS3DWB10IS_ISPU_DUMMYCFG14              0x5FU
#define IIS3DWB10IS_ISPU_DUMMYCFG15              0x60U
#define IIS3DWB10IS_ISPU_DUMMYCFG16              0x61U

#define IIS3DWB10IS_SLEEPCNT_TIME_L              0x62U
#define IIS3DWB10IS_SLEEPCNT_TIME_H              0x63U

#define IIS3DWB10IS_INT_CTRL3                    0x6EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_ispu_int_b              : 1;
  uint8_t int1_ispu_int_a              : 1;
  uint8_t int2_timer                   : 1;
  uint8_t int1_timer                   : 1;
  uint8_t not_used0                    : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 4;
  uint8_t int1_timer                   : 1;
  uint8_t int2_timer                   : 1;
  uint8_t int1_ispu_int_a              : 1;
  uint8_t int2_ispu_int_b              : 1;
#endif /* DRV_BYTE_ORDER */
} iis3dwb10is_int_ctrl3_t;

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

uint64_t iis3dwb10is_from_lsb_to_us(uint64_t lsb);
float_t iis3dwb10is_from_lsb_to_celsius(int16_t lsb);

float_t iis3dwb10is_16b_from_fs50g_to_mg(int16_t lsb);
float_t iis3dwb10is_16b_from_fs100g_to_mg(int16_t lsb);
float_t iis3dwb10is_16b_from_fs200g_to_mg(int16_t lsb);
float_t iis3dwb10is_from_fs50g_to_mg(int32_t lsb);
float_t iis3dwb10is_from_fs100g_to_mg(int32_t lsb);
float_t iis3dwb10is_from_fs200g_to_mg(int32_t lsb);

typedef enum
{
  IIS3DWB10IS_MAIN_MEM_BANK =                    0x0,
  IIS3DWB10IS_ISPU_MEM_BANK =                    0x3,
} iis3dwb10is_mem_bank_t;
int32_t iis3dwb10is_mem_bank_set(const stmdev_ctx_t *ctx, iis3dwb10is_mem_bank_t val);
int32_t iis3dwb10is_mem_bank_get(const stmdev_ctx_t *ctx, iis3dwb10is_mem_bank_t *val);

typedef struct
{
  uint8_t sw_por : 1; /* Complete reset of sensor */
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
    IIS3DWB10IS_ODR_80KHz      = 0x7,
  } odr;
} iis3dwb10is_data_rate_t;
int32_t iis3dwb10is_xl_data_rate_set(const stmdev_ctx_t *ctx, iis3dwb10is_data_rate_t val);
int32_t iis3dwb10is_xl_data_rate_get(const stmdev_ctx_t *ctx, iis3dwb10is_data_rate_t *val);

typedef enum
{
  IIS3DWB10IS_50g   = 0,
  IIS3DWB10IS_100g  = 1,
  IIS3DWB10IS_200g  = 2,
} iis3dwb10is_fs_xl_t;
int32_t iis3dwb10is_xl_full_scale_set(const stmdev_ctx_t *ctx, iis3dwb10is_fs_xl_t val);
int32_t iis3dwb10is_xl_full_scale_get(const stmdev_ctx_t *ctx, iis3dwb10is_fs_xl_t *val);

int32_t iis3dwb10is_block_data_update_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dwb10is_block_data_update_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef struct
{
  enum
  {
    IIS3DWB10IS_WRAPAROUND_DISABLED  = 0x0,
    IIS3DWB10IS_WRAPAROUND_1_EN      = 0x1,
    IIS3DWB10IS_WRAPAROUND_2_EN      = 0x2,
  } rounding;

  uint8_t x_axis_en : 1;
  uint8_t y_axis_en : 1;
  uint8_t z_axis_en : 1;
} iis3dwb10is_xl_data_cfg_t;
int32_t iis3dwb10is_xl_data_config_set(const stmdev_ctx_t *ctx, iis3dwb10is_xl_data_cfg_t val);
int32_t iis3dwb10is_xl_data_config_get(const stmdev_ctx_t *ctx, iis3dwb10is_xl_data_cfg_t *val);

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


typedef struct
{
  enum
  {
    IIS3DWB10IS_I3C_BUS_AVAIL_TIME_50US = 0x0,
    IIS3DWB10IS_I3C_BUS_AVAIL_TIME_2US  = 0x1,
  } bus_act_sel;
  uint8_t i3c_int_en                   : 1;
  uint8_t i3c_disable                  : 1;
  uint8_t sda_pu_en                    : 1;
} iis3dwb10is_i3c_cfg_t;
int32_t iis3dwb10is_i3c_configure_set(const stmdev_ctx_t *ctx, iis3dwb10is_i3c_cfg_t val);
int32_t iis3dwb10is_i3c_configure_get(const stmdev_ctx_t *ctx, iis3dwb10is_i3c_cfg_t *val);

typedef enum
{
  IIS3DWB10IS_SPI_4_WIRE  = 0x0, /* SPI 4 wires */
  IIS3DWB10IS_SPI_3_WIRE  = 0x1, /* SPI 3 wires */
} iis3dwb10is_spi_mode_t;
int32_t iis3dwb10is_spi_mode_set(const stmdev_ctx_t *ctx, iis3dwb10is_spi_mode_t val);
int32_t iis3dwb10is_spi_mode_get(const stmdev_ctx_t *ctx, iis3dwb10is_spi_mode_t *val);

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

typedef struct
{
  uint8_t batch_xl   : 1;
  uint8_t batch_temp : 1;
  uint8_t batch_qvar : 1;
  uint8_t batch_ispu : 1;
  enum
  {
    IIS3DWB10IS_TMSTMP_NOT_BATCHED = 0x0,
    IIS3DWB10IS_TMSTMP_DEC_1       = 0x1,
    IIS3DWB10IS_TMSTMP_DEC_8       = 0x2,
    IIS3DWB10IS_TMSTMP_DEC_32      = 0x3,
  } batch_ts;
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
  uint16_t fifo_level          : 12;
  uint8_t  fifo_full           : 1;
  uint8_t  fifo_ovr            : 1;
  uint8_t  fifo_th             : 1;
} iis3dwb10is_fifo_status_t;

int32_t iis3dwb10is_fifo_status_get(const stmdev_ctx_t *ctx, iis3dwb10is_fifo_status_t *val);

#define FIFO_ROW_LEN 10U

typedef struct
{
  enum
  {
    IIS3DWB10IS_TAG_EMPTY                 = 0x00,
    IIS3DWB10IS_TAG_QVAR                  = 0x08,
    IIS3DWB10IS_TAG_XL                    = 0x10,
    IIS3DWB10IS_TAG_TEMP                  = 0x18,
    IIS3DWB10IS_TAG_TS                    = 0x20,
    IIS3DWB10IS_TAG_TEMP_QVAR             = 0x28,
  } tag;

  uint8_t raw[FIFO_ROW_LEN - 1];
  struct
  {
    int32_t x_raw    : 20;
    int32_t y_raw    : 20;
    int32_t z_raw    : 20;
  } xl;
  int16_t temp_raw;
  int16_t qvar;
  uint64_t ts_raw;
} iis3dwb10is_fifo_out_raw_t;
int32_t iis3dwb10is_fifo_process(uint8_t *fifo_buf, iis3dwb10is_fifo_out_raw_t *val);

int32_t iis3dwb10is_fifo_out_raw_get(const stmdev_ctx_t *ctx, uint8_t *fifo_buf, uint16_t cnt);

int32_t iis3dwb10is_counter_odr_cfg_set(const stmdev_ctx_t *ctx, uint16_t val);
int32_t iis3dwb10is_counter_odr_cfg_get(const stmdev_ctx_t *ctx, uint16_t *val);

typedef struct
{
  uint8_t qvar_en              : 1;
  uint8_t qvar1_pad_en         : 1;
  uint8_t qvar2_pad_en         : 1;
  uint8_t qvar_switch          : 1;
  uint8_t lpf                  : 1;
  uint8_t hpf                  : 1;
  enum
  {
    IIS3DWB10IS_QVAR_GAIN_1X    = 0x0,
    IIS3DWB10IS_QVAR_GAIN_2X    = 0x1,
  } c_zin;
} iis3dwb10is_qvar_mode_t;
int32_t iis3dwb10is_qvar_mode_set(const stmdev_ctx_t *ctx, iis3dwb10is_qvar_mode_t val);
int32_t iis3dwb10is_qvar_mode_get(const stmdev_ctx_t *ctx, iis3dwb10is_qvar_mode_t *val);

typedef struct
{
  enum
  {
    IIS3DWB10IS_SLP_TICK_SLOW   = 0x0,
    IIS3DWB10IS_SLP_TICK_FAST    = 0x1,
  } tick_sel;
  uint8_t enable               : 1;
  uint8_t preload_en           : 1;
  uint8_t reset                : 1;
  uint16_t preload_val;
  uint16_t threshold_val;
} iis3dwb10is_slpcnt_cfg_t;
int32_t iis3dwb10is_sleepcnt_cfg_set(const stmdev_ctx_t *ctx, iis3dwb10is_slpcnt_cfg_t val);
int32_t iis3dwb10is_sleepcnt_cfg_get(const stmdev_ctx_t *ctx, iis3dwb10is_slpcnt_cfg_t *val);

int32_t iis3dwb10is_sleepcnt_time_set(const stmdev_ctx_t *ctx, uint16_t val);
int32_t iis3dwb10is_sleepcnt_time_get(const stmdev_ctx_t *ctx, uint16_t *val);

int32_t iis3dwb10is_burst_ui_trigger_set(const stmdev_ctx_t *ctx, uint8_t val);

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

typedef enum
{
  IIS3DWB10IS_DRDY_LEVEL   = 0x0,
  IIS3DWB10IS_DRDY_PULSED  = 0x1,
} iis3dwb10is_data_ready_mode_t;
int32_t iis3dwb10is_data_ready_mode_set(const stmdev_ctx_t *ctx, iis3dwb10is_data_ready_mode_t val);
int32_t iis3dwb10is_data_ready_mode_get(const stmdev_ctx_t *ctx,
                                        iis3dwb10is_data_ready_mode_t *val);

typedef struct
{
  uint8_t int_drdy_xl          : 1;
  uint8_t int_drdy_qvar        : 1;
  uint8_t int_sleep_cnt        : 1;
  uint8_t int_ext_trig         : 1;
  uint8_t int_fifo_th          : 1;
  uint8_t int_fifo_ovr         : 1;
  uint8_t int_fifo_full        : 1;
  uint8_t int_drdy_temp        : 1;
  uint8_t int_boot             : 1;
  uint8_t int_sleep_ispu       : 1;
  uint8_t int2_on_int1         : 1;
} iis3dwb10is_pin_int_route_t;
int32_t iis3dwb10is_pin_int1_route_set(const stmdev_ctx_t *ctx,
                                       iis3dwb10is_pin_int_route_t val);
int32_t iis3dwb10is_pin_int1_route_get(const stmdev_ctx_t *ctx,
                                       iis3dwb10is_pin_int_route_t *val);
int32_t iis3dwb10is_pin_int2_route_set(const stmdev_ctx_t *ctx,
                                       iis3dwb10is_pin_int_route_t val);
int32_t iis3dwb10is_pin_int2_route_get(const stmdev_ctx_t *ctx,
                                       iis3dwb10is_pin_int_route_t *val);

typedef struct
{
  uint8_t drdy_xl              : 1;
  uint8_t drdy_qvar            : 1;
  uint8_t drdy_temp            : 1;
  uint8_t sleepcnt_ia          : 1;
  uint8_t ext_trig_ia          : 1;
  uint8_t ispu_ia              : 1;
  uint8_t timestamp_endcount   : 1;
} iis3dwb10is_data_ready_t;
int32_t iis3dwb10is_data_ready_get(const stmdev_ctx_t *ctx, iis3dwb10is_data_ready_t *val);

int32_t iis3dwb10is_temperature_raw_get(const stmdev_ctx_t *ctx, int16_t *val);
int32_t iis3dwb10is_acceleration_16b_raw_get(const stmdev_ctx_t *ctx, int16_t *val);
int32_t iis3dwb10is_acceleration_raw_get(const stmdev_ctx_t *ctx, int32_t *val);

int32_t iis3dwb10is_ispu_reset(const stmdev_ctx_t *ctx);
int32_t iis3dwb10is_ispu_timestamp_en_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dwb10is_ispu_timestamp_en_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  IIS3DWB10IS_ISPU_RATE_IDLE  = 0x0,
  IIS3DWB10IS_ISPU_RATE_2KHz5 = 0x2,
  IIS3DWB10IS_ISPU_RATE_5KHz  = 0x3,
  IIS3DWB10IS_ISPU_RATE_10KHz = 0x4,
  IIS3DWB10IS_ISPU_RATE_20KHz = 0x5,
  IIS3DWB10IS_ISPU_RATE_40KHz = 0x6,
  IIS3DWB10IS_ISPU_RATE_80KHz = 0x7,
} iis3dwb10is_ispu_rate_t;
int32_t iis3dwb10is_ispu_rate_set(const stmdev_ctx_t *ctx, iis3dwb10is_ispu_rate_t val);
int32_t iis3dwb10is_ispu_rate_get(const stmdev_ctx_t *ctx, iis3dwb10is_ispu_rate_t *val);

int32_t iis3dwb10is_ispu_bdu_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dwb10is_ispu_bdu_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dwb10is_grant_ispu_regs_access_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t iis3dwb10is_grant_ispu_regs_access_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t iis3dwb10is_ispu_trigger_interrupt(const stmdev_ctx_t *ctx, uint8_t val);

typedef struct
{
  uint8_t status_a_sl;
  uint8_t status_b_sl;
} iis3dwb10is_ispu_status_t;

int32_t iis3dwb10is_ispu_status_get(const stmdev_ctx_t *ctx, iis3dwb10is_ispu_status_t *val);

typedef struct
{
  uint8_t ctrl_access          : 1;
  uint8_t core_sleep           : 1;
} iis3dwb10is_ispu_dev_status_t;

int32_t iis3dwb10is_ispu_dev_status_get(const stmdev_ctx_t *ctx,
                                        iis3dwb10is_ispu_dev_status_t *val);

typedef struct
{
  uint8_t fifo_en             : 1;
  uint8_t sleepcnt_en         : 1;
  uint8_t qvar_en             : 1;
  uint8_t ext_trg_en          : 1;
  uint8_t user_trg_en         : 1;
  uint8_t temp_en             : 1;
} iis3dwb10is_ispu_loprio_cfg_t;

int32_t iis3dwb10is_ispu_loprio_cfg_set(const stmdev_ctx_t *ctx,
                                        iis3dwb10is_ispu_loprio_cfg_t val);
int32_t iis3dwb10is_ispu_loprio_cfg_get(const stmdev_ctx_t *ctx,
                                        iis3dwb10is_ispu_loprio_cfg_t *val);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* IIS3DWB10IS_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
