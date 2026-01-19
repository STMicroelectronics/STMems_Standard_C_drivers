/*
 ******************************************************************************
 * @file    asm330ab1_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          asm330ab1_reg.c driver.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ASM330AB1_REGS_H
#define ASM330AB1_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup ASM330AB1
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

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*stmdev_write_ptr)(void *ctx, uint8_t reg, const uint8_t *data, uint16_t len);
typedef int32_t (*stmdev_read_ptr)(void *ctx, uint8_t reg, uint8_t *data, uint16_t len);
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

  /** private data **/
  void *priv_data;
} stmdev_ctx_t;

typedef struct
{
  uint8_t use_safespi_bus               : 1;
  uint8_t reserved_1                    : 7;
} asm330ab1_priv_t;

/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#define ASM330AB1_ID                                   0x74U

/** @addtogroup  Common Page
  * @{
  *
  */

#define ASM330AB1_PAGE_SEL                             0x00U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t page_sel                 : 5;
  uint8_t sw_por                   : 1;
  uint8_t lock                     : 1;
  uint8_t eoi                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t eoi                      : 1;
  uint8_t lock                     : 1;
  uint8_t sw_por                   : 1;
  uint8_t page_sel                 : 5;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_page_sel_t;

#define ASM330AB1_PIN_CTRL0                            0x01U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t scl_sda_pd_dis               : 1;
  uint8_t int1_pd_dis                  : 1;
  uint8_t int2_pd_dis                  : 1;
  uint8_t int3_pd_dis                  : 1;
  uint8_t not_used                     : 1;
  uint8_t scl_sda_pu_en                : 1;
  uint8_t cs_pu_dis                    : 1;
  uint8_t ssd_pu_dis                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t ssd_pu_dis                   : 1;
  uint8_t cs_pu_dis                    : 1;
  uint8_t scl_sda_pu_en                : 1;
  uint8_t not_used                     : 1;
  uint8_t int3_pd_dis                  : 1;
  uint8_t int2_pd_dis                  : 1;
  uint8_t int1_pd_dis                  : 1;
  uint8_t scl_sda_pd_dis               : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_pin_ctrl0_t;

#define ASM330AB1_PIN_CTRL1                            0x02U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0                    : 2;
  uint8_t sdo_drive                    : 2;
  uint8_t pu_dis_ta0                   : 1;
  uint8_t ibhr_por_en                  : 1;
  uint8_t sdo_pu_en                    : 1;
  uint8_t not_used1                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1                    : 1;
  uint8_t sdo_pu_en                    : 1;
  uint8_t ibhr_por_en                  : 1;
  uint8_t pu_dis_ta0                   : 1;
  uint8_t sdo_drive                    : 2;
  uint8_t not_used0                    : 2;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_pin_ctrl1_t;

#define ASM330AB1_CRC_W                                0x04U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t crc_w                        : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t crc_w                        : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_crc_w_t;

#define ASM330AB1_CRC_R                                0x05U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t crc_r                        : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t crc_r                        : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_crc_r_t;

#define ASM330AB1_TEST_IF                              0x07U

#define ASM330AB1_INT3_CTRL                            0x0CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int3_drdy_xl                 : 1;
  uint8_t int3_drdy_g                  : 1;
  uint8_t int3_drdy_temp               : 1;
  uint8_t int3_drdy_fusa               : 1;
  uint8_t fault_out_sel                : 2;
  uint8_t not_used0                    : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 2;
  uint8_t fault_out_sel                : 2;
  uint8_t int3_drdy_fusa               : 1;
  uint8_t int3_drdy_temp               : 1;
  uint8_t int3_drdy_g                  : 1;
  uint8_t int3_drdy_xl                 : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_int3_ctrl_t;

#define ASM330AB1_INT1_CTRL                            0x0DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int1_drdy_xl                 : 1;
  uint8_t int1_drdy_g                  : 1;
  uint8_t int1_drdy_temp               : 1;
  uint8_t int2_on_int1                 : 1;
  uint8_t int1_drdy_fusa               : 1;
  uint8_t not_used0                    : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 3;
  uint8_t int1_drdy_fusa               : 1;
  uint8_t int2_on_int1                 : 1;
  uint8_t int1_drdy_temp               : 1;
  uint8_t int1_drdy_g                  : 1;
  uint8_t int1_drdy_xl                 : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_int1_ctrl_t;

#define ASM330AB1_INT2_CTRL                            0x0EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_drdy_xl                 : 1;
  uint8_t int2_drdy_g                  : 1;
  uint8_t int2_drdy_temp               : 1;
  uint8_t int2_in_lh                   : 1;
  uint8_t int2_timestamp               : 1;
  uint8_t int2_cap_en                  : 1;
  uint8_t not_used0                    : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 2;
  uint8_t int2_cap_en                  : 1;
  uint8_t int2_timestamp               : 1;
  uint8_t int2_in_lh                   : 1;
  uint8_t int2_drdy_temp               : 1;
  uint8_t int2_drdy_g                  : 1;
  uint8_t int2_drdy_xl                 : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_int2_ctrl_t;

#define ASM330AB1_WHO_AM_I                             0x0FU

#define ASM330AB1_CTRL1                                0x10U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t odr_xl                   : 4;
  uint8_t op_mode_xl               : 3;
  uint8_t not_used                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used                 : 1;
  uint8_t op_mode_xl               : 3;
  uint8_t odr_xl                   : 4;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_ctrl1_t;

#define ASM330AB1_CTRL2                                0x11U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t odr_g                    : 4;
  uint8_t op_mode_g                : 3;
  uint8_t not_used                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used                 : 1;
  uint8_t op_mode_g                : 3;
  uint8_t odr_g                    : 4;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_ctrl2_t;

#define ASM330AB1_CTRL3                                0x12U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sw_reset                 : 1;
  uint8_t timestamp_en             : 1;
  uint8_t if_inc                   : 1;
  uint8_t spi_crc_en               : 1;
  uint8_t global_fusa_en           : 1;
  uint8_t timestamp_ext_en         : 1;
  uint8_t bdu                      : 1;
  uint8_t boot                     : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                     : 1;
  uint8_t bdu                      : 1;
  uint8_t timestamp_ext_en         : 1;
  uint8_t global_fusa_en           : 1;
  uint8_t spi_crc_en               : 1;
  uint8_t if_inc                   : 1;
  uint8_t timestamp_en             : 1;
  uint8_t sw_reset                 : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_ctrl3_t;

#define ASM330AB1_CTRL4                                0x13U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t protocol_ibi_en          : 1;
  uint8_t drdy_pulsed              : 1;
  uint8_t not_used0                : 1;
  uint8_t drdy_mask                : 1;
  uint8_t not_used1                : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1                : 4;
  uint8_t drdy_mask                : 1;
  uint8_t not_used0                : 1;
  uint8_t drdy_pulsed              : 1;
  uint8_t protocol_ibi_en          : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_ctrl4_t;

#define ASM330AB1_CTRL5                                0x14U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int_en_i3c               : 1;
  uint8_t bus_act_sel              : 2;
  uint8_t not_used0                : 3;
  uint8_t i3c_crc_en               : 1;
  uint8_t i3c_line_rel             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t i3c_line_rel             : 1;
  uint8_t i3c_crc_en               : 1;
  uint8_t not_used0                : 3;
  uint8_t bus_act_sel              : 2;
  uint8_t int_en_i3c               : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_ctrl5_t;

#define ASM330AB1_CTRL6                                0x15U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fs_g                     : 3;
  uint8_t lpf1_g_bw                : 4;
  uint8_t not_used                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used                 : 1;
  uint8_t lpf1_g_bw                : 4;
  uint8_t fs_g                     : 3;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_ctrl6_t;

#define ASM330AB1_CTRL7                                0x16U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lpf1_g_en                : 1;
  uint8_t not_used                 : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used                 : 7;
  uint8_t lpf1_g_en                : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_ctrl7_t;

#define ASM330AB1_CTRL8                                0x17U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fs_xl                    : 2;
  uint8_t not_used                 : 3;
  uint8_t lpf2_xl_bw               : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t lpf2_xl_bw               : 3;
  uint8_t not_used                 : 3;
  uint8_t fs_xl                    : 2;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_ctrl8_t;

#define ASM330AB1_CTRL9                                0x18U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t usr_on_off_out           : 1;
  uint8_t usr_off_w                : 1;
  uint8_t not_used0                : 1;
  uint8_t lpf2_xl_en               : 1;
  uint8_t not_used1                : 1;
  uint8_t xl_fastsettl_mode        : 1;
  uint8_t not_used2                : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used2                : 2;
  uint8_t xl_fastsettl_mode        : 1;
  uint8_t not_used1                : 1;
  uint8_t lpf2_xl_en               : 1;
  uint8_t not_used0                : 1;
  uint8_t usr_off_w                : 1;
  uint8_t usr_on_off_out           : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_ctrl9_t;

#define ASM330AB1_CTRL10                               0x19U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t st_xl                    : 2;
  uint8_t st_g                     : 2;
  uint8_t not_used0                : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                : 4;
  uint8_t st_g                     : 2;
  uint8_t st_xl                    : 2;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_ctrl10_t;

#define ASM330AB1_DEVICE_STATUS                        0x1AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t boot_check               : 1;
  uint8_t gy_startup               : 1;
  uint8_t not_used0                : 2;
  uint8_t cap_ack                  : 1;
  uint8_t xl_startup               : 1;
  uint8_t timestamp_ext_ack        : 1;
  uint8_t not_used1                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1                : 1;
  uint8_t timestamp_ext_ack        : 1;
  uint8_t xl_startup               : 1;
  uint8_t cap_ack                  : 1;
  uint8_t not_used0                : 2;
  uint8_t gy_startup               : 1;
  uint8_t boot_check               : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_device_status_t;

#define ASM330AB1_FUSA_STATUS                          0x1DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl_fusa_status           : 1;
  uint8_t gy_fusa_status           : 1;
  uint8_t temp_fusa_status         : 1;
  uint8_t com_if_fusa_status       : 1;
  uint8_t xl_fusa_range            : 1;
  uint8_t gy_fusa_range            : 1;
  uint8_t lock_fusa_status         : 1;
  uint8_t eoi_fusa_status          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t eoi_fusa_status          : 1;
  uint8_t lock_fusa_status         : 1;
  uint8_t gy_fusa_range            : 1;
  uint8_t xl_fusa_range            : 1;
  uint8_t com_if_fusa_status       : 1;
  uint8_t temp_fusa_status         : 1;
  uint8_t gy_fusa_status           : 1;
  uint8_t xl_fusa_status           : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_fusa_status_t;

#define ASM330AB1_STATUS                               0x1EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                         : 1;
  uint8_t gda                          : 1;
  uint8_t tda                          : 1;
  uint8_t not_used0                    : 4;
  uint8_t timestamp_endcount           : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t timestamp_endcount           : 1;
  uint8_t not_used0                    : 4;
  uint8_t tda                          : 1;
  uint8_t gda                          : 1;
  uint8_t xlda                         : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_status_t;

#define ASM330AB1_OUT_TEMP_L                           0x20U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t temp                         : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t temp                         : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_out_temp_l_t;

#define ASM330AB1_OUT_TEMP_H                           0x21U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t temp                         : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t temp                         : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_out_temp_h_t;

#define ASM330AB1_OUTX_L_G                             0x22U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outx_g                       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outx_g                       : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_outx_l_g_t;

#define ASM330AB1_OUTX_H_G                             0x23U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outx_g                       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outx_g                       : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_outx_h_g_t;

#define ASM330AB1_OUTY_L_G                             0x24U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outy_g                       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outy_g                       : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_outy_l_g_t;

#define ASM330AB1_OUTY_H_G                             0x25U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outy_g                       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outy_g                       : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_outy_h_g_t;

#define ASM330AB1_OUTZ_L_G                             0x26U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outz_g                       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outz_g                       : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_outz_l_g_t;

#define ASM330AB1_OUTZ_H_G                             0x27U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outz_g                       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outz_g                       : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_outz_h_g_t;

#define ASM330AB1_OUTX_L_A                             0x28U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outx_a                       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outx_a                       : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_outx_l_a_t;

#define ASM330AB1_OUTX_H_A                             0x29U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outx_a                       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outx_a                       : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_outx_h_a_t;

#define ASM330AB1_OUTY_L_A                             0x2AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outy_a                       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outy_a                       : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_outy_l_a_t;

#define ASM330AB1_OUTY_H_A                             0x2BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outy_a                       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outy_a                       : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_outy_h_a_t;

#define ASM330AB1_OUTZ_L_A                             0x2CU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outz_a                       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outz_a                       : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_outz_l_a_t;

#define ASM330AB1_OUTZ_H_A                             0x2DU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outz_a                       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outz_a                       : 8;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_outz_h_a_t;

/* Timestamp at last data ready event */
#define ASM330AB1_TIMESTAMP_ODR0                       0x2EU
#define ASM330AB1_TIMESTAMP_ODR1                       0x2FU
#define ASM330AB1_TIMESTAMP_ODR2                       0x30U
#define ASM330AB1_TIMESTAMP_ODR3                       0x31U

/**
  * @}
  *
  */

/** @addtogroup  Main Page
  * @{
  *
  */
#define ASM330AB1_MAIN_PAGE                            0x0

#define ASM330AB1_HAODR_CFG                            0x62
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t haodr_sel                : 2;
  uint8_t not_used                 : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used                 : 6;
  uint8_t haodr_sel                : 2;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_haodr_cfg_t;

/* Timestamp at first data output register */
#define ASM330AB1_TIMESTAMP0                           0x40U
#define ASM330AB1_TIMESTAMP1                           0x41U
#define ASM330AB1_TIMESTAMP2                           0x42U
#define ASM330AB1_TIMESTAMP3                           0x43U

/* Timestamp at last external trigger event */
#define ASM330AB1_TIMESTAMP_EXT0                       0x44U
#define ASM330AB1_TIMESTAMP_EXT1                       0x45U
#define ASM330AB1_TIMESTAMP_EXT2                       0x46U
#define ASM330AB1_TIMESTAMP_EXT3                       0x47U

/**
  * @}
  *
  */

/** @addtogroup  Fu.Sa. Page
  * @{
  *
  */
#define ASM330AB1_FUSA_PAGE                            0x7

#define ASM330AB1_CLEAR                                0x32
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t group                    : 3;
  uint8_t not_used                 : 5;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used                 : 5;
  uint8_t group                    : 3;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_clear_t;

#define ASM330AB1_SUM_STATUS                           0x33
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl_status_x              : 1;
  uint8_t xl_status_y              : 1;
  uint8_t xl_status_z              : 1;
  uint8_t gy_status_x              : 1;
  uint8_t gy_status_y              : 1;
  uint8_t gy_status_z              : 1;
  uint8_t if_status                : 1;
  uint8_t com_status               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t com_status               : 1;
  uint8_t if_status                : 1;
  uint8_t gy_status_z              : 1;
  uint8_t gy_status_y              : 1;
  uint8_t gy_status_x              : 1;
  uint8_t xl_status_z              : 1;
  uint8_t xl_status_y              : 1;
  uint8_t xl_status_x              : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_sum_status_t;

#define ASM330AB1_SUM_RANGE                            0x34
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl_range_x               : 1;
  uint8_t xl_range_y               : 1;
  uint8_t xl_range_z               : 1;
  uint8_t gy_range_x               : 1;
  uint8_t gy_range_y               : 1;
  uint8_t gy_range_z               : 1;
  uint8_t not_used                 : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used                 : 2;
  uint8_t gy_range_z               : 1;
  uint8_t gy_range_y               : 1;
  uint8_t gy_range_x               : 1;
  uint8_t xl_range_z               : 1;
  uint8_t xl_range_y               : 1;
  uint8_t xl_range_x               : 1;
#endif /* DRV_BYTE_ORDER */
} asm330ab1_sum_range_t;

#define ASM330AB1_RESERVED_PAGE_1                      0x01
#define ASM330AB1_RESERVED_PAGE_16                     0x10
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
int32_t __weak asm330ab1_read_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                                  uint8_t *data,
                                  uint16_t len);
int32_t __weak asm330ab1_write_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                                   uint8_t *data,
                                   uint16_t len);

float_t asm330ab1_from_fs2g_to_mg(int16_t lsb);
float_t asm330ab1_from_fs4g_to_mg(int16_t lsb);
float_t asm330ab1_from_fs8g_to_mg(int16_t lsb);
float_t asm330ab1_from_fs16g_to_mg(int16_t lsb);

float_t asm330ab1_from_fs125_to_mdps(int16_t lsb);
float_t asm330ab1_from_fs250_to_mdps(int16_t lsb);
float_t asm330ab1_from_fs500_to_mdps(int16_t lsb);
float_t asm330ab1_from_fs1000_to_mdps(int16_t lsb);
float_t asm330ab1_from_fs2000_to_mdps(int16_t lsb);

float_t asm330ab1_from_lsb_to_celsius(int16_t lsb);

int32_t asm330ab1_device_id_get(const stmdev_ctx_t *ctx, uint8_t *id);

int32_t asm330ab1_reboot(const stmdev_ctx_t *ctx);
int32_t asm330ab1_sw_por(const stmdev_ctx_t *ctx);
int32_t asm330ab1_sw_reset(const stmdev_ctx_t *ctx);

int32_t asm330ab1_page_sel_set(const stmdev_ctx_t *ctx, uint8_t page);
int32_t asm330ab1_page_sel_get(const stmdev_ctx_t *ctx, uint8_t *page);

int32_t asm330ab1_eoi_set(const stmdev_ctx_t *ctx);
int32_t asm330ab1_pages_lock(const stmdev_ctx_t *ctx, uint8_t val);

int32_t asm330ab1_check_spi_communication(const stmdev_ctx_t *ctx);

int32_t asm330ab1_sensor_power_down(const stmdev_ctx_t *ctx);
int32_t asm330ab1_sensor_start_up(const stmdev_ctx_t *ctx);

int32_t asm330ab1_check_faults(const stmdev_ctx_t *ctx);

int32_t asm330ab1_fusa_fault_clear(const stmdev_ctx_t *ctx, uint8_t group);

typedef struct
{
  uint16_t xl_status_x                 : 1;
  uint16_t xl_status_y                 : 1;
  uint16_t xl_status_z                 : 1;
  uint16_t gy_status_x                 : 1;
  uint16_t gy_status_y                 : 1;
  uint16_t gy_status_z                 : 1;
  uint16_t if_status                   : 1;
  uint16_t com_status                  : 1;
  uint16_t xl_range_x                  : 1;
  uint16_t xl_range_y                  : 1;
  uint16_t xl_range_z                  : 1;
  uint16_t gy_range_x                  : 1;
  uint16_t gy_range_y                  : 1;
  uint16_t gy_range_z                  : 1;
} asm330ab1_fusa_faults_t;
int32_t asm330ab1_fusa_status_read(const stmdev_ctx_t *ctx, asm330ab1_fusa_faults_t *status);

typedef enum
{
  ASM330AB1_SDO_DRIVE_10PF_14PF    = 0x0,
  ASM330AB1_SDO_DRIVE_15PF_29PF    = 0x2,
  ASM330AB1_SDO_DRIVE_20PF_54PF    = 0x1,
  ASM330AB1_SDO_DRIVE_55PF_100PF   = 0x3,
} asm330ab1_sdo_drive_val_t;

typedef struct
{
  asm330ab1_sdo_drive_val_t sdo_drive;

  uint16_t scl_sda_pd_dis              : 1; /* 1 = pull down disconnected */
  uint16_t int1_pd_dis                 : 1; /* 1 = pull down disconnected */
  uint16_t int2_pd_dis                 : 1; /* 1 = pull down disconnected */
  uint16_t int3_pd_dis                 : 1; /* 1 = pull down disconnected */
  uint16_t scl_sda_pu_en               : 1; /* 1 = pull up enable */
  uint16_t cs_pu_dis                   : 1; /* 1 = pull up disconnected */
  uint16_t ssd_pu_dis                  : 1; /* 1 = pull up disconnected */
  uint16_t pu_dis_ta0                  : 1; /* 1 = pull up disconnected */
  uint16_t ibhr_por_en                 : 1; /* 1 = global reset */
  uint16_t sdo_pu_en                   : 1; /* 1 = pull up enable */
} asm330ab1_pin_conf_t;
int32_t asm330ab1_pin_conf_set(const stmdev_ctx_t *ctx, const asm330ab1_pin_conf_t *val);
int32_t asm330ab1_pin_conf_get(const stmdev_ctx_t *ctx, asm330ab1_pin_conf_t *val);

int32_t asm330ab1_crc_spi_write_ops(const stmdev_ctx_t *ctx, uint8_t *crc);
int32_t asm330ab1_crc_spi_read_ops(const stmdev_ctx_t *ctx, uint8_t *crc);

typedef enum
{
  ASM330AB1_HA00_ODR_POWER_DOWN = 0x00, /* high_accuracy = 0x0 */
  ASM330AB1_HA00_ODR_AT_15Hz    = 0x03,
  ASM330AB1_HA00_ODR_AT_30Hz    = 0x04,
  ASM330AB1_HA00_ODR_AT_60Hz    = 0x05,
  ASM330AB1_HA00_ODR_AT_120Hz   = 0x06,
  ASM330AB1_HA00_ODR_AT_240Hz   = 0x07,
  ASM330AB1_HA00_ODR_AT_480Hz   = 0x08,
  ASM330AB1_HA00_ODR_AT_960Hz   = 0x09,
  ASM330AB1_HA00_ODR_AT_1920Hz  = 0x0A,
  ASM330AB1_HA00_ODR_AT_3840Hz  = 0x0B,

  ASM330AB1_HA02_ODR_POWER_DOWN = 0x20, /* high_accuracy = 0x2 */
  ASM330AB1_HA02_ODR_AT_12Hz5   = 0x23,
  ASM330AB1_HA02_ODR_AT_25Hz    = 0x24,
  ASM330AB1_HA02_ODR_AT_50Hz    = 0x25,
  ASM330AB1_HA02_ODR_AT_100Hz   = 0x26,
  ASM330AB1_HA02_ODR_AT_200Hz   = 0x27,
  ASM330AB1_HA02_ODR_AT_400Hz   = 0x28,
  ASM330AB1_HA02_ODR_AT_800Hz   = 0x29,
  ASM330AB1_HA02_ODR_AT_1600Hz  = 0x2A,
  ASM330AB1_HA02_ODR_AT_3200Hz  = 0x2B,

  ASM330AB1_HA03_ODR_POWER_DOWN = 0x30, /* high_accuracy = 0x3 */
  ASM330AB1_HA03_ODR_AT_13Hz    = 0x33,
  ASM330AB1_HA03_ODR_AT_26Hz    = 0x34,
  ASM330AB1_HA03_ODR_AT_52Hz    = 0x35,
  ASM330AB1_HA03_ODR_AT_104Hz   = 0x36,
  ASM330AB1_HA03_ODR_AT_208Hz   = 0x37,
  ASM330AB1_HA03_ODR_AT_417Hz   = 0x38,
  ASM330AB1_HA03_ODR_AT_833Hz   = 0x39,
  ASM330AB1_HA03_ODR_AT_1667Hz  = 0x3A,
  ASM330AB1_HA03_ODR_AT_3333Hz  = 0x3B,
} asm330ab1_odr_t;
int32_t asm330ab1_xl_data_rate_set(const stmdev_ctx_t *ctx, asm330ab1_odr_t val);
int32_t asm330ab1_xl_data_rate_get(const stmdev_ctx_t *ctx, asm330ab1_odr_t *val);
int32_t asm330ab1_gy_data_rate_set(const stmdev_ctx_t *ctx, asm330ab1_odr_t val);
int32_t asm330ab1_gy_data_rate_get(const stmdev_ctx_t *ctx, asm330ab1_odr_t *val);

#define ASM330AB1_HA_MODE 1 /* op_mode_xl and op_mode_g are always set to this */

typedef enum
{
  ASM330AB1_HA03_ODR_0  = 0x0,
  ASM330AB1_HA03_ODR_2  = 0x2,
  ASM330AB1_HA03_ODR_3  = 0x3,
} asm330ab1_haodr_cfg_val_t;
int32_t asm330ab1_haodr_cfg_set(const stmdev_ctx_t *ctx, asm330ab1_haodr_cfg_val_t val);
int32_t asm330ab1_haodr_cfg_get(const stmdev_ctx_t *ctx, asm330ab1_haodr_cfg_val_t *val);

typedef enum
{
  ASM330AB1_2g  = 0x0,
  ASM330AB1_4g  = 0x1,
  ASM330AB1_8g  = 0x2,
  ASM330AB1_16g = 0x3,
} asm330ab1_xl_full_scale_t;
int32_t asm330ab1_xl_full_scale_set(const stmdev_ctx_t *ctx, asm330ab1_xl_full_scale_t val);
int32_t asm330ab1_xl_full_scale_get(const stmdev_ctx_t *ctx, asm330ab1_xl_full_scale_t *val);

typedef enum
{
  ASM330AB1_125dps  = 0x0,
  ASM330AB1_250dps  = 0x1,
  ASM330AB1_500dps  = 0x2,
  ASM330AB1_1000dps = 0x3,
  ASM330AB1_2000dps = 0x4,
} asm330ab1_gy_full_scale_t;
int32_t asm330ab1_gy_full_scale_set(const stmdev_ctx_t *ctx, asm330ab1_gy_full_scale_t val);
int32_t asm330ab1_gy_full_scale_get(const stmdev_ctx_t *ctx, asm330ab1_gy_full_scale_t *val);

typedef struct
{
  uint8_t drdy_xl                      : 1;
  uint8_t drdy_g                       : 1;
  uint8_t drdy_temp                    : 1;
  uint8_t drdy_fusa                    : 1;
  uint8_t timestamp                    : 1;
  uint8_t in_lh                        : 1;
  uint8_t cap_en                       : 1;
} asm330ab1_pin_int_route_t;

int32_t asm330ab1_pin_int1_route_set(const stmdev_ctx_t *ctx, asm330ab1_pin_int_route_t *val);
int32_t asm330ab1_pin_int1_route_get(const stmdev_ctx_t *ctx, asm330ab1_pin_int_route_t *val);
int32_t asm330ab1_pin_int2_route_set(const stmdev_ctx_t *ctx, asm330ab1_pin_int_route_t *val);
int32_t asm330ab1_pin_int2_route_get(const stmdev_ctx_t *ctx, asm330ab1_pin_int_route_t *val);
int32_t asm330ab1_pin_int3_route_set(const stmdev_ctx_t *ctx, asm330ab1_pin_int_route_t *val);
int32_t asm330ab1_pin_int3_route_get(const stmdev_ctx_t *ctx, asm330ab1_pin_int_route_t *val);

typedef struct
{
  enum
  {
    ASM330AB1_DRDY_LATCHED = 0x0,
    ASM330AB1_DRDY_PULSED  = 0x1,
  } mode;
  enum
  {
    ASM330AB1_DRDY_NOT_MASKED = 0x0,
    ASM330AB1_DRDY_MASKED     = 0x1,
  } mask;
} asm330ab1_data_ready_mode_t;
int32_t asm330ab1_data_ready_mode_set(const stmdev_ctx_t *ctx, asm330ab1_data_ready_mode_t val);
int32_t asm330ab1_data_ready_mode_get(const stmdev_ctx_t *ctx, asm330ab1_data_ready_mode_t *val);

int32_t asm330ab1_temperature_raw_get(const stmdev_ctx_t *ctx, int16_t *val);

int32_t asm330ab1_angular_rate_raw_get(const stmdev_ctx_t *ctx, int16_t *val);

int32_t asm330ab1_acceleration_raw_get(const stmdev_ctx_t *ctx, int16_t *val);

int32_t asm330ab1_timestamp_enable(const stmdev_ctx_t *ctx, uint8_t val);
int32_t asm330ab1_timestamp_us_get(const stmdev_ctx_t *ctx, uint64_t *val);
int32_t asm330ab1_timestamp_odr_us_get(const stmdev_ctx_t *ctx, uint64_t *val);
int32_t asm330ab1_timestamp_ext_us_get(const stmdev_ctx_t *ctx, uint64_t *val);

int32_t asm330ab1_device_status_get(const stmdev_ctx_t *ctx, asm330ab1_device_status_t *status);
int32_t asm330ab1_status_get(const stmdev_ctx_t *ctx, asm330ab1_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* ASM330AB1_REGS_H */
