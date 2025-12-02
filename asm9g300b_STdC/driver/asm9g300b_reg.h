/*
 ******************************************************************************
 * @file    asm9g300b_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          asm9g300b_reg.c driver.
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
#ifndef ASM9G300B_REGS_H
#define ASM9G300B_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup ASM9G300B
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

/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#define ASM9G300B_ARSX                                 0x01
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t reg_arsx                            : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t reg_arsx                            : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_arsx_t;

#define ASM9G300B_ARSY                                 0x02
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t reg_arsy                            : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t reg_arsy                            : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_arsy_t;

#define ASM9G300B_ARSZ                                 0x03
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t reg_arsz                            : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t reg_arsz                            : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_arsz_t;

#define ASM9G300B_ACCX                                 0x04
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t reg_accx                            : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t reg_accx                            : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_accx_t;

#define ASM9G300B_ACCY                                 0x05
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t reg_accy                            : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t reg_accy                            : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_accy_t;

#define ASM9G300B_ACCZ                                 0x06
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t reg_accz                            : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t reg_accz                            : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_accz_t;

#define ASM9G300B_TEMP                                 0x07
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t reg_temp                            : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t reg_temp                            : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_temp_t;

#define ASM9G300B_MGPX                                 0x08
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t reg_mgpx                            : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t reg_mgpx                            : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_mgpx_t;

#define ASM9G300B_MGPY                                 0x09
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t reg_mgpy                            : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t reg_mgpy                            : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_mgpy_t;

#define ASM9G300B_MGPZ                                 0x0A
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t reg_mgpz                            : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t reg_mgpz                            : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_mgpz_t;

#define ASM9G300B_SSUMOK                               0x0E
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t s_ok_arsx                           : 1;
  uint16_t s_ok_arsy                           : 1;
  uint16_t s_ok_arsz                           : 1;
  uint16_t s_ok_accx                           : 1;
  uint16_t s_ok_accy                           : 1;
  uint16_t s_ok_accz                           : 1;
  uint16_t s_ok_mgpx                           : 1;
  uint16_t s_ok_mgpy                           : 1;
  uint16_t s_ok_mgpz                           : 1;
  uint16_t not_used0                           : 1;
  uint16_t s_ok_c                              : 1;
  uint16_t eoi                                 : 1;
  uint16_t not_used1                           : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t not_used1                           : 4;
  uint16_t eoi                                 : 1;
  uint16_t s_ok_c                              : 1;
  uint16_t not_used0                           : 1;
  uint16_t s_ok_mgpz                           : 1;
  uint16_t s_ok_mgpy                           : 1;
  uint16_t s_ok_mgpx                           : 1;
  uint16_t s_ok_accz                           : 1;
  uint16_t s_ok_accy                           : 1;
  uint16_t s_ok_accx                           : 1;
  uint16_t s_ok_arsz                           : 1;
  uint16_t s_ok_arsy                           : 1;
  uint16_t s_ok_arsx                           : 1;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_ssumok_t;

#define ASM9G300B_SSUMRNG                              0x0F
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t srng_arsx                           : 1;
  uint16_t srng_arsy                           : 1;
  uint16_t srng_arsz                           : 1;
  uint16_t srng_accx                           : 1;
  uint16_t srng_accy                           : 1;
  uint16_t srng_accz                           : 1;
  uint16_t srng_mgpx                           : 1;
  uint16_t srng_mgpy                           : 1;
  uint16_t srng_mgpz                           : 1;
  uint16_t not_used0                           : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t not_used0                           : 7;
  uint16_t srng_mgpz                           : 1;
  uint16_t srng_mgpy                           : 1;
  uint16_t srng_mgpx                           : 1;
  uint16_t srng_accz                           : 1;
  uint16_t srng_accy                           : 1;
  uint16_t srng_accx                           : 1;
  uint16_t srng_arsz                           : 1;
  uint16_t srng_arsy                           : 1;
  uint16_t srng_arsx                           : 1;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_ssumrng_t;

#define ASM9G300B_STATARSX                             0x10
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t drive_stator_ok_rx                  : 1;
  uint16_t selftestauto_rate_tc_ok_rx          : 1;
  uint16_t rotor_ok_rx                         : 1;
  uint16_t start_ok_rx                         : 1;
  uint16_t rate_not_sat_rx                     : 1;
  uint16_t qc_amp_ok_rx                        : 1;
  uint16_t sns_stator_ok_rx                    : 1;
  uint16_t gyroselftestauto_ok_rx              : 1;
  uint16_t drive_disp_ok_rx                    : 1;
  uint16_t drive_control_ok_rx                 : 1;
  uint16_t qcctrlok_rx                         : 1;
  uint16_t adc_not_sat_rx                      : 1;
  uint16_t fdrvok_rx                           : 1;
  uint16_t pll_lock_ok_rx                      : 1;
  uint16_t cp_ok_rx                            : 1;
  uint16_t rate_ref_ok_rx                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t rate_ref_ok_rx                      : 1;
  uint16_t cp_ok_rx                            : 1;
  uint16_t pll_lock_ok_rx                      : 1;
  uint16_t fdrvok_rx                           : 1;
  uint16_t adc_not_sat_rx                      : 1;
  uint16_t qcctrlok_rx                         : 1;
  uint16_t drive_control_ok_rx                 : 1;
  uint16_t drive_disp_ok_rx                    : 1;
  uint16_t gyroselftestauto_ok_rx              : 1;
  uint16_t sns_stator_ok_rx                    : 1;
  uint16_t qc_amp_ok_rx                        : 1;
  uint16_t rate_not_sat_rx                     : 1;
  uint16_t start_ok_rx                         : 1;
  uint16_t rotor_ok_rx                         : 1;
  uint16_t selftestauto_rate_tc_ok_rx          : 1;
  uint16_t drive_stator_ok_rx                  : 1;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_statarsx_t;

#define ASM9G300B_STATARSZ                             0x11
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t drive_stator_ok_rz                  : 1;
  uint16_t selftestauto_rate_tc_ok_rz          : 1;
  uint16_t rotor_ok_rz                         : 1;
  uint16_t start_ok_rz                         : 1;
  uint16_t rate_not_sat_rz                     : 1;
  uint16_t qc_amp_ok_rz                        : 1;
  uint16_t sns_stator_ok_rz                    : 1;
  uint16_t gyroselftestauto_ok_rz              : 1;
  uint16_t drive_disp_ok_rz                    : 1;
  uint16_t drive_control_ok_rz                 : 1;
  uint16_t qcctrlok_rz                         : 1;
  uint16_t adc_not_sat_rz                      : 1;
  uint16_t fdrvok_rz                           : 1;
  uint16_t pll_lock_ok_rz                      : 1;
  uint16_t cp_ok_rz                            : 1;
  uint16_t rate_ref_ok_rz                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t rate_ref_ok_rz                      : 1;
  uint16_t cp_ok_rz                            : 1;
  uint16_t pll_lock_ok_rz                      : 1;
  uint16_t fdrvok_rz                           : 1;
  uint16_t adc_not_sat_rz                      : 1;
  uint16_t qcctrlok_rz                         : 1;
  uint16_t drive_control_ok_rz                 : 1;
  uint16_t drive_disp_ok_rz                    : 1;
  uint16_t gyroselftestauto_ok_rz              : 1;
  uint16_t sns_stator_ok_rz                    : 1;
  uint16_t qc_amp_ok_rz                        : 1;
  uint16_t rate_not_sat_rz                     : 1;
  uint16_t start_ok_rz                         : 1;
  uint16_t rotor_ok_rz                         : 1;
  uint16_t selftestauto_rate_tc_ok_rz          : 1;
  uint16_t drive_stator_ok_rz                  : 1;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_statarsz_t;

#define ASM9G300B_STATACCX                             0x12
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t a2d_nsat_ok_ax                      : 1;
  uint16_t selftestauto_tc_ok_ax               : 1;
  uint16_t ref_ok_ax                           : 1;
  uint16_t selftestauto_ok_ax                  : 1;
  uint16_t rotorbond_ok_ax                     : 1;
  uint16_t bw_shield_ok_ax                     : 1;
  uint16_t not_used                            : 10;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t not_used                            : 10;
  uint16_t bw_shield_ok_ax                     : 1;
  uint16_t rotorbond_ok_ax                     : 1;
  uint16_t selftestauto_ok_ax                  : 1;
  uint16_t ref_ok_ax                           : 1;
  uint16_t selftestauto_tc_ok_ax               : 1;
  uint16_t a2d_nsat_ok_ax                      : 1;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_stataccx_t;

#define ASM9G300B_STATACCY                             0x13
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t a2d_nsat_ok_ay                      : 1;
  uint16_t selftestauto_tc_ok_ay               : 1;
  uint16_t ref_ok_ay                           : 1;
  uint16_t selftestauto_ok_ay                  : 1;
  uint16_t rotorbond_ok_ay                     : 1;
  uint16_t bw_shield_ok_ay                     : 1;
  uint16_t not_used                            : 10;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t not_used                            : 10;
  uint16_t bw_shield_ok_ay                     : 1;
  uint16_t rotorbond_ok_ay                     : 1;
  uint16_t selftestauto_ok_ay                  : 1;
  uint16_t ref_ok_ay                           : 1;
  uint16_t selftestauto_tc_ok_ay               : 1;
  uint16_t a2d_nsat_ok_ay                      : 1;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_stataccy_t;

#define ASM9G300B_STATCOM1                             0x14
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t not_testmode                        : 1;
  uint16_t bg_ok                               : 1;
  uint16_t ts_ok                               : 1;
  uint16_t spi_no_err                          : 1;
  uint16_t selftestauto_losscap_ok_latch       : 1;
  uint16_t sar_ok                              : 1;
  uint16_t selftestauto_nvm_ok_latch           : 1;
  uint16_t reg_ctrl_wr_ok_latch                : 1;
  uint16_t dgnd_ok                             : 1;
  uint16_t avdd_headroom_ok                    : 1;
  uint16_t osc_ok                              : 1;
  uint16_t reg_flip_no_err                     : 1;
  uint16_t nvm_flip_no_err                     : 1;
  uint16_t ana1_ok                             : 1;
  uint16_t ana2_ok                             : 1;
  uint16_t not_used                            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t not_used                            : 1;
  uint16_t ana2_ok                             : 1;
  uint16_t ana1_ok                             : 1;
  uint16_t nvm_flip_no_err                     : 1;
  uint16_t reg_flip_no_err                     : 1;
  uint16_t osc_ok                              : 1;
  uint16_t avdd_headroom_ok                    : 1;
  uint16_t dgnd_ok                             : 1;
  uint16_t reg_ctrl_wr_ok_latch                : 1;
  uint16_t selftestauto_nvm_ok_latch           : 1;
  uint16_t sar_ok                              : 1;
  uint16_t selftestauto_losscap_ok_latch       : 1;
  uint16_t spi_no_err                          : 1;
  uint16_t ts_ok                               : 1;
  uint16_t bg_ok                               : 1;
  uint16_t not_testmode                        : 1;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_statcom1_t;

#define ASM9G300B_STATCOM2                             0x15
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t avdd_high_ok                        : 1;
  uint16_t avdd_ok                             : 1;
  uint16_t vrega_high_ok                       : 1;
  uint16_t vrega_ok                            : 1;
  uint16_t vregd_high_ok                       : 1;
  uint16_t vregd_ok                            : 1;
  uint16_t selftestauto_sarbist_ok             : 1;
  uint16_t tsrepeat_ok                         : 1;
  uint16_t not_used                            : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t not_used                            : 8;
  uint16_t tsrepeat_ok                         : 1;
  uint16_t selftestauto_sarbist_ok             : 1;
  uint16_t vregd_ok                            : 1;
  uint16_t vregd_high_ok                       : 1;
  uint16_t vrega_ok                            : 1;
  uint16_t vrega_high_ok                       : 1;
  uint16_t avdd_ok                             : 1;
  uint16_t avdd_high_ok                        : 1;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_statcom2_t;

#define ASM9G300B_CONFIG01                             0x16
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t iir_bw_sel_ax                       : 2;
  uint16_t iir_bw_sel_ay                       : 2;
  uint16_t iir_bw_sel_az                       : 2;
  uint16_t iir_bw_sel_rx                       : 2;
  uint16_t iir_bw_sel_rz                       : 2;
  uint16_t sdo_drv                             : 3;
  uint16_t not_used                            : 1;
  uint16_t selftestautoxldis                   : 1;
  uint16_t selftestautogyrodis                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t selftestautogyrodis                 : 1;
  uint16_t selftestautoxldis                   : 1;
  uint16_t not_used                            : 1;
  uint16_t sdo_drv                             : 3;
  uint16_t iir_bw_sel_rz                       : 2;
  uint16_t iir_bw_sel_rx                       : 2;
  uint16_t iir_bw_sel_az                       : 2;
  uint16_t iir_bw_sel_ay                       : 2;
  uint16_t iir_bw_sel_ax                       : 2;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_config01_t;

#define ASM9G300B_TEST_REG                             0x17
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t test_reg                            : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t test_reg                            : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_test_reg_t;

#define ASM9G300B_CONFIG02                             0x18
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t sensor_cmd                          : 7;
  uint16_t gyromst0dps                         : 1;
  uint16_t ax_pos_st                           : 1;
  uint16_t ax_neg_st                           : 1;
  uint16_t ay_pos_st                           : 1;
  uint16_t ay_neg_st                           : 1;
  uint16_t az_pos_st                           : 1;
  uint16_t az_neg_st                           : 1;
  uint16_t mst_rx                              : 1;
  uint16_t mst_rz                              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t mst_rz                              : 1;
  uint16_t mst_rx                              : 1;
  uint16_t az_neg_st                           : 1;
  uint16_t az_pos_st                           : 1;
  uint16_t ay_neg_st                           : 1;
  uint16_t ay_pos_st                           : 1;
  uint16_t ax_neg_st                           : 1;
  uint16_t ax_pos_st                           : 1;
  uint16_t gyromst0dps                         : 1;
  uint16_t sensor_cmd                          : 7;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_config02_t;

#define ASM9G300B_MODE                                 0x19
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t mode                                : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t mode                                : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_mode_t;

#define ASM9G300B_ID_TRACKING0                         0x1A
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t tracking0                           : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t tracking0                           : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_tracking0_t;

#define ASM9G300B_COMBO_ID                             0x1B
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t combo_id                            : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t combo_id                            : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_combo_id_t;

#define ASM9G300B_ASIC_ID                              0x1C
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t asic_id                             : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t asic_id                             : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_asic_id_t;

#define ASM9G300B_ID_TRACKING1                         0x1D
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t tracking1                           : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t tracking1                           : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_tracking1_t;

#define ASM9G300B_ID_TRACKING2                         0x1E
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t tracking2                           : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t tracking2                           : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_tracking2_t;

#define ASM9G300B_CONFIG04                             0x1F
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t tdebrng_ax                          : 3;
  uint16_t tdebrng_ay                          : 3;
  uint16_t tdebrng_az                          : 3;
  uint16_t tdebrng_rx                          : 3;
  uint16_t tdebrng_rz                          : 3;
  uint16_t not_used                            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t not_used                            : 1;
  uint16_t tdebrng_rz                          : 3;
  uint16_t tdebrng_rx                          : 3;
  uint16_t tdebrng_az                          : 3;
  uint16_t tdebrng_ay                          : 3;
  uint16_t tdebrng_ax                          : 3;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_config04_t;

#define ASM9G300B_STATARSY                             0x20
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t drive_stator_ok_ry                  : 1;
  uint16_t selftestauto_rate_tc_ok_ry          : 1;
  uint16_t rotor_ok_ry                         : 1;
  uint16_t start_ok_ry                         : 1;
  uint16_t rate_not_sat_ry                     : 1;
  uint16_t qc_amp_ok_ry                        : 1;
  uint16_t sns_stator_ok_ry                    : 1;
  uint16_t gyroselftestauto_ok_ry              : 1;
  uint16_t drive_disp_ok_ry                    : 1;
  uint16_t drive_control_ok_ry                 : 1;
  uint16_t qcctrlok_ry                         : 1;
  uint16_t adc_not_sat_ry                      : 1;
  uint16_t fdrvok_ry                           : 1;
  uint16_t pll_lock_ok_ry                      : 1;
  uint16_t cp_ok_ry                            : 1;
  uint16_t rate_ref_ok_ry                      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t rate_ref_ok_ry                      : 1;
  uint16_t cp_ok_ry                            : 1;
  uint16_t pll_lock_ok_ry                      : 1;
  uint16_t fdrvok_ry                           : 1;
  uint16_t adc_not_sat_ry                      : 1;
  uint16_t qcctrlok_ry                         : 1;
  uint16_t drive_control_ok_ry                 : 1;
  uint16_t drive_disp_ok_ry                    : 1;
  uint16_t gyroselftestauto_ok_ry              : 1;
  uint16_t sns_stator_ok_ry                    : 1;
  uint16_t qc_amp_ok_ry                        : 1;
  uint16_t rate_not_sat_ry                     : 1;
  uint16_t start_ok_ry                         : 1;
  uint16_t rotor_ok_ry                         : 1;
  uint16_t selftestauto_rate_tc_ok_ry          : 1;
  uint16_t drive_stator_ok_ry                  : 1;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_statarsy_t;

#define ASM9G300B_STATACCZ                             0x21
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t a2d_nsat_ok_az                      : 1;
  uint16_t selftestauto_tc_ok_az               : 1;
  uint16_t ref_ok_az                           : 1;
  uint16_t selftestauto_ok_az                  : 1;
  uint16_t rotorbond_ok_az                     : 1;
  uint16_t bw_shield_ok_az                     : 1;
  uint16_t not_used                            : 10;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t not_used                            : 10;
  uint16_t bw_shield_ok_az                     : 1;
  uint16_t rotorbond_ok_az                     : 1;
  uint16_t selftestauto_ok_az                  : 1;
  uint16_t ref_ok_az                           : 1;
  uint16_t selftestauto_tc_ok_az               : 1;
  uint16_t a2d_nsat_ok_az                      : 1;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_stataccz_t;

#define ASM9G300B_CONFIG05                             0x23
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t zclampconfig                        : 2;
  uint16_t vrega_ok_status                     : 1;
  uint16_t avdd_ok_status                      : 1;
  uint16_t selftestauto_nvm_crc_fail           : 1;
  uint16_t por_nvm_partial_download            : 1;
  uint16_t iqdiffratery_fail                   : 1;
  uint16_t iqdiffraterz_fail                   : 1;
  uint16_t iqdiffraterx_fail                   : 1;
  uint16_t end_of_pwrup_latch_xl               : 1;
  uint16_t end_of_nvm_download_phase           : 1;
  uint16_t gyro_startup_wait                   : 1;
  uint16_t gyrotest_wait                       : 1;
  uint16_t end_of_pwrup_latch_gyro             : 1;
  uint16_t soft_rst_active_latch               : 1;
  uint16_t hard_rst_active_latch               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t hard_rst_active_latch               : 1;
  uint16_t soft_rst_active_latch               : 1;
  uint16_t end_of_pwrup_latch_gyro             : 1;
  uint16_t gyrotest_wait                       : 1;
  uint16_t gyro_startup_wait                   : 1;
  uint16_t end_of_nvm_download_phase           : 1;
  uint16_t end_of_pwrup_latch_xl               : 1;
  uint16_t iqdiffraterx_fail                   : 1;
  uint16_t iqdiffraterz_fail                   : 1;
  uint16_t iqdiffratery_fail                   : 1;
  uint16_t por_nvm_partial_download            : 1;
  uint16_t selftestauto_nvm_crc_fail           : 1;
  uint16_t avdd_ok_status                      : 1;
  uint16_t vrega_ok_status                     : 1;
  uint16_t zclampconfig                        : 2;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_config05_t;

#define ASM9G300B_CONFIG06                             0x24
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t iir_bw_sel_ry                       : 2;
  uint16_t mst_ry                              : 1;
  uint16_t tdebrng_ry                          : 3;
  uint16_t not_used                            : 10;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t not_used                            : 10;
  uint16_t tdebrng_ry                          : 3;
  uint16_t mst_ry                              : 1;
  uint16_t iir_bw_sel_ry                       : 2;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_config06_t;

#define ASM9G300B_I_FIR_ARSX                           0x25
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t i_fir_arsx                          : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t i_fir_arsx                          : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_i_fir_arsx_t;

#define ASM9G300B_I_FIR_ARSY                           0x26
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t i_fir_arsy                          : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t i_fir_arsy                          : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_i_fir_arsy_t;

#define ASM9G300B_I_FIR_ARSZ                           0x27
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t i_fir_arsz                          : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t i_fir_arsz                          : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_i_fir_arsz_t;

#define ASM9G300B_Q_FIR_ARSX                           0x28
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t q_fir_arsx                          : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t q_fir_arsx                          : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_q_fir_arsx_t;

#define ASM9G300B_Q_FIR_ARSY                           0x29
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t q_fir_arsy                          : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t q_fir_arsy                          : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_q_fir_arsy_t;

#define ASM9G300B_Q_FIR_ARSZ                           0x2A
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t q_fir_arsz                          : 16;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t q_fir_arsz                          : 16;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_q_fir_arsz_t;

#define ASM9G300B_ST_STATUS_ARS                        0x2B
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t auto_st_status_rx                   : 2;
  uint16_t auto_st_status_ry                   : 2;
  uint16_t auto_st_status_rz                   : 2;
  uint16_t manual_st_status_rx                 : 1;
  uint16_t manual_st_status_ry                 : 1;
  uint16_t manual_st_status_rz                 : 1;
  uint16_t caploss_status                      : 1;
  uint16_t not_used                            : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t not_used                            : 6;
  uint16_t caploss_status                      : 1;
  uint16_t manual_st_status_rz                 : 1;
  uint16_t manual_st_status_ry                 : 1;
  uint16_t manual_st_status_rx                 : 1;
  uint16_t auto_st_status_rz                   : 2;
  uint16_t auto_st_status_ry                   : 2;
  uint16_t auto_st_status_rx                   : 2;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_st_status_ars_t;

#define ASM9G300B_ST_STATUS_ACC                        0x2C
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint16_t auto_st_status_ax                   : 2;
  uint16_t auto_st_status_ay                   : 2;
  uint16_t auto_st_status_az                   : 2;
  uint16_t manual_st_status_ax                 : 1;
  uint16_t manual_st_status_ay                 : 1;
  uint16_t manual_st_status_az                 : 1;
  uint16_t not_used                            : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint16_t not_used                            : 7;
  uint16_t manual_st_status_az                 : 1;
  uint16_t manual_st_status_ay                 : 1;
  uint16_t manual_st_status_ax                 : 1;
  uint16_t auto_st_status_az                   : 2;
  uint16_t auto_st_status_ay                   : 2;
  uint16_t auto_st_status_ax                   : 2;
#endif /* DRV_BYTE_ORDER */
} asm9g300b_st_status_acc_t;


#define ASM9G300B_PAGE                                 0x7F

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

int32_t asm9g300b_read_frame(const stmdev_ctx_t *ctx, uint8_t *reg,
                             uint8_t *data);
int32_t asm9g300b_write_frame(const stmdev_ctx_t *ctx, uint8_t reg,
                              uint16_t data);

typedef enum
{
  ASM9G300B_CMD_SOFT_RESET         = 0x03,
  ASM9G300B_CMD_HARD_RESET         = 0x07,
  ASM9G300B_CMD_WAKE_UP            = 0x09,
  ASM9G300B_CMD_LP_MODE            = 0x0F,
  ASM9G300B_CMD_START_SELF_TEST    = 0x11,
  ASM9G300B_CMD_STOP_SELF_TEST     = 0x12,
  ASM9G300B_CMD_XL_AUTO_SELF_TEST  = 0x1D,
  ASM9G300B_CMD_GY_AUTO_SELF_TEST  = 0x1E,
  ASM9G300B_CMD_ALL_AUTO_SELF_TEST = 0x1F,
  ASM9G300B_CMD_GY_RESTART         = 0x21,
  ASM9G300B_CMD_EOI                = 0x7F,
} asm9g300b_commands_t;
int32_t asm9g300b_set_command(const stmdev_ctx_t *ctx, asm9g300b_commands_t cmd);

/* IIR Filter values */
#define ASM9G300B_IIR_FILTER_60HZ   0x0
#define ASM9G300B_IIR_FILTER_30HZ   0x1
#define ASM9G300B_IIR_FILTER_15HZ   0x2
#define ASM9G300B_IIR_FILTER_300HZ  0x3
#define ASM9G300B_IIR_FILTER_227HZ  0x3

int32_t asm9g300b_startup(const stmdev_ctx_t *ctx);

int32_t asm9g300b_check_spi_communication(const stmdev_ctx_t *ctx);

int32_t asm9g300b_summary_status_get(const stmdev_ctx_t *ctx, uint16_t *status);
int32_t asm9g300b_summary_sig_range_status_get(const stmdev_ctx_t *ctx, uint16_t *status);
int32_t asm9g300b_ars_status_get(const stmdev_ctx_t *ctx, uint16_t *status);
int32_t asm9g300b_acc_status_get(const stmdev_ctx_t *ctx, uint16_t *status);
int32_t asm9g300b_com_status_get(const stmdev_ctx_t *ctx, uint32_t *status);

/* read ARS data */
int32_t asm9g300b_ars_data_get(const stmdev_ctx_t *ctx, int16_t *raw);

/* read ACC data */
int32_t asm9g300b_acc_data_get(const stmdev_ctx_t *ctx, int16_t *raw);

/* read MGP data  */
int32_t asm9g300b_mgp_data_get(const stmdev_ctx_t *ctx, int16_t *raw);

/* read TEMP data  */
int32_t asm9g300b_temp_data_get(const stmdev_ctx_t *ctx, int16_t *raw);

/* transform lsb data into engineering units */
int32_t asm9g300b_from_ars_lsb_to_mdps(int16_t lsb);
int32_t asm9g300b_from_acc_lsb_to_mms2(int16_t lsb);
int32_t asm9g300b_from_mgp_lsb_to_mms2(int16_t lsb);
int32_t asm9g300b_from_temp_lsb_to_celsius(int16_t lsb);

/* read device serial number */
int32_t asm9g300b_getSerialNum(const stmdev_ctx_t *ctx, uint32_t *s_num);

#ifdef __cplusplus
}
#endif

#endif /* ASM9G300B_REGS_H */
