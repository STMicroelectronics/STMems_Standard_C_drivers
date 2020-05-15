/*
 ******************************************************************************
 * @file    lis3dsh_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          lis3dsh_reg.c driver.
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
#ifndef LIS3DSH_REGS_H
#define LIS3DSH_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup LIS3DSH
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

/** @defgroup LIS3DSH_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> 3D if SA0=1 -> 3B **/
#define LIS3DSH_I2C_ADD_L                    0x3D
#define LIS3DSH_I2C_ADD_H                    0x3B

/** Device Identification (Who am I) **/
#define LIS3DSH_ID                           0x3F

/**
  * @}
  *
  */

#define LIS3DSH_OUT_T                        0x0CU
#define LIS3DSH_INFO1                        0x0DU
#define LIS3DSH_INFO2                        0x0EU
#define LIS3DSH_WHO_AM_I                     0x0FU

#define LIS3DSH_OFF_X                        0x10U
#define LIS3DSH_OFF_Y                        0x11U
#define LIS3DSH_OFF_Z                        0x12U

#define LIS3DSH_CS_X                         0x13U
#define LIS3DSH_CS_Y                         0x14U
#define LIS3DSH_CS_Z                         0x15U

#define LIS3DSH_LC_L                         0x16U
#define LIS3DSH_LC_H                         0x17U

#define LIS3DSH_STAT                         0x18U
typedef struct {
  uint8_t drdy                : 1;
  uint8_t dor                 : 1;
  uint8_t int_sm2             : 1;
  uint8_t int_sm1             : 1;
  uint8_t sync1               : 1;
  uint8_t sync2               : 1;
  uint8_t syncw               : 1;
  uint8_t l_count             : 1;  //alias LONG
} lis3dsh_stat_t;

#define LIS3DSH_PEAK1                        0x19U
#define LIS3DSH_PEAK2                        0x1AU

#define LIS3DSH_VFC_1                        0x1BU
#define LIS3DSH_VFC_2                        0x1CU
#define LIS3DSH_VFC_3                        0x1DU
#define LIS3DSH_VFC_4                        0x1EU

#define LIS3DSH_THRS3                        0x1FU
#define LIS3DSH_CTRL_REG4                    0x20U
typedef struct {
  uint8_t xen                 : 1;
  uint8_t yen                 : 1;
  uint8_t zen                 : 1;
  uint8_t bdu                 : 1;
  uint8_t odr                 : 4;
} lis3dsh_ctrl_reg4_t;

#define LIS3DSH_CTRL_REG1                    0x21U
typedef struct {
  uint8_t sm1_en              : 1;
  uint8_t not_used_01         : 2;
  uint8_t sm1_pin             : 1;
  uint8_t not_used_02         : 1;
  uint8_t hyst_1              : 3;
} lis3dsh_ctrl_reg1_t;

#define LIS3DSH_CTRL_REG2                    0x22U
typedef struct {
  uint8_t sm2_en              : 1;
  uint8_t not_used_01         : 2;
  uint8_t sm2_pin             : 1;
  uint8_t not_used_02         : 1;
  uint8_t hyst_2              : 3;
} lis3dsh_ctrl_reg2_t;

#define LIS3DSH_CTRL_REG3                    0x23U
typedef struct {
  uint8_t strt                : 1;
  uint8_t not_used_01         : 1;
  uint8_t vfilt               : 1;
  uint8_t int1_en             : 1;
  uint8_t int2_en             : 1;
  uint8_t iel                 : 1;
  uint8_t iea                 : 1;
  uint8_t dr_en               : 1;
} lis3dsh_ctrl_reg3_t;

#define LIS3DSH_CTRL_REG5                    0x24U
typedef struct {
  uint8_t sim                 : 1;
  uint8_t st                  : 2;
  uint8_t fscale              : 3;
  uint8_t bw                  : 2;
} lis3dsh_ctrl_reg5_t;

#define LIS3DSH_CTRL_REG6                    0x25U
typedef struct {
  uint8_t p2_boot             : 1;
  uint8_t p1_overrun          : 1;
  uint8_t p1_wtm              : 1;
  uint8_t p1_empty            : 1;
  uint8_t add_inc             : 1;
  uint8_t wtm_en              : 1;
  uint8_t fifo_en             : 1;
  uint8_t boot                : 1;
} lis3dsh_ctrl_reg6_t;

#define LIS3DSH_STATUS                       0x27U
typedef struct {
  uint8_t xda                  : 1;
  uint8_t yda                  : 1;
  uint8_t zda                  : 1;
  uint8_t zyxda                : 1;
  uint8_t _xor                 : 1;
  uint8_t yor                  : 1;
  uint8_t zor                  : 1;
  uint8_t zyxor                : 1;
} lis3dsh_status_t;

#define LIS3DSH_OUT_X_L                      0x28U
#define LIS3DSH_OUT_X_H                      0x29U
#define LIS3DSH_OUT_Y_L                      0x2AU
#define LIS3DSH_OUT_Y_H                      0x2BU
#define LIS3DSH_OUT_Z_L                      0x2CU
#define LIS3DSH_OUT_Z_H                      0x2DU
#define LIS3DSH_FIFO_CTRL                    0X2EU
typedef struct {
  uint8_t wtmp                 : 5;
  uint8_t fmode                : 3;
} lis3dsh_fifo_ctrl_t;

#define LIS3DSH_FIFO_SRC                     0x2FU
typedef struct {
  uint8_t fss                  : 5;
  uint8_t empty                : 1;
  uint8_t ovrn_fifo            : 1;
  uint8_t wtm                  : 1;
} lis3dsh_fifo_src_t;

/* State Machine 1 */

#define LIS3DSH_ST0_1                        0x40U
#define LIS3DSH_ST1_1                        0x41U
#define LIS3DSH_ST2_1                        0x42U
#define LIS3DSH_ST3_1                        0x43U
#define LIS3DSH_ST4_1                        0x44U
#define LIS3DSH_ST5_1                        0x45U
#define LIS3DSH_ST6_1                        0x46U
#define LIS3DSH_ST7_1                        0x47U
#define LIS3DSH_ST8_1                        0x48U
#define LIS3DSH_ST9_1                        0x49U
#define LIS3DSH_ST10_1                       0x4AU
#define LIS3DSH_ST11_1                       0x4BU
#define LIS3DSH_ST12_1                       0x4CU
#define LIS3DSH_ST13_1                       0x4DU
#define LIS3DSH_ST14_1                       0x4EU
#define LIS3DSH_ST15_1                       0x4FU
#define LIS3DSH_TIM4_1                       0x50U
#define LIS3DSH_TIM3_1                       0x51U
#define LIS3DSH_TIM2_1_L                     0x52U
#define LIS3DSH_TIM2_1_H                     0x53U
#define LIS3DSH_TIM1_1_L                     0x54U
#define LIS3DSH_TIM1_1_H                     0x55U
#define LIS3DSH_THRS2_1                      0x56U
#define LIS3DSH_THRS1_1                      0x57U
#define LIS3DSH_MASK1_B                      0x59U
typedef struct {
  uint8_t n_v                  : 1;
  uint8_t p_v                  : 1;
  uint8_t n_z                  : 1;
  uint8_t p_z                  : 1;
  uint8_t n_y                  : 1;
  uint8_t p_y                  : 1;
  uint8_t n_x                  : 1;
  uint8_t p_x                  : 1;
} lis3dsh_mask1_b_t;

#define LIS3DSH_MASK1_A                      0x5AU
typedef struct {
  uint8_t n_v                  : 1;
  uint8_t p_v                  : 1;
  uint8_t n_z                  : 1;
  uint8_t p_z                  : 1;
  uint8_t n_y                  : 1;
  uint8_t p_y                  : 1;
  uint8_t n_x                  : 1;
  uint8_t p_x                  : 1;
} lis3dsh_mask1_a_t;

#define LIS3DSH_SETT1                        0x5BU
typedef struct {
  uint8_t sitr                 : 1;
  uint8_t r_tam                : 1;
  uint8_t thr3_ma              : 1;
  uint8_t not_used_01          : 2;
  uint8_t abs                  : 1;
  uint8_t thr3_sa              : 1;
  uint8_t p_det                : 1;
} lis3dsh_sett1_t;

#define LIS3DSH_PR1                          0x5CU
typedef struct {
  uint8_t pp                   : 4;
  uint8_t rp                   : 4;
} lis3dsh_pr1_t;

#define LIS3DSH_TC1_L                        0x5DU
#define LIS3DSH_TC1_H                        0x5EU
#define LIS3DSH_OUTS1                        0x5FU
typedef struct {
  uint8_t n_v                  : 1;
  uint8_t p_v                  : 1;
  uint8_t n_z                  : 1;
  uint8_t p_z                  : 1;
  uint8_t n_y                  : 1;
  uint8_t p_y                  : 1;
  uint8_t n_x                  : 1;
  uint8_t p_x                  : 1;
} lis3dsh_outs1_t;

/* State Machine 2 */

#define LIS3DSH_ST0_2                        0x60U
#define LIS3DSH_ST1_2                        0x61U
#define LIS3DSH_ST2_2                        0x62U
#define LIS3DSH_ST3_2                        0x63U
#define LIS3DSH_ST4_2                        0x64U
#define LIS3DSH_ST5_2                        0x65U
#define LIS3DSH_ST6_2                        0x66U
#define LIS3DSH_ST7_2                        0x67U
#define LIS3DSH_ST8_2                        0x68U
#define LIS3DSH_ST9_2                        0x69U
#define LIS3DSH_ST10_2                       0x6AU
#define LIS3DSH_ST11_2                       0x6BU
#define LIS3DSH_ST12_2                       0x6CU
#define LIS3DSH_ST13_2                       0x6DU
#define LIS3DSH_ST14_2                       0x6EU
#define LIS3DSH_ST15_2                       0x6FU
#define LIS3DSH_TIM4_2                       0x70U
#define LIS3DSH_TIM3_2                       0x71U
#define LIS3DSH_TIM2_2_L                     0x72U
#define LIS3DSH_TIM2_2_H                     0x73U
#define LIS3DSH_TIM1_2_L                     0x74U
#define LIS3DSH_TIM1_2_H                     0x75U
#define LIS3DSH_THRS2_2                      0x76U
#define LIS3DSH_THRS1_2                      0x77U
#define LIS3DSH_DES2                         0x78U
#define LIS3DSH_MASK2_B                      0x79U
typedef struct {
  uint8_t n_v                  : 1;
  uint8_t p_v                  : 1;
  uint8_t n_z                  : 1;
  uint8_t p_z                  : 1;
  uint8_t n_y                  : 1;
  uint8_t p_y                  : 1;
  uint8_t n_x                  : 1;
  uint8_t p_x                  : 1;
} lis3dsh_mask2_b_t;

#define LIS3DSH_MASK2_A                      0x7AU
typedef struct {
  uint8_t n_v                  : 1;
  uint8_t p_v                  : 1;
  uint8_t n_z                  : 1;
  uint8_t p_z                  : 1;
  uint8_t n_y                  : 1;
  uint8_t p_y                  : 1;
  uint8_t n_x                  : 1;
  uint8_t p_x                  : 1;
} lis3dsh_mask2_a_t;

#define LIS3DSH_SETT2                        0x7BU
typedef struct {
  uint8_t sitr                 : 1;
  uint8_t r_tam                : 1;
  uint8_t thr3_ma              : 1;
  uint8_t d_cs                 : 1;
  uint8_t radi                 : 1;
  uint8_t abs                  : 1;
  uint8_t thr3_sa              : 1;
  uint8_t p_det                : 1;
} lis3dsh_sett2_t;

#define LIS3DSH_PR2                          0x7CU
typedef struct {
  uint8_t pp                   : 4;
  uint8_t rp                   : 4;
} lis3dsh_pr2_t;

#define LIS3DSH_TC2_L                        0x7DU
#define LIS3DSH_TC2_H                        0x7EU
#define LIS3DSH_OUTS2                        0x7FU
typedef struct {
  uint8_t n_v                  : 1;
  uint8_t p_v                  : 1;
  uint8_t n_z                  : 1;
  uint8_t p_z                  : 1;
  uint8_t n_y                  : 1;
  uint8_t p_y                  : 1;
  uint8_t n_x                  : 1;
  uint8_t p_x                  : 1;
} lis3dsh_outs2_t;

/**
  * @defgroup LIS3DSH_Register_Union
  * @brief    This union group all the registers that has a bitfield
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
  lis3dsh_stat_t                           stat;
  lis3dsh_ctrl_reg4_t                      ctrl_reg4;
  lis3dsh_ctrl_reg1_t                      ctrl_reg1;
  lis3dsh_ctrl_reg2_t                      ctrl_reg2;
  lis3dsh_ctrl_reg3_t                      ctrl_reg3;
  lis3dsh_ctrl_reg5_t                      ctrl_reg5;
  lis3dsh_ctrl_reg6_t                      ctrl_reg6;
  lis3dsh_status_t                         status;
  lis3dsh_fifo_ctrl_t                      fifo_ctrl;
  lis3dsh_fifo_src_t                       fifo_src;
  lis3dsh_mask1_b_t                        mask1_b;
  lis3dsh_mask1_a_t                        mask1_a;
  lis3dsh_sett1_t                          sett1;
  lis3dsh_pr1_t                            pr1;
  lis3dsh_outs1_t                          outs1;
  lis3dsh_mask2_b_t                        mask2_b;
  lis3dsh_mask2_a_t                        mask2_a;
  lis3dsh_sett2_t                          sett2;
  lis3dsh_pr2_t                            pr2;
  lis3dsh_outs2_t                          outs2;
  bitwise_t                                bitwise;
  uint8_t                                  byte;
} lis3dsh_reg_t;

/**
  * @}
  *
  */

int32_t lis3dsh_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);
int32_t lis3dsh_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);

extern float_t lis3dsh_from_fs2_to_mg(int16_t lsb);
extern float_t lis3dsh_from_fs4_to_mg(int16_t lsb);
extern float_t lis3dsh_from_fs6_to_mg(int16_t lsb);
extern float_t lis3dsh_from_fs8_to_mg(int16_t lsb);
extern float_t lis3dsh_from_fs16_to_mg(int16_t lsb);
extern float_t lis3dsh_from_lsb_to_celsius(int8_t lsb);

typedef struct {
  uint8_t whoami;
  uint8_t info1;
  uint8_t info2;
} lis3dsh_id_t;
int32_t lis3dsh_id_get(stmdev_ctx_t *ctx, lis3dsh_id_t *val);

typedef enum {
  LIS3DSH_SEL_BY_HW   = 0x00, /* bus mode select by HW (SPI 3W disable) */
  LIS3DSH_SPI_3W      = 0x01, /* SDO / SDI share the same pin */
} lis3dsh_bus_mode_t;
int32_t lis3dsh_bus_mode_set(stmdev_ctx_t *ctx, lis3dsh_bus_mode_t *val);
int32_t lis3dsh_bus_mode_get(stmdev_ctx_t *ctx, lis3dsh_bus_mode_t *val);

typedef enum {
  LIS3DSH_DRV_RDY   = 0x00, /* Initialize the device for driver usage */
  LIS3DSH_BOOT      = 0x01, /* Restore calib. param. ( it takes 10ms ) */
  LIS3DSH_RESET     = 0x02, /* Reset configuration registers */
} lis3dsh_init_t;
int32_t lis3dsh_init_set(stmdev_ctx_t *ctx, lis3dsh_init_t val);

typedef struct {
  uint8_t sw_reset           : 1; /* Restoring configuration registers */
  uint8_t boot               : 1; /* Restoring calibration parameters */
  uint8_t drdy_xl            : 1; /* Accelerometer data ready */
  uint8_t ovrn_xl            : 1; /* Accelerometer data overrun */
} lis3dsh_status_var_t;
int32_t lis3dsh_status_get(stmdev_ctx_t *ctx, lis3dsh_status_var_t *val);

typedef struct {
  uint8_t active_low : 1; /* 1 = active low / 0 = active high */
  uint8_t latched    : 1; /* Signals 1 = latched / 0 = pulsed */
} lis3dsh_int_mode_t;
int32_t lis3dsh_interrupt_mode_set(stmdev_ctx_t *ctx,
                                    lis3dsh_int_mode_t *val);
int32_t lis3dsh_interrupt_mode_get(stmdev_ctx_t *ctx,
                                    lis3dsh_int_mode_t *val);

typedef struct {
  uint8_t drdy_xl       : 1; /* Accelerometer data ready. */
  uint8_t fifo_empty    : 1; /* FIFO empty indication. */
  uint8_t fifo_th       : 1; /* FIFO threshold reached */
  uint8_t fifo_full     : 1; /* FIFO full */
  uint8_t fsm1          : 1; /* State machine 1 interrupt event */
  uint8_t fsm2          : 1; /* State machine 2 interrupt event */
} lis3dsh_pin_int1_route_t;
int32_t lis3dsh_pin_int1_route_set(stmdev_ctx_t *ctx,
                                   lis3dsh_pin_int1_route_t *val);
int32_t lis3dsh_pin_int1_route_get(stmdev_ctx_t *ctx,
                                   lis3dsh_pin_int1_route_t *val);

typedef struct {
  uint8_t fsm1          : 1; /* State machine 1 interrupt event */
  uint8_t fsm2          : 1; /* State machine 2 interrupt event */
  uint8_t boot          : 1; /* Restoring calibration parameters */
} lis3dsh_pin_int2_route_t;
int32_t lis3dsh_pin_int2_route_set(stmdev_ctx_t *ctx, 
                                   lis3dsh_pin_int2_route_t *val);
int32_t lis3dsh_pin_int2_route_get(stmdev_ctx_t *ctx,
                                   lis3dsh_pin_int2_route_t *val);

typedef struct {
  uint8_t drdy_xl          : 1; /* Accelerometer data ready */
  uint8_t ovrn_xl          : 1; /* Accelerometer data overrun */
  uint8_t fsm_lc           : 1; /* long counter flag (for both SM) */
  uint8_t fsm_ext_sync     : 1; /* Synchronization with ext-host requested */
  uint8_t fsm1_wait_fsm2   : 1; /* fsm1 wait fsm2 */
  uint8_t fsm2_wait_fsm1   : 1; /* fsm2 wait fsm1 */
  uint8_t fsm1             : 1; /* fsm 1 interrupt event */
  uint8_t fsm2             : 1; /* fsm 2 interrupt event */
  uint8_t fifo_ovr         : 1; /* FIFO overrun */
  uint8_t fifo_empty       : 1; /* FIFO empty indication. */
  uint8_t fifo_full        : 1; /* FIFO full */
  uint8_t fifo_th          : 1; /* FIFO threshold reached */
} lis3dsh_all_sources_t;
int32_t lis3dsh_all_sources_get(stmdev_ctx_t *ctx,
                                lis3dsh_all_sources_t *val);

typedef struct {
  enum {
    LIS3DSH_OFF    = 0x00, /* in power down */
    LIS3DSH_3Hz125 = 0x01, /* Data rate @3.125 Hz */
    LIS3DSH_6Hz25  = 0x02, /* Data rate @6.25 Hz */
    LIS3DSH_12Hz5  = 0x03, /* Data rate @12.5 Hz */
    LIS3DSH_25Hz   = 0x04, /* Data rate @25 Hz */
    LIS3DSH_50Hz   = 0x05, /* Data rate @50 Hz */
    LIS3DSH_100Hz  = 0x06, /* Data rate @100 Hz */
    LIS3DSH_400Hz  = 0x07, /* Data rate @400 Hz */
    LIS3DSH_800Hz  = 0x08, /* Data rate @800 Hz */
    LIS3DSH_1kHz6  = 0x09, /* Data rate @1600 Hz */
  } odr;
  enum {
    LIS3DSH_2g   = 0,
    LIS3DSH_4g   = 1,
    LIS3DSH_6g   = 2,
    LIS3DSH_8g   = 3,
    LIS3DSH_16g  = 4,
  } fs;
} lis3dsh_md_t;
int32_t lis3dsh_mode_set(stmdev_ctx_t *ctx, lis3dsh_md_t *val);
int32_t lis3dsh_mode_get(stmdev_ctx_t *ctx, lis3dsh_md_t *val);

typedef struct {
  struct {
    float mg[3];
    int16_t raw[3];
  }xl;
  struct {
    float deg_c;
    int8_t raw;
  }heat;
} lis3dsh_data_t;
int32_t lis3dsh_data_get(stmdev_ctx_t *ctx, lis3dsh_md_t *md,
                         lis3dsh_data_t *data);

typedef enum {
  LIS3DSH_ST_DISABLE   = 0,
  LIS3DSH_ST_POSITIVE  = 1,
  LIS3DSH_ST_NEGATIVE  = 2,
} lis3dsh_st_t;
int32_t lis3dsh_self_test_set(stmdev_ctx_t *ctx, lis3dsh_st_t val);
int32_t lis3dsh_self_test_get(stmdev_ctx_t *ctx, lis3dsh_st_t *val);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /*LIS3DSH_DRIVER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
