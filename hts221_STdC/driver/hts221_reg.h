/*
 ******************************************************************************
 * @file    hts221_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          hts221_reg.c driver.
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
#ifndef HTS221_REGS_H
#define HTS221_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup HTS221
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

/** @defgroup HTS221_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  **/
#define HTS221_I2C_ADDRESS         0xBFU

/** Device Identification (Who am I) **/
#define HTS221_ID                  0xBCU

/**
  * @}
  *
  */

#define HTS221_WHO_AM_I            0x0FU
#define HTS221_AV_CONF             0x10U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t avgh                 : 3;
  uint8_t avgt                 : 3;
  uint8_t not_used_01          : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01          : 2;
  uint8_t avgt                 : 3;
  uint8_t avgh                 : 3;
#endif /* DRV_BYTE_ORDER */
} hts221_av_conf_t;

#define HTS221_CTRL_REG1           0x20U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t odr                  : 2;
  uint8_t bdu                  : 1;
  uint8_t not_used_01          : 4;
  uint8_t pd                   : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t pd                   : 1;
  uint8_t not_used_01          : 4;
  uint8_t bdu                  : 1;
  uint8_t odr                  : 2;
#endif /* DRV_BYTE_ORDER */
} hts221_ctrl_reg1_t;

#define HTS221_CTRL_REG2           0x21U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t one_shot             : 1;
  uint8_t heater               : 1;
  uint8_t not_used_01          : 5;
  uint8_t boot                 : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot                 : 1;
  uint8_t not_used_01          : 5;
  uint8_t heater               : 1;
  uint8_t one_shot             : 1;
#endif /* DRV_BYTE_ORDER */
} hts221_ctrl_reg2_t;

#define HTS221_CTRL_REG3           0x22U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01          : 2;
  uint8_t drdy                 : 1;
  uint8_t not_used_02          : 3;
  uint8_t pp_od                : 1;
  uint8_t drdy_h_l             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t drdy_h_l             : 1;
  uint8_t pp_od                : 1;
  uint8_t not_used_02          : 3;
  uint8_t drdy                 : 1;
  uint8_t not_used_01          : 2;
#endif /* DRV_BYTE_ORDER */
} hts221_ctrl_reg3_t;

#define HTS221_STATUS_REG          0x27U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t t_da                 : 1;
  uint8_t h_da                 : 1;
  uint8_t not_used_01          : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01          : 6;
  uint8_t h_da                 : 1;
  uint8_t t_da                 : 1;
#endif /* DRV_BYTE_ORDER */
} hts221_status_reg_t;

#define HTS221_HUMIDITY_OUT_L      0x28U
#define HTS221_HUMIDITY_OUT_H      0x29U
#define HTS221_TEMP_OUT_L          0x2AU
#define HTS221_TEMP_OUT_H          0x2BU
#define HTS221_H0_RH_X2            0x30U
#define HTS221_H1_RH_X2            0x31U
#define HTS221_T0_DEGC_X8          0x32U
#define HTS221_T1_DEGC_X8          0x33U
#define HTS221_T1_T0_MSB           0x35U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t t0_msb               : 2;
  uint8_t t1_msb               : 2;
  uint8_t not_used_01          : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01          : 4;
  uint8_t t1_msb               : 2;
  uint8_t t0_msb               : 2;
#endif /* DRV_BYTE_ORDER */
} hts221_t1_t0_msb_t;

#define HTS221_H0_T0_OUT_L         0x36U
#define HTS221_H0_T0_OUT_H         0x37U
#define HTS221_H1_T0_OUT_L         0x3AU
#define HTS221_H1_T0_OUT_H         0x3BU
#define HTS221_T0_OUT_L            0x3CU
#define HTS221_T0_OUT_H            0x3DU
#define HTS221_T1_OUT_L            0x3EU
#define HTS221_T1_OUT_H            0x3FU

/**
  * @defgroup HTS221_Register_Union
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
  hts221_av_conf_t        av_conf;
  hts221_ctrl_reg1_t      ctrl_reg1;
  hts221_ctrl_reg2_t      ctrl_reg2;
  hts221_ctrl_reg3_t      ctrl_reg3;
  hts221_status_reg_t     status_reg;
  hts221_t1_t0_msb_t      t1_t0_msb;
  bitwise_t               bitwise;
  uint8_t                 byte;
} hts221_reg_t;

/**
  * @}
  *
  */

int32_t hts221_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                        uint16_t len);
int32_t hts221_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);

typedef enum {
  HTS221_H_AVG_4    = 0,
  HTS221_H_AVG_8    = 1,
  HTS221_H_AVG_16   = 2,
  HTS221_H_AVG_32   = 3,
  HTS221_H_AVG_64   = 4,
  HTS221_H_AVG_128  = 5,
  HTS221_H_AVG_256  = 6,
  HTS221_H_AVG_512  = 7,
  HTS221_H_AVG_ND   = 8,
} hts221_avgh_t;
int32_t hts221_humidity_avg_set(stmdev_ctx_t *ctx, hts221_avgh_t val);
int32_t hts221_humidity_avg_get(stmdev_ctx_t *ctx, hts221_avgh_t *val);

typedef enum {
  HTS221_T_AVG_2   = 0,
  HTS221_T_AVG_4   = 1,
  HTS221_T_AVG_8   = 2,
  HTS221_T_AVG_16  = 3,
  HTS221_T_AVG_32  = 4,
  HTS221_T_AVG_64  = 5,
  HTS221_T_AVG_128 = 6,
  HTS221_T_AVG_256 = 7,
  HTS221_T_AVG_ND  = 8,
} hts221_avgt_t;
int32_t hts221_temperature_avg_set(stmdev_ctx_t *ctx, hts221_avgt_t val);
int32_t hts221_temperature_avg_get(stmdev_ctx_t *ctx, hts221_avgt_t *val);

typedef enum {
  HTS221_ONE_SHOT  = 0,
  HTS221_ODR_1Hz   = 1,
  HTS221_ODR_7Hz   = 2,
  HTS221_ODR_12Hz5 = 3,
  HTS221_ODR_ND    = 4,
} hts221_odr_t;
int32_t hts221_data_rate_set(stmdev_ctx_t *ctx, hts221_odr_t val);
int32_t hts221_data_rate_get(stmdev_ctx_t *ctx, hts221_odr_t *val);

int32_t hts221_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t hts221_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t hts221_one_shoot_trigger_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t hts221_one_shoot_trigger_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t hts221_temp_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t hts221_hum_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t hts221_humidity_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t hts221_temperature_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t hts221_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t hts221_power_on_set(stmdev_ctx_t *ctx, uint8_t val);

int32_t hts221_power_on_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t hts221_heater_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t hts221_heater_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t hts221_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t hts221_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t hts221_status_get(stmdev_ctx_t *ctx, hts221_status_reg_t *val);

int32_t hts221_drdy_on_int_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t hts221_drdy_on_int_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  HTS221_PUSH_PULL   = 0,
  HTS221_OPEN_DRAIN  = 1,
  HTS221_PIN_MODE_ND = 2,
} hts221_pp_od_t;
int32_t hts221_pin_mode_set(stmdev_ctx_t *ctx, hts221_pp_od_t val);
int32_t hts221_pin_mode_get(stmdev_ctx_t *ctx, hts221_pp_od_t *val);

typedef enum {
  HTS221_ACTIVE_HIGH = 0,
  HTS221_ACTIVE_LOW  = 1,
  HTS221_ACTIVE_ND   = 2,
} hts221_drdy_h_l_t;
int32_t hts221_int_polarity_set(stmdev_ctx_t *ctx, hts221_drdy_h_l_t val);
int32_t hts221_int_polarity_get(stmdev_ctx_t *ctx, hts221_drdy_h_l_t *val);

int32_t hts221_hum_rh_point_0_get(stmdev_ctx_t *ctx, float_t *val);
int32_t hts221_hum_rh_point_1_get(stmdev_ctx_t *ctx, float_t *val);

int32_t hts221_temp_deg_point_0_get(stmdev_ctx_t *ctx, float_t *val);
int32_t hts221_temp_deg_point_1_get(stmdev_ctx_t *ctx, float_t *val);

int32_t hts221_hum_adc_point_0_get(stmdev_ctx_t *ctx, float_t *val);
int32_t hts221_hum_adc_point_1_get(stmdev_ctx_t *ctx, float_t *val);

int32_t hts221_temp_adc_point_0_get(stmdev_ctx_t *ctx, float_t *val);
int32_t hts221_temp_adc_point_1_get(stmdev_ctx_t *ctx, float_t *val);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /*HTS221_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
