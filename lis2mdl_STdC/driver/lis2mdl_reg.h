/*
 ******************************************************************************
 * @file    lis2mdl_reg.h
 * @author  MEMS Software Solution Team
 * @date    06-October-2017
 * @brief   This file contains all the functions prototypes for the
 *          lis2mdl_reg.c driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIS2MDL_DRIVER__H
#define __LIS2MDL_DRIVER__H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup lis2mdl
 * @{
 */

#ifndef __MEMS_SHARED__TYPES
#define __MEMS_SHARED__TYPES

/** @defgroup ST_MEMS_common_types
  * @{
  */

typedef union{
	int16_t i16bit[3];
	uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
	int16_t i16bit;
	uint8_t u8bit[2];
} axis1bit16_t;

typedef union{
	int32_t i32bit[3];
	uint8_t u8bit[12];
} axis3bit32_t;

typedef union{
	int32_t i32bit;
	uint8_t u8bit[4];
} axis1bit32_t;

typedef struct {
   uint8_t bit0       : 1;
   uint8_t bit1       : 1;
   uint8_t bit2       : 1;
   uint8_t bit3       : 1;
   uint8_t bit4       : 1;
   uint8_t bit5       : 1;
   uint8_t bit6       : 1;
   uint8_t bit7       : 1;
} bitwise_t;

#define PROPERTY_DISABLE                (0)
#define PROPERTY_ENABLE                 (1)

#endif /*__MEMS_SHARED__TYPES*/

/**
  * @}
  */

/** @defgroup lis2mdl_interface
  * @{
  */

typedef int32_t (*lis2mdl_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*lis2mdl_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
  /** Component mandatory fields **/
  lis2mdl_write_ptr  write_reg;
  lis2mdl_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} lis2mdl_ctx_t;

/**
  * @}
  */

/** @defgroup lis2mdl_Infos
  * @{
  */
  /** I2C Device Address 8 bit format **/
#define LIS2MDL_I2C_ADD   0x3D

/** Device Identification (Who am I) **/
#define LIS2MDL_ID            0x40

/**
  * @}
  */

/**
  * @defgroup lis2mdl_Sensitivity
  * @{
  */

#define LIS2MDL_FROM_LSB_TO_mG(lsb)     (float)(lsb * 1.5f)
#define LIS2MDL_FROM_LSB_TO_degC(lsb)   (float)(lsb / 8.0f) + (25.0f)

/**
  * @}
  */

#define LIS2MDL_OFFSET_X_REG_L          0x45
#define LIS2MDL_OFFSET_X_REG_H          0x46
#define LIS2MDL_OFFSET_Y_REG_L          0x47
#define LIS2MDL_OFFSET_Y_REG_H          0x48
#define LIS2MDL_OFFSET_Z_REG_L          0x49
#define LIS2MDL_OFFSET_Z_REG_H          0x4A
#define LIS2MDL_WHO_AM_I                0x4F
#define LIS2MDL_CFG_REG_A               0x60
typedef struct {
  uint8_t md                     : 2;
  uint8_t odr                    : 2;
  uint8_t lp                     : 1;
  uint8_t soft_rst               : 1;
  uint8_t reboot                 : 1;
  uint8_t comp_temp_en           : 1;
} lis2mdl_cfg_reg_a_t;

#define LIS2MDL_CFG_REG_B               0x61
typedef struct {
  uint8_t lpf                    : 1;
  uint8_t set_rst                : 2; /* OFF_CANC + Set_FREQ */
  uint8_t int_on_dataoff         : 1;
  uint8_t off_canc_one_shot      : 1;
  uint8_t not_used_01            : 3;
} lis2mdl_cfg_reg_b_t;

#define LIS2MDL_CFG_REG_C               0x62
typedef struct {
  uint8_t drdy_on_pin            : 1;
  uint8_t self_test              : 1;
  uint8_t not_used_01            : 1;
  uint8_t ble                    : 1;
  uint8_t bdu                    : 1;
  uint8_t i2c_dis                : 1;
  uint8_t int_on_pin             : 1;
  uint8_t not_used_02            : 1;
} lis2mdl_cfg_reg_c_t;

#define LIS2MDL_INT_CRTL_REG            0x63
typedef struct {
  uint8_t ien                    : 1;
  uint8_t iel                    : 1;
  uint8_t iea                    : 1;
  uint8_t not_used_01            : 2;
  uint8_t zien                   : 1;
  uint8_t yien                   : 1;
  uint8_t xien                   : 1;
} lis2mdl_int_crtl_reg_t;

#define LIS2MDL_INT_SOURCE_REG          0x64
typedef struct {
  uint8_t _int                    : 1;
  uint8_t mroi                   : 1;
  uint8_t n_th_s_z               : 1;
  uint8_t n_th_s_y               : 1;
  uint8_t n_th_s_x               : 1;
  uint8_t p_th_s_z               : 1;
  uint8_t p_th_s_y               : 1;
  uint8_t p_th_s_x               : 1;
} lis2mdl_int_source_reg_t;

#define LIS2MDL_INT_THS_L_REG           0x65
#define LIS2MDL_INT_THS_H_REG           0x66
#define LIS2MDL_STATUS_REG              0x67
typedef struct {
  uint8_t xda                    : 1;
  uint8_t yda                    : 1;
  uint8_t zda                    : 1;
  uint8_t zyxda                  : 1;
  uint8_t xor                    : 1;
  uint8_t yor                    : 1;
  uint8_t zor                    : 1;
  uint8_t zyxor                  : 1;
} lis2mdl_status_reg_t;

#define LIS2MDL_OUTX_L_REG              0x68
#define LIS2MDL_OUTX_H_REG              0x69
#define LIS2MDL_OUTY_L_REG              0x6A
#define LIS2MDL_OUTY_H_REG              0x6B
#define LIS2MDL_OUTZ_L_REG              0x6C
#define LIS2MDL_OUTZ_H_REG              0x6D
#define LIS2MDL_TEMP_OUT_L_REG          0x6E
#define LIS2MDL_TEMP_OUT_H_REG          0x6F

typedef union{
  lis2mdl_cfg_reg_a_t            cfg_reg_a;
  lis2mdl_cfg_reg_b_t            cfg_reg_b;
  lis2mdl_cfg_reg_c_t            cfg_reg_c;
  lis2mdl_int_crtl_reg_t         int_crtl_reg;
  lis2mdl_int_source_reg_t       int_source_reg;
  lis2mdl_status_reg_t           status_reg;
  bitwise_t                      bitwise;
  uint8_t                        byte;
} lis2mdl_reg_t;

int32_t lis2mdl_read_reg(lis2mdl_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);
int32_t lis2mdl_write_reg(lis2mdl_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);

int32_t lis2mdl_mag_user_offset_set(lis2mdl_ctx_t *ctx, uint8_t *buff);
int32_t lis2mdl_mag_user_offset_get(lis2mdl_ctx_t *ctx, uint8_t *buff);
typedef enum {
  LIS2MDL_CONTINUOUS_MODE  = 0,
  LIS2MDL_SINGLE_TRIGGER   = 1,
  LIS2MDL_POWER_DOWN       = 2,
} lis2mdl_md_t;
int32_t lis2mdl_operating_mode_set(lis2mdl_ctx_t *ctx, lis2mdl_md_t val);
int32_t lis2mdl_operating_mode_get(lis2mdl_ctx_t *ctx, lis2mdl_md_t *val);

typedef enum {
  LIS2MDL_ODR_10Hz   = 0,
  LIS2MDL_ODR_20Hz   = 1,
  LIS2MDL_ODR_50Hz   = 2,
  LIS2MDL_ODR_100Hz  = 3,
} lis2mdl_odr_t;
int32_t lis2mdl_data_rate_set(lis2mdl_ctx_t *ctx, lis2mdl_odr_t val);
int32_t lis2mdl_data_rate_get(lis2mdl_ctx_t *ctx, lis2mdl_odr_t *val);

typedef enum {
  LIS2MDL_HIGH_RESOLUTION  = 0,
  LIS2MDL_LOW_POWER        = 1,
} lis2mdl_lp_t;
int32_t lis2mdl_power_mode_set(lis2mdl_ctx_t *ctx, lis2mdl_lp_t val);
int32_t lis2mdl_power_mode_get(lis2mdl_ctx_t *ctx, lis2mdl_lp_t *val);

int32_t lis2mdl_offset_temp_comp_set(lis2mdl_ctx_t *ctx, uint8_t val);
int32_t lis2mdl_offset_temp_comp_get(lis2mdl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LIS2MDL_ODR_DIV_2  = 0,
  LIS2MDL_ODR_DIV_4  = 1,
} lis2mdl_lpf_t;
int32_t lis2mdl_low_pass_bandwidth_set(lis2mdl_ctx_t *ctx,
                                       lis2mdl_lpf_t val);
int32_t lis2mdl_low_pass_bandwidth_get(lis2mdl_ctx_t *ctx,
                                       lis2mdl_lpf_t *val);

typedef enum {
  LIS2MDL_SET_SENS_ODR_DIV_63        = 0,
  LIS2MDL_SENS_OFF_CANC_EVERY_ODR    = 1,
  LIS2MDL_SET_SENS_ONLY_AT_POWER_ON  = 2,
} lis2mdl_set_rst_t;
int32_t lis2mdl_set_rst_mode_set(lis2mdl_ctx_t *ctx,
                                 lis2mdl_set_rst_t val);
int32_t lis2mdl_set_rst_mode_get(lis2mdl_ctx_t *ctx,
                                 lis2mdl_set_rst_t *val);

int32_t lis2mdl_set_rst_sensor_single_set(lis2mdl_ctx_t *ctx,
                                          uint8_t val);
int32_t lis2mdl_set_rst_sensor_single_get(lis2mdl_ctx_t *ctx,
                                          uint8_t *val);

int32_t lis2mdl_block_data_update_set(lis2mdl_ctx_t *ctx, uint8_t val);
int32_t lis2mdl_block_data_update_get(lis2mdl_ctx_t *ctx, uint8_t *val);

int32_t lis2mdl_mag_data_ready_get(lis2mdl_ctx_t *ctx, uint8_t *val);

int32_t lis2mdl_mag_data_ovr_get(lis2mdl_ctx_t *ctx, uint8_t *val);

int32_t lis2mdl_magnetic_raw_get(lis2mdl_ctx_t *ctx, uint8_t *buff);

int32_t lis2mdl_temperature_raw_get(lis2mdl_ctx_t *ctx, uint8_t *buff);

int32_t lis2mdl_device_id_get(lis2mdl_ctx_t *ctx, uint8_t *buff);

int32_t lis2mdl_reset_set(lis2mdl_ctx_t *ctx, uint8_t val);
int32_t lis2mdl_reset_get(lis2mdl_ctx_t *ctx, uint8_t *val);

int32_t lis2mdl_boot_set(lis2mdl_ctx_t *ctx, uint8_t val);
int32_t lis2mdl_boot_get(lis2mdl_ctx_t *ctx, uint8_t *val);

int32_t lis2mdl_self_test_set(lis2mdl_ctx_t *ctx, uint8_t val);
int32_t lis2mdl_self_test_get(lis2mdl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LIS2MDL_LSB_AT_LOW_ADD  = 0,
  LIS2MDL_MSB_AT_LOW_ADD  = 1,
} lis2mdl_ble_t;
int32_t lis2mdl_data_format_set(lis2mdl_ctx_t *ctx, lis2mdl_ble_t val);
int32_t lis2mdl_data_format_get(lis2mdl_ctx_t *ctx, lis2mdl_ble_t *val);

int32_t lis2mdl_status_get(lis2mdl_ctx_t *ctx, lis2mdl_status_reg_t *val);

typedef enum {
  LIS2MDL_CHECK_BEFORE  = 0,
  LIS2MDL_CHECK_AFTER   = 1,
} lis2mdl_int_on_dataoff_t;
int32_t lis2mdl_offset_int_conf_set(lis2mdl_ctx_t *ctx,
                                    lis2mdl_int_on_dataoff_t val);
int32_t lis2mdl_offset_int_conf_get(lis2mdl_ctx_t *ctx,
                                    lis2mdl_int_on_dataoff_t *val);

int32_t lis2mdl_drdy_on_pin_set(lis2mdl_ctx_t *ctx, uint8_t val);
int32_t lis2mdl_drdy_on_pin_get(lis2mdl_ctx_t *ctx, uint8_t *val);

int32_t lis2mdl_int_on_pin_set(lis2mdl_ctx_t *ctx, uint8_t val);
int32_t lis2mdl_int_on_pin_get(lis2mdl_ctx_t *ctx, uint8_t *val);

int32_t lis2mdl_int_gen_conf_set(lis2mdl_ctx_t *ctx,
                                 lis2mdl_int_crtl_reg_t *val);
int32_t lis2mdl_int_gen_conf_get(lis2mdl_ctx_t *ctx,
                                 lis2mdl_int_crtl_reg_t *val);

int32_t lis2mdl_int_gen_source_get(lis2mdl_ctx_t *ctx,
                                   lis2mdl_int_source_reg_t *val);

int32_t lis2mdl_int_gen_treshold_set(lis2mdl_ctx_t *ctx, uint8_t *buff);
int32_t lis2mdl_int_gen_treshold_get(lis2mdl_ctx_t *ctx, uint8_t *buff);
typedef enum {
  LIS2MDL_I2C_ENABLE   = 0,
  LIS2MDL_I2C_DISABLE  = 1,
} lis2mdl_i2c_dis_t;
int32_t lis2mdl_i2c_interface_set(lis2mdl_ctx_t *ctx,
                                  lis2mdl_i2c_dis_t val);
int32_t lis2mdl_i2c_interface_get(lis2mdl_ctx_t *ctx,
                                  lis2mdl_i2c_dis_t *val);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__LIS2MDL_DRIVER__H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
