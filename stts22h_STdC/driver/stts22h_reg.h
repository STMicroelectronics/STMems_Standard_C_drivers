/*
 ******************************************************************************
 * @file    stts22h_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          stts22h_reg.c driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STTS22H_REGS_H
#define STTS22H_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup STTS22H
  * @{
  *
  */

/** @defgroup STTS22H_sensors_common_types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

/**
  * @defgroup axisXbitXX_t
  * @brief    These unions are useful to represent different sensors data type.
  *           These unions are not need by the driver.
  *
  *           REMOVING the unions you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
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

/**
  * @}
  *
  */

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

#endif /* MEMS_SHARED_TYPES */

/**
  * @}
  *
  */

/** @addtogroup  STTS22H_Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*stts22h_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*stts22h_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
  /** Component mandatory fields **/
  stts22h_write_ptr  write_reg;
  stts22h_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stts22h_ctx_t;

/**
  * @}
  *
  */

/** @defgroup STTS22H_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format **/
#define STTS22H_I2C_ADD_6K8     0x71U
#define STTS22H_I2C_ADD_12K     0x79U
#define STTS22H_I2C_ADD_20K     0x7DU
#define STTS22H_I2C_ADD_33K     0x7FU

/** Device Identification (Who am I) **/
#define STTS22H_ID              0xA0U

/**
  * @}
  *
  */

#define STTS22H_WHOAMI                       0x01U
#define STTS22H_TEMP_H_LIMIT                 0x02U
typedef struct {
  uint8_t thl                 : 8;
} stts22h_temp_h_limit_t;

#define STTS22H_TEMP_L_LIMIT                 0x03U
typedef struct {
  uint8_t tll                 : 8;
} stts22h_temp_l_limit_t;

#define STTS22H_CTRL                         0x04U
typedef struct {
  uint8_t one_shot            : 1;
  uint8_t time_out_dis        : 1;
  uint8_t freerun             : 1;
  uint8_t if_add_inc          : 1;
  uint8_t avg                 : 2;
  uint8_t bdu                 : 1;
  uint8_t low_odr_en          : 1;
} stts22h_ctrl_t;

#define STTS22H_STATUS                       0x05U
typedef struct {
  uint8_t busy                : 1;
  uint8_t over_thh            : 1;
  uint8_t under_thl           : 1;
  uint8_t not_used_01         : 5;
} stts22h_status_t;

#define STTS22H_TEMP_L_OUT                   0x06U
#define STTS22H_TEMP_H_OUT                   0x07U

/**
  * @defgroup STTS22H_Register_Union
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
  stts22h_temp_h_limit_t      temp_h_limit;
  stts22h_temp_l_limit_t      temp_l_limit;
  stts22h_ctrl_t              ctrl;
  stts22h_status_t            status;
  bitwise_t                   bitwise;
  uint8_t                     byte;
} stts22h_reg_t;

/**
  * @}
  *
  */

int32_t stts22h_read_reg(stts22h_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);
int32_t stts22h_write_reg(stts22h_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);

extern float_t stts22h_from_lsb_to_celsius(int16_t lsb);

typedef enum {
  STTS22H_POWER_DOWN   = 0x00,
  STTS22H_ONE_SHOT     = 0x01,
  STTS22H_1Hz          = 0x04,
  STTS22H_25Hz         = 0x02,
  STTS22H_50Hz         = 0x12,
  STTS22H_100Hz        = 0x22,
  STTS22H_200Hz        = 0x32,
} stts22h_odr_temp_t;
int32_t stts22h_temp_data_rate_set(stts22h_ctx_t *ctx,
                                   stts22h_odr_temp_t val);
int32_t stts22h_temp_data_rate_get(stts22h_ctx_t *ctx,
                                   stts22h_odr_temp_t *val);

int32_t stts22h_block_data_update_set(stts22h_ctx_t *ctx, uint8_t val);
int32_t stts22h_block_data_update_get(stts22h_ctx_t *ctx, uint8_t *val);

int32_t stts22h_temp_flag_data_ready_get(stts22h_ctx_t *ctx, uint8_t *val);

int32_t stts22h_temperature_raw_get(stts22h_ctx_t *ctx, int16_t *buff);

int32_t stts22h_dev_id_get(stts22h_ctx_t *ctx, uint8_t *buff);

typedef struct {
  uint8_t busy             : 1;
} stts22h_dev_status_t;
int32_t stts22h_dev_status_get(stts22h_ctx_t *ctx, stts22h_dev_status_t *val);

typedef enum {
  STTS22H_SMBUS_TIMEOUT_ENABLE    = 0,
  STTS22H_SMBUS_TIMEOUT_DISABLE   = 1,
} stts22h_smbus_md_t;
int32_t stts22h_smbus_interface_set(stts22h_ctx_t *ctx,
                                    stts22h_smbus_md_t val);
int32_t stts22h_smbus_interface_get(stts22h_ctx_t *ctx,
                                    stts22h_smbus_md_t *val);

int32_t stts22h_auto_increment_set(stts22h_ctx_t *ctx, uint8_t val);
int32_t stts22h_auto_increment_get(stts22h_ctx_t *ctx, uint8_t *val);

int32_t stts22h_temp_trshld_high_set(stts22h_ctx_t *ctx, uint8_t val);
int32_t stts22h_temp_trshld_high_get(stts22h_ctx_t *ctx, uint8_t *val);

int32_t stts22h_temp_trshld_low_set(stts22h_ctx_t *ctx, uint8_t val);
int32_t stts22h_temp_trshld_low_get(stts22h_ctx_t *ctx, uint8_t *val);

typedef struct {
  uint8_t under_thl             : 1;
  uint8_t over_thh             : 1;
} stts22h_temp_trlhd_src_t;
int32_t stts22h_temp_trshld_src_get(stts22h_ctx_t *ctx,
                                    stts22h_temp_trlhd_src_t *val);

/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* STTS22H_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
