/*
 ******************************************************************************
 * @file    hts221_reg.h
 * @author  MEMS Software Solution Team
 * @date    21-September-2017
 * @brief   This file contains all the functions prototypes for the
 *          hts221_reg.c driver.
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
#ifndef __HTS221_DRIVER__H
#define __HTS221_DRIVER__H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup hts221
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

/** @defgroup hts221_interface
  * @{
  */

typedef int32_t (*hts221_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*hts221_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
  /** Component mandatory fields **/
  hts221_write_ptr  write_reg;
  hts221_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} hts221_ctx_t;

/**
  * @}
  */

/** @defgroup hts221_Infos
  * @{
  */
  /** I2C Device Address 8 bit format  **/
#define HTS221_I2C_ADDRESS   0xBF

/** Device Identification (Who am I) **/
#define HTS221_ID            0xBC

/**
  * @}
  */

#define HTS221_WHO_AM_I            0x0F
#define HTS221_AV_CONF             0x10
typedef struct {
  uint8_t avgh                 : 3;
  uint8_t avgt                 : 3;
  uint8_t not_used_01          : 2;
} hts221_av_conf_t;

#define HTS221_CTRL_REG1           0x20
typedef struct {
  uint8_t odr                  : 2;
  uint8_t bdu                  : 1;
  uint8_t not_used_01          : 4;
  uint8_t pd                   : 1;
} hts221_ctrl_reg1_t;

#define HTS221_CTRL_REG2           0x21
typedef struct {
  uint8_t one_shot             : 1;
  uint8_t heater               : 1;
  uint8_t not_used_01          : 5;
  uint8_t boot                 : 1;
} hts221_ctrl_reg2_t;

#define HTS221_CTRL_REG3           0x22
typedef struct {
  uint8_t not_used_01          : 2;
  uint8_t drdy                 : 1;
  uint8_t not_used_02          : 3;
  uint8_t pp_od                : 1;
  uint8_t drdy_h_l             : 1;
} hts221_ctrl_reg3_t;          

#define HTS221_STATUS_REG          0x27
typedef struct {
  uint8_t t_da                 : 1;
  uint8_t h_da                 : 1;
  uint8_t not_used_01          : 6;
} hts221_status_reg_t;

#define HTS221_HUMIDITY_OUT_L      0x28
#define HTS221_HUMIDITY_OUT_H      0x29
#define HTS221_TEMP_OUT_L          0x2A
#define HTS221_TEMP_OUT_H          0x2B
#define HTS221_H0_RH_X2            0x30
#define HTS221_H1_RH_X2            0x31
#define HTS221_T0_DEGC_X8          0x32
#define HTS221_T1_DEGC_X8          0x33
#define HTS221_T1_T0_MSB           0x35
typedef struct {
  uint8_t t0_msb               : 2;
  uint8_t t1_msb               : 2;
  uint8_t not_used_01          : 4;
} hts221_t1_t0_msb_t;

#define HTS221_H0_T0_OUT_L         0x36
#define HTS221_H0_T0_OUT_H         0x37
#define HTS221_H1_T0_OUT_L         0x3A
#define HTS221_H1_T0_OUT_H         0x3B
#define HTS221_T0_OUT_L            0x3C
#define HTS221_T0_OUT_H            0x3D
#define HTS221_T1_OUT_L            0x3E
#define HTS221_T1_OUT_H            0x3F

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
int32_t hts221_read_reg(hts221_ctx_t *ctx, uint8_t reg, uint8_t* data,
                        uint16_t len);
int32_t hts221_write_reg(hts221_ctx_t *ctx, uint8_t reg, uint8_t* data,
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
} hts221_avgh_t;
int32_t hts221_humidity_avg_set(hts221_ctx_t *ctx, hts221_avgh_t val);
int32_t hts221_humidity_avg_get(hts221_ctx_t *ctx, hts221_avgh_t *val);

typedef enum {
  HTS221_T_AVG_2   = 0,
  HTS221_T_AVG_4   = 1,
  HTS221_T_AVG_8   = 2,
  HTS221_T_AVG_16  = 3,
  HTS221_T_AVG_32  = 4,
  HTS221_T_AVG_64  = 5,
  HTS221_T_AVG_128 = 6,
  HTS221_T_AVG_256 = 7,
} hts221_avgt_t;
int32_t hts221_temperature_avg_set(hts221_ctx_t *ctx, hts221_avgt_t val);
int32_t hts221_temperature_avg_get(hts221_ctx_t *ctx, hts221_avgt_t *val);

typedef enum {
  HTS221_ONE_SHOT  = 0,
  HTS221_ODR_1Hz   = 1,
  HTS221_ODR_7Hz   = 2,
  HTS221_ODR_12Hz5 = 3,
} hts221_odr_t;
int32_t hts221_data_rate_set(hts221_ctx_t *ctx, hts221_odr_t val);
int32_t hts221_data_rate_get(hts221_ctx_t *ctx, hts221_odr_t *val);

int32_t hts221_block_data_update_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_block_data_update_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_one_shoot_trigger_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_one_shoot_trigger_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_temp_data_ready_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_hum_data_ready_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_humidity_raw_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_temperature_raw_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_device_id_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_power_on_set(hts221_ctx_t *ctx, uint8_t val);

int32_t hts221_power_on_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_heater_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_heater_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_boot_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_boot_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_status_get(hts221_ctx_t *ctx, hts221_status_reg_t *val);

int32_t hts221_drdy_on_int_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_drdy_on_int_get(hts221_ctx_t *ctx, uint8_t *val);

typedef enum {
  HTS221_PUSH_PULL  = 0,
  HTS221_OPEN_DRAIN = 1,
} hts221_pp_od_t;
int32_t hts221_pin_mode_set(hts221_ctx_t *ctx, hts221_pp_od_t val);
int32_t hts221_pin_mode_get(hts221_ctx_t *ctx, hts221_pp_od_t *val);

typedef enum {
  HTS221_ACTIVE_HIGH = 0,
  HTS221_ACTIVE_LOW  = 1,
} hts221_drdy_h_l_t;
int32_t hts221_int_polarity_set(hts221_ctx_t *ctx, hts221_drdy_h_l_t val);
int32_t hts221_int_polarity_get(hts221_ctx_t *ctx, hts221_drdy_h_l_t *val);

int32_t hts221_hum_rh_point_0_get(hts221_ctx_t *ctx, uint8_t *buff);
int32_t hts221_hum_rh_point_1_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_temp_deg_point_0_get(hts221_ctx_t *ctx, uint8_t *buff);
int32_t hts221_temp_deg_point_1_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_hum_adc_point_0_get(hts221_ctx_t *ctx, uint8_t *buff);
int32_t hts221_hum_adc_point_1_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_temp_adc_point_0_get(hts221_ctx_t *ctx, uint8_t *buff);
int32_t hts221_temp_adc_point_1_get(hts221_ctx_t *ctx, uint8_t *buff);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__HTS221_DRIVER__H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
