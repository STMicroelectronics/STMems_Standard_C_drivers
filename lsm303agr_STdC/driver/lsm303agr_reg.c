/*
 ******************************************************************************
 * @file    lsm303agr_reg.c
 * @author  MEMS Software Solution Team
 * @date    12-October-2017
 * @brief   LSM303AGR driver file
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

#include "lsm303agr_reg.h"

/**
  * @addtogroup  lsm303agr
  * @brief  This file provides a set of functions needed to drive the
  *         lsm303agr enanced inertial module.
  * @{
  */

/**
  * @addtogroup  interfaces_functions
  * @brief  This section provide a set of functions used to read and write
  *         a generic register of the device.
  * @{
  */

/**
  * @brief  Read generic device register
  *
  * @param  lsm303agr_ctx_t* ctx: read / write interface definitions
  * @param  uint8_t reg: register to read
  * @param  uint8_t* data: pointer to buffer that store the data read
  * @param  uint16_t len: number of consecutive register to read
  *
  */
int32_t lsm303agr_read_reg(lsm303agr_ctx_t* ctx, uint8_t reg, uint8_t* data,
                          uint16_t len)
{
  return ctx->read_reg(ctx->handle, reg, data, len);
}

/**
  * @brief  Write generic device register
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t reg: register to write
  * @param  uint8_t* data: pointer to data to write in register reg
  * @param  uint16_t len: number of consecutive register to write
  *
*/
int32_t lsm303agr_write_reg(lsm303agr_ctx_t* ctx, uint8_t reg, uint8_t* data,
                           uint16_t len)
{
  return ctx->write_reg(ctx->handle, reg, data, len);
}

/**
  * @}
  */

/**
  * @addtogroup  data_generation_c
  * @brief   This section group all the functions concerning data generation
  * @{
  */

/**
  * @brief  temp_status_reg: [get]  Temperature status register.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm303agr_temp_status_reg_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_STATUS_REG_AUX_A, buff, 1);
}
/**
  * @brief  temp_data_ready: [get]  Temperature data available.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tda in reg STATUS_REG_AUX_A
  *
  */
int32_t lsm303agr_temp_data_ready_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_STATUS_REG_AUX_A,
                                &reg.byte, 1);
  *val = reg.status_reg_aux_a.tda;

  return mm_error;
}
/**
  * @brief  temp_data_ovr: [get]  Temperature data overrun.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tor in reg STATUS_REG_AUX_A
  *
  */
int32_t lsm303agr_temp_data_ovr_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_STATUS_REG_AUX_A,
                                &reg.byte, 1);
  *val = reg.status_reg_aux_a.tor;

  return mm_error;
}
/**
  * @brief  temperature_raw: [get]  Temperature output value.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm303agr_temperature_raw_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_OUT_TEMP_L_A, buff, 2);
}
/**
  * @brief  temperature_meas: [set]  Temperature sensor enable.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_temp_en_a_t: change the values of temp_en in
  *                             reg TEMP_CFG_REG_A
  *
  */
int32_t lsm303agr_temperature_meas_set(lsm303agr_ctx_t *ctx,
                                      lsm303agr_temp_en_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_TEMP_CFG_REG_A,
                                &reg.byte, 1);
  reg.temp_cfg_reg_a.temp_en = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_TEMP_CFG_REG_A,
                                 &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  temperature_meas: [get]  Temperature sensor enable.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_temp_en_a_t: Get the values of temp_en in
  *                                reg TEMP_CFG_REG_A
  *
  */
int32_t lsm303agr_temperature_meas_get(lsm303agr_ctx_t *ctx,
                                      lsm303agr_temp_en_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_TEMP_CFG_REG_A, &reg.byte, 1);
  *val = (lsm303agr_temp_en_a_t) reg.temp_cfg_reg_a.temp_en;

  return mm_error;
}

/**
  * @brief  xl_operating_mode: [set]  Operating mode selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_op_md_a_t val: change the values of lpen in reg 
  *                                  CTRL_REG1_A and HR in reg CTRL_REG4_A
  *
  */
int32_t lsm303agr_xl_operating_mode_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_op_md_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;
  uint8_t lpen, hr;

  if ( val == LSM303AGR_HR_12bit ){
    lpen = 0;
    hr   = 1;
  } else if (val == LSM303AGR_NM_10bit) {
    lpen = 0;
    hr   = 0;
  } else {
    lpen = 1;
    hr   = 0;
  }

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG1_A, &reg.byte, 1);
  reg.ctrl_reg1_a.lpen = lpen;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG1_A, &reg.byte, 1);
  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);
  reg.ctrl_reg4_a.hr = hr;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_operating_mode: [get]  Operating mode selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_op_md_a_t val: get the values of lpen in reg 
  *                                  CTRL_REG1_A and HR in 
  *                                  reg CTRL_REG4_AG1_A
  *
  */
int32_t lsm303agr_xl_operating_mode_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_op_md_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;
  uint8_t lpen, hr;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG1_A, &reg.byte, 1);
  lpen = reg.ctrl_reg1_a.lpen;
  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);
  hr = reg.ctrl_reg4_a.hr;

  if ( lpen ){
    *val = LSM303AGR_LP_8bit;
  } else if (hr) {
    *val = LSM303AGR_HR_12bit;
  } else{
    *val = LSM303AGR_NM_10bit;
  }

  return mm_error;
}

/**
  * @brief  xl_data_rate: [set]  Output data rate selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_odr_a_t: change the values of odr in reg CTRL_REG1_A
  *
  */
int32_t lsm303agr_xl_data_rate_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_odr_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG1_A,
                                &reg.byte, 1);
  reg.ctrl_reg1_a.odr = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG1_A,
                                 &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_data_rate: [get]  Output data rate selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_odr_a_t: Get the values of odr in reg CTRL_REG1_A
  *
  */
int32_t lsm303agr_xl_data_rate_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_odr_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG1_A, &reg.byte, 1);
  *val = (lsm303agr_odr_a_t) reg.ctrl_reg1_a.odr;

  return mm_error;
}

/**
  * @brief   xl_high_pass_on_outputs: [set] High pass data from internal
  *                                         filter sent to output register
  *                                         and FIFO
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of fds in reg CTRL_REG2_A
  *
  */
int32_t lsm303agr_xl_high_pass_on_outputs_set(lsm303agr_ctx_t *ctx,
                                              uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG2_A, &reg.byte, 1);
  reg.ctrl_reg2_a.fds = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG2_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   xl_high_pass_on_outputs: [get] High pass data from internal
  *                                         filter sent to output register
  *                                         and FIFO
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fds in reg CTRL_REG2_A
  *
  */
int32_t lsm303agr_xl_high_pass_on_outputs_get(lsm303agr_ctx_t *ctx,
                                              uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG2_A, &reg.byte, 1);
  *val = reg.ctrl_reg2_a.fds;

  return mm_error;
}

/**
  * @brief   xl_high_pass_bandwidth: [set]  High-pass filter cutoff
  *                                         frequency selection
  *
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_hpcf_a_t: change the values of hpcf in reg CTRL_REG2_A
  *
  * HPCF[2:1]\ft @1Hz    @10Hz  @25Hz  @50Hz @100Hz @200Hz @400Hz @1kHz6 ft@5kHz
  * AGGRESSIVE   0.02Hz  0.2Hz  0.5Hz  1Hz   2Hz    4Hz    8Hz    32Hz   100Hz
  * STRONG       0.008Hz 0.08Hz 0.2Hz  0.5Hz 1Hz    2Hz    4Hz    16Hz   50Hz
  * MEDIUM       0.004Hz 0.04Hz 0.1Hz  0.2Hz 0.5Hz  1Hz    2Hz    8Hz    25Hz
  * LIGHT        0.002Hz 0.02Hz 0.05Hz 0.1Hz 0.2Hz  0.5Hz  1Hz    4Hz    12Hz
  *
  */
int32_t lsm303agr_xl_high_pass_bandwidth_set(lsm303agr_ctx_t *ctx,
                                             lsm303agr_hpcf_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG2_A, &reg.byte, 1);
  reg.ctrl_reg2_a.hpcf = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG2_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   xl_high_pass_bandwidth: [get]  High-pass filter cutoff
  *                                         frequency selection
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_hpcf_a_t: Get the values of hpcf in reg CTRL_REG2_A
  *
  * HPCF[2:1]\ft @1Hz    @10Hz  @25Hz  @50Hz @100Hz @200Hz @400Hz @1kHz6 ft@5kHz
  * AGGRESSIVE   0.02Hz  0.2Hz  0.5Hz  1Hz   2Hz    4Hz    8Hz    32Hz   100Hz
  * STRONG       0.008Hz 0.08Hz 0.2Hz  0.5Hz 1Hz    2Hz    4Hz    16Hz   50Hz
  * MEDIUM       0.004Hz 0.04Hz 0.1Hz  0.2Hz 0.5Hz  1Hz    2Hz    8Hz    25Hz
  * LIGHT        0.002Hz 0.02Hz 0.05Hz 0.1Hz 0.2Hz  0.5Hz  1Hz    4Hz    12Hz
  *
  */
int32_t lsm303agr_xl_high_pass_bandwidth_get(lsm303agr_ctx_t *ctx,
                                             lsm303agr_hpcf_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG2_A, &reg.byte, 1);
  *val = (lsm303agr_hpcf_a_t) reg.ctrl_reg2_a.hpcf;

  return mm_error;
}

/**
  * @brief  xl_high_pass_mode: [set]  High-pass filter mode selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_hpcf_a_t: change the values of hpm in reg CTRL_REG2_A
  *
  */
int32_t lsm303agr_xl_high_pass_mode_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_hpm_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG2_A, &reg.byte, 1);
  reg.ctrl_reg2_a.hpm = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG2_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_high_pass_mode: [get]  High-pass filter mode selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_hpcf_a_t: Get the values of hpm in reg CTRL_REG2_A
  *
  */
int32_t lsm303agr_xl_high_pass_mode_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_hpm_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG2_A, &reg.byte, 1);
  *val = (lsm303agr_hpm_a_t) reg.ctrl_reg2_a.hpm;

  return mm_error;
}

/**
  * @brief  xl_full_scale: [set]  Full-scale configuration.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_fs_a_t: change the values of fs in reg CTRL_REG4_A
  *
  */
int32_t lsm303agr_xl_full_scale_set(lsm303agr_ctx_t *ctx,
                                    lsm303agr_fs_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);
  reg.ctrl_reg4_a.fs = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_full_scale: [get]  Full-scale configuration.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_fs_a_t: Get the values of fs in reg CTRL_REG4_A
  *
  */
int32_t lsm303agr_xl_full_scale_get(lsm303agr_ctx_t *ctx,
                                    lsm303agr_fs_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);
  *val = (lsm303agr_fs_a_t) reg.ctrl_reg4_a.fs;

  return mm_error;
}

/**
  * @brief  xl_block_data_update: [set] Blockdataupdate.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of bdu in reg CTRL_REG4_A
  *
  */
int32_t lsm303agr_xl_block_data_update_set(lsm303agr_ctx_t *ctx,
                                           uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);
  reg.ctrl_reg4_a.bdu = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_block_data_update: [get] Blockdataupdate.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of bdu in reg CTRL_REG4_A
  *
  */
int32_t lsm303agr_xl_block_data_update_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);
  *val = reg.ctrl_reg4_a.bdu;

  return mm_error;
}

/**
  * @brief  xl_filter_reference: [set]Reference value for interrupt generation.
  *                                   LSB = ~16@2g / ~31@4g / ~63@8g / ~127@16g
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that contains data to write
  *
  */
int32_t lsm303agr_xl_filter_reference_set(lsm303agr_ctx_t *ctx,
                                          uint8_t *buff)
{
  return lsm303agr_write_reg(ctx, LSM303AGR_REFERENCE_A, buff, 1);
}

/**
  * @brief  xl_filter_reference: [get]Reference value for interrupt generation.
  *                                   LSB = ~16@2g / ~31@4g / ~63@8g / ~127@16g
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm303agr_xl_filter_reference_get(lsm303agr_ctx_t *ctx,
                                          uint8_t *buff)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_REFERENCE_A, buff, 1);
}
/**
  * @brief  xl_data_ready: [get]  Acceleration set of data available.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of zyxda in reg STATUS_REG_A
  *
  */
int32_t lsm303agr_xl_data_ready_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_STATUS_REG_A, &reg.byte, 1);
  *val = reg.status_reg_a.zyxda;

  return mm_error;
}
/**
  * @brief  xl_data_ovr: [get]  Acceleration set of data overrun.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of zyxor in reg STATUS_REG_A
  *
  */
int32_t lsm303agr_xl_data_ovr_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_STATUS_REG_A, &reg.byte, 1);
  *val = reg.status_reg_a.zyxor;

  return mm_error;
}
/**
  * @brief  acceleration_raw: [get]  Acceleration output value.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm303agr_acceleration_raw_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_OUT_X_L_A, buff, 6);
}

/**
  * @brief  mag_user_offset: [set]  These registers comprise a 3 group of
  *                                 16-bit number and represent hard-iron
  *                                 offset in order to compensate environmental
  *                                 effects. Data format is the same of
  *                                 output data raw: two’s complement with
  *                                 1LSb = 1.5mG. These values act on the
  *                                 magnetic output data value in order to
  *                                 delete the environmental offset.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that contains data to write
  *
  */
int32_t lsm303agr_mag_user_offset_set(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  return lsm303agr_write_reg(ctx, LSM303AGR_OFFSET_X_REG_L_M, buff, 6);
}

/**
  * @brief  mag_user_offset: [get]  These registers comprise a 3 group of
  *                                 16-bit number and represent hard-iron
  *                                 offset in order to compensate environmental
  *                                 effects. Data format is the same of
  *                                 output data raw: two’s complement with
  *                                 1LSb = 1.5mG. These values act on the
  *                                 magnetic output data value in order to
  *                                 delete the environmental offset.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm303agr_mag_user_offset_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_OFFSET_X_REG_L_M, buff, 6);
}

/**
  * @brief  operating_mode: [set]  Operating mode selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_md_t: change the values of md in reg CFG_REG_A_M
  *
  */
int32_t lsm303agr_mag_operating_mode_set(lsm303agr_ctx_t *ctx,
                                         lsm303agr_md_m_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);
  reg.cfg_reg_a_m.md = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  operating_mode: [get]  Operating mode selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_md_t: Get the values of md in reg CFG_REG_A_M
  *
  */
int32_t lsm303agr_mag_operating_mode_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_md_m_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);
  *val = (lsm303agr_md_m_t) reg.cfg_reg_a_m.md;

  return mm_error;
}

/**
  * @brief  data_rate: [set]  Output data rate selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_mg_odr_m_t: change the values of odr in reg CFG_REG_A_M
  *
  */
int32_t lsm303agr_mag_data_rate_set(lsm303agr_ctx_t *ctx,
                                    lsm303agr_mg_odr_m_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);
  reg.cfg_reg_a_m.odr = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  data_rate: [get]  Output data rate selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_mg_odr_m_tv: Get the values of odr in reg CFG_REG_A_M
  *
  */
int32_t lsm303agr_mag_data_rate_get(lsm303agr_ctx_t *ctx,
                                    lsm303agr_mg_odr_m_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);
  *val = (lsm303agr_mg_odr_m_t) reg.cfg_reg_a_m.odr;

  return mm_error;
}

/**
  * @brief  power_mode: [set]  Enables high-resolution/low-power mode.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_lp_m_t: change the values of lp in reg CFG_REG_A_M
  *
  */
int32_t lsm303agr_mag_power_mode_set(lsm303agr_ctx_t *ctx,
                                     lsm303agr_lp_m_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);
  reg.cfg_reg_a_m.lp = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  power_mode: [get]  Enables high-resolution/low-power mode.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_lp_m_t: Get the values of lp in reg CFG_REG_A_M
  *
  */
int32_t lsm303agr_mag_power_mode_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_lp_m_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);
  *val = (lsm303agr_lp_m_t) reg.cfg_reg_a_m.lp;

  return mm_error;
}

/**
  * @brief  mag_offset_temp_comp: [set]  Enables the magnetometer temperature
  *                                  compensation.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of comp_temp_en in reg CFG_REG_A_M
  *
  */
int32_t lsm303agr_mag_offset_temp_comp_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);
  reg.cfg_reg_a_m.comp_temp_en = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  mag_offset_temp_comp: [get]  Enables the magnetometer temperature
  *                                  compensation.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of comp_temp_en in reg CFG_REG_A_M
  *
  */
int32_t lsm303agr_mag_offset_temp_comp_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);
  *val = reg.cfg_reg_a_m.comp_temp_en;

  return mm_error;
}

/**
  * @brief  mag_low_pass_bandwidth: [set]  Low-pass bandwidth selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_lpf_t: change the values of lpf in reg CFG_REG_B_M
  *
  */
int32_t lsm303agr_mag_low_pass_bandwidth_set(lsm303agr_ctx_t *ctx,
                                             lsm303agr_lpf_m_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_B_M, &reg.byte, 1);
  reg.cfg_reg_b_m.lpf = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_B_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  mag_low_pass_bandwidth: [get]  Low-pass bandwidth selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_lpf_t: Get the values of lpf in reg CFG_REG_B_M
  *
  */
int32_t lsm303agr_mag_low_pass_bandwidth_get(lsm303agr_ctx_t *ctx,
                                             lsm303agr_lpf_m_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_B_M, &reg.byte, 1);
  *val = (lsm303agr_lpf_m_t) reg.cfg_reg_b_m.lpf;

  return mm_error;
}

/**
  * @brief  mag_set_rst_mode: [set]
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_set_rst_m_t: change the values of set_rst in
  *                                reg CFG_REG_B_M
  *
  */
int32_t lsm303agr_mag_set_rst_mode_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_set_rst_m_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_B_M, &reg.byte, 1);
  reg.cfg_reg_b_m.set_rst = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_B_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  mag_set_rst_mode: [get]
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_set_rst_m_t: Get the values of set_rst in
  *                                reg CFG_REG_B_M
  *
  */
int32_t lsm303agr_mag_set_rst_mode_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_set_rst_m_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_B_M, &reg.byte, 1);
  *val = (lsm303agr_set_rst_m_t) reg.cfg_reg_b_m.set_rst;

  return mm_error;
}

/**
  * @brief   mag_set_rst_sensor_single: [set] Enables offset cancellation
  *                                       in single measurement mode.
  *                                       The OFF_CANC bit must be set
  *                                       to 1 when enabling offset
  *                                       cancellation in single measurement
  *                                       mode this means a call function:
  *                                       mag_set_rst_mode
  *                                       (SENS_OFF_CANC_EVERY_ODR) is need.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of off_canc_one_shot in
  *                      reg CFG_REG_B_M
  *
  */
int32_t lsm303agr_mag_set_rst_sensor_single_set(lsm303agr_ctx_t *ctx,
                                                uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_B_M, &reg.byte, 1);
  reg.cfg_reg_b_m.off_canc_one_shot = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_B_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   mag_set_rst_sensor_single: [get] Enables offset cancellation
  *                                       in single measurement mode.
  *                                       The OFF_CANC bit must be set to
  *                                       1 when enabling offset cancellation
  *                                       in single measurement mode this
  *                                       means a call function:
  *                                       mag_set_rst_mode
  *                                       (SENS_OFF_CANC_EVERY_ODR) is need.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of off_canc_one_shot in reg CFG_REG_B_M
  *
  */
int32_t lsm303agr_mag_set_rst_sensor_single_get(lsm303agr_ctx_t *ctx,
                                                uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_B_M, &reg.byte, 1);
  *val = reg.cfg_reg_b_m.off_canc_one_shot;

  return mm_error;
}

/**
  * @brief  mag_block_data_update: [set] Blockdataupdate.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of bdu in reg CFG_REG_C_M
  *
  */
int32_t lsm303agr_mag_block_data_update_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);
  reg.cfg_reg_c_m.bdu = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  mag_block_data_update: [get] Blockdataupdate.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of bdu in reg CFG_REG_C_M
  *
  */
int32_t lsm303agr_mag_block_data_update_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);
  *val = reg.cfg_reg_c_m.bdu;

  return mm_error;
}

/**
  * @brief  mag_data_ready: [get]  Magnetic set of data available.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of zyxda in reg STATUS_REG_M
  *
  */
int32_t lsm303agr_mag_data_ready_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_STATUS_REG_M, &reg.byte, 1);
  *val = reg.status_reg_m.zyxda;

  return mm_error;
}

/**
  * @brief  mag_data_ovr: [get]  Magnetic set of data overrun.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of zyxor in reg STATUS_REG_M
  *
  */
int32_t lsm303agr_mag_data_ovr_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_STATUS_REG_M, &reg.byte, 1);
  *val = reg.status_reg_m.zyxor;

  return mm_error;
}

/**
  * @brief  magnetic_raw: [get]  Magnetic output value.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm303agr_magnetic_raw_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_OUTX_L_REG_M, buff, 6);
}

/**
  * @}
  */

/**
  * @addtogroup  common
  * @brief   This section group common usefull functions
  * @{
  */

/**
  * @brief  xl_device_id: [get] DeviceWhoamI.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm303agr_xl_device_id_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_WHO_AM_I_A, buff, 1);
}
/**
  * @brief  xl_self_test: [set] Selftest.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_st_a_t: change the values of st in reg CTRL_REG4_A
  *
  */
int32_t lsm303agr_xl_self_test_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_st_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);
  reg.ctrl_reg4_a.st = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_self_test: [get] Selftest.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_st_a_t: Get the values of st in reg CTRL_REG4_A
  *
  */
int32_t lsm303agr_xl_self_test_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_st_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);
  *val = (lsm303agr_st_a_t) reg.ctrl_reg4_a.st;

  return mm_error;
}

/**
  * @brief  xl_data_format: [set]  Big/Little Endian data selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_ble_a_t: change the values of ble in reg CTRL_REG4_A
  *
  */
int32_t lsm303agr_xl_data_format_set(lsm303agr_ctx_t *ctx,
                                     lsm303agr_ble_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);
  reg.ctrl_reg4_a.ble = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_data_format: [get]  Big/Little Endian data selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_ble_a_t: Get the values of ble in reg CTRL_REG4_A
  *
  */
int32_t lsm303agr_xl_data_format_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_ble_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);
  *val = (lsm303agr_ble_a_t) reg.ctrl_reg4_a.ble;

  return mm_error;
}

/**
  * @brief  xl_boot: [set]  Reboot memory content. Reload the
  *                      calibration parameters
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of boot in reg CTRL_REG5_A
  *
  */
int32_t lsm303agr_xl_boot_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);
  reg.ctrl_reg5_a.boot = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_boot: [get]  Reboot memory content. Reload the calibration
  *                      parameters
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of boot in reg CTRL_REG5_A
  *
  */
int32_t lsm303agr_xl_boot_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);
  *val = reg.ctrl_reg5_a.boot;

  return mm_error;
}

/**
  * @brief  xl_status: [get]  Info about device status.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_status_reg_a_t: register STATUS_REG_A
  *
  */
int32_t lsm303agr_xl_status_get(lsm303agr_ctx_t *ctx,
                                lsm303agr_status_reg_a_t *val)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_STATUS_REG_A, (uint8_t*) val, 1);
}

/**
  * @brief  mag_device_id: [get] DeviceWhoamI.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm303agr_mag_device_id_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_WHO_AM_I_M, buff, 1);
}

/**
  * @brief  mag_reset: [set]  Software reset. Restore the default values in
  *                       user registers.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of soft_rst in reg CFG_REG_A_M
  *
  */
int32_t lsm303agr_mag_reset_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);
  reg.cfg_reg_a_m.soft_rst = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  mag_reset: [get]  Software reset. Restore the default values
  *                       in user registers.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of soft_rst in reg CFG_REG_A_M
  *
  */
int32_t lsm303agr_mag_reset_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);
  *val = reg.cfg_reg_a_m.soft_rst;

  return mm_error;
}

/**
  * @brief  mag_boot: [set]  Reboot memory content. Reload the calibration
  *                      parameters.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of reboot in reg CFG_REG_A_M
  *
  */
int32_t lsm303agr_mag_boot_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);
  reg.cfg_reg_a_m.reboot = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  mag_boot: [get]  Reboot memory content. Reload the
  *                      calibration parameters.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of reboot in reg CFG_REG_A_M
  *
  */
int32_t lsm303agr_mag_boot_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_A_M, &reg.byte, 1);
  *val = reg.cfg_reg_a_m.reboot;

  return mm_error;
}

/**
  * @brief  mag_self_test: [set] Selftest.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of self_test in reg CFG_REG_C_M
  *
  */
int32_t lsm303agr_mag_self_test_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);
  reg.cfg_reg_c_m.self_test = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  mag_self_test: [get] Selftest.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of self_test in reg CFG_REG_C_M
  *
  */
int32_t lsm303agr_mag_self_test_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);
  *val = reg.cfg_reg_c_m.self_test;

  return mm_error;
}

/**
  * @brief  data_format: [set]  Big/Little Endian data selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_ble_m_t: change the values of ble in reg CFG_REG_C_M
  *
  */
int32_t lsm303agr_mag_data_format_set(lsm303agr_ctx_t *ctx,
                                      lsm303agr_ble_m_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);
  reg.cfg_reg_c_m.ble = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  data_format: [get]  Big/Little Endian data selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_ble_m_t: Get the values of ble in reg CFG_REG_C_M
  *
  */
int32_t lsm303agr_mag_data_format_get(lsm303agr_ctx_t *ctx,
                                      lsm303agr_ble_m_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);
  *val = (lsm303agr_ble_m_t) reg.cfg_reg_c_m.ble;

  return mm_error;
}

/**
  * @brief  status: [get]  Info about device status.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_status_reg_m_t: registers STATUS_REG_M
  *
  */
int32_t lsm303agr_mag_status_get(lsm303agr_ctx_t *ctx,
                                 lsm303agr_status_reg_m_t *val)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_STATUS_REG_M, (uint8_t*) val, 1);
}

/**
  * @}
  */

/**
  * @addtogroup   interrupts_generator_1_for_xl
  * @brief   This section group all the functions that manage the first
  *          interrupts generator of accelerometer
  * @{
  */

/**
  * @brief  xl_int1_gen_conf: [set]  Interrupt generator 1 configuration
  *                                  register
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_int1_cfg_a_t: register INT1_CFG_A
  *
  */
int32_t lsm303agr_xl_int1_gen_conf_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_int1_cfg_a_t *val)
{
  return lsm303agr_write_reg(ctx, LSM303AGR_INT1_CFG_A, (uint8_t*) val, 1);
}

/**
  * @brief  xl_int1_gen_conf: [get]  Interrupt generator 1 configuration
  *                                  register
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_int1_cfg_a_t: register INT1_CFG_A
  *
  */
int32_t lsm303agr_xl_int1_gen_conf_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_int1_cfg_a_t *val)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_INT1_CFG_A, (uint8_t*) val, 1);
}

/**
  * @brief  xl_int1_gen_source: [get]  Interrupt generator 1 source register
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_int1_src_a_t: Registers INT1_SRC_A
  *
  */
int32_t lsm303agr_xl_int1_gen_source_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_int1_src_a_t *val)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_INT1_SRC_A, (uint8_t*) val, 1);
}
/**
  * @brief  xl_int1_gen_threshold: [set]  User-defined threshold value for xl
  *                                    interrupt event on generator 1.
  *                             LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of ths in reg INT1_THS_A
  *
  */
int32_t lsm303agr_xl_int1_gen_threshold_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_INT1_THS_A, &reg.byte, 1);
  reg.int1_ths_a.ths = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_INT1_THS_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_int1_gen_threshold: [get]  User-defined threshold value for xl
  *                                    interrupt event on generator 1.
  *                             LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ths in reg INT1_THS_A
  *
  */
int32_t lsm303agr_xl_int1_gen_threshold_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_INT1_THS_A, &reg.byte, 1);
  *val = reg.int1_ths_a.ths;

  return mm_error;
}

/**
  * @brief  xl_int1_gen_duration: [set]  The minimum duration (LSb = 1/ODR)
  *                                   of the Interrupt 1 event to be
  *                                   recognized.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of d in reg INT1_DURATION_A
  *
  */
int32_t lsm303agr_xl_int1_gen_duration_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_INT1_DURATION_A, &reg.byte, 1);
  reg.int1_duration_a.d = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_INT1_DURATION_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_int1_gen_duration: [get]  The minimum duration (LSb = 1/ODR)
  *                                   of the Interrupt 1 event to be
  *                                   recognized.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of d in reg INT1_DURATION_A
  *
  */
int32_t lsm303agr_xl_int1_gen_duration_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_INT1_DURATION_A, &reg.byte, 1);
  *val = reg.int1_duration_a.d;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup   interrupts_generator_2_for_xl
  * @brief   This section group all the functions that manage the second
  *          interrupts generator for accelerometer
  * @{
  */

/**
  * @brief  xl_int2_gen_conf: [set]  Interrupt generator 2 configuration
  *                                  register
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_int2_cfg_a_t: registers INT2_CFG_A
  *
  */
int32_t lsm303agr_xl_int2_gen_conf_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_int2_cfg_a_t *val)
{
  return lsm303agr_write_reg(ctx, LSM303AGR_INT2_CFG_A, (uint8_t*) val, 1);
}

/**
  * @brief  xl_int2_gen_conf: [get]  Interrupt generator 2 configuration
  *                                  register
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_int2_cfg_a_t: registers INT2_CFG_A
  *
  */
int32_t lsm303agr_xl_int2_gen_conf_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_int2_cfg_a_t *val)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_INT2_CFG_A, (uint8_t*) val, 1);
}
/**
  * @brief  xl_int2_gen_source: [get]  Interrupt generator 2 source register
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_int2_src_a_t: registers INT2_SRC_A
  *
  */
int32_t lsm303agr_xl_int2_gen_source_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_int2_src_a_t *val)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_INT2_SRC_A, (uint8_t*) val, 1);
}
/**
  * @brief  xl_int2_gen_threshold: [set]  User-defined threshold value for xl
  *                                    interrupt event on generator 2.
  *                             LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of ths in reg INT2_THS_A
  *
  */
int32_t lsm303agr_xl_int2_gen_threshold_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_INT2_THS_A, &reg.byte, 1);
  reg.int2_ths_a.ths = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_INT2_THS_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_int2_gen_threshold: [get]  User-defined threshold value for
  *                                    xl interrupt event on generator 2.
  *                             LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ths in reg INT2_THS_A
  *
  */
int32_t lsm303agr_xl_int2_gen_threshold_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_INT2_THS_A, &reg.byte, 1);
  *val = reg.int2_ths_a.ths;

  return mm_error;
}

/**
  * @brief  xl_int2_gen_duration: [set]  The minimum duration (LSb = 1/ODR)
  *                                   of the Interrupt 1 event to be
  *                                   recognized.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of d in reg INT2_DURATION_A
  *
  */
int32_t lsm303agr_xl_int2_gen_duration_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_INT2_DURATION_A, 
                                &reg.byte, 1);
  reg.int2_duration_a.d = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_INT2_DURATION_A,
                                 &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_int2_gen_duration: [get]  The minimum duration (LSb = 1/ODR)
  *                                   of the Interrupt 1 event to be
  *                                   recognized.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of d in reg INT2_DURATION_A
  *
  */
int32_t lsm303agr_xl_int2_gen_duration_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_INT2_DURATION_A,
                                &reg.byte, 1);
  *val = reg.int2_duration_a.d;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  interrupt_pins_xl
  * @brief   This section group all the functions that manage interrupt
  *          pins of accelerometer 
  * @{
  */

/**
  * @brief  xl_high_pass_int_conf: [set]  High-pass filter on interrupts/tap
  *                                    generator
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_hp_a_t: change the values of hp in reg CTRL_REG2_A
  *
  */
int32_t lsm303agr_xl_high_pass_int_conf_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_hp_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG2_A, &reg.byte, 1);
  reg.ctrl_reg2_a.hp = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG2_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_high_pass_int_conf: [get]  High-pass filter on interrupts/tap
  *                                    generator
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_hp_a_t: Get the values of hp in reg CTRL_REG2_A
  *
  */
int32_t lsm303agr_xl_high_pass_int_conf_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_hp_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG2_A, &reg.byte, 1);
  *val = (lsm303agr_hp_a_t) reg.ctrl_reg2_a.hp;

  return mm_error;
}

/**
  * @brief  xl_pin_int1_config: [set]  Int1 pin routing configuration register.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_ctrl_reg3_a_t: registers CTRL_REG3_A
  *
  */
int32_t lsm303agr_xl_pin_int1_config_set(lsm303agr_ctx_t *ctx,
                                     lsm303agr_ctrl_reg3_a_t *val)
{
  return lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG3_A, (uint8_t*) val, 1);
}

/**
  * @brief  xl_pin_int1_config: [get]  Int1 pin routing configuration register.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_ctrl_reg3_a_t: registers CTRL_REG3_A
  *
  */
int32_t lsm303agr_xl_pin_int1_config_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_ctrl_reg3_a_t *val)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG3_A, (uint8_t*) val, 1);
}
/**
  * @brief  xl_int2_pin_detect_4d: [set]  4D enable: 4D detection is enabled
  *                                    on INT2 pin when 6D bit on
  *                                    INT2_CFG_A (34h) is set to 1.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of d4d_int2 in reg CTRL_REG5_A
  *
  */
int32_t lsm303agr_xl_int2_pin_detect_4d_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);
  reg.ctrl_reg5_a.d4d_int2 = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_int2_pin_detect_4d: [get]  4D enable: 4D detection is enabled
  *                                    on INT2 pin when 6D bit on
  *                                    INT2_CFG_A (34h) is set to 1.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of d4d_int2 in reg CTRL_REG5_A
  *
  */
int32_t lsm303agr_xl_int2_pin_detect_4d_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);
  *val = reg.ctrl_reg5_a.d4d_int2;

  return mm_error;
}

/**
  * @brief   xl_int2pin_notification_mode: [set]  Latch interrupt request
  *                                             on INT2_SRC_A (35h) register,
  *                                             with INT2_SRC_A (35h) register
  *                                             cleared by reading INT2_SRC_A
  *                                             (35h) itself.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_lir_int2_a_t: change the values of lir_int2 in
  *                               reg CTRL_REG5_A
  *
  */
int32_t lsm303agr_xl_int2pin_notification_mode_set(lsm303agr_ctx_t *ctx,
                                                 lsm303agr_lir_int2_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);
  reg.ctrl_reg5_a.lir_int2 = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   xl_int2pin_notification_mode: [get] Latch interrupt request on
  *                                            INT2_SRC_A (35h) register, with
  *                                            INT2_SRC_A (35h) register 
  *                                            cleared by reading 
  *                                            INT2_SRC_A (35h)itself.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_lir_int2_a_t: Get the values of lir_int2 in 
  *                                 reg CTRL_REG5_A
  *
  */
int32_t lsm303agr_xl_int2pin_notification_mode_get(lsm303agr_ctx_t *ctx,
                                                lsm303agr_lir_int2_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);
  *val = (lsm303agr_lir_int2_a_t) reg.ctrl_reg5_a.lir_int2;

  return mm_error;
}

/**
  * @brief  xl_int1_pin_detect_4d: [set]  4D enable: 4D detection is enabled
  *                                    on INT1 pin when 6D bit on INT1_CFG_A
  *                                    (30h) is set to 1.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of d4d_int1 in reg CTRL_REG5_A
  *
  */
int32_t lsm303agr_xl_int1_pin_detect_4d_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);
  reg.ctrl_reg5_a.d4d_int1 = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_int1_pin_detect_4d: [get]  4D enable: 4D detection is enabled
  *                                    on INT1 pin when 6D bit on INT1_CFG_A
  *                                    (30h) is set to 1.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of d4d_int1 in reg CTRL_REG5_A
  *
  */
int32_t lsm303agr_xl_int1_pin_detect_4d_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);
  *val = reg.ctrl_reg5_a.d4d_int1;

  return mm_error;
}

/**
  * @brief   xl_int1pin_notification_mode: [set]  Latch interrupt request on
  *                                             INT1_SRC_A (31h), with 
  *                                             INT1_SRC_A(31h) register 
  *                                             cleared by reading 
  *                                             INT1_SRC_A (31h) itself.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_lir_int1_a_t: change the values of lir_int1 in
  *                               reg CTRL_REG5_A
  *
  */
int32_t lsm303agr_xl_int1pin_notification_mode_set(lsm303agr_ctx_t *ctx,
                                                 lsm303agr_lir_int1_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);
  reg.ctrl_reg5_a.lir_int1 = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   xl_int1pin_notification_mode: [get]  Latch interrupt request on
  *                                             INT1_SRC_A (31h), with
  *                                             INT1_SRC_A (31h) register 
  *                                             cleared by reading INT1_SRC_A
  *                                             (31h) itself.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_lir_int1_a_t: Get the values of lir_int1 in
  *                                 reg CTRL_REG5_A
  *
  */
int32_t lsm303agr_xl_int1pin_notification_mode_get(lsm303agr_ctx_t *ctx,
                                                lsm303agr_lir_int1_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);
  *val = (lsm303agr_lir_int1_a_t) reg.ctrl_reg5_a.lir_int1;

  return mm_error;
}

/**
  * @brief  _xl_pin_int2_config: [set]  Int2 pin routing configuration register.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_ctrl_reg6_a_t: registers CTRL_REG6_A
  *
  */
int32_t lsm303agr_xl_pin_int2_config_set(lsm303agr_ctx_t *ctx,
                                     lsm303agr_ctrl_reg6_a_t *val)
{
  return lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG6_A, (uint8_t*) val, 1);
}

/**
  * @brief  _xl_pin_int2_config: [get]  Int2 pin routing configuration register.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_ctrl_reg6_a_t: registers CTRL_REG6_A
  *
  */
int32_t lsm303agr_xl_pin_int2_config_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_ctrl_reg6_a_t *val)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG6_A, (uint8_t*) val, 1);
}

/**
  * @}
  */
  
  /**
  * @addtogroup  mag interrupts
  * @brief   This section group all the functions that manage the 
  *          magnetometer interrupts
  * @{
  */

/**
  * @brief  offset_int_conf: [set]  The interrupt block recognition checks
  *                                 data after/before the hard-iron correction
  *                                 to discover the interrupt.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_int_on_dataoff_m_t: change the values of int_on_dataoff
  *                                       in reg CFG_REG_B_M
  *
  */
int32_t lsm303agr_mag_offset_int_conf_set(lsm303agr_ctx_t *ctx,
                                          lsm303agr_int_on_dataoff_m_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_B_M, &reg.byte, 1);
  reg.cfg_reg_b_m.int_on_dataoff = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_B_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  offset_int_conf: [get]  The interrupt block recognition checks
  *                                 data after/before the hard-iron correction
  *                                 to discover the interrupt.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_int_on_dataoff_m_t: Get the values of int_on_dataoff in
  *                                       reg CFG_REG_B_M
  *
  */
int32_t lsm303agr_mag_offset_int_conf_get(lsm303agr_ctx_t *ctx,
                                          lsm303agr_int_on_dataoff_m_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_B_M, &reg.byte, 1);
  *val = (lsm303agr_int_on_dataoff_m_t) reg.cfg_reg_b_m.int_on_dataoff;

  return mm_error;
}

/**
  * @brief  mag_drdy_on_pin: [set]  Data-ready signal on INT_DRDY pin.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of drdy_on_pin in reg CFG_REG_C_M
  *
  */
int32_t lsm303agr_mag_drdy_on_pin_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);
  reg.cfg_reg_c_m.int_mag = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  mag_drdy_on_pin: [get]  Data-ready signal on INT_DRDY pin.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of drdy_on_pin in reg CFG_REG_C_M
  *
  */
int32_t lsm303agr_mag_drdy_on_pin_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);
  *val = reg.cfg_reg_c_m.int_mag;

  return mm_error;
}

/**
  * @brief  mag_int_on_pin: [set]  Interrupt signal on INT_DRDY pin.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of int_on_pin in reg CFG_REG_C_M
  *
  */
int32_t lsm303agr_mag_int_on_pin_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);
  reg.cfg_reg_c_m.int_mag_pin = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  mag_int_on_pin: [get]  Interrupt signal on INT_DRDY pin.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of int_on_pin in reg CFG_REG_C_M
  *
  */
int32_t lsm303agr_mag_int_on_pin_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);
  *val = reg.cfg_reg_c_m.int_mag_pin;

  return mm_error;
}

/**
  * @brief  mag_int_gen_conf: [set] Interrupt generator configuration
  *                                 register
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_int_crtl_reg_m_t: registers INT_CRTL_REG_M
  *
  */
int32_t lsm303agr_mag_int_gen_conf_set(lsm303agr_ctx_t *ctx,
                                 lsm303agr_int_crtl_reg_m_t *val)
{
  return lsm303agr_write_reg(ctx, LSM303AGR_INT_CRTL_REG_M, (uint8_t*) val, 1);
}

/**
  * @brief  mag_int_gen_conf: [get]  Interrupt generator configuration
  *                                  register
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_int_crtl_reg_m_t: registers INT_CRTL_REG_M
  *
  */
int32_t lsm303agr_mag_int_gen_conf_get(lsm303agr_ctx_t *ctx,
                                 lsm303agr_int_crtl_reg_m_t *val)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_INT_CRTL_REG_M,
                            (uint8_t*) val, 1);
}

/**
  * @brief  mag_int_gen_source: [get]  Interrupt generator source register
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_int_source_reg_m_t: registers INT_SOURCE_REG_M
  *
  */
int32_t lsm303agr_mag_int_gen_source_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_int_source_reg_m_t *val)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_INT_SOURCE_REG_M,
                            (uint8_t*) val, 1);
}

/**
  * @brief  mag_int_gen_treshold: [set]  User-defined threshold value for xl
  *                                  interrupt event on generator.
  *                                  Data format is the same of output
  *                                  data raw: two’s complement with
  *                                  1LSb = 1.5mG.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that contains data to write
  *
  */
int32_t lsm303agr_mag_int_gen_treshold_set(lsm303agr_ctx_t *ctx,
                                           uint8_t *buff)
{
  return lsm303agr_write_reg(ctx, LSM303AGR_INT_THS_L_REG_M, buff, 2);
}

/**
  * @brief  mag_int_gen_treshold: [get]  User-defined threshold value for
  *                                  xl interrupt event on generator.
  *                                  Data format is the same of output
  *                                  data raw: two’s complement with
  *                                  1LSb = 1.5mG.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm303agr_mag_int_gen_treshold_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *buff)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_INT_THS_L_REG_M, buff, 2);
}

/**
  * @}
  */

/**
  * @addtogroup  accelerometer_fifo
  * @brief   This section group all the functions concerning the xl 
  *          fifo usage
  * @{
  */

/**
  * @brief  xl_fifo: [set] FIFOenable.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of fifo_en in reg CTRL_REG5_A
  *
  */
int32_t lsm303agr_xl_fifo_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);
  reg.ctrl_reg5_a.fifo_en = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_fifo: [get] FIFOenable.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fifo_en in reg CTRL_REG5_A
  *
  */
int32_t lsm303agr_xl_fifo_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG5_A, &reg.byte, 1);
  *val = reg.ctrl_reg5_a.fifo_en;

  return mm_error;
}

/**
  * @brief  xl_fifo_watermark: [set]  FIFO watermark level selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of fth in reg FIFO_CTRL_REG_A
  *
  */
int32_t lsm303agr_xl_fifo_watermark_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_FIFO_CTRL_REG_A,
                                &reg.byte, 1);
  reg.fifo_ctrl_reg_a.fth = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_FIFO_CTRL_REG_A,
                                 &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_fifo_watermark: [get]  FIFO watermark level selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fth in reg FIFO_CTRL_REG_A
  *
  */
int32_t lsm303agr_xl_fifo_watermark_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_FIFO_CTRL_REG_A,
                                &reg.byte, 1);
  *val = reg.fifo_ctrl_reg_a.fth;

  return mm_error;
}

/**
  * @brief  xl_fifo_trigger_event: [set]  Trigger FIFO selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_tr_a_t: change the values of tr in reg FIFO_CTRL_REG_A
  *
  */
int32_t lsm303agr_xl_fifo_trigger_event_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_tr_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_FIFO_CTRL_REG_A,
                                &reg.byte, 1);
  reg.fifo_ctrl_reg_a.tr = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_FIFO_CTRL_REG_A,
                                 &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_fifo_trigger_event: [get]  Trigger FIFO selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_tr_a_t: Get the values of tr in reg FIFO_CTRL_REG_A
  *
  */
int32_t lsm303agr_xl_fifo_trigger_event_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_tr_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_FIFO_CTRL_REG_A,
                                &reg.byte, 1);
  *val = (lsm303agr_tr_a_t) reg.fifo_ctrl_reg_a.tr;

  return mm_error;
}

/**
  * @brief  xl_fifo_mode: [set]  FIFO mode selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_fm_a_t: change the values of fm in reg FIFO_CTRL_REG_A
  *
  */
int32_t lsm303agr_xl_fifo_mode_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_fm_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_FIFO_CTRL_REG_A,
                                &reg.byte, 1);
  reg.fifo_ctrl_reg_a.fm = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_FIFO_CTRL_REG_A,
                                 &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_fifo_mode: [get]  FIFO mode selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_fm_t: Get the values of fm in reg FIFO_CTRL_REG_A
  *
  */
int32_t lsm303agr_xl_fifo_mode_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_fm_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_FIFO_CTRL_REG_A,
                                &reg.byte, 1);
  *val = (lsm303agr_fm_a_t) reg.fifo_ctrl_reg_a.fm;

  return mm_error;
}

/**
  * @brief  xl_fifo_status: [get]  FIFO status register.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_fifo_src_reg_a_t: registers FIFO_SRC_REG_A
  *
  */
int32_t lsm303agr_xl_fifo_status_get(lsm303agr_ctx_t *ctx,
                                 lsm303agr_fifo_src_reg_a_t *val)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_FIFO_SRC_REG_A,
                            (uint8_t*) val, 1);
}
/**
  * @brief  xl_fifo_data_level: [get]  FIFO stored data level.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fss in reg FIFO_SRC_REG_A
  *
  */
int32_t lsm303agr_xl_fifo_data_level_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_FIFO_SRC_REG_A,
                                &reg.byte, 1);
  *val = reg.fifo_src_reg_a.fss;

  return mm_error;
}
/**
  * @brief  xl_fifo_empty_flag: [get]  Empty FIFO status flag.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of empty in reg FIFO_SRC_REG_A
  *
  */
int32_t lsm303agr_xl_fifo_empty_flag_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_FIFO_SRC_REG_A,
                                &reg.byte, 1);
  *val = reg.fifo_src_reg_a.empty;

  return mm_error;
}
/**
  * @brief  xl_fifo_ovr_flag: [get]  FIFO overrun status flag.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ovrn_fifo in reg FIFO_SRC_REG_A
  *
  */
int32_t lsm303agr_xl_fifo_ovr_flag_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_FIFO_SRC_REG_A,
                                &reg.byte, 1);
  *val = reg.fifo_src_reg_a.ovrn_fifo;

  return mm_error;
}
/**
  * @brief  xl_fifo_fth_flag: [get]  FIFO watermark status.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of wtm in reg FIFO_SRC_REG_A
  *
  */
int32_t lsm303agr_xl_fifo_fth_flag_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_FIFO_SRC_REG_A, &reg.byte, 1);
  *val = reg.fifo_src_reg_a.wtm;

  return mm_error;
}
/**
  * @}
  */

/**
  * @addtogroup  tap_generator
  * @brief   This section group all the functions that manage the tap and
  *          double tap event generation
  * @{
  */

/**
  * @brief  tap_conf: [set]  Tap/Double Tap generator configuration register
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_click_cfg_a_t: registers CLICK_CFG_A
  *
  */
int32_t lsm303agr_tap_conf_set(lsm303agr_ctx_t *ctx,
                               lsm303agr_click_cfg_a_t *val)
{
  return lsm303agr_write_reg(ctx, LSM303AGR_CLICK_CFG_A, (uint8_t*) val, 1);
}

/**
  * @brief  tap_conf: [get]  Tap/Double Tap generator configuration register
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_click_cfg_a_t: registers CLICK_CFG_A
  *
  */
int32_t lsm303agr_tap_conf_get(lsm303agr_ctx_t *ctx,
                               lsm303agr_click_cfg_a_t *val)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_CLICK_CFG_A, (uint8_t*) val, 1);
}
/**
  * @brief  tap_source: [get]  Tap/Double Tap generator source register
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_click_cfg_t: registers CLICK_SRC_A
  *
  */
int32_t lsm303agr_tap_source_get(lsm303agr_ctx_t *ctx,
                                 lsm303agr_click_src_a_t *val)
{
  return lsm303agr_read_reg(ctx, LSM303AGR_CLICK_SRC_A, (uint8_t*) val, 1);
}
/**
  * @brief  tap_threshold: [set]  User-defined threshold value for Tap/Double
  *                               Tap event. (1 LSB = full scale/128)
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of ths in reg CLICK_THS_A
  *
  */
int32_t lsm303agr_tap_threshold_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CLICK_THS_A, &reg.byte, 1);
  reg.click_ths_a.ths = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CLICK_THS_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  tap_threshold: [get]  User-defined threshold value for Tap/Double
  *                               Tap event. (1 LSB = full scale/128)
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ths in reg CLICK_THS_A
  *
  */
int32_t lsm303agr_tap_threshold_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CLICK_THS_A, &reg.byte, 1);
  *val = reg.click_ths_a.ths;

  return mm_error;
}

/**
  * @brief  shock_dur: [set]  The maximum time (1 LSB = 1/ODR) interval
  *                           that can elapse between the start of the
  *                           click-detection procedure and when the
  *                           acceleration falls back below the threshold.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of tli in reg TIME_LIMIT_A
  *
  */
int32_t lsm303agr_shock_dur_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_TIME_LIMIT_A, &reg.byte, 1);
  reg.time_limit_a.tli = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_TIME_LIMIT_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  shock_dur: [get]  The maximum time (1 LSB = 1/ODR) interval
  *                           that can elapse between the start of the
  *                           click-detection procedure and when the
  *                           acceleration falls back below the threshold.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tli in reg TIME_LIMIT_A
  *
  */
int32_t lsm303agr_shock_dur_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_TIME_LIMIT_A, &reg.byte, 1);
  *val = reg.time_limit_a.tli;

  return mm_error;
}

/**
  * @brief  quiet_dur: [set]  The time (1 LSB = 1/ODR) interval that
  *                           starts after the first click detection
  *                           where the click-detection procedure is
  *                           disabled, in cases where the device is
  *                           configured for double-click detection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of tla in reg TIME_LATENCY_A
  *
  */
int32_t lsm303agr_quiet_dur_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_TIME_LATENCY_A,
                                &reg.byte, 1);
  reg.time_latency_a.tla = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_TIME_LATENCY_A,
                                 &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  quiet_dur: [get]  The time (1 LSB = 1/ODR) interval that
  *                           starts after the first click detection
  *                           where the click-detection procedure is
  *                           disabled, in cases where the device is
  *                           configured for double-click detection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tla in reg TIME_LATENCY_A
  *
  */
int32_t lsm303agr_quiet_dur_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_TIME_LATENCY_A,
                                &reg.byte, 1);
  *val = reg.time_latency_a.tla;

  return mm_error;
}

/**
  * @brief  double_tap_timeout: [set]  The maximum interval of time
  *                                    (1 LSB = 1/ODR) that can elapse
  *                                    after the end of the latency interval
  *                                    in which the click-detection procedure
  *                                    can start, in cases where the device is
  *                                    configured for double-click detection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of tw in reg TIME_WINDOW_A
  *
  */
int32_t lsm303agr_double_tap_timeout_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_TIME_WINDOW_A, &reg.byte, 1);
  reg.time_window_a.tw = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_TIME_WINDOW_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  double_tap_timeout: [get]  The maximum interval of time
  *                                    (1 LSB = 1/ODR) that can elapse
  *                                    after the end of the latency interval
  *                                    in which the click-detection procedure
  *                                    can start, in cases where the device
  *                                    is configured for double-click
  *                                    detection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tw in reg TIME_WINDOW_A
  *
  */
int32_t lsm303agr_double_tap_timeout_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_TIME_WINDOW_A, &reg.byte, 1);
  *val = reg.time_window_a.tw;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup   activity_inactivity_xl
  * @brief   This section group all the functions concerning activity
  *          inactivity functionality foe accelerometer
  * @{
  */

/**
  * @brief  act_threshold: [set]  Sleep-to-wake, return-to-sleep activation
  *                               threshold in low-power mode.
  *                        1 LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of acth in reg ACT_THS_A
  *
  */
int32_t lsm303agr_act_threshold_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_ACT_THS_A, &reg.byte, 1);
  reg.act_ths_a.acth = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_ACT_THS_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  act_threshold: [get]  Sleep-to-wake, return-to-sleep activation
  *                               threshold in low-power mode.
  *                        1 LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of acth in reg ACT_THS_A
  *
  */
int32_t lsm303agr_act_threshold_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_ACT_THS_A, &reg.byte, 1);
  *val = reg.act_ths_a.acth;

  return mm_error;
}

/**
  * @brief  act_timeout: [set]  Sleep-to-wake, return-to-sleep
  *                             duration = (8*1[LSb]+1)/ODR.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of actd in reg ACT_DUR_A
  *
  */
int32_t lsm303agr_act_timeout_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_ACT_DUR_A, &reg.byte, 1);
  reg.act_dur_a.actd = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_ACT_DUR_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  act_timeout: [get]  Sleep-to-wake, return-to-sleep
  *                             duration = (8*1[LSb]+1)/ODR.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of actd in reg ACT_DUR_A
  *
  */
int32_t lsm303agr_act_timeout_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_ACT_DUR_A, &reg.byte, 1);
  *val = reg.act_dur_a.actd;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  serial_interface
  * @brief   This section group all the functions concerning serial
  *          interface management
  * @{
  */

/**
  * @brief  xl_spi_mode: [set]  SPI Serial Interface Mode selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_sim_a_t: change the values of sim in reg CTRL_REG4_A
  *
  */
int32_t lsm303agr_xl_spi_mode_set(lsm303agr_ctx_t *ctx,
                                  lsm303agr_sim_a_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);
  reg.ctrl_reg4_a.spi_enable = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  xl_spi_mode: [get]  SPI Serial Interface Mode selection.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_sim_a_t: Get the values of sim in reg CTRL_REG4_A
  *
  */
int32_t lsm303agr_xl_spi_mode_get(lsm303agr_ctx_t *ctx,
                                  lsm303agr_sim_a_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CTRL_REG4_A, &reg.byte, 1);
  *val = (lsm303agr_sim_a_t) reg.ctrl_reg4_a.spi_enable;

  return mm_error;
}
/**
  * @brief  i2c_interface: [set]  Enable/Disable I2C interface.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_i2c_dis_m_t: change the values of i2c_dis in
  *                                reg CFG_REG_C_M
  *
  */
int32_t lsm303agr_mag_i2c_interface_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_i2c_dis_m_t val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);
  reg.cfg_reg_c_m.i2c_dis = val;
  mm_error = lsm303agr_write_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  i2c_interface: [get]  Enable/Disable I2C interface.
  *
  * @param  lsm303agr_ctx_t *ctx: read / write interface definitions
  * @param  lsm303agr_i2c_dis_m_t: Get the values of i2c_dis in
  *                                reg CFG_REG_C_M
  *
  */
int32_t lsm303agr_mag_i2c_interface_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_i2c_dis_m_t *val)
{
  lsm303agr_reg_t reg;
  int32_t mm_error;

  mm_error = lsm303agr_read_reg(ctx, LSM303AGR_CFG_REG_C_M, &reg.byte, 1);
  *val = (lsm303agr_i2c_dis_m_t) reg.cfg_reg_c_m.i2c_dis;

  return mm_error;
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/