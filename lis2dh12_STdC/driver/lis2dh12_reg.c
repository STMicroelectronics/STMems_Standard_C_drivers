/*
 ******************************************************************************
 * @file    lis2dh12_reg.c
 * @author  MEMS Software Solution Team
 * @date    05-October-2017
 * @brief   LIS2DH12 driver file
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

#include "lis2dh12_reg.h"

/**
  * @addtogroup  lis2dh12
  * @brief  This file provides a set of functions needed to drive the
  *         lis2dh12 enanced inertial module.
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
  * @param  lis2dh12_ctx_t* ctx: read / write interface definitions
  * @param  uint8_t reg: register to read
  * @param  uint8_t* data: pointer to buffer that store the data read
  * @param  uint16_t len: number of consecutive register to read
  *
  */
int32_t lis2dh12_read_reg(lis2dh12_ctx_t* ctx, uint8_t reg, uint8_t* data,
                          uint16_t len)
{
  return ctx->read_reg(ctx->handle, reg, data, len);
}

/**
  * @brief  Write generic device register
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t reg: register to write
  * @param  uint8_t* data: pointer to data to write in register reg
  * @param  uint16_t len: number of consecutive register to write
  *
*/
int32_t lis2dh12_write_reg(lis2dh12_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis2dh12_temp_status_reg_get(lis2dh12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_STATUS_REG_AUX, buff, 1);
}
/**
  * @brief  temp_data_ready: [get]  Temperature data available.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tda in reg STATUS_REG_AUX
  *
  */
int32_t lis2dh12_temp_data_ready_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_STATUS_REG_AUX, &reg.byte, 1);
  *val = reg.status_reg_aux.tda;

  return mm_error;
}
/**
  * @brief  temp_data_ovr: [get]  Temperature data overrun.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tor in reg STATUS_REG_AUX
  *
  */
int32_t lis2dh12_temp_data_ovr_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_STATUS_REG_AUX, &reg.byte, 1);
  *val = reg.status_reg_aux.tor;

  return mm_error;
}
/**
  * @brief  temperature_raw: [get]  Temperature output value.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis2dh12_temperature_raw_get(lis2dh12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_OUT_TEMP_L, buff, 2);
}
/**
  * @brief  temperature_meas: [set]  Temperature sensor enable.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_temp_en_t: change the values of temp_en in
  *                             reg TEMP_CFG_REG
  *
  */
int32_t lis2dh12_temperature_meas_set(lis2dh12_ctx_t *ctx,
                                      lis2dh12_temp_en_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_TEMP_CFG_REG, &reg.byte, 1);
  reg.temp_cfg_reg.temp_en = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_TEMP_CFG_REG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  temperature_meas: [get]  Temperature sensor enable.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_temp_en_t: Get the values of temp_en in reg TEMP_CFG_REG
  *
  */
int32_t lis2dh12_temperature_meas_get(lis2dh12_ctx_t *ctx,
                                      lis2dh12_temp_en_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_TEMP_CFG_REG, &reg.byte, 1);
  *val = (lis2dh12_temp_en_t) reg.temp_cfg_reg.temp_en;

  return mm_error;
}

/**
  * @brief  operating_mode: [set]  Operating mode selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_op_md_t val: change the values of lpen in reg CTRL_REG1
  *                               and HR in reg CTRL_REG4
  *
  */
int32_t lis2dh12_operating_mode_set(lis2dh12_ctx_t *ctx, lis2dh12_op_md_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;
  uint8_t lpen, hr;

  if ( val == LIS2DH12_HR_12bit ){
    lpen = 0;
    hr   = 1;
  } else if (val == LIS2DH12_NM_10bit) {
    lpen = 0;
    hr   = 0;
  } else {
    lpen = 1;
    hr   = 0;
  }

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.lpen = lpen;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG1, &reg.byte, 1);
  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.hr = hr;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  operating_mode: [get]  Operating mode selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of lpen in reg CTRL_REG1
  *
  */
int32_t lis2dh12_operating_mode_get(lis2dh12_ctx_t *ctx, lis2dh12_op_md_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;
  uint8_t lpen, hr;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG1, &reg.byte, 1);
  lpen = reg.ctrl_reg1.lpen;
  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);
  hr = reg.ctrl_reg4.hr;

  if ( lpen ){
    *val = LIS2DH12_LP_8bit;
  } else if (hr) {
    *val = LIS2DH12_HR_12bit;
  } else{
    *val = LIS2DH12_NM_10bit;
  }

  return mm_error;
}

/**
  * @brief  data_rate: [set]  Output data rate selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_odr_t: change the values of odr in reg CTRL_REG1
  *
  */
int32_t lis2dh12_data_rate_set(lis2dh12_ctx_t *ctx, lis2dh12_odr_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.odr = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG1, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  data_rate: [get]  Output data rate selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_odr_t: Get the values of odr in reg CTRL_REG1
  *
  */
int32_t lis2dh12_data_rate_get(lis2dh12_ctx_t *ctx, lis2dh12_odr_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG1, &reg.byte, 1);
  *val = (lis2dh12_odr_t) reg.ctrl_reg1.odr;

  return mm_error;
}

/**
  * @brief   high_pass_on_outputs: [set]  High pass data from internal
  *                                       filter sent to output register
  *                                       and FIFO
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of fds in reg CTRL_REG2
  *
  */
int32_t lis2dh12_high_pass_on_outputs_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.fds = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   high_pass_on_outputs: [get]  High pass data from internal
  *                                       filter sent to output register
  *                                       and FIFO
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fds in reg CTRL_REG2
  *
  */
int32_t lis2dh12_high_pass_on_outputs_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG2, &reg.byte, 1);
  *val = reg.ctrl_reg2.fds;

  return mm_error;
}

/**
  * @brief   high_pass_bandwidth: [set]  High-pass filter cutoff
  *                                      frequency selection
  *
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_hpcf_t: change the values of hpcf in reg CTRL_REG2
  *
  * HPCF[2:1]\ft @1Hz    @10Hz  @25Hz  @50Hz @100Hz @200Hz @400Hz @1kHz6 ft@5kHz
  * AGGRESSIVE   0.02Hz  0.2Hz  0.5Hz  1Hz   2Hz    4Hz    8Hz    32Hz   100Hz
  * STRONG       0.008Hz 0.08Hz 0.2Hz  0.5Hz 1Hz    2Hz    4Hz    16Hz   50Hz
  * MEDIUM       0.004Hz 0.04Hz 0.1Hz  0.2Hz 0.5Hz  1Hz    2Hz    8Hz    25Hz
  * LIGHT        0.002Hz 0.02Hz 0.05Hz 0.1Hz 0.2Hz  0.5Hz  1Hz    4Hz    12Hz
  *
  */
int32_t lis2dh12_high_pass_bandwidth_set(lis2dh12_ctx_t *ctx,
                                         lis2dh12_hpcf_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.hpcf = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   high_pass_bandwidth: [get]  High-pass filter cutoff
  *                                      frequency selection
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_hpcf_t: Get the values of hpcf in reg CTRL_REG2
  *
  * HPCF[2:1]\ft @1Hz    @10Hz  @25Hz  @50Hz @100Hz @200Hz @400Hz @1kHz6 ft@5kHz
  * AGGRESSIVE   0.02Hz  0.2Hz  0.5Hz  1Hz   2Hz    4Hz    8Hz    32Hz   100Hz
  * STRONG       0.008Hz 0.08Hz 0.2Hz  0.5Hz 1Hz    2Hz    4Hz    16Hz   50Hz
  * MEDIUM       0.004Hz 0.04Hz 0.1Hz  0.2Hz 0.5Hz  1Hz    2Hz    8Hz    25Hz
  * LIGHT        0.002Hz 0.02Hz 0.05Hz 0.1Hz 0.2Hz  0.5Hz  1Hz    4Hz    12Hz
  *
  */
int32_t lis2dh12_high_pass_bandwidth_get(lis2dh12_ctx_t *ctx,
                                         lis2dh12_hpcf_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG2, &reg.byte, 1);
  *val = (lis2dh12_hpcf_t) reg.ctrl_reg2.hpcf;

  return mm_error;
}

/**
  * @brief  high_pass_mode: [set]  High-pass filter mode selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_hpm_t: change the values of hpm in reg CTRL_REG2
  *
  */
int32_t lis2dh12_high_pass_mode_set(lis2dh12_ctx_t *ctx, lis2dh12_hpm_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.hpm = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  high_pass_mode: [get]  High-pass filter mode selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_hpm_t: Get the values of hpm in reg CTRL_REG2
  *
  */
int32_t lis2dh12_high_pass_mode_get(lis2dh12_ctx_t *ctx, lis2dh12_hpm_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG2, &reg.byte, 1);
  *val = (lis2dh12_hpm_t) reg.ctrl_reg2.hpm;

  return mm_error;
}

/**
  * @brief  full_scale: [set]  Full-scale configuration.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_fs_t: change the values of fs in reg CTRL_REG4
  *
  */
int32_t lis2dh12_full_scale_set(lis2dh12_ctx_t *ctx, lis2dh12_fs_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.fs = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  full_scale: [get]  Full-scale configuration.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_fs_t: Get the values of fs in reg CTRL_REG4
  *
  */
int32_t lis2dh12_full_scale_get(lis2dh12_ctx_t *ctx, lis2dh12_fs_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);
  *val = (lis2dh12_fs_t) reg.ctrl_reg4.fs;

  return mm_error;
}

/**
  * @brief  block_data_update: [set] Blockdataupdate.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of bdu in reg CTRL_REG4
  *
  */
int32_t lis2dh12_block_data_update_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.bdu = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  block_data_update: [get] Blockdataupdate.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of bdu in reg CTRL_REG4
  *
  */
int32_t lis2dh12_block_data_update_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);
  *val = reg.ctrl_reg4.bdu;

  return mm_error;
}

/**
  * @brief  filter_reference: [set]  Reference value for interrupt generation.
  *                                  LSB = ~16@2g / ~31@4g / ~63@8g / ~127@16g
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that contains data to write
  *
  */
int32_t lis2dh12_filter_reference_set(lis2dh12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dh12_write_reg(ctx, LIS2DH12_REFERENCE, buff, 1);
}

/**
  * @brief  filter_reference: [get]  Reference value for interrupt generation.
  *                                  LSB = ~16@2g / ~31@4g / ~63@8g / ~127@16g
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis2dh12_filter_reference_get(lis2dh12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_REFERENCE, buff, 1);
}
/**
  * @brief  xl_data_ready: [get]  Acceleration set of data available.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of zyxda in reg STATUS_REG
  *
  */
int32_t lis2dh12_xl_data_ready_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_STATUS_REG, &reg.byte, 1);
  *val = reg.status_reg.zyxda;

  return mm_error;
}
/**
  * @brief  xl_data_ovr: [get]  Acceleration set of data overrun.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of zyxor in reg STATUS_REG
  *
  */
int32_t lis2dh12_xl_data_ovr_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_STATUS_REG, &reg.byte, 1);
  *val = reg.status_reg.zyxor;

  return mm_error;
}
/**
  * @brief  acceleration_raw: [get]  Acceleration output value.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis2dh12_acceleration_raw_get(lis2dh12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_OUT_X_L, buff, 6);
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
  * @brief  device_id: [get] DeviceWhoamI.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis2dh12_device_id_get(lis2dh12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_WHO_AM_I, buff, 1);
}
/**
  * @brief  self_test: [set] Selftest.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_st_t: change the values of st in reg CTRL_REG4
  *
  */
int32_t lis2dh12_self_test_set(lis2dh12_ctx_t *ctx, lis2dh12_st_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.st = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  self_test: [get] Selftest.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_st_t: Get the values of st in reg CTRL_REG4
  *
  */
int32_t lis2dh12_self_test_get(lis2dh12_ctx_t *ctx, lis2dh12_st_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);
  *val = (lis2dh12_st_t) reg.ctrl_reg4.st;

  return mm_error;
}

/**
  * @brief  data_format: [set]  Big/Little Endian data selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_ble_t: change the values of ble in reg CTRL_REG4
  *
  */
int32_t lis2dh12_data_format_set(lis2dh12_ctx_t *ctx, lis2dh12_ble_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.ble = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  data_format: [get]  Big/Little Endian data selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_ble_t: Get the values of ble in reg CTRL_REG4
  *
  */
int32_t lis2dh12_data_format_get(lis2dh12_ctx_t *ctx, lis2dh12_ble_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);
  *val = (lis2dh12_ble_t) reg.ctrl_reg4.ble;

  return mm_error;
}

/**
  * @brief  boot: [set]  Reboot memory content. Reload the
  *                      calibration parameters
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of boot in reg CTRL_REG5
  *
  */
int32_t lis2dh12_boot_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);
  reg.ctrl_reg5.boot = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  boot: [get]  Reboot memory content. Reload the calibration
  *                      parameters
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of boot in reg CTRL_REG5
  *
  */
int32_t lis2dh12_boot_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);
  *val = reg.ctrl_reg5.boot;

  return mm_error;
}

/**
  * @brief  status: [get]  Info about device status.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_status_reg_t: register STATUS_REG
  *
  */
int32_t lis2dh12_status_get(lis2dh12_ctx_t *ctx, lis2dh12_status_reg_t *val)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_STATUS_REG, (uint8_t*) val, 1);
}
/**
  * @}
  */

/**
  * @addtogroup   interrupts_generator_1
  * @brief   This section group all the functions that manage the first
  *          interrupts generator
  * @{
  */

/**
  * @brief  int1_gen_conf: [set]  Interrupt generator 1 configuration register
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_int1_cfg_t: register INT1_CFG
  *
  */
int32_t lis2dh12_int1_gen_conf_set(lis2dh12_ctx_t *ctx,
                                   lis2dh12_int1_cfg_t *val)
{
  return lis2dh12_write_reg(ctx, LIS2DH12_INT1_CFG, (uint8_t*) val, 1);
}

/**
  * @brief  int1_gen_conf: [get]  Interrupt generator 1 configuration register
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_int1_cfg_t: register INT1_CFG
  *
  */
int32_t lis2dh12_int1_gen_conf_get(lis2dh12_ctx_t *ctx,
                                   lis2dh12_int1_cfg_t *val)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_INT1_CFG, (uint8_t*) val, 1);
}

/**
  * @brief  int1_gen_source: [get]  Interrupt generator 1 source register
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_int1_src_t: Registers INT1_SRC
  *
  */
int32_t lis2dh12_int1_gen_source_get(lis2dh12_ctx_t *ctx,
                                     lis2dh12_int1_src_t *val)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_INT1_SRC, (uint8_t*) val, 1);
}
/**
  * @brief  int1_gen_threshold: [set]  User-defined threshold value for xl
  *                                    interrupt event on generator 1.
  *                             LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of ths in reg INT1_THS
  *
  */
int32_t lis2dh12_int1_gen_threshold_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_INT1_THS, &reg.byte, 1);
  reg.int1_ths.ths = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_INT1_THS, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int1_gen_threshold: [get]  User-defined threshold value for xl
  *                                    interrupt event on generator 1.
  *                             LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ths in reg INT1_THS
  *
  */
int32_t lis2dh12_int1_gen_threshold_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_INT1_THS, &reg.byte, 1);
  *val = reg.int1_ths.ths;

  return mm_error;
}

/**
  * @brief  int1_gen_duration: [set]  The minimum duration (LSb = 1/ODR)
  *                                   of the Interrupt 1 event to be
  *                                   recognized.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of d in reg INT1_DURATION
  *
  */
int32_t lis2dh12_int1_gen_duration_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_INT1_DURATION, &reg.byte, 1);
  reg.int1_duration.d = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_INT1_DURATION, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int1_gen_duration: [get]  The minimum duration (LSb = 1/ODR)
  *                                   of the Interrupt 1 event to be
  *                                   recognized.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of d in reg INT1_DURATION
  *
  */
int32_t lis2dh12_int1_gen_duration_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_INT1_DURATION, &reg.byte, 1);
  *val = reg.int1_duration.d;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup   interrupts_generator_2
  * @brief   This section group all the functions that manage the second
  *          interrupts generator
  * @{
  */

/**
  * @brief  int2_gen_conf: [set]  Interrupt generator 2 configuration register
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_int2_cfg_t: registers INT2_CFG
  *
  */
int32_t lis2dh12_int2_gen_conf_set(lis2dh12_ctx_t *ctx,
                                   lis2dh12_int2_cfg_t *val)
{
  return lis2dh12_write_reg(ctx, LIS2DH12_INT2_CFG, (uint8_t*) val, 1);
}

/**
  * @brief  int2_gen_conf: [get]  Interrupt generator 2 configuration register
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_int2_cfg_t: registers INT2_CFG
  *
  */
int32_t lis2dh12_int2_gen_conf_get(lis2dh12_ctx_t *ctx,
                                   lis2dh12_int2_cfg_t *val)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_INT2_CFG, (uint8_t*) val, 1);
}
/**
  * @brief  int2_gen_source: [get]  Interrupt generator 2 source register
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_int2_src_t: registers INT2_SRC
  *
  */
int32_t lis2dh12_int2_gen_source_get(lis2dh12_ctx_t *ctx,
                                     lis2dh12_int2_src_t *val)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_INT2_SRC, (uint8_t*) val, 1);
}
/**
  * @brief  int2_gen_threshold: [set]  User-defined threshold value for xl
  *                                    interrupt event on generator 2.
  *                             LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of ths in reg INT2_THS
  *
  */
int32_t lis2dh12_int2_gen_threshold_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_INT2_THS, &reg.byte, 1);
  reg.int2_ths.ths = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_INT2_THS, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int2_gen_threshold: [get]  User-defined threshold value for
  *                                    xl interrupt event on generator 2.
  *                             LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ths in reg INT2_THS
  *
  */
int32_t lis2dh12_int2_gen_threshold_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_INT2_THS, &reg.byte, 1);
  *val = reg.int2_ths.ths;

  return mm_error;
}

/**
  * @brief  int2_gen_duration: [set]  The minimum duration (LSb = 1/ODR)
  *                                   of the Interrupt 1 event to be
  *                                   recognized.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of d in reg INT2_DURATION
  *
  */
int32_t lis2dh12_int2_gen_duration_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_INT2_DURATION, &reg.byte, 1);
  reg.int2_duration.d = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_INT2_DURATION, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int2_gen_duration: [get]  The minimum duration (LSb = 1/ODR)
  *                                   of the Interrupt 1 event to be
  *                                   recognized.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of d in reg INT2_DURATION
  *
  */
int32_t lis2dh12_int2_gen_duration_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_INT2_DURATION, &reg.byte, 1);
  *val = reg.int2_duration.d;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  interrupt_pins
  * @brief   This section group all the functions that manage interrup pins
  * @{
  */

/**
  * @brief  high_pass_int_conf: [set]  High-pass filter on interrupts/tap
  *                                    generator
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_hp_t: change the values of hp in reg CTRL_REG2
  *
  */
int32_t lis2dh12_high_pass_int_conf_set(lis2dh12_ctx_t *ctx,
                                        lis2dh12_hp_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.hp = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  high_pass_int_conf: [get]  High-pass filter on interrupts/tap
  *                                    generator
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_hp_t: Get the values of hp in reg CTRL_REG2
  *
  */
int32_t lis2dh12_high_pass_int_conf_get(lis2dh12_ctx_t *ctx,
                                        lis2dh12_hp_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG2, &reg.byte, 1);
  *val = (lis2dh12_hp_t) reg.ctrl_reg2.hp;

  return mm_error;
}

/**
  * @brief  pin_int1_config: [set]  Int1 pin routing configuration register.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_ctrl_reg3_t: registers CTRL_REG3
  *
  */
int32_t lis2dh12_pin_int1_config_set(lis2dh12_ctx_t *ctx,
                                     lis2dh12_ctrl_reg3_t *val)
{
  return lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG3, (uint8_t*) val, 1);
}

/**
  * @brief  pin_int1_config: [get]  Int1 pin routing configuration register.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_ctrl_reg3_t: registers CTRL_REG3
  *
  */
int32_t lis2dh12_pin_int1_config_get(lis2dh12_ctx_t *ctx,
                                     lis2dh12_ctrl_reg3_t *val)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG3, (uint8_t*) val, 1);
}
/**
  * @brief  int2_pin_detect_4d: [set]  4D enable: 4D detection is enabled
  *                                    on INT2 pin when 6D bit on
  *                                    INT2_CFG (34h) is set to 1.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of d4d_int2 in reg CTRL_REG5
  *
  */
int32_t lis2dh12_int2_pin_detect_4d_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);
  reg.ctrl_reg5.d4d_int2 = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int2_pin_detect_4d: [get]  4D enable: 4D detection is enabled
  *                                    on INT2 pin when 6D bit on
  *                                    INT2_CFG (34h) is set to 1.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of d4d_int2 in reg CTRL_REG5
  *
  */
int32_t lis2dh12_int2_pin_detect_4d_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);
  *val = reg.ctrl_reg5.d4d_int2;

  return mm_error;
}

/**
  * @brief   int2_pin_notification_mode: [set]  Latch interrupt request
  *                                             on INT2_SRC (35h) register,
  *                                             with INT2_SRC (35h) register
  *                                             cleared by reading INT2_SRC
  *                                             (35h) itself.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_lir_int2_t: change the values of lir_int2 in reg CTRL_REG5
  *
  */
int32_t lis2dh12_int2_pin_notification_mode_set(lis2dh12_ctx_t *ctx, lis2dh12_lir_int2_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);
  reg.ctrl_reg5.lir_int2 = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   int2_pin_notification_mode: [get]  Latch interrupt request on
  *                                             INT2_SRC (35h) register, with
  *                                             INT2_SRC (35h) register cleared
  *                                             by reading INT2_SRC (35h)itself.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_lir_int2_t: Get the values of lir_int2 in reg CTRL_REG5
  *
  */
int32_t lis2dh12_int2_pin_notification_mode_get(lis2dh12_ctx_t *ctx,
                                                lis2dh12_lir_int2_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);
  *val = (lis2dh12_lir_int2_t) reg.ctrl_reg5.lir_int2;

  return mm_error;
}

/**
  * @brief  int1_pin_detect_4d: [set]  4D enable: 4D detection is enabled
  *                                    on INT1 pin when 6D bit on INT1_CFG
  *                                    (30h) is set to 1.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of d4d_int1 in reg CTRL_REG5
  *
  */
int32_t lis2dh12_int1_pin_detect_4d_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);
  reg.ctrl_reg5.d4d_int1 = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int1_pin_detect_4d: [get]  4D enable: 4D detection is enabled
  *                                    on INT1 pin when 6D bit on INT1_CFG
  *                                    (30h) is set to 1.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of d4d_int1 in reg CTRL_REG5
  *
  */
int32_t lis2dh12_int1_pin_detect_4d_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);
  *val = reg.ctrl_reg5.d4d_int1;

  return mm_error;
}

/**
  * @brief   int1_pin_notification_mode: [set]  Latch interrupt request on
  *                                             INT1_SRC (31h), with INT1_SRC
  *                                             (31h) register cleared by
  *                                             reading INT1_SRC (31h) itself.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_lir_int1_t: change the values of lir_int1 in reg CTRL_REG5
  *
  */
int32_t lis2dh12_int1_pin_notification_mode_set(lis2dh12_ctx_t *ctx,
                                                lis2dh12_lir_int1_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);
  reg.ctrl_reg5.lir_int1 = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   int1_pin_notification_mode: [get]  Latch interrupt request on
  *                                             INT1_SRC (31h), with INT1_SRC
  *                                             (31h) register cleared by
  *                                             reading INT1_SRC (31h) itself.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_lir_int1_t: Get the values of lir_int1 in reg CTRL_REG5
  *
  */
int32_t lis2dh12_int1_pin_notification_mode_get(lis2dh12_ctx_t *ctx,
                                                lis2dh12_lir_int1_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);
  *val = (lis2dh12_lir_int1_t) reg.ctrl_reg5.lir_int1;

  return mm_error;
}

/**
  * @brief  pin_int2_config: [set]  Int2 pin routing configuration register.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_ctrl_reg6_t: registers CTRL_REG6
  *
  */
int32_t lis2dh12_pin_int2_config_set(lis2dh12_ctx_t *ctx,
                                     lis2dh12_ctrl_reg6_t *val)
{
  return lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG6, (uint8_t*) val, 1);
}

/**
  * @brief  pin_int2_config: [get]  Int2 pin routing configuration register.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_ctrl_reg6_t: registers CTRL_REG6
  *
  */
int32_t lis2dh12_pin_int2_config_get(lis2dh12_ctx_t *ctx,
                                     lis2dh12_ctrl_reg6_t *val)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG6, (uint8_t*) val, 1);
}
/**
  * @}
  */

/**
  * @addtogroup  fifo
  * @brief   This section group all the functions concerning the fifo usage
  * @{
  */

/**
  * @brief  fifo: [set] FIFOenable.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of fifo_en in reg CTRL_REG5
  *
  */
int32_t lis2dh12_fifo_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);
  reg.ctrl_reg5.fifo_en = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  fifo: [get] FIFOenable.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fifo_en in reg CTRL_REG5
  *
  */
int32_t lis2dh12_fifo_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG5, &reg.byte, 1);
  *val = reg.ctrl_reg5.fifo_en;

  return mm_error;
}

/**
  * @brief  fifo_watermark: [set]  FIFO watermark level selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of fth in reg FIFO_CTRL_REG
  *
  */
int32_t lis2dh12_fifo_watermark_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_FIFO_CTRL_REG, &reg.byte, 1);
  reg.fifo_ctrl_reg.fth = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_FIFO_CTRL_REG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  fifo_watermark: [get]  FIFO watermark level selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fth in reg FIFO_CTRL_REG
  *
  */
int32_t lis2dh12_fifo_watermark_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_FIFO_CTRL_REG, &reg.byte, 1);
  *val = reg.fifo_ctrl_reg.fth;

  return mm_error;
}

/**
  * @brief  fifo_trigger_event: [set]  Trigger FIFO selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_tr_t: change the values of tr in reg FIFO_CTRL_REG
  *
  */
int32_t lis2dh12_fifo_trigger_event_set(lis2dh12_ctx_t *ctx,
                                        lis2dh12_tr_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_FIFO_CTRL_REG, &reg.byte, 1);
  reg.fifo_ctrl_reg.tr = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_FIFO_CTRL_REG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  fifo_trigger_event: [get]  Trigger FIFO selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_tr_t: Get the values of tr in reg FIFO_CTRL_REG
  *
  */
int32_t lis2dh12_fifo_trigger_event_get(lis2dh12_ctx_t *ctx,
                                        lis2dh12_tr_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_FIFO_CTRL_REG, &reg.byte, 1);
  *val = (lis2dh12_tr_t) reg.fifo_ctrl_reg.tr;

  return mm_error;
}

/**
  * @brief  fifo_mode: [set]  FIFO mode selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_fm_t: change the values of fm in reg FIFO_CTRL_REG
  *
  */
int32_t lis2dh12_fifo_mode_set(lis2dh12_ctx_t *ctx, lis2dh12_fm_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_FIFO_CTRL_REG, &reg.byte, 1);
  reg.fifo_ctrl_reg.fm = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_FIFO_CTRL_REG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  fifo_mode: [get]  FIFO mode selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_fm_t: Get the values of fm in reg FIFO_CTRL_REG
  *
  */
int32_t lis2dh12_fifo_mode_get(lis2dh12_ctx_t *ctx, lis2dh12_fm_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_FIFO_CTRL_REG, &reg.byte, 1);
  *val = (lis2dh12_fm_t) reg.fifo_ctrl_reg.fm;

  return mm_error;
}

/**
  * @brief  fifo_status: [get]  FIFO status register.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_fifo_src_reg_t: registers FIFO_SRC_REG
  *
  */
int32_t lis2dh12_fifo_status_get(lis2dh12_ctx_t *ctx,
                                 lis2dh12_fifo_src_reg_t *val)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_FIFO_SRC_REG, (uint8_t*) val, 1);
}
/**
  * @brief  fifo_data_level: [get]  FIFO stored data level.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fss in reg FIFO_SRC_REG
  *
  */
int32_t lis2dh12_fifo_data_level_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_FIFO_SRC_REG, &reg.byte, 1);
  *val = reg.fifo_src_reg.fss;

  return mm_error;
}
/**
  * @brief  fifo_empty_flag: [get]  Empty FIFO status flag.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of empty in reg FIFO_SRC_REG
  *
  */
int32_t lis2dh12_fifo_empty_flag_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_FIFO_SRC_REG, &reg.byte, 1);
  *val = reg.fifo_src_reg.empty;

  return mm_error;
}
/**
  * @brief  fifo_ovr_flag: [get]  FIFO overrun status flag.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ovrn_fifo in reg FIFO_SRC_REG
  *
  */
int32_t lis2dh12_fifo_ovr_flag_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_FIFO_SRC_REG, &reg.byte, 1);
  *val = reg.fifo_src_reg.ovrn_fifo;

  return mm_error;
}
/**
  * @brief  fifo_fth_flag: [get]  FIFO watermark status.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of wtm in reg FIFO_SRC_REG
  *
  */
int32_t lis2dh12_fifo_fth_flag_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_FIFO_SRC_REG, &reg.byte, 1);
  *val = reg.fifo_src_reg.wtm;

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
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_click_cfg_t: registers CLICK_CFG
  *
  */
int32_t lis2dh12_tap_conf_set(lis2dh12_ctx_t *ctx, lis2dh12_click_cfg_t *val)
{
  return lis2dh12_write_reg(ctx, LIS2DH12_CLICK_CFG, (uint8_t*) val, 1);
}

/**
  * @brief  tap_conf: [get]  Tap/Double Tap generator configuration register
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_click_cfg_t: registers CLICK_CFG
  *
  */
int32_t lis2dh12_tap_conf_get(lis2dh12_ctx_t *ctx, lis2dh12_click_cfg_t *val)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_CLICK_CFG, (uint8_t*) val, 1);
}
/**
  * @brief  tap_source: [get]  Tap/Double Tap generator source register
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_click_cfg_t: registers CLICK_SRC
  *
  */
int32_t lis2dh12_tap_source_get(lis2dh12_ctx_t *ctx, lis2dh12_click_src_t *val)
{
  return lis2dh12_read_reg(ctx, LIS2DH12_CLICK_SRC, (uint8_t*) val, 1);
}
/**
  * @brief  tap_threshold: [set]  User-defined threshold value for Tap/Double
  *                               Tap event. (1 LSB = full scale/128)
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of ths in reg CLICK_THS
  *
  */
int32_t lis2dh12_tap_threshold_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CLICK_THS, &reg.byte, 1);
  reg.click_ths.ths = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CLICK_THS, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  tap_threshold: [get]  User-defined threshold value for Tap/Double
  *                               Tap event. (1 LSB = full scale/128)
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ths in reg CLICK_THS
  *
  */
int32_t lis2dh12_tap_threshold_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CLICK_THS, &reg.byte, 1);
  *val = reg.click_ths.ths;

  return mm_error;
}

/**
  * @brief   tap_notification_mode: [set]  If the LIR_Click bit is not set,
  *                                        the interrupt is kept high for the
  *                                        duration of the latency window.
  *                                        If the LIR_Click bit is set, the
  *                                        interrupt is kept high until the
  *                                        CLICK_SRC (39h) register is read.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_lir_click_t: change the values of lir_click in
  *                               reg CLICK_THS
  *
  */
int32_t lis2dh12_tap_notification_mode_set(lis2dh12_ctx_t *ctx,
                                           lis2dh12_lir_click_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CLICK_THS, &reg.byte, 1);
  reg.click_ths.lir_click = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CLICK_THS, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   tap_notification_mode: [get]  If the LIR_Click bit is not set,
  *                                        the interrupt is kept high for the
  *                                        duration of the latency window.
  *                                        If the LIR_Click bit is set, the
  *                                        interrupt is kept high until the
  *                                        CLICK_SRC (39h) register is read.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_lir_click_t: Get the values of lir_click in reg CLICK_THS
  *
  */
int32_t lis2dh12_tap_notification_mode_get(lis2dh12_ctx_t *ctx,
                                           lis2dh12_lir_click_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CLICK_THS, &reg.byte, 1);
  *val = (lis2dh12_lir_click_t) reg.click_ths.lir_click;

  return mm_error;
}

/**
  * @brief  shock_dur: [set]  The maximum time (1 LSB = 1/ODR) interval
  *                           that can elapse between the start of the
  *                           click-detection procedure and when the
  *                           acceleration falls back below the threshold.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of tli in reg TIME_LIMIT
  *
  */
int32_t lis2dh12_shock_dur_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_TIME_LIMIT, &reg.byte, 1);
  reg.time_limit.tli = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_TIME_LIMIT, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  shock_dur: [get]  The maximum time (1 LSB = 1/ODR) interval
  *                           that can elapse between the start of the
  *                           click-detection procedure and when the
  *                           acceleration falls back below the threshold.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tli in reg TIME_LIMIT
  *
  */
int32_t lis2dh12_shock_dur_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_TIME_LIMIT, &reg.byte, 1);
  *val = reg.time_limit.tli;

  return mm_error;
}

/**
  * @brief  quiet_dur: [set]  The time (1 LSB = 1/ODR) interval that
  *                           starts after the first click detection
  *                           where the click-detection procedure is
  *                           disabled, in cases where the device is
  *                           configured for double-click detection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of tla in reg TIME_LATENCY
  *
  */
int32_t lis2dh12_quiet_dur_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_TIME_LATENCY, &reg.byte, 1);
  reg.time_latency.tla = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_TIME_LATENCY, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  quiet_dur: [get]  The time (1 LSB = 1/ODR) interval that
  *                           starts after the first click detection
  *                           where the click-detection procedure is
  *                           disabled, in cases where the device is
  *                           configured for double-click detection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tla in reg TIME_LATENCY
  *
  */
int32_t lis2dh12_quiet_dur_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_TIME_LATENCY, &reg.byte, 1);
  *val = reg.time_latency.tla;

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
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of tw in reg TIME_WINDOW
  *
  */
int32_t lis2dh12_double_tap_timeout_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_TIME_WINDOW, &reg.byte, 1);
  reg.time_window.tw = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_TIME_WINDOW, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  double_tap_timeout: [get]  The maximum interval of time
  *                                    (1 LSB = 1/ODR) that can elapse
  *                                    after the end of the latency interval
  *                                    in which the click-detection procedure
  *                                    can start, in cases where the device
  *                                    is configured for double-click detection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tw in reg TIME_WINDOW
  *
  */
int32_t lis2dh12_double_tap_timeout_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_TIME_WINDOW, &reg.byte, 1);
  *val = reg.time_window.tw;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup   activity_inactivity
  * @brief   This section group all the functions concerning activity
  *          inactivity functionality
  * @{
  */

/**
  * @brief  act_threshold: [set]  Sleep-to-wake, return-to-sleep activation
  *                               threshold in low-power mode.
  *                        1 LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of acth in reg ACT_THS
  *
  */
int32_t lis2dh12_act_threshold_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_ACT_THS, &reg.byte, 1);
  reg.act_ths.acth = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_ACT_THS, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  act_threshold: [get]  Sleep-to-wake, return-to-sleep activation
  *                               threshold in low-power mode.
  *                        1 LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of acth in reg ACT_THS
  *
  */
int32_t lis2dh12_act_threshold_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_ACT_THS, &reg.byte, 1);
  *val = reg.act_ths.acth;

  return mm_error;
}

/**
  * @brief  act_timeout: [set]  Sleep-to-wake, return-to-sleep
  *                             duration = (8*1[LSb]+1)/ODR.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of actd in reg ACT_DUR
  *
  */
int32_t lis2dh12_act_timeout_set(lis2dh12_ctx_t *ctx, uint8_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_ACT_DUR, &reg.byte, 1);
  reg.act_dur.actd = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_ACT_DUR, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  act_timeout: [get]  Sleep-to-wake, return-to-sleep
  *                             duration = (8*1[LSb]+1)/ODR.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of actd in reg ACT_DUR
  *
  */
int32_t lis2dh12_act_timeout_get(lis2dh12_ctx_t *ctx, uint8_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_ACT_DUR, &reg.byte, 1);
  *val = reg.act_dur.actd;

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
  * @brief  pin_sdo_sa0_mode: [set]  Connect/Disconnect SDO/SA0 internal
  *                                  pull-up.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_sdo_pu_disc_t: change the values of sdo_pu_disc in
  *         reg CTRL_REG0
  *
  */
int32_t lis2dh12_pin_sdo_sa0_mode_set(lis2dh12_ctx_t *ctx,
                                      lis2dh12_sdo_pu_disc_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG0, &reg.byte, 1);
  reg.ctrl_reg0.sdo_pu_disc = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG0, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_sdo_sa0_mode: [get]  Connect/Disconnect SDO/SA0 internal
  *                                  pull-up.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_sdo_pu_disc_t: Get the values of sdo_pu_disc in
  *                                 reg CTRL_REG0
  *
  */
int32_t lis2dh12_pin_sdo_sa0_mode_get(lis2dh12_ctx_t *ctx,
                                      lis2dh12_sdo_pu_disc_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG0, &reg.byte, 1);
  *val = (lis2dh12_sdo_pu_disc_t) reg.ctrl_reg0.sdo_pu_disc;

  return mm_error;
}

/**
  * @brief  spi_mode: [set]  SPI Serial Interface Mode selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_sim_t: change the values of sim in reg CTRL_REG4
  *
  */
int32_t lis2dh12_spi_mode_set(lis2dh12_ctx_t *ctx, lis2dh12_sim_t val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.sim = val;
  mm_error = lis2dh12_write_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  spi_mode: [get]  SPI Serial Interface Mode selection.
  *
  * @param  lis2dh12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dh12_sim_t: Get the values of sim in reg CTRL_REG4
  *
  */
int32_t lis2dh12_spi_mode_get(lis2dh12_ctx_t *ctx, lis2dh12_sim_t *val)
{
  lis2dh12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dh12_read_reg(ctx, LIS2DH12_CTRL_REG4, &reg.byte, 1);
  *val = (lis2dh12_sim_t) reg.ctrl_reg4.sim;

  return mm_error;
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/