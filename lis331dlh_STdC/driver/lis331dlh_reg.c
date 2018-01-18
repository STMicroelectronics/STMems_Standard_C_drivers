/*
 ******************************************************************************
 * @file    lis331dlh_reg.c
 * @author  MEMS Software Solution Team
 * @date    20-December-2017
 * @brief   LIS331DLH driver file
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

#include "lis331dlh_reg.h"

/**
  * @addtogroup  lis331dlh
  * @brief  This file provides a set of functions needed to drive the
  *         lis331dlh enanced inertial module.
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
  * @param  lis331dlh_ctx_t* ctx: read / write interface definitions
  * @param  uint8_t reg: register to read
  * @param  uint8_t* data: pointer to buffer that store the data read
  * @param  uint16_t len: number of consecutive register to read
  *
  */
int32_t lis331dlh_read_reg(lis331dlh_ctx_t* ctx, uint8_t reg, uint8_t* data,
                           uint16_t len)
{
  return ctx->read_reg(ctx->handle, reg, data, len);
}

/**
  * @brief  Write generic device register
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t reg: register to write
  * @param  uint8_t* data: pointer to data to write in register reg
  * @param  uint16_t len: number of consecutive register to write
  *
*/
int32_t lis331dlh_write_reg(lis331dlh_ctx_t* ctx, uint8_t reg, uint8_t* data,
                            uint16_t len)
{
  return ctx->write_reg(ctx->handle, reg, data, len);
}

/**
  * @}
  */

/**
  * @addtogroup  data_generation_c
  * @brief   This section groups all the functions concerning data generation
  * @{
  */

/**
  * @brief  axis_x_data: [set]  X axis enable/disable.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of xen in reg CTRL_REG1
  *
  */
int32_t lis331dlh_axis_x_data_set(lis331dlh_ctx_t *ctx, uint8_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.xen = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG1, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  axis_x_data: [get]  X axis enable/disable.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of xen in reg CTRL_REG1
  *
  */
int32_t lis331dlh_axis_x_data_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG1, &reg.byte, 1);
  *val = reg.ctrl_reg1.xen;

  return mm_error;
}

/**
  * @brief  axis_y_data: [set]  Y axis enable/disable.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of yen in reg CTRL_REG1
  *
  */
int32_t lis331dlh_axis_y_data_set(lis331dlh_ctx_t *ctx, uint8_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.yen = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG1, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  axis_y_data: [get]  Y axis enable/disable.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of yen in reg CTRL_REG1
  *
  */
int32_t lis331dlh_axis_y_data_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG1, &reg.byte, 1);
  *val = reg.ctrl_reg1.yen;

  return mm_error;
}

/**
  * @brief  axis_z_data: [set]  Z axis enable/disable.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of zen in reg CTRL_REG1
  *
  */
int32_t lis331dlh_axis_z_data_set(lis331dlh_ctx_t *ctx, uint8_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.zen = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG1, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  axis_z_data: [get]  Z axis enable/disable.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of zen in reg CTRL_REG1
  *
  */
int32_t lis331dlh_axis_z_data_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG1, &reg.byte, 1);
  *val = reg.ctrl_reg1.zen;

  return mm_error;
}

/**
  * @brief  data_rate: [set]   Accelerometer data rate selection.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_dr_t: change the values of dr in reg CTRL_REG1
  *
  */
int32_t lis331dlh_data_rate_set(lis331dlh_ctx_t *ctx, lis331dlh_dr_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.pm = val & 0x07;
  reg.ctrl_reg1.dr = ( val & 0x30 ) >> 4;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG1, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  data_rate: [get]   Accelerometer data rate selection.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_dr_t: Get the values of dr in reg CTRL_REG1
  *
  */
int32_t lis331dlh_data_rate_get(lis331dlh_ctx_t *ctx, lis331dlh_dr_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG1, &reg.byte, 1);
  *val = (lis331dlh_dr_t) ((reg.ctrl_reg1.dr << 4) + reg.ctrl_reg1.pm);

  return mm_error;
}

/**
  * @brief  reference_mode: [set]  High pass filter mode selection.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_hpm_t: change the values of hpm in reg CTRL_REG2
  *
  */
int32_t lis331dlh_reference_mode_set(lis331dlh_ctx_t *ctx,
                                     lis331dlh_hpm_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.hpm = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  reference_mode: [get]  High pass filter mode selection.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_hpm_t: Get the values of hpm in reg CTRL_REG2
  *
  */
int32_t lis331dlh_reference_mode_get(lis331dlh_ctx_t *ctx,
                                     lis331dlh_hpm_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG2, &reg.byte, 1);
  *val = (lis331dlh_hpm_t) reg.ctrl_reg2.hpm;

  return mm_error;
}

/**
  * @brief  full_scale: [set]  Accelerometer full-scale selection.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_fs_t: change the values of fs in reg CTRL_REG4
  *
  */
int32_t lis331dlh_full_scale_set(lis331dlh_ctx_t *ctx, lis331dlh_fs_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.fs = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  full_scale: [get]  Accelerometer full-scale selection.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_fs_t: Get the values of fs in reg CTRL_REG4
  *
  */
int32_t lis331dlh_full_scale_get(lis331dlh_ctx_t *ctx, lis331dlh_fs_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);
  *val = (lis331dlh_fs_t) reg.ctrl_reg4.fs;

  return mm_error;
}

/**
  * @brief  block_data_update: [set] Blockdataupdate.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of bdu in reg CTRL_REG4
  *
  */
int32_t lis331dlh_block_data_update_set(lis331dlh_ctx_t *ctx, uint8_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.bdu = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  block_data_update: [get] Blockdataupdate.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of bdu in reg CTRL_REG4
  *
  */
int32_t lis331dlh_block_data_update_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);
  *val = reg.ctrl_reg4.bdu;

  return mm_error;
}

/**
  * @brief  status_reg: [get] The STATUS_REG register is read by the
  *                           primary interface
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_status_reg_t: registers STATUS_REG
  *
  */
int32_t lis331dlh_status_reg_get(lis331dlh_ctx_t *ctx,
                                 lis331dlh_status_reg_t *val)
{
  return lis331dlh_read_reg(ctx, LIS331DLH_STATUS_REG, (uint8_t*) val, 1);
}

/**
  * @brief  flag_data_ready: [get]  Accelerometer new data available.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of zyxda in reg STATUS_REG
  *
  */
int32_t lis331dlh_flag_data_ready_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_STATUS_REG, &reg.byte, 1);
  *val = reg.status_reg.zyxda;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  Dataoutput
  * @brief   This section groups all the data output functions.
  * @{
  */

/**
  * @brief  acceleration_raw: [get] Linear acceleration output register.
  *                                 The value is expressed as a 16-bit word
  *                                 in two’s complement.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis331dlh_acceleration_raw_get(lis331dlh_ctx_t *ctx, uint8_t *buff)
{
  return lis331dlh_read_reg(ctx, LIS331DLH_OUT_X_L, buff, 6);
}

/**
  * @}
  */

/**
  * @addtogroup  common
  * @brief   This section groups common usefull functions.
  * @{
  */

/**
  * @brief  device_id: [get] DeviceWhoamI.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis331dlh_device_id_get(lis331dlh_ctx_t *ctx, uint8_t *buff)
{
  return lis331dlh_read_reg(ctx, LIS331DLH_WHO_AM_I, buff, 1);
}

/**
  * @brief  boot: [set] Reboot memory content. Reload the calibration
  *                     parameters.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of boot in reg CTRL_REG2
  *
  */
int32_t lis331dlh_boot_set(lis331dlh_ctx_t *ctx, uint8_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.boot = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  boot: [get] Reboot memory content. Reload the calibration
  *                     parameters.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of boot in reg CTRL_REG2
  *
  */
int32_t lis331dlh_boot_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG2, &reg.byte, 1);
  *val = reg.ctrl_reg2.boot;

  return mm_error;
}

/**
  * @brief  self_test: [set]  Linear acceleration sensor self-test enable.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_st_t: change the values of st in reg CTRL_REG4
  *
  */
int32_t lis331dlh_self_test_set(lis331dlh_ctx_t *ctx, lis331dlh_st_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.st = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  self_test: [get]  Linear acceleration sensor self-test enable.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_st_t: Get the values of st in reg CTRL_REG4
  *
  */
int32_t lis331dlh_self_test_get(lis331dlh_ctx_t *ctx, lis331dlh_st_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);
  *val = (lis331dlh_st_t) reg.ctrl_reg4.st;

  return mm_error;
}

/**
  * @brief  data_format: [set]  Big/Little Endian Data selection.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_ble_t: change the values of ble in reg CTRL_REG4
  *
  */
int32_t lis331dlh_data_format_set(lis331dlh_ctx_t *ctx, lis331dlh_ble_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.ble = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  data_format: [get]  Big/Little Endian Data selection.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_ble_t: Get the values of ble in reg CTRL_REG4
  *
  */
int32_t lis331dlh_data_format_get(lis331dlh_ctx_t *ctx, lis331dlh_ble_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);
  *val = (lis331dlh_ble_t) reg.ctrl_reg4.ble;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  filters
  * @brief   This section group all the functions concerning the
  *          filters configuration.
  * @{
  */

/**
  * @brief  hp_bandwidth: [set] High pass filter cut-off frequency
  *                             configuration.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_hpcf_t: change the values of hpcf in reg CTRL_REG2
  *
  */
int32_t lis331dlh_hp_bandwidth_set(lis331dlh_ctx_t *ctx, lis331dlh_hpcf_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.hpcf = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  hp_bandwidth: [get] High pass filter cut-off frequency
  *                             configuration.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_hpcf_t: Get the values of hpcf in reg CTRL_REG2
  *
  */
int32_t lis331dlh_hp_bandwidth_get(lis331dlh_ctx_t *ctx,
                                   lis331dlh_hpcf_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG2, &reg.byte, 1);
  *val = (lis331dlh_hpcf_t) reg.ctrl_reg2.hpcf;

  return mm_error;
}

/**
  * @brief  hp_path: [set]  Select High Pass filter path
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_hpen_t: change the values of hpen in reg CTRL_REG2
  *
  */
int32_t lis331dlh_hp_path_set(lis331dlh_ctx_t *ctx, lis331dlh_hpen_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.hpen = val & 0x03;
  reg.ctrl_reg2.fds = (val & 0x04) >> 2;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  hp_path: [get]  Select High Pass filter path
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_hpen_t: Get the values of hpen in reg CTRL_REG2
  *
  */
int32_t lis331dlh_hp_path_get(lis331dlh_ctx_t *ctx, lis331dlh_hpen_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG2, &reg.byte, 1);
  *val = (lis331dlh_hpen_t) ( (reg.ctrl_reg2.fds << 2) + reg.ctrl_reg2.hpen );

  return mm_error;
}

/**
  * @brief  hp_reset: [get] Reading at this address zeroes instantaneously
  *                         the content of the internal high pass-filter.
  *                         If the high pass filter is enabled all three axes
  *                         are instantaneously set to 0g. This allows to
  *                         overcome the settling time of the high pass
  *                         filter.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis331dlh_hp_reset_get(lis331dlh_ctx_t *ctx)
{
  uint8_t dummy;
  return lis331dlh_read_reg(ctx, LIS331DLH_HP_FILTER_RESET, &dummy, 1);
}

/**
  * @brief  hp_reference_value: [set]  Reference value for high-pass filter.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of ref in reg REFERENCE
  *
  */
int32_t lis331dlh_hp_reference_value_set(lis331dlh_ctx_t *ctx, uint8_t val)
{
  return lis331dlh_write_reg(ctx, LIS331DLH_REFERENCE, &val, 1);
}

/**
  * @brief  hp_reference_value: [get]  Reference value for high-pass filter.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ref in reg REFERENCE
  *
  */
int32_t lis331dlh_hp_reference_value_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  return lis331dlh_read_reg(ctx, LIS331DLH_REFERENCE, val, 1);
}

/**
  * @}
  */

/**
  * @addtogroup   main_serial_interface
  * @brief   This section groups all the functions concerning serial
  *          interface management.
  * @{
  */

/**
  * @brief  spi_mode: [set]  SPI 3- or 4-wire interface.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_sim_t: change the values of sim in reg CTRL_REG4
  *
  */
int32_t lis331dlh_spi_mode_set(lis331dlh_ctx_t *ctx, lis331dlh_sim_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.sim = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  spi_mode: [get]  SPI 3- or 4-wire interface.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_sim_t: Get the values of sim in reg CTRL_REG4
  *
  */
int32_t lis331dlh_spi_mode_get(lis331dlh_ctx_t *ctx, lis331dlh_sim_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG4, &reg.byte, 1);
  *val = (lis331dlh_sim_t) reg.ctrl_reg4.sim;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  interrupt_pins
  * @brief   This section groups all the functions that manage interrupt pins.
  * @{
  */

/**
  * @brief  pin_int1_route: [set]  Data signal on INT 1 pad control bits.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_i1_cfg_t: change the values of i1_cfg in reg CTRL_REG3
  *
  */
int32_t lis331dlh_pin_int1_route_set(lis331dlh_ctx_t *ctx,
                                     lis331dlh_i1_cfg_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);
  reg.ctrl_reg3.i1_cfg = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_int1_route: [get]  Data signal on INT 1 pad control bits.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_i1_cfg_t: Get the values of i1_cfg in reg CTRL_REG3
  *
  */
int32_t lis331dlh_pin_int1_route_get(lis331dlh_ctx_t *ctx,
                                     lis331dlh_i1_cfg_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);
  *val = (lis331dlh_i1_cfg_t) reg.ctrl_reg3.i1_cfg;

  return mm_error;
}

/**
  * @brief  int1_notification: [set] Latch interrupt request on INT1_SRC
  *                                  register, with INT1_SRC register cleared
  *                                  by reading INT1_SRC register.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_lir1_t: change the values of lir1 in reg CTRL_REG3
  *
  */
int32_t lis331dlh_int1_notification_set(lis331dlh_ctx_t *ctx,
                                        lis331dlh_lir1_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);
  reg.ctrl_reg3.lir1 = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int1_notification: [get] Latch interrupt request on INT1_SRC
  *                                  register, with INT1_SRC register cleared
  *                                  by reading INT1_SRC register.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_lir1_t: Get the values of lir1 in reg CTRL_REG3
  *
  */
int32_t lis331dlh_int1_notification_get(lis331dlh_ctx_t *ctx,
                                        lis331dlh_lir1_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);
  *val = (lis331dlh_lir1_t) reg.ctrl_reg3.lir1;

  return mm_error;
}

/**
  * @brief  pin_int2_route: [set]  Data signal on INT 2 pad control bits.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_i2_cfg_t: change the values of i2_cfg in reg CTRL_REG3
  *
  */
int32_t lis331dlh_pin_int2_route_set(lis331dlh_ctx_t *ctx,
                                     lis331dlh_i2_cfg_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);
  reg.ctrl_reg3.i2_cfg = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_int2_route: [get]  Data signal on INT 2 pad control bits.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_i2_cfg_t: Get the values of i2_cfg in reg CTRL_REG3
  *
  */
int32_t lis331dlh_pin_int2_route_get(lis331dlh_ctx_t *ctx,
                                     lis331dlh_i2_cfg_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);
  *val = (lis331dlh_i2_cfg_t) reg.ctrl_reg3.i2_cfg;

  return mm_error;
}

/**
  * @brief  int2_notification: [set] Latch interrupt request on
  *                                  INT2_SRC register, with INT2_SRC
  *                                  register cleared by reading INT2_SRC
  *                                  itself.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_lir2_t: change the values of lir2 in reg CTRL_REG3
  *
  */
int32_t lis331dlh_int2_notification_set(lis331dlh_ctx_t *ctx,
                                        lis331dlh_lir2_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);
  reg.ctrl_reg3.lir2 = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int2_notification: [get] Latch interrupt request on INT2_SRC
  *                                  register, with INT2_SRC register cleared
  *                                  by reading INT2_SRC itself.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_lir2_t: Get the values of lir2 in reg CTRL_REG3
  *
  */
int32_t lis331dlh_int2_notification_get(lis331dlh_ctx_t *ctx,
                                        lis331dlh_lir2_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);
  *val = (lis331dlh_lir2_t) reg.ctrl_reg3.lir2;

  return mm_error;
}

/**
  * @brief  pin_mode: [set]  Push-pull/open drain selection on interrupt pads.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_pp_od_t: change the values of pp_od in reg CTRL_REG3
  *
  */
int32_t lis331dlh_pin_mode_set(lis331dlh_ctx_t *ctx, lis331dlh_pp_od_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);
  reg.ctrl_reg3.pp_od = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_mode: [get]  Push-pull/open drain selection on interrupt pads.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_pp_od_t: Get the values of pp_od in reg CTRL_REG3
  *
  */
int32_t lis331dlh_pin_mode_get(lis331dlh_ctx_t *ctx, lis331dlh_pp_od_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);
  *val = (lis331dlh_pp_od_t) reg.ctrl_reg3.pp_od;

  return mm_error;
}

/**
  * @brief  pin_polarity: [set]  Interrupt active-high/low.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_ihl_t: change the values of ihl in reg CTRL_REG3
  *
  */
int32_t lis331dlh_pin_polarity_set(lis331dlh_ctx_t *ctx, lis331dlh_ihl_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);
  reg.ctrl_reg3.ihl = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_polarity: [get]  Interrupt active-high/low.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_ihl_t: Get the values of ihl in reg CTRL_REG3
  *
  */
int32_t lis331dlh_pin_polarity_get(lis331dlh_ctx_t *ctx, lis331dlh_ihl_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG3, &reg.byte, 1);
  *val = (lis331dlh_ihl_t) reg.ctrl_reg3.ihl;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  interrupt_on_threshold
  * @brief   This section groups all the functions that manage the interrupt
  *          on threshold event generation.
  * @{
  */

/**
  * @brief   int1_on_threshold_conf: [set] Configure the interrupt 1
  *                                        threshold sign.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  int1_on_th_conf_t: enable sign and axis for interrupt on threshold
  *
  */
int32_t lis331dlh_int1_on_threshold_conf_set(lis331dlh_ctx_t *ctx,
                                              int1_on_th_conf_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT1_CFG, &reg.byte, 1);
  reg.int1_cfg.xlie  = val.int1_xlie;
  reg.int1_cfg.xhie  = val.int1_xhie;
  reg.int1_cfg.ylie  = val.int1_ylie;
  reg.int1_cfg.yhie  = val.int1_yhie;
  reg.int1_cfg.zlie  = val.int1_zlie;
  reg.int1_cfg.zhie  = val.int1_zhie;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_INT1_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   int1_on_threshold_conf: [get] Configure the interrupt 1
  *                                        threshold sign.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  int1_on_th_conf_t: enable sign and axis for interrupt on threshold
  *
  */
int32_t lis331dlh_int1_on_threshold_conf_get(lis331dlh_ctx_t *ctx,
                                             int1_on_th_conf_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT1_CFG, &reg.byte, 1);
  val->int1_xlie = reg.int1_cfg.xlie;
  val->int1_xhie = reg.int1_cfg.xhie;
  val->int1_ylie = reg.int1_cfg.ylie;
  val->int1_yhie = reg.int1_cfg.yhie;
  val->int1_zlie = reg.int1_cfg.zlie;
  val->int1_zhie = reg.int1_cfg.zhie;

  return mm_error;
}

/**
  * @brief   int1_on_threshold_mode: [set] AND/OR combination of
  *                                        Interrupt 1 events.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_aoi_t: change the values of aoi in reg INT1_CFG
  *
  */
int32_t lis331dlh_int1_on_threshold_mode_set(lis331dlh_ctx_t *ctx,
                                             lis331dlh_int1_aoi_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT1_CFG, &reg.byte, 1);
  reg.int1_cfg.aoi = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_INT1_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   int1_on_threshold_mode: [get] AND/OR combination of
  *                                        Interrupt 1 events.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_aoi_t: Get the values of aoi in reg INT1_CFG
  *
  */
int32_t lis331dlh_int1_on_threshold_mode_get(lis331dlh_ctx_t *ctx,
                                             lis331dlh_int1_aoi_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT1_CFG, &reg.byte, 1);
  *val = (lis331dlh_int1_aoi_t) reg.int1_cfg.aoi;

  return mm_error;
}

/**
  * @brief  int1_src: [get] Interrupt generator 1 on threshold
  *                         source register.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_int1_src_t: registers INT1_SRC
  *
  */
int32_t lis331dlh_int1_src_get(lis331dlh_ctx_t *ctx,
                               lis331dlh_int1_src_t *val)
{
  return lis331dlh_read_reg(ctx, LIS331DLH_INT1_SRC, (uint8_t*) val, 1);
}

/**
  * @brief  int1_treshold: [set]  Interrupt 1 threshold.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of ths in reg INT1_THS
  *
  */
int32_t lis331dlh_int1_treshold_set(lis331dlh_ctx_t *ctx, uint8_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT1_THS, &reg.byte, 1);
  reg.int1_ths.ths = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_INT1_THS, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int1_treshold: [get]  Interrupt 1 threshold.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ths in reg INT1_THS
  *
  */
int32_t lis331dlh_int1_treshold_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT1_THS, &reg.byte, 1);
  *val = reg.int1_ths.ths;

  return mm_error;
}

/**
  * @brief  int1_dur: [set]  Duration value for interrupt 1 generator.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of d in reg INT1_DURATION
  *
  */
int32_t lis331dlh_int1_dur_set(lis331dlh_ctx_t *ctx, uint8_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT1_DURATION, &reg.byte, 1);
  reg.int1_duration.d = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_INT1_DURATION, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int1_dur: [get]  Duration value for interrupt 1 generator.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of d in reg INT1_DURATION
  *
  */
int32_t lis331dlh_int1_dur_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT1_DURATION, &reg.byte, 1);
  *val = reg.int1_duration.d;

  return mm_error;
}

/**
  * @brief   int2_on_threshold_conf: [set] Configure the interrupt 2
  *                                        threshold sign.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  int2_on_th_conf_t: enable sign and axis for interrupt on threshold
  *
  */
int32_t lis331dlh_int2_on_threshold_conf_set(lis331dlh_ctx_t *ctx,
                                              int2_on_th_conf_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT2_CFG, &reg.byte, 1);
  reg.int2_cfg.xlie  = val.int2_xlie;
  reg.int2_cfg.xhie  = val.int2_xhie;
  reg.int2_cfg.ylie  = val.int2_ylie;
  reg.int2_cfg.yhie  = val.int2_yhie;
  reg.int2_cfg.zlie  = val.int2_zlie;
  reg.int2_cfg.zhie  = val.int2_zhie;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_INT2_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   int2_on_threshold_conf: [get] Configure the interrupt 2
  *                                        threshold sign.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  int2_on_th_conf_t: enable sign and axis for interrupt on threshold
  *
  */
int32_t lis331dlh_int2_on_threshold_conf_get(lis331dlh_ctx_t *ctx,
                                              int2_on_th_conf_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT2_CFG, &reg.byte, 1);
  val->int2_xlie = reg.int2_cfg.xlie;
  val->int2_xhie = reg.int2_cfg.xhie;
  val->int2_ylie = reg.int2_cfg.ylie;
  val->int2_yhie = reg.int2_cfg.yhie;
  val->int2_zlie = reg.int2_cfg.zlie;
  val->int2_zhie = reg.int2_cfg.zhie;

  return mm_error;
}

/**
  * @brief   int2_on_threshold_mode: [set] AND/OR combination of
  *                                        Interrupt 2 events.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_aoi_t: change the values of aoi in reg INT2_CFG
  *
  */
int32_t lis331dlh_int2_on_threshold_mode_set(lis331dlh_ctx_t *ctx,
                                             lis331dlh_int2_aoi_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT2_CFG, &reg.byte, 1);
  reg.int2_cfg.aoi = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_INT2_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   int2_on_threshold_mode: [get] AND/OR combination of
  *                                        Interrupt 2 events.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_aoi_t: Get the values of aoi in reg INT2_CFG
  *
  */
int32_t lis331dlh_int2_on_threshold_mode_get(lis331dlh_ctx_t *ctx,
                                             lis331dlh_int2_aoi_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT2_CFG, &reg.byte, 1);
  *val = (lis331dlh_int2_aoi_t) reg.int2_cfg.aoi;

  return mm_error;
}

/**
  * @brief  int2_src: [get] Interrupt generator 1 on threshold source
  *                         register.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_int2_src_t: registers INT2_SRC
  *
  */
int32_t lis331dlh_int2_src_get(lis331dlh_ctx_t *ctx,
                               lis331dlh_int2_src_t *val)
{
  return lis331dlh_read_reg(ctx, LIS331DLH_INT2_SRC, (uint8_t*) val, 1);
}

/**
  * @brief  int2_treshold: [set]  Interrupt 2 threshold.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of ths in reg INT2_THS
  *
  */
int32_t lis331dlh_int2_treshold_set(lis331dlh_ctx_t *ctx, uint8_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT2_THS, &reg.byte, 1);
  reg.int2_ths.ths = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_INT2_THS, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int2_treshold: [get]  Interrupt 2 threshold.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ths in reg INT2_THS
  *
  */
int32_t lis331dlh_int2_treshold_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT2_THS, &reg.byte, 1);
  *val = reg.int2_ths.ths;

  return mm_error;
}

/**
  * @brief  int2_dur: [set]  Duration value for interrupt 2 generator.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of d in reg INT2_DURATION
  *
  */
int32_t lis331dlh_int2_dur_set(lis331dlh_ctx_t *ctx, uint8_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT2_DURATION, &reg.byte, 1);
  reg.int2_duration.d = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_INT2_DURATION, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int2_dur: [get]  Duration value for interrupt 2 generator.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of d in reg INT2_DURATION
  *
  */
int32_t lis331dlh_int2_dur_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT2_DURATION, &reg.byte, 1);
  *val = reg.int2_duration.d;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  Wake_Up_event
  * @brief  This section groups all the functions that manage the Wake Up
  *         event generation.
  * @{
  */

/**
  * @brief  wkup_to_sleep: [set] Turn-on mode selection for sleep to
  *                              wake function.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of turnon in reg CTRL_REG5
  *
  */
int32_t lis331dlh_wkup_to_sleep_set(lis331dlh_ctx_t *ctx, uint8_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG5, &reg.byte, 1);
  reg.ctrl_reg5.turnon = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_CTRL_REG5, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  wkup_to_sleep: [get] Turn-on mode selection for sleep to
  *                              wake function.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of turnon in reg CTRL_REG5
  *
  */
int32_t lis331dlh_wkup_to_sleep_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_CTRL_REG5, &reg.byte, 1);
  *val = reg.ctrl_reg5.turnon;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup   Six_position_detection
  * @brief   This section groups all the functions concerning six
  *          position detection (6D).
  * @{
  */

/**
  * @brief  int1_6d_mode: [set]  Configure the 6d on interrupt 1 generator.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_6d_t: change the values of 6d in reg INT1_CFG
  *
  */
int32_t lis331dlh_int1_6d_mode_set(lis331dlh_ctx_t *ctx,
                                   lis331dlh_int1_6d_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT1_CFG, &reg.byte, 1);
  reg.int1_cfg._6d = val & 0x01;
  reg.int1_cfg.aoi = (val & 0x02) >> 1;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_INT1_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int1_6d_mode: [get]  Configure the 6d on interrupt 1 generator.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_6d_t: Get the values of 6d in reg INT1_CFG
  *
  */
int32_t lis331dlh_int1_6d_mode_get(lis331dlh_ctx_t *ctx,
                                   lis331dlh_int1_6d_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT1_CFG, &reg.byte, 1);
  *val = (lis331dlh_int1_6d_t) ((reg.int1_cfg.aoi << 1) + reg.int1_cfg._6d);

  return mm_error;
}

/**
  * @brief  int1_6d_src: [get]  6D on interrupt generator 1 source register.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_: registers INT1_SRC
  *
  */
int32_t lis331dlh_int1_6d_src_get(lis331dlh_ctx_t *ctx,
                                  lis331dlh_int1_src_t *val)
{
  return lis331dlh_read_reg(ctx, LIS331DLH_INT1_SRC, (uint8_t*) val, 1);
}

/**
  * @brief  int1_6d_treshold: [set]  Interrupt 1 threshold.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of ths in reg INT1_THS
  *
  */
int32_t lis331dlh_int1_6d_treshold_set(lis331dlh_ctx_t *ctx, uint8_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT1_THS, &reg.byte, 1);
  reg.int1_ths.ths = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_INT1_THS, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int1_6d_treshold: [get]  Interrupt 1 threshold.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ths in reg INT1_THS
  *
  */
int32_t lis331dlh_int1_6d_treshold_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT1_THS, &reg.byte, 1);
  *val = reg.int1_ths.ths;

  return mm_error;
}

/**
  * @brief  int2_6d_mode: [set]   Configure the 6d on interrupt 2 generator.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_6d_t: change the values of 6d in reg INT2_CFG
  *
  */
int32_t lis331dlh_int2_6d_mode_set(lis331dlh_ctx_t *ctx,
                                   lis331dlh_int2_6d_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT2_CFG, &reg.byte, 1);
  reg.int2_cfg._6d = val & 0x01;
  reg.int2_cfg.aoi = (val & 0x02) >> 1;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_INT2_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int2_6d_mode: [get]   Configure the 6d on interrupt 2 generator.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_6d_t: Get the values of 6d in reg INT2_CFG
  *
  */
int32_t lis331dlh_int2_6d_mode_get(lis331dlh_ctx_t *ctx,
                                   lis331dlh_int2_6d_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT2_CFG, &reg.byte, 1);
  *val = (lis331dlh_int2_6d_t) ((reg.int2_cfg.aoi << 1) + reg.int2_cfg._6d);

  return mm_error;
}

/**
  * @brief  int2_6d_src: [get]  6D on interrupt generator 2 source register.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  lis331dlh_int2_src_t: registers INT2_SRC
  *
  */
int32_t lis331dlh_int2_6d_src_get(lis331dlh_ctx_t *ctx,
                                  lis331dlh_int2_src_t *val)
{
  return lis331dlh_read_reg(ctx, LIS331DLH_INT2_SRC, (uint8_t*) val, 1);
}

/**
  * @brief  int2_6d_treshold: [set]  Interrupt 2 threshold.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of ths in reg INT2_THS
  *
  */
int32_t lis331dlh_int2_6d_treshold_set(lis331dlh_ctx_t *ctx, uint8_t val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT2_THS, &reg.byte, 1);
  reg.int2_ths.ths = val;
  mm_error = lis331dlh_write_reg(ctx, LIS331DLH_INT2_THS, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int2_6d_treshold: [get]  Interrupt 2 threshold.
  *
  * @param  lis331dlh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ths in reg INT2_THS
  *
  */
int32_t lis331dlh_int2_6d_treshold_get(lis331dlh_ctx_t *ctx, uint8_t *val)
{
  lis331dlh_reg_t reg;
  int32_t mm_error;

  mm_error = lis331dlh_read_reg(ctx, LIS331DLH_INT2_THS, &reg.byte, 1);
  *val = reg.int2_ths.ths;

  return mm_error;
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/