/*
 ******************************************************************************
 * @file    lis2dw12_reg.c
 * @author  MEMS Software Solution Team
 * @date    25-January-2018
 * @brief   LIS2DW12 driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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

#include "lis2dw12_reg.h"

/**
  * @addtogroup  lis2dw12
  * @brief  This file provides a set of functions needed to drive the
  *         lis2dw12 enanced inertial module.
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
  * @param  lis2dw12_ctx_t* ctx: read / write interface definitions
  * @param  uint8_t reg: register to read
  * @param  uint8_t* data: pointer to buffer that store the data read
  * @param  uint16_t len: number of consecutive register to read
  *
  */
int32_t lis2dw12_read_reg(lis2dw12_ctx_t* ctx, uint8_t reg, uint8_t* data,
                          uint16_t len)
{
  return ctx->read_reg(ctx->handle, reg, data, len);
}

/**
  * @brief  Write generic device register
  * 
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t reg: register to write
  * @param  uint8_t* data: pointer to data to write in register reg
  * @param  uint16_t len: number of consecutive register to write
  *
*/
int32_t lis2dw12_write_reg(lis2dw12_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
  * @brief  power_mode: [set]  Select accelerometer operating modes.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_mode_t: change the values of mode / lp_mode in reg CTRL1
  *                          and low_noise in reg CTRL6
  *
  */
int32_t lis2dw12_power_mode_set(lis2dw12_ctx_t *ctx, lis2dw12_mode_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL1, &reg.byte, 1);
  reg.ctrl1.mode = ( val & 0x0C ) >> 2;
  reg.ctrl1.lp_mode = val & 0x03 ;  
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL1, &reg.byte, 1);

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, &reg.byte, 1);
  reg.ctrl6.low_noise = ( val & 0x10 ) >> 4;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL6, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  power_mode: [get]  Select accelerometer operating modes.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_mode_t: change the values of mode / lp_mode in reg CTRL1
  *                          and low_noise in reg CTRL6
  *
  */
int32_t lis2dw12_power_mode_get(lis2dw12_ctx_t *ctx, lis2dw12_mode_t *val)
{
  lis2dw12_reg_t reg[2];
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL1, &reg[0].byte, 1);
  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, &reg[1].byte, 1);
  *val = (lis2dw12_mode_t) ((reg[1].ctrl6.low_noise << 4) + 
                            (reg[0].ctrl1.mode << 2) + reg[0].ctrl1.lp_mode);

  return mm_error;
}

/**
  * @brief  data_rate: [set]  Accelerometer data rate selection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_odr_t: change the values of odr in reg CTRL1
  *
  */
int32_t lis2dw12_data_rate_set(lis2dw12_ctx_t *ctx, lis2dw12_odr_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL1, &reg.byte, 1);
  reg.ctrl1.odr = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL1, &reg.byte, 1);
  
  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);
  reg.ctrl3.slp_mode = ( val & 0x30 ) >> 4;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  data_rate: [get]  Accelerometer data rate selection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_odr_t: Get the values of odr in reg CTRL1
  *
  */
int32_t lis2dw12_data_rate_get(lis2dw12_ctx_t *ctx, lis2dw12_odr_t *val)
{
  lis2dw12_reg_t reg[2];
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL1, &reg[0].byte, 1);
  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, &reg[1].byte, 1);
  *val = (lis2dw12_odr_t) ((reg[1].ctrl3.slp_mode << 4) + reg[0].ctrl1.odr);

  return mm_error;
}

/**
  * @brief  block_data_update: [set] Blockdataupdate.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of bdu in reg CTRL2
  *
  */
int32_t lis2dw12_block_data_update_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  reg.ctrl2.bdu = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  block_data_update: [get] Blockdataupdate.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of bdu in reg CTRL2
  *
  */
int32_t lis2dw12_block_data_update_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  *val = reg.ctrl2.bdu;

  return mm_error;
}

/**
  * @brief  full_scale: [set]  Accelerometer full-scale selection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_fs_t: change the values of fs in reg CTRL6
  *
  */
int32_t lis2dw12_full_scale_set(lis2dw12_ctx_t *ctx, lis2dw12_fs_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, &reg.byte, 1);
  reg.ctrl6.fs = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL6, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  full_scale: [get]  Accelerometer full-scale selection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_fs_t: Get the values of fs in reg CTRL6
  *
  */
int32_t lis2dw12_full_scale_get(lis2dw12_ctx_t *ctx, lis2dw12_fs_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, &reg.byte, 1);
  *val = (lis2dw12_fs_t) reg.ctrl6.fs;

  return mm_error;
}

/**
  * @brief  status_reg: [get]  The STATUS_REG register of the device.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_: union of registers from STATUS to 
  *
  */
int32_t lis2dw12_status_reg_get(lis2dw12_ctx_t *ctx, lis2dw12_status_t *val)
{
  return lis2dw12_read_reg(ctx, LIS2DW12_STATUS, (uint8_t*) val, 1);
}
/**
  * @brief  flag_data_ready: [get]  Accelerometer new data available.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of drdy in reg STATUS
  *
  */
int32_t lis2dw12_flag_data_ready_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_STATUS, &reg.byte, 1);
  *val = reg.status.drdy;

  return mm_error;
}
/**
  * @brief  all_sources: [get] Read all the interrupt/status flag of
  *                            the device.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_all_sources: registers STATUS_DUP, WAKE_UP_SRC,
  *                               TAP_SRC, SIXD_SRC, ALL_INT_SRC
  *
  */
int32_t lis2dw12_all_sources_get(lis2dw12_ctx_t *ctx,
                                 lis2dw12_all_sources_t *val)
{
  return lis2dw12_read_reg(ctx, LIS2DW12_STATUS_DUP, (uint8_t*) val, 5);
}
/**
  * @brief  usr_offset_x: [set] Accelerometer X-axis user offset correction
  *                                expressed in two’s complement, weight
  *                                depends on bit USR_OFF_W. The value must be
  *                                in the range [-127 127].
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that contains data to write
  *
  */
int32_t lis2dw12_usr_offset_x_set(lis2dw12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dw12_write_reg(ctx, LIS2DW12_X_OFS_USR, buff, 1);
}

/**
  * @brief  usr_offset_x: [get] Accelerometer X-axis user offset
  *                                correction expressed in two’s complement, 
  *                                weight depends on bit USR_OFF_W. 
  *                                The value must be in the range [-127 127].
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis2dw12_usr_offset_x_get(lis2dw12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dw12_read_reg(ctx, LIS2DW12_X_OFS_USR, buff, 1);
}
/**
  * @brief  usr_offset_y: [set] Accelerometer Y-axis user offset
  *                                correction expressed in two’s complement,
  *                                weight depends on bit USR_OFF_W.
  *                                The value must be in the range [-127 127].
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that contains data to write
  *
  */
int32_t lis2dw12_usr_offset_y_set(lis2dw12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dw12_write_reg(ctx, LIS2DW12_Y_OFS_USR, buff, 1);
}

/**
  * @brief  usr_offset_y: [get] Accelerometer Y-axis user offset
  *                                correction expressed in two’s complement,
  *                                weight depends on bit USR_OFF_W.
  *                                The value must be in the range [-127 127].
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis2dw12_usr_offset_y_get(lis2dw12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dw12_read_reg(ctx, LIS2DW12_Y_OFS_USR, buff, 1);
}
/**
  * @brief  usr_offset_z: [set] Accelerometer Z-axis user offset
  *                                correction expressed in two’s complement,
  *                                weight depends on bit USR_OFF_W.
  *                                The value must be in the range [-127 127].
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that contains data to write
  *
  */
int32_t lis2dw12_usr_offset_z_set(lis2dw12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dw12_write_reg(ctx, LIS2DW12_Z_OFS_USR, buff, 1);
}

/**
  * @brief  usr_offset_z: [get] Accelerometer Z-axis user offset
  *                                correction expressed in two’s complement,
  *                                weight depends on bit USR_OFF_W.
  *                                The value must be in the range [-127 127].
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis2dw12_usr_offset_z_get(lis2dw12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dw12_read_reg(ctx, LIS2DW12_Z_OFS_USR, buff, 1);
}
/**
  * @brief  offset_weight: [set] Weight of XL user offset bits of
  *                                 registers X_OFS_USR, Y_OFS_USR, Z_OFS_USR.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_usr_off_w_t: change the values of usr_off_w in
  *                               reg CTRL_REG7
  *
  */
int32_t lis2dw12_offset_weight_set(lis2dw12_ctx_t *ctx,
                                      lis2dw12_usr_off_w_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  reg.ctrl_reg7.usr_off_w = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  offset_weight: [get] Weight of XL user offset bits of
  *                                 registers X_OFS_USR, Y_OFS_USR, Z_OFS_USR.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_usr_off_w_t: Get the values of usr_off_w in reg CTRL_REG7
  *
  */
int32_t lis2dw12_offset_weight_get(lis2dw12_ctx_t *ctx,
                                      lis2dw12_usr_off_w_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  *val = (lis2dw12_usr_off_w_t) reg.ctrl_reg7.usr_off_w;

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
  * @brief  temperature_raw: [get] Temperature data output register (r).
  *                                L and H registers together express a
  *                                16-bit word in two’s complement.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis2dw12_temperature_raw_get(lis2dw12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dw12_read_reg(ctx, LIS2DW12_OUT_T_L, buff, 2);
}

/**
  * @brief  acceleration_raw: [get] Linear acceleration output register.
  *                                 The value is expressed as a 16-bit word
  *                                 in two’s complement.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis2dw12_acceleration_raw_get(lis2dw12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dw12_read_reg(ctx, LIS2DW12_OUT_X_L, buff, 6);
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
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lis2dw12_device_id_get(lis2dw12_ctx_t *ctx, uint8_t *buff)
{
  return lis2dw12_read_reg(ctx, LIS2DW12_WHO_AM_I, buff, 1);
}

/**
  * @brief  auto_increment: [set] Register address automatically incremented
  *                               during multiple byte access with a
  *                               serial interface.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of if_add_inc in reg CTRL2
  *
  */
int32_t lis2dw12_auto_increment_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  reg.ctrl2.if_add_inc = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  auto_increment: [get]  Register address automatically
  *                                incremented during multiple byte access
  *                                with a serial interface.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of if_add_inc in reg CTRL2
  *
  */
int32_t lis2dw12_auto_increment_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  *val = reg.ctrl2.if_add_inc;

  return mm_error;
}

/**
  * @brief  reset: [set] Software reset. Restore the default values in
  *                      user registers.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of soft_reset in reg CTRL2
  *
  */
int32_t lis2dw12_reset_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  reg.ctrl2.soft_reset = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  reset: [get] Software reset. Restore the default values in
  *                      user registers.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of soft_reset in reg CTRL2
  *
  */
int32_t lis2dw12_reset_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  *val = reg.ctrl2.soft_reset;

  return mm_error;
}

/**
  * @brief  boot: [set] Reboot memory content. Reload the calibration
  *                     parameters.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of boot in reg CTRL2
  *
  */
int32_t lis2dw12_boot_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  reg.ctrl2.boot = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  boot: [get] Reboot memory content. Reload the calibration
  *                     parameters.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of boot in reg CTRL2
  *
  */
int32_t lis2dw12_boot_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  *val = reg.ctrl2.boot;

  return mm_error;
}

/**
  * @brief  self_test: [set]  Sensor self-test enable.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_st_t: change the values of st in reg CTRL3
  *
  */
int32_t lis2dw12_self_test_set(lis2dw12_ctx_t *ctx, lis2dw12_st_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);
  reg.ctrl3.st = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  self_test: [get]  Sensor self-test enable.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_st_t: Get the values of st in reg CTRL3
  *
  */
int32_t lis2dw12_self_test_get(lis2dw12_ctx_t *ctx, lis2dw12_st_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);
  *val = (lis2dw12_st_t) reg.ctrl3.st;

  return mm_error;
}

/**
  * @brief  data_ready_mode: [set]  Data-ready pulsed / letched mode.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_drdy_pulsed_t: change the values of drdy_pulsed in
  *                                 reg CTRL_REG7
  *
  */
int32_t lis2dw12_data_ready_mode_set(lis2dw12_ctx_t *ctx,
                                     lis2dw12_drdy_pulsed_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  reg.ctrl_reg7.drdy_pulsed = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  data_ready_mode: [get]  Data-ready pulsed / letched mode.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_drdy_pulsed_t: Get the values of drdy_pulsed in
  *                                 reg CTRL_REG7
  *
  */
int32_t lis2dw12_data_ready_mode_get(lis2dw12_ctx_t *ctx,
                                     lis2dw12_drdy_pulsed_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  *val = (lis2dw12_drdy_pulsed_t) reg.ctrl_reg7.drdy_pulsed;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  filters
  * @brief   This section group all the functions concerning the filters
  *          configuration.
  * @{
  */

/**
  * @brief  filter_path: [set]  Accelerometer filtering path for outputs.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_fds_t: change the values of fds in reg CTRL6
  *
  */
int32_t lis2dw12_filter_path_set(lis2dw12_ctx_t *ctx, lis2dw12_fds_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, &reg.byte, 1);
  reg.ctrl6.fds = ( val & 0x10 ) >> 4;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL6, &reg.byte, 1);
  
  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  reg.ctrl_reg7.usr_off_on_out = val & 0x01;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);  

  return mm_error;
}

/**
  * @brief  filter_path: [get]  Accelerometer filtering path for outputs.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_fds_t: Get the values of fds in reg CTRL6
  *
  */
int32_t lis2dw12_filter_path_get(lis2dw12_ctx_t *ctx, lis2dw12_fds_t *val)
{
  lis2dw12_reg_t reg[2];
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, &reg[0].byte, 1);
  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg[1].byte, 1);
  
  *val = (lis2dw12_fds_t) ((reg[0].ctrl6.fds << 4 ) + reg[1].ctrl_reg7.usr_off_on_out);

  return mm_error;
}

/**
  * @brief   filter_bandwidth: [set] Accelerometer cutoff filter frequency.
  *                                     Valid for low and high pass filter.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_bw_filt_t: change the values of bw_filt in reg CTRL6
  *
  */
int32_t lis2dw12_filter_bandwidth_set(lis2dw12_ctx_t *ctx,
                                         lis2dw12_bw_filt_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, &reg.byte, 1);
  reg.ctrl6.bw_filt = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL6, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   filter_bandwidth: [get] Accelerometer cutoff filter frequency.
  *                                     Valid for low and high pass filter.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_bw_filt_t: Get the values of bw_filt in reg CTRL6
  *
  */
int32_t lis2dw12_filter_bandwidth_get(lis2dw12_ctx_t *ctx,
                                         lis2dw12_bw_filt_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL6, &reg.byte, 1);
  *val = (lis2dw12_bw_filt_t) reg.ctrl6.bw_filt;

  return mm_error;
}

/**
  * @brief  reference_mode: [set]  Enable HP filter reference mode.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of hp_ref_mode in reg CTRL_REG7
  *
  */
int32_t lis2dw12_reference_mode_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  reg.ctrl_reg7.hp_ref_mode = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  reference_mode: [get]  Enable HP filter reference mode.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of hp_ref_mode in reg CTRL_REG7
  *
  */
int32_t lis2dw12_reference_mode_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  *val = reg.ctrl_reg7.hp_ref_mode;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup   main_serial_interface
  * @brief   This section groups all the functions concerning main serial
  *          interface management (not auxiliary)
  * @{
  */

/**
  * @brief  spi_mode: [set]  SPI Serial Interface Mode selection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_sim_t: change the values of sim in reg CTRL2
  *
  */
int32_t lis2dw12_spi_mode_set(lis2dw12_ctx_t *ctx, lis2dw12_sim_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  reg.ctrl2.sim = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  spi_mode: [get]  SPI Serial Interface Mode selection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_sim_t: Get the values of sim in reg CTRL2
  *
  */
int32_t lis2dw12_spi_mode_get(lis2dw12_ctx_t *ctx, lis2dw12_sim_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  *val = (lis2dw12_sim_t) reg.ctrl2.sim;

  return mm_error;
}

/**
  * @brief  i2c_interface: [set]  Disable / Enable I2C interface.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_i2c_disable_t: change the values of i2c_disable in
  *                                 reg CTRL2
  *
  */
int32_t lis2dw12_i2c_interface_set(lis2dw12_ctx_t *ctx,
                                   lis2dw12_i2c_disable_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  reg.ctrl2.i2c_disable = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  i2c_interface: [get]  Disable / Enable I2C interface.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_i2c_disable_t: Get the values of i2c_disable in reg CTRL2
  *
  */
int32_t lis2dw12_i2c_interface_get(lis2dw12_ctx_t *ctx,
                                   lis2dw12_i2c_disable_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  *val = (lis2dw12_i2c_disable_t) reg.ctrl2.i2c_disable;

  return mm_error;
}

/**
  * @brief  cs_mode: [set]  Disconnect CS pull-up.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_cs_pu_disc_t: change the values of cs_pu_disc in reg CTRL2
  *
  */
int32_t lis2dw12_cs_mode_set(lis2dw12_ctx_t *ctx, lis2dw12_cs_pu_disc_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  reg.ctrl2.cs_pu_disc = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  cs_mode: [get]  Disconnect CS pull-up.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_cs_pu_disc_t: Get the values of cs_pu_disc in reg CTRL2
  *
  */
int32_t lis2dw12_cs_mode_get(lis2dw12_ctx_t *ctx, lis2dw12_cs_pu_disc_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL2, &reg.byte, 1);
  *val = (lis2dw12_cs_pu_disc_t) reg.ctrl2.cs_pu_disc;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  interrupt_pins
  * @brief   This section groups all the functions that manage interrup pins
  * @{
  */

/**
  * @brief  pin_polarity: [set]  Interrupt active-high/low.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_h_lactive_t: change the values of h_lactive in reg CTRL3
  *
  */
int32_t lis2dw12_pin_polarity_set(lis2dw12_ctx_t *ctx,
                                  lis2dw12_h_lactive_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);
  reg.ctrl3.h_lactive = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_polarity: [get]  Interrupt active-high/low.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_h_lactive_t: Get the values of h_lactive in reg CTRL3
  *
  */
int32_t lis2dw12_pin_polarity_get(lis2dw12_ctx_t *ctx,
                                  lis2dw12_h_lactive_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);
  *val = (lis2dw12_h_lactive_t) reg.ctrl3.h_lactive;

  return mm_error;
}

/**
  * @brief  int_notification: [set]  Latched/pulsed interrupt.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_lir_t: change the values of lir in reg CTRL3
  *
  */
int32_t lis2dw12_int_notification_set(lis2dw12_ctx_t *ctx,
                                      lis2dw12_lir_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);
  reg.ctrl3.lir = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int_notification: [get]  Latched/pulsed interrupt.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_lir_t: Get the values of lir in reg CTRL3
  *
  */
int32_t lis2dw12_int_notification_get(lis2dw12_ctx_t *ctx,
                                      lis2dw12_lir_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);
  *val = (lis2dw12_lir_t) reg.ctrl3.lir;

  return mm_error;
}

/**
  * @brief  pin_mode: [set]  Push-pull/open drain selection on interrupt pads.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_pp_od_t: change the values of pp_od in reg CTRL3
  *
  */
int32_t lis2dw12_pin_mode_set(lis2dw12_ctx_t *ctx, lis2dw12_pp_od_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);
  reg.ctrl3.pp_od = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_mode: [get]  Push-pull/open drain selection on interrupt pads.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_pp_od_t: Get the values of pp_od in reg CTRL3
  *
  */
int32_t lis2dw12_pin_mode_get(lis2dw12_ctx_t *ctx, lis2dw12_pp_od_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL3, &reg.byte, 1);
  *val = (lis2dw12_pp_od_t) reg.ctrl3.pp_od;

  return mm_error;
}

/**
  * @brief  pin_int1_route: [set] Select the signal that need to
  *                               route on int1 pad.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_: register CTRL4_INT1_PAD_CTRL.
  *
  */
int32_t lis2dw12_pin_int1_route_set(lis2dw12_ctx_t *ctx,
                                    lis2dw12_ctrl4_int1_pad_ctrl_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;
  
  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  
  if (val->int1_tap || val->int1_ff || val->int1_wu || val->int1_single_tap ||
    val->int1_6d){  
    reg.ctrl_reg7.interrupts_enable = PROPERTY_ENABLE;   
  } 
  else{
    reg.ctrl_reg7.interrupts_enable = PROPERTY_DISABLE;
  }
  
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL4_INT1_PAD_CTRL,
                                (uint8_t*) val, 1);  
  
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1); 
                            
  return mm_error;                          
}

/**
  * @brief  pin_int1_route: [get] Select the signal that need to route on
  *                               int1 pad.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_: register CTRL4_INT1_PAD_CTRL.
  *
  */
int32_t lis2dw12_pin_int1_route_get(lis2dw12_ctx_t *ctx,
                                    lis2dw12_ctrl4_int1_pad_ctrl_t *val)
{
  return lis2dw12_read_reg(ctx, LIS2DW12_CTRL4_INT1_PAD_CTRL,
                           (uint8_t*) val, 1);
}
/**
  * @brief  pin_int2_route: [set] Select the signal that need to route on
  *                               int2 pad.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_: register CTRL5_INT2_PAD_CTRL.
  *
  */
int32_t lis2dw12_pin_int2_route_set(lis2dw12_ctx_t *ctx,
                                    lis2dw12_ctrl5_int2_pad_ctrl_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;
  
  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  
  if ( val->int2_sleep_state || val->int2_sleep_chg ){  
    reg.ctrl_reg7.interrupts_enable = PROPERTY_ENABLE;   
  } 
  else{
    reg.ctrl_reg7.interrupts_enable = PROPERTY_DISABLE;
  }
  
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL5_INT2_PAD_CTRL,
                                (uint8_t*) val, 1);  
  
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1); 
                            
  return mm_error;     
}

/**
  * @brief  pin_int2_route: [get] Select the signal that need to route on
  *                               int2 pad.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_: register CTRL5_INT2_PAD_CTRL
  *
  */
int32_t lis2dw12_pin_int2_route_get(lis2dw12_ctx_t *ctx,
                                    lis2dw12_ctrl5_int2_pad_ctrl_t *val)
{
  return lis2dw12_read_reg(ctx, LIS2DW12_CTRL5_INT2_PAD_CTRL,
                           (uint8_t*) val, 1);
}
/**
  * @brief  all_on_int1: [set] All interrupt signals become available
  *                            on INT1 pin.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of int2_on_int1 in reg CTRL_REG7
  *
  */
int32_t lis2dw12_all_on_int1_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  reg.ctrl_reg7.int2_on_int1 = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  all_on_int1: [get] All interrupt signals become available
  *                            on INT1 pin.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of int2_on_int1 in reg CTRL_REG7
  *
  */
int32_t lis2dw12_all_on_int1_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  *val = reg.ctrl_reg7.int2_on_int1;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  Wake_Up_event
  * @brief   This section groups all the functions that manage the Wake
  *          Up event generation.
  * @{
  */

/**
  * @brief  wkup_threshold: [set]  Threshold for wakeup.1 LSB = FS_XL / 64.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of wk_ths in reg WAKE_UP_THS
  *
  */
int32_t lis2dw12_wkup_threshold_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_THS, &reg.byte, 1);
  reg.wake_up_ths.wk_ths = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_WAKE_UP_THS, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  wkup_threshold: [get]  Threshold for wakeup.1 LSB = FS_XL / 64.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of wk_ths in reg WAKE_UP_THS
  *
  */
int32_t lis2dw12_wkup_threshold_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_THS, &reg.byte, 1);
  *val = reg.wake_up_ths.wk_ths;

  return mm_error;
}

/**
  * @brief  wkup_dur: [set]  Wake up duration event.1LSb = 1 / ODR.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of wake_dur in reg WAKE_UP_DUR
  *
  */
int32_t lis2dw12_wkup_dur_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR, &reg.byte, 1);
  reg.wake_up_dur.wake_dur = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_WAKE_UP_DUR, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  wkup_dur: [get]  Wake up duration event.1LSb = 1 / ODR.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of wake_dur in reg WAKE_UP_DUR
  *
  */
int32_t lis2dw12_wkup_dur_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR, &reg.byte, 1);
  *val = reg.wake_up_dur.wake_dur;

  return mm_error;
}

/**
  * @brief  wkup_feed_data: [set]  Data sent to wake-up interrupt function.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_usr_off_on_wu_t: change the values of usr_off_on_wu in
  *                                   reg CTRL_REG7
  *
  */
int32_t lis2dw12_wkup_feed_data_set(lis2dw12_ctx_t *ctx,
                                    lis2dw12_usr_off_on_wu_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  reg.ctrl_reg7.usr_off_on_wu = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  wkup_feed_data: [get]  Data sent to wake-up interrupt function.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_usr_off_on_wu_t: Get the values of usr_off_on_wu in
  *                                   reg CTRL_REG7
  *
  */
int32_t lis2dw12_wkup_feed_data_get(lis2dw12_ctx_t *ctx,
                                    lis2dw12_usr_off_on_wu_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  *val = (lis2dw12_usr_off_on_wu_t) reg.ctrl_reg7.usr_off_on_wu;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup   Activity/Inactivity_detection
  * @brief   This section groups all the functions concerning
  *          activity/inactivity detection.
  * @{
  */

/**
  * @brief  act_mode: [set] Config activity / inactivity or 
  *                         stationary / motion detection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_sleep_on_t: change the values of sleep_on / stationary in
  *                              reg WAKE_UP_THS / WAKE_UP_DUR
  *
  */
int32_t lis2dw12_act_mode_set(lis2dw12_ctx_t *ctx, lis2dw12_sleep_on_t val)
{
  lis2dw12_reg_t reg[2];
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_THS, &reg[0].byte, 2);
  reg[0].wake_up_ths.sleep_on = val & 0x01;
  reg[1].wake_up_dur.stationary = (val & 0x02) >> 1;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_WAKE_UP_THS, &reg[0].byte, 2);

  return mm_error;
}

/**
  * @brief  act_mode: [get] Config activity / inactivity or 
  *                         stationary / motion detection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_sleep_on_t: Get the values of sleep_on in reg WAKE_UP_THS
  *
  */
int32_t lis2dw12_act_mode_get(lis2dw12_ctx_t *ctx, lis2dw12_sleep_on_t *val)
{
  lis2dw12_reg_t reg[2];
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_THS, &reg[0].byte, 2);
  *val = (lis2dw12_sleep_on_t) ((reg[1].wake_up_dur.stationary << 1) 
                                + reg[0].wake_up_ths.sleep_on);

  return mm_error;
}

/**
  * @brief  act_sleep_dur: [set] Duration to go in sleep mode.
  *                              1 LSb = 512 / ODR.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of sleep_dur in reg WAKE_UP_DUR
  *
  */
int32_t lis2dw12_act_sleep_dur_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR, &reg.byte, 1);
  reg.wake_up_dur.sleep_dur = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_WAKE_UP_DUR, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  act_sleep_dur: [get] Duration to go in sleep mode.
  *                              1 LSb = 512 / ODR.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of sleep_dur in reg WAKE_UP_DUR
  *
  */
int32_t lis2dw12_act_sleep_dur_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR, &reg.byte, 1);
  *val = reg.wake_up_dur.sleep_dur;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  tap_generator
  * @brief   This section groups all the functions that manage the tap
  *          and double tap event generation.
  * @{
  */

/**
  * @brief  tap_threshold_x: [set]  Threshold for tap recognition.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of tap_thsx in reg TAP_THS_X
  *
  */
int32_t lis2dw12_tap_threshold_x_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_X, &reg.byte, 1);
  reg.tap_ths_x.tap_thsx = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_X, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  tap_threshold_x: [get]  Threshold for tap recognition.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tap_thsx in reg TAP_THS_X
  *
  */
int32_t lis2dw12_tap_threshold_x_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_X, &reg.byte, 1);
  *val = reg.tap_ths_x.tap_thsx;

  return mm_error;
}

/**
  * @brief  tap_threshold_y: [set]  Threshold for tap recognition.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of tap_thsy in reg TAP_THS_Y
  *
  */
int32_t lis2dw12_tap_threshold_y_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Y, &reg.byte, 1);
  reg.tap_ths_y.tap_thsy = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_Y, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  tap_threshold_y: [get]  Threshold for tap recognition.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tap_thsy in reg TAP_THS_Y
  *
  */
int32_t lis2dw12_tap_threshold_y_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Y, &reg.byte, 1);
  *val = reg.tap_ths_y.tap_thsy;

  return mm_error;
}

/**
  * @brief  tap_axis_priority: [set] Selection of axis priority for
  *                                  TAP detection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_tap_prior_t: change the values of tap_prior in
  *                               reg TAP_THS_Y
  *
  */
int32_t lis2dw12_tap_axis_priority_set(lis2dw12_ctx_t *ctx,
                                       lis2dw12_tap_prior_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Y, &reg.byte, 1);
  reg.tap_ths_y.tap_prior = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_Y, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  tap_axis_priority: [get] Selection of axis priority for
  *                                  TAP detection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_tap_prior_t: Get the values of tap_prior in reg TAP_THS_Y
  *
  */
int32_t lis2dw12_tap_axis_priority_get(lis2dw12_ctx_t *ctx,
                                       lis2dw12_tap_prior_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Y, &reg.byte, 1);
  *val = (lis2dw12_tap_prior_t) reg.tap_ths_y.tap_prior;

  return mm_error;
}

/**
  * @brief  tap_threshold_z: [set]  Threshold for tap recognition.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of tap_thsz in reg TAP_THS_Z
  *
  */
int32_t lis2dw12_tap_threshold_z_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, &reg.byte, 1);
  reg.tap_ths_z.tap_thsz = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_Z, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  tap_threshold_z: [get]  Threshold for tap recognition.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tap_thsz in reg TAP_THS_Z
  *
  */
int32_t lis2dw12_tap_threshold_z_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, &reg.byte, 1);
  *val = reg.tap_ths_z.tap_thsz;

  return mm_error;
}

/**
  * @brief  tap_detection_on_z: [set]  Enable Z direction in tap recognition.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of tap_z_en in reg TAP_THS_Z
  *
  */
int32_t lis2dw12_tap_detection_on_z_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, &reg.byte, 1);
  reg.tap_ths_z.tap_z_en = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_Z, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  tap_detection_on_z: [get]  Enable Z direction in tap recognition.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tap_z_en in reg TAP_THS_Z
  *
  */
int32_t lis2dw12_tap_detection_on_z_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, &reg.byte, 1);
  *val = reg.tap_ths_z.tap_z_en;

  return mm_error;
}

/**
  * @brief  tap_detection_on_y: [set]  Enable Y direction in tap recognition.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of tap_y_en in reg TAP_THS_Z
  *
  */
int32_t lis2dw12_tap_detection_on_y_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, &reg.byte, 1);
  reg.tap_ths_z.tap_y_en = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_Z, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  tap_detection_on_y: [get]  Enable Y direction in tap recognition.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tap_y_en in reg TAP_THS_Z
  *
  */
int32_t lis2dw12_tap_detection_on_y_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, &reg.byte, 1);
  *val = reg.tap_ths_z.tap_y_en;

  return mm_error;
}

/**
  * @brief  tap_detection_on_x: [set]  Enable X direction in tap recognition.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of tap_x_en in reg TAP_THS_Z
  *
  */
int32_t lis2dw12_tap_detection_on_x_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, &reg.byte, 1);
  reg.tap_ths_z.tap_x_en = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_Z, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  tap_detection_on_x: [get]  Enable X direction in tap recognition.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of tap_x_en in reg TAP_THS_Z
  *
  */
int32_t lis2dw12_tap_detection_on_x_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_Z, &reg.byte, 1);
  *val = reg.tap_ths_z.tap_x_en;

  return mm_error;
}

/**
  * @brief  tap_shock: [set] Maximum duration is the maximum time of an
  *                          overthreshold signal detection to be recognized
  *                          as a tap event. The default value of these bits
  *                          is 00b which corresponds to 4*ODR_XL time.
  *                          If the SHOCK[1:0] bits are set to a different
  *                          value, 1LSB corresponds to 8*ODR_XL time.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of shock in reg INT_DUR
  *
  */
int32_t lis2dw12_tap_shock_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_INT_DUR, &reg.byte, 1);
  reg.int_dur.shock = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_INT_DUR, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  tap_shock: [get] Maximum duration is the maximum time of an
  *                          overthreshold signal detection to be
  *                          recognized as a tap event.
  *                          The default value of these bits is 00b which
  *                          corresponds to 4*ODR_XL time.
  *                          If the SHOCK[1:0] bits are set to a different
  *                          value, 1LSB corresponds to 8*ODR_XL time.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of shock in reg INT_DUR
  *
  */
int32_t lis2dw12_tap_shock_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_INT_DUR, &reg.byte, 1);
  *val = reg.int_dur.shock;

  return mm_error;
}

/**
  * @brief  tap_quiet: [set] Quiet time is the time after the first
  *                          detected tap in which there must not be any
  *                          overthreshold event.
  *                          The default value of these bits is 00b which
  *                          corresponds to 2*ODR_XL time.
  *                          If the QUIET[1:0] bits are set to a different
  *                          value, 1LSB corresponds to 4*ODR_XL time.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of quiet in reg INT_DUR
  *
  */
int32_t lis2dw12_tap_quiet_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_INT_DUR, &reg.byte, 1);
  reg.int_dur.quiet = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_INT_DUR, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  tap_quiet: [get] Quiet time is the time after the first
  *                          detected tap in which there must not be
  *                          any overthreshold event.
  *                          The default value of these bits is 00b which
  *                          corresponds to 2*ODR_XL time.
  *                          If the QUIET[1:0] bits are set to a different
  *                          value, 1LSB corresponds to 4*ODR_XL time.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of quiet in reg INT_DUR
  *
  */
int32_t lis2dw12_tap_quiet_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_INT_DUR, &reg.byte, 1);
  *val = reg.int_dur.quiet;

  return mm_error;
}

/**
  * @brief  tap_dur: [set] When double tap recognition is enabled,
  *                        this register expresses the maximum time
  *                        between two consecutive detected taps to 
  *                        determine a double tap event.
  *                        The default value of these bits is 0000b
  *                        which corresponds to 16*ODR_XL time.
  *                        If the DUR[3:0] bits are set to a different value,
  *                        1LSB corresponds to 32*ODR_XL time.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of latency in reg INT_DUR
  *
  */
int32_t lis2dw12_tap_dur_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_INT_DUR, &reg.byte, 1);
  reg.int_dur.latency = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_INT_DUR, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  tap_dur: [get] When double tap recognition is enabled,
  *                        this register expresses the maximum time
  *                        between two consecutive detected taps to
  *                        determine a double tap event.
  *                        The default value of these bits is 0000b
  *                        which corresponds to 16*ODR_XL time.
  *                        If the DUR[3:0] bits are set to a different
  *                        value, 1LSB corresponds to 32*ODR_XL time.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of latency in reg INT_DUR
  *
  */
int32_t lis2dw12_tap_dur_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_INT_DUR, &reg.byte, 1);
  *val = reg.int_dur.latency;

  return mm_error;
}

/**
  * @brief  tap_mode: [set]  Single/double-tap event enable.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_single_double_tap_t: change the values of
  *                                       single_double_tap in reg WAKE_UP_THS
  *
  */
int32_t lis2dw12_tap_mode_set(lis2dw12_ctx_t *ctx,
                              lis2dw12_single_double_tap_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_THS, &reg.byte, 1);
  reg.wake_up_ths.single_double_tap = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_WAKE_UP_THS, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  tap_mode: [get]  Single/double-tap event enable.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_single_double_tap_t: Get the values of single_double_tap
  *                                       in reg WAKE_UP_THS
  *
  */
int32_t lis2dw12_tap_mode_get(lis2dw12_ctx_t *ctx,
                              lis2dw12_single_double_tap_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_THS, &reg.byte, 1);
  *val = (lis2dw12_single_double_tap_t) reg.wake_up_ths.single_double_tap;

  return mm_error;
}

/**
  * @brief  tap_src: [get]  Read the tap / double tap source register.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_tap_src: union of registers from TAP_SRC to 
  *
  */
int32_t lis2dw12_tap_src_get(lis2dw12_ctx_t *ctx, lis2dw12_tap_src_t *val)
{
  return lis2dw12_read_reg(ctx, LIS2DW12_TAP_SRC, (uint8_t*) val, 1);
}
/**
  * @}
  */

/**
  * @addtogroup   Six_position_detection(6D/4D)
  * @brief   This section groups all the functions concerning six
  *          position detection (6D).
  * @{
  */

/**
  * @brief  6d_threshold: [set]  Threshold for 4D/6D function.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of 6d_ths in reg TAP_THS_X
  *
  */
int32_t lis2dw12_6d_threshold_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_X, &reg.byte, 1);
  reg.tap_ths_x._6d_ths = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_X, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  6d_threshold: [get]  Threshold for 4D/6D function.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of 6d_ths in reg TAP_THS_X
  *
  */
int32_t lis2dw12_6d_threshold_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_X, &reg.byte, 1);
  *val = reg.tap_ths_x._6d_ths;

  return mm_error;
}

/**
  * @brief  4d_mode: [set]  4D orientation detection enable.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of 4d_en in reg TAP_THS_X
  *
  */
int32_t lis2dw12_4d_mode_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_X, &reg.byte, 1);
  reg.tap_ths_x._4d_en = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_TAP_THS_X, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  4d_mode: [get]  4D orientation detection enable.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of 4d_en in reg TAP_THS_X
  *
  */
int32_t lis2dw12_4d_mode_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_TAP_THS_X, &reg.byte, 1);
  *val = reg.tap_ths_x._4d_en;

  return mm_error;
}

/**
  * @brief  6d_src: [get]  Read the 6D tap source register.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_6d_src: union of registers from SIXD_SRC to 
  *
  */
int32_t lis2dw12_6d_src_get(lis2dw12_ctx_t *ctx, lis2dw12_sixd_src_t *val)
{
  return lis2dw12_read_reg(ctx, LIS2DW12_SIXD_SRC, (uint8_t*) val, 1);
}
/**
  * @brief  6d_feed_data: [set]  Data sent to 6D interrupt function.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_lpass_on6d_t: change the values of lpass_on6d in
  *                                reg CTRL_REG7
  *
  */
int32_t lis2dw12_6d_feed_data_set(lis2dw12_ctx_t *ctx,
                                  lis2dw12_lpass_on6d_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  reg.ctrl_reg7.lpass_on6d = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  6d_feed_data: [get]  Data sent to 6D interrupt function.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_lpass_on6d_t: Get the values of lpass_on6d in
  *                                reg CTRL_REG7
  *
  */
int32_t lis2dw12_6d_feed_data_get(lis2dw12_ctx_t *ctx,
                                  lis2dw12_lpass_on6d_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_CTRL_REG7, &reg.byte, 1);
  *val = (lis2dw12_lpass_on6d_t) reg.ctrl_reg7.lpass_on6d;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  free_fall
  * @brief   This section group all the functions concerning
  *          the free fall detection.
  * @{
  */

/**
  * @brief  ff_dur: [set]  Wake up duration event.1LSb = 1 / ODR.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of ff_dur in 
  *                      reg WAKE_UP_DUR /F REE_FALL
  *
  */
int32_t lis2dw12_ff_dur_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg[2];
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR, &reg[0].byte, 2);
  reg[0].wake_up_dur.ff_dur = (val & 0x20) >> 5;
  reg[1].free_fall.ff_dur = val & 0x1F;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_WAKE_UP_DUR, &reg[0].byte, 2);

  return mm_error;
}

/**
  * @brief  ff_dur: [get]  Wake up duration event.1LSb = 1 / ODR.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ff_dur in
  *                  reg WAKE_UP_DUR /F REE_FALL  
  *
  */
int32_t lis2dw12_ff_dur_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg[2];
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_WAKE_UP_DUR, &reg[0].byte, 2);
  *val = (reg[0].wake_up_dur.ff_dur << 5) + reg[1].free_fall.ff_dur;

  return mm_error;
}

/**
  * @brief  ff_threshold: [set]  Free fall threshold setting.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_ff_ths_t: change the values of ff_ths in reg FREE_FALL
  *
  */
int32_t lis2dw12_ff_threshold_set(lis2dw12_ctx_t *ctx, lis2dw12_ff_ths_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_FREE_FALL, &reg.byte, 1);
  reg.free_fall.ff_ths = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_FREE_FALL, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  ff_threshold: [get]  Free fall threshold setting.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_ff_ths_t: Get the values of ff_ths in reg FREE_FALL
  *
  */
int32_t lis2dw12_ff_threshold_get(lis2dw12_ctx_t *ctx,
                                  lis2dw12_ff_ths_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_FREE_FALL, &reg.byte, 1);
  *val = (lis2dw12_ff_ths_t) reg.free_fall.ff_ths;

  return mm_error;
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
  * @brief  fifo_watermark: [set]  FIFO watermark level selection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of fth in reg FIFO_CTRL
  *
  */
int32_t lis2dw12_fifo_watermark_set(lis2dw12_ctx_t *ctx, uint8_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_CTRL, &reg.byte, 1);
  reg.fifo_ctrl.fth = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_FIFO_CTRL, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  fifo_watermark: [get]  FIFO watermark level selection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fth in reg FIFO_CTRL
  *
  */
int32_t lis2dw12_fifo_watermark_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_CTRL, &reg.byte, 1);
  *val = reg.fifo_ctrl.fth;

  return mm_error;
}

/**
  * @brief  fifo_mode: [set]  FIFO mode selection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_fmode_t: change the values of fmode in reg FIFO_CTRL
  *
  */
int32_t lis2dw12_fifo_mode_set(lis2dw12_ctx_t *ctx, lis2dw12_fmode_t val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_CTRL, &reg.byte, 1);
  reg.fifo_ctrl.fmode = val;
  mm_error = lis2dw12_write_reg(ctx, LIS2DW12_FIFO_CTRL, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  fifo_mode: [get]  FIFO mode selection.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  lis2dw12_fmode_t: Get the values of fmode in reg FIFO_CTRL
  *
  */
int32_t lis2dw12_fifo_mode_get(lis2dw12_ctx_t *ctx, lis2dw12_fmode_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_CTRL, &reg.byte, 1);
  *val = (lis2dw12_fmode_t) reg.fifo_ctrl.fmode;

  return mm_error;
}

/**
  * @brief  fifo_data_level: [get]  Number of unread samples stored in FIFO.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of diff in reg FIFO_SAMPLES
  *
  */
int32_t lis2dw12_fifo_data_level_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_SAMPLES, &reg.byte, 1);
  *val = reg.fifo_samples.diff;

  return mm_error;
}
/**
  * @brief  fifo_ovr_flag: [get]  FIFO overrun status.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fifo_ovr in reg FIFO_SAMPLES
  *
  */
int32_t lis2dw12_fifo_ovr_flag_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_SAMPLES, &reg.byte, 1);
  *val = reg.fifo_samples.fifo_ovr;

  return mm_error;
}
/**
  * @brief  fifo_wtm_flag: [get]  FIFO threshold status flag.
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fifo_fth in reg FIFO_SAMPLES
  *
  */
int32_t lis2dw12_fifo_wtm_flag_get(lis2dw12_ctx_t *ctx, uint8_t *val)
{
  lis2dw12_reg_t reg;
  int32_t mm_error;

  mm_error = lis2dw12_read_reg(ctx, LIS2DW12_FIFO_SAMPLES, &reg.byte, 1);
  *val = reg.fifo_samples.fifo_fth;

  return mm_error;
}
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/