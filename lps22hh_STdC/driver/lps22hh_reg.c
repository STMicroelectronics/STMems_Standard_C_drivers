/*
 ******************************************************************************
 * @file    lps22hh_reg.c
 * @author  MEMS Software Solution Team
 * @date    14-December-2017
 * @brief   LPS22HH driver file
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

#include "lps22hh_reg.h"

/**
  * @addtogroup  lps22hh
  * @brief  This file provides a set of functions needed to drive the
  *         lps22hh enanced inertial module.
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
  * @param  lps22hh_ctx_t* ctx: read / write interface definitions
  * @param  uint8_t reg: register to read
  * @param  uint8_t* data: pointer to buffer that store the data read
  * @param  uint16_t len: number of consecutive register to read
  *
  */
int32_t lps22hh_read_reg(lps22hh_ctx_t* ctx, uint8_t reg, uint8_t* data,
                         uint16_t len)
{
  return ctx->read_reg(ctx->handle, reg, data, len);
}

/**
  * @brief  Write generic device register
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t reg: register to write
  * @param  uint8_t* data: pointer to data to write in register reg
  * @param  uint16_t len: number of consecutive register to write
  *
*/
int32_t lps22hh_write_reg(lps22hh_ctx_t* ctx, uint8_t reg, uint8_t* data,
                          uint16_t len)
{
  return ctx->write_reg(ctx->handle, reg, data, len);
}

/**
  * @}
  */

/**
  * @addtogroup  data_generation_c
  * @brief   This section groups all the functions concerning data generation.
  * @{
  */

/**
  * @brief  autozero_rst: [set]  Reset Autozero function.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of reset_az in reg INTERRUPT_CFG
  *
  */
int32_t lps22hh_autozero_rst_set(lps22hh_ctx_t *ctx, uint8_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);
  reg.interrupt_cfg.reset_az = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  autozero_rst: [get]  Reset Autozero function.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of reset_az in reg INTERRUPT_CFG
  *
  */
int32_t lps22hh_autozero_rst_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);
  *val = reg.interrupt_cfg.reset_az;

  return mm_error;
}

/**
  * @brief  autozero: [set]  Enable Autozero function.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of autozero in reg INTERRUPT_CFG
  *
  */
int32_t lps22hh_autozero_set(lps22hh_ctx_t *ctx, uint8_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);
  reg.interrupt_cfg.autozero = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  autozero: [get]  Enable Autozero function.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of autozero in reg INTERRUPT_CFG
  *
  */
int32_t lps22hh_autozero_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);
  *val = reg.interrupt_cfg.autozero;

  return mm_error;
}

/**
  * @brief  pressure_snap_rst: [set]  Reset AutoRifP function.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of reset_arp in reg INTERRUPT_CFG
  *
  */
int32_t lps22hh_pressure_snap_rst_set(lps22hh_ctx_t *ctx, uint8_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);
  reg.interrupt_cfg.reset_arp = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pressure_snap_rst: [get]  Reset AutoRifP function.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of reset_arp in reg INTERRUPT_CFG
  *
  */
int32_t lps22hh_pressure_snap_rst_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);
  *val = reg.interrupt_cfg.reset_arp;

  return mm_error;
}

/**
  * @brief  pressure_snap: [set]  Enable AutoRifP function.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of autorifp in reg INTERRUPT_CFG
  *
  */
int32_t lps22hh_pressure_snap_set(lps22hh_ctx_t *ctx, uint8_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);
  reg.interrupt_cfg.autorifp = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pressure_snap: [get]  Enable AutoRifP function.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of autorifp in reg INTERRUPT_CFG
  *
  */
int32_t lps22hh_pressure_snap_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);
  *val = reg.interrupt_cfg.autorifp;

  return mm_error;
}

/**
  * @brief  block_data_update: [set] Blockdataupdate.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of bdu in reg CTRL_REG1
  *
  */
int32_t lps22hh_block_data_update_set(lps22hh_ctx_t *ctx, uint8_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.bdu = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_CTRL_REG1, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  block_data_update: [get] Blockdataupdate.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of bdu in reg CTRL_REG1
  *
  */
int32_t lps22hh_block_data_update_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG1, &reg.byte, 1);
  *val = reg.ctrl_reg1.bdu;

  return mm_error;
}

/**
  * @brief  data_rate: [set]  Output data rate selection.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_odr_t: change the values of odr in reg CTRL_REG1
  *
  */
int32_t lps22hh_data_rate_set(lps22hh_ctx_t *ctx, lps22hh_odr_t val)
{
  lps22hh_reg_t reg[2];
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG1, &reg[0].byte, 2);
  reg[0].ctrl_reg1.odr = val & 0x03;
  reg[1].ctrl_reg2.low_noise_en = (val & 0x10) >> 4;
  reg[1].ctrl_reg2.one_shot = (val & 0x08) >> 3;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_CTRL_REG1, &reg[0].byte, 2);

  return mm_error;
}

/**
  * @brief  data_rate: [get]  Output data rate selection.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_odr_t: Get the values of odr in reg CTRL_REG1
  *
  */
int32_t lps22hh_data_rate_get(lps22hh_ctx_t *ctx, lps22hh_odr_t *val)
{
  lps22hh_reg_t reg[2];
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG1, &reg[0].byte, 2);
  *val = (lps22hh_odr_t) ( (reg[1].ctrl_reg2.low_noise_en << 4) +
         (reg[1].ctrl_reg2.one_shot << 3) + reg[0].ctrl_reg1.odr );

  return mm_error;
}

/**
  * @brief  pressure_ref: [set] The Reference pressure value is a 16-bit data
  *                             expressed as 2’s complement. The value is used
  *                             when AUTOZERO or AUTORIFP function is enabled.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that contains data to write
  *
  */
int32_t lps22hh_pressure_ref_set(lps22hh_ctx_t *ctx, uint8_t *buff)
{
  return lps22hh_write_reg(ctx, LPS22HH_REF_P_XL, buff, 2);
}

/**
  * @brief  pressure_ref: [get] The Reference pressure value is a 16-bit
  *                             data expressed as 2’s complement.
  *                             The value is used when AUTOZERO or AUTORIFP
  *                             function is enabled.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lps22hh_pressure_ref_get(lps22hh_ctx_t *ctx, uint8_t *buff)
{
  return lps22hh_read_reg(ctx, LPS22HH_REF_P_XL, buff, 2);
}

/**
  * @brief  pressure_offset: [set] The pressure offset value is 16-bit data
  *                                that can be used to implement one-point
  *                                calibration (OPC) after soldering.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that contains data to write
  *
  */
int32_t lps22hh_pressure_offset_set(lps22hh_ctx_t *ctx, uint8_t *buff)
{
  return lps22hh_write_reg(ctx, LPS22HH_RPDS_L, buff, 2);
}

/**
  * @brief  pressure_offset: [get] The pressure offset value is 16-bit
  *                                data that can be used to implement
  *                                one-point calibration (OPC) after
  *                                soldering.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lps22hh_pressure_offset_get(lps22hh_ctx_t *ctx, uint8_t *buff)
{
  return lps22hh_read_reg(ctx, LPS22HH_RPDS_L, buff, 2);
}

/**
  * @brief  all_sources: [get] Read all the interrupt/status flag of
  *                            the device.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_all_sources: registers STATUS,FIFO_STATUS2,INT_SOURCE
  *
  */
int32_t lps22hh_all_sources_get(lps22hh_ctx_t *ctx, lps22hh_all_sources_t *val)
{
  lps22hh_reg_t reg[3];
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_INT_SOURCE, &reg[0].byte, 1);
  mm_error = lps22hh_read_reg(ctx, LPS22HH_FIFO_STATUS2, &reg[1].byte, 2);

  val->int_source   = reg[0].int_source;
  val->fifo_status2 = reg[1].fifo_status2;
  val->status       = reg[2].status;

  return mm_error;
}

/**
  * @brief  status_reg: [get] The STATUS_REG register is read by
  *                           the primary interface
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_: union of registers from STATUS to STATUS_REG
  *
  */
int32_t lps22hh_status_reg_get(lps22hh_ctx_t *ctx, lps22hh_status_t *val)
{
  return lps22hh_read_reg(ctx, LPS22HH_STATUS, (uint8_t*) val, 1);
}

/**
  * @brief   press_flag_data_ready: [get]  Pressure new data available.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of p_da in reg STATUS
  *
  */
int32_t lps22hh_press_flag_data_ready_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_STATUS, &reg.byte, 1);
  *val = reg.status.p_da;

  return mm_error;
}

/**
  * @brief   temp_flag_data_ready: [get]  Temperature data available.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of t_da in reg STATUS
  *
  */
int32_t lps22hh_temp_flag_data_ready_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_STATUS, &reg.byte, 1);
  *val = reg.status.t_da;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  data_output
  * @brief   This section groups all the data output functions.
  * @{
  */

/**
  * @brief  pressure_raw: [get]  Pressure output value.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lps22hh_pressure_raw_get(lps22hh_ctx_t *ctx, uint8_t *buff)
{
  return lps22hh_read_reg(ctx, LPS22HH_PRESSURE_OUT_XL, buff, 3);
}

/**
  * @brief  temperature_raw: [get]  Temperature output value.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lps22hh_temperature_raw_get(lps22hh_ctx_t *ctx, uint8_t *buff)
{
  return lps22hh_read_reg(ctx, LPS22HH_TEMP_OUT_L, buff, 2);
}

/**
  * @brief  fifo_pressure_raw: [get]  Pressure output from FIFO value.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lps22hh_fifo_pressure_raw_get(lps22hh_ctx_t *ctx, uint8_t *buff)
{
  return lps22hh_read_reg(ctx, LPS22HH_FIFO_DATA_OUT_PRESS_XL, buff, 3);
}

/**
  * @brief   fifo_temperature_raw: [get]  Temperature output from FIFO value.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lps22hh_fifo_temperature_raw_get(lps22hh_ctx_t *ctx, uint8_t *buff)
{
  return lps22hh_read_reg(ctx, LPS22HH_FIFO_DATA_OUT_TEMP_L, buff, 2);
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
  * @brief  device_id: [get] DeviceWhoamI
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lps22hh_device_id_get(lps22hh_ctx_t *ctx, uint8_t *buff)
{
  return lps22hh_read_reg(ctx, LPS22HH_WHO_AM_I, buff, 1);
}

/**
  * @brief  reset: [set] Software reset. Restore the default values
  *                      in user registers.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of swreset in reg CTRL_REG2
  *
  */
int32_t lps22hh_reset_set(lps22hh_ctx_t *ctx, uint8_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.swreset = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  reset: [get] Software reset. Restore the default values
  *                      in user registers.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of swreset in reg CTRL_REG2
  *
  */
int32_t lps22hh_reset_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);
  *val = reg.ctrl_reg2.swreset;

  return mm_error;
}

/**
  * @brief  auto_increment: [set] Register address automatically
  *                               incremented during a multiple byte access
  *                               with a serial interface.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of if_add_inc in reg CTRL_REG2
  *
  */
int32_t lps22hh_auto_increment_set(lps22hh_ctx_t *ctx, uint8_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.if_add_inc = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  auto_increment: [get] Register address automatically
  *                               incremented during a multiple byte
  *                               access with a serial interface.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of if_add_inc in reg CTRL_REG2
  *
  */
int32_t lps22hh_auto_increment_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);
  *val = reg.ctrl_reg2.if_add_inc;

  return mm_error;
}

/**
  * @brief  boot: [set] Reboot memory content. Reload the calibration
  *                     parameters.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of boot in reg CTRL_REG2
  *
  */
int32_t lps22hh_boot_set(lps22hh_ctx_t *ctx, uint8_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.boot = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  boot: [get] Reboot memory content. Reload the calibration
  *                     parameters.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of boot in reg CTRL_REG2
  *
  */
int32_t lps22hh_boot_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);
  *val = reg.ctrl_reg2.boot;

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
  * @brief  lp_bandwidth: [set]  Low-pass bandwidth selection.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_lpfp_cfg_t: change the values of lpfp_cfg in
  *                             reg CTRL_REG1
  *
  */
int32_t lps22hh_lp_bandwidth_set(lps22hh_ctx_t *ctx, lps22hh_lpfp_cfg_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.lpfp_cfg = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_CTRL_REG1, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  lp_bandwidth: [get]  Low-pass bandwidth selection.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_lpfp_cfg_t: Get the values of lpfp_cfg in reg CTRL_REG1
  *
  */
int32_t lps22hh_lp_bandwidth_get(lps22hh_ctx_t *ctx, lps22hh_lpfp_cfg_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG1, &reg.byte, 1);
  *val = (lps22hh_lpfp_cfg_t) reg.ctrl_reg1.lpfp_cfg;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  serial_interface
  * @brief   This section groups all the functions concerning  serial
  *          interface management
  * @{
  */

/**
  * @brief  i2c_interface: [set]  Enable/Disable I2C interface.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_i2c_disable_t: change the values of i2c_disable in
  *                                reg IF_CTRL
  *
  */
int32_t lps22hh_i2c_interface_set(lps22hh_ctx_t *ctx,
                                  lps22hh_i2c_disable_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_IF_CTRL, &reg.byte, 1);
  reg.if_ctrl.i2c_disable = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_IF_CTRL, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  i2c_interface: [get]  Enable/Disable I2C interface.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_i2c_disable_t: Get the values of i2c_disable in reg
  *                                IF_CTRL
  *
  */
int32_t lps22hh_i2c_interface_get(lps22hh_ctx_t *ctx,
                                  lps22hh_i2c_disable_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_IF_CTRL, &reg.byte, 1);
  *val = (lps22hh_i2c_disable_t) reg.if_ctrl.i2c_disable;

  return mm_error;
}

/**
  * @brief  i3c_interface: [set]  I3C Enable/Disable communication protocol
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t lps22hh_i3c_disable_t: change the values of int_en_i3c in
  *                                        reg IF_CTRL
  *
  */
int32_t lps22hh_i3c_interface_set(lps22hh_ctx_t *ctx,
                                  lps22hh_i3c_disable_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_IF_CTRL, &reg.byte, 1);
  reg.if_ctrl.i3c_disable = (val & 0x01);
  reg.if_ctrl.int_en_i3c = (val & 0x10) >> 4;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_IF_CTRL, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  i3c_interface: [get]  I3C Enable/Disable communication protocol
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_i3c_disable_t: change the values of int_en_i3c in reg
  *                                IF_CTRL
  *
  */
int32_t lps22hh_i3c_interface_get(lps22hh_ctx_t *ctx,
                                  lps22hh_i3c_disable_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_IF_CTRL, &reg.byte, 1);
  *val = (lps22hh_i3c_disable_t)((reg.if_ctrl.int_en_i3c << 4) +
         reg.if_ctrl.int_en_i3c);

  return mm_error;
}

/**
  * @brief  sdo_sa0_mode: [set]  Enable/Disable pull-up on SDO pin.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_sdo_pu_en_t: change the values of sdo_pu_en in reg
  *                              IF_CTRL
  *
  */
int32_t lps22hh_sdo_sa0_mode_set(lps22hh_ctx_t *ctx, lps22hh_pu_en_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_IF_CTRL, &reg.byte, 1);
  reg.if_ctrl.sdo_pu_en = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_IF_CTRL, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  sdo_sa0_mode: [get]  Enable/Disable pull-up on SDO pin.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_sdo_pu_en_t: Get the values of sdo_pu_en in reg IF_CTRL
  *
  */
int32_t lps22hh_sdo_sa0_mode_get(lps22hh_ctx_t *ctx, lps22hh_pu_en_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_IF_CTRL, &reg.byte, 1);
  *val = (lps22hh_pu_en_t) reg.if_ctrl.sdo_pu_en;

  return mm_error;
}

/**
  * @brief  sda_mode: [set]  Connect/Disconnect SDO/SA0 internal pull-up.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_sda_pu_en_t: change the values of sda_pu_en in reg
  *                              IF_CTRL
  *
  */
int32_t lps22hh_sda_mode_set(lps22hh_ctx_t *ctx, lps22hh_pu_en_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_IF_CTRL, &reg.byte, 1);
  reg.if_ctrl.sda_pu_en = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_IF_CTRL, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  sda_mode: [get]  Connect/Disconnect SDO/SA0 internal pull-up.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_sda_pu_en_t: Get the values of sda_pu_en in reg IF_CTRL
  *
  */
int32_t lps22hh_sda_mode_get(lps22hh_ctx_t *ctx, lps22hh_pu_en_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_IF_CTRL, &reg.byte, 1);
  *val = (lps22hh_pu_en_t) reg.if_ctrl.sda_pu_en;

  return mm_error;
}

/**
  * @brief  spi_mode: [set]  SPI Serial Interface Mode selection.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_sim_t: change the values of sim in reg CTRL_REG1
  *
  */
int32_t lps22hh_spi_mode_set(lps22hh_ctx_t *ctx, lps22hh_sim_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.sim = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_CTRL_REG1, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  spi_mode: [get]  SPI Serial Interface Mode selection.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_sim_t: Get the values of sim in reg CTRL_REG1
  *
  */
int32_t lps22hh_spi_mode_get(lps22hh_ctx_t *ctx, lps22hh_sim_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG1, &reg.byte, 1);
  *val = (lps22hh_sim_t) reg.ctrl_reg1.sim;

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
  * @brief  int_notification: [set] Latch interrupt request to the
  *                                 INT_SOURCE (24h) register.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_lir_t: change the values of lir in reg INTERRUPT_CFG
  *
  */
int32_t lps22hh_int_notification_set(lps22hh_ctx_t *ctx, lps22hh_lir_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);
  reg.interrupt_cfg.lir = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int_notification: [get] Latch interrupt request to the
  *                                 INT_SOURCE (24h) register.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_lir_t: Get the values of lir in reg INTERRUPT_CFG
  *
  */
int32_t lps22hh_int_notification_get(lps22hh_ctx_t *ctx, lps22hh_lir_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);
  *val = (lps22hh_lir_t) reg.interrupt_cfg.lir;

  return mm_error;
}

/**
  * @brief  pin_mode: [set]  Push-pull/open drain selection on interrupt pads.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_pp_od_t: change the values of pp_od in reg CTRL_REG2
  *
  */
int32_t lps22hh_pin_mode_set(lps22hh_ctx_t *ctx, lps22hh_pp_od_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.pp_od = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_mode: [get]  Push-pull/open drain selection on interrupt pads.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_pp_od_t: Get the values of pp_od in reg CTRL_REG2
  *
  */
int32_t lps22hh_pin_mode_get(lps22hh_ctx_t *ctx, lps22hh_pp_od_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);
  *val = (lps22hh_pp_od_t) reg.ctrl_reg2.pp_od;

  return mm_error;
}

/**
  * @brief  pin_polarity: [set]  Interrupt active-high/low.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_int_h_l_t: change the values of int_h_l in reg CTRL_REG2
  *
  */
int32_t lps22hh_pin_polarity_set(lps22hh_ctx_t *ctx, lps22hh_int_h_l_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.int_h_l = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_polarity: [get]  Interrupt active-high/low.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_int_h_l_t: Get the values of int_h_l in reg CTRL_REG2
  *
  */
int32_t lps22hh_pin_polarity_get(lps22hh_ctx_t *ctx, lps22hh_int_h_l_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_CTRL_REG2, &reg.byte, 1);
  *val = (lps22hh_int_h_l_t) reg.ctrl_reg2.int_h_l;

  return mm_error;
}

/**
  * @brief  pin_int_route: [set] Select the signal that need to route
  *                              on int pad
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_: registers CTRL_REG3
  *
  */
int32_t lps22hh_pin_int_route_set(lps22hh_ctx_t *ctx,
                                  lps22hh_ctrl_reg3_t *val)
{
  return lps22hh_write_reg(ctx, LPS22HH_CTRL_REG3, (uint8_t*) val, 1);
}

/**
  * @brief  pin_int_route: [get] Select the signal that need to route
  *                              on int pad
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_: registers CTRL_REG3
  *
  */
int32_t lps22hh_pin_int_route_get(lps22hh_ctx_t *ctx,
                                  lps22hh_ctrl_reg3_t *val)
{
  return lps22hh_read_reg(ctx, LPS22HH_CTRL_REG3, (uint8_t*) val, 1);
}

/**
  * @}
  */

/**
  * @addtogroup   interrupt_on_threshold
  * @brief   This section groups all the functions that manage the interrupt
  *          on threshold event generation.
  * @{
  */

/**
  * @brief  int_on_threshold: [set] Enable interrupt generation on
  *                                 pressure low/high event.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_pe_t: change the values of pe in reg INTERRUPT_CFG
  *
  */
int32_t lps22hh_int_on_threshold_set(lps22hh_ctx_t *ctx, lps22hh_pe_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);
  reg.interrupt_cfg.pe = val;

  if (val)
    reg.interrupt_cfg.diff_en = 1;
  else
    reg.interrupt_cfg.diff_en = 0;

  mm_error = lps22hh_write_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int_on_threshold: [get] Enable interrupt generation on
  *                                 pressure low/high event.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_pe_t: Get the values of pe in reg INTERRUPT_CFG
  *
  */
int32_t lps22hh_int_on_threshold_get(lps22hh_ctx_t *ctx, lps22hh_pe_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_INTERRUPT_CFG, &reg.byte, 1);
  *val = (lps22hh_pe_t) reg.interrupt_cfg.pe;

  return mm_error;
}

/**
  * @brief  int_treshold: [set] User-defined threshold value for
  *                             pressure interrupt event.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that contains data to write
  *
  */
int32_t lps22hh_int_treshold_set(lps22hh_ctx_t *ctx, uint8_t *buff)
{
  return lps22hh_write_reg(ctx, LPS22HH_THS_P_L, buff, 2);
}

/**
  * @brief  int_treshold: [get] User-defined threshold value for
  *                             pressure interrupt event.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lps22hh_int_treshold_get(lps22hh_ctx_t *ctx, uint8_t *buff)
{
  return lps22hh_read_reg(ctx, LPS22HH_THS_P_L, buff, 2);
}

/**
  * @}
  */

/**
  * @addtogroup  fifo
  * @brief   This section group all the functions concerning the fifo usage.
  * @{
  */

/**
  * @brief  fifo_mode: [set]
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_f_mode_t: change the values of f_mode in reg FIFO_CTRL
  *
  */
int32_t lps22hh_fifo_mode_set(lps22hh_ctx_t *ctx, lps22hh_f_mode_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_FIFO_CTRL, &reg.byte, 1);
  reg.fifo_ctrl.f_mode = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_FIFO_CTRL, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  fifo_mode: [get]
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_f_mode_t: Get the values of f_mode in reg FIFO_CTRL
  *
  */
int32_t lps22hh_fifo_mode_get(lps22hh_ctx_t *ctx, lps22hh_f_mode_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_FIFO_CTRL, &reg.byte, 1);
  *val = (lps22hh_f_mode_t) reg.fifo_ctrl.f_mode;

  return mm_error;
}

/**
  * @brief  fifo_stop_on_wtm: [set] Sensing chain FIFO stop values
  *                                 memorization at threshold level.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of stop_on_fth in reg FIFO_CTRL
  *
  */
int32_t lps22hh_fifo_stop_on_wtm_set(lps22hh_ctx_t *ctx, uint8_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_FIFO_CTRL, &reg.byte, 1);
  reg.fifo_ctrl.stop_on_fth = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_FIFO_CTRL, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  fifo_stop_on_wtm: [get] Sensing chain FIFO stop values
  *                                 memorization at threshold level.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of stop_on_fth in reg FIFO_CTRL
  *
  */
int32_t lps22hh_fifo_stop_on_wtm_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_FIFO_CTRL, &reg.byte, 1);
  *val = reg.fifo_ctrl.stop_on_fth;

  return mm_error;
}

/**
  * @brief  fifo_watermark: [set]  FIFO watermark level selection.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of wtm in reg FIFO_WTM
  *
  */
int32_t lps22hh_fifo_watermark_set(lps22hh_ctx_t *ctx, uint8_t val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_FIFO_WTM, &reg.byte, 1);
  reg.fifo_wtm.wtm = val;
  mm_error = lps22hh_write_reg(ctx, LPS22HH_FIFO_WTM, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  fifo_watermark: [get]  FIFO watermark level selection.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of wtm in reg FIFO_WTM
  *
  */
int32_t lps22hh_fifo_watermark_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_FIFO_WTM, &reg.byte, 1);
  *val = reg.fifo_wtm.wtm;

  return mm_error;
}

/**
  * @brief  fifo_data_level: [get]  FIFO stored data level.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lps22hh_fifo_data_level_get(lps22hh_ctx_t *ctx, uint8_t *buff)
{
  return lps22hh_read_reg(ctx, LPS22HH_FIFO_STATUS1, buff, 1);
}

/**
  * @brief  fifo_src: [get]  Read all the FIFO status flag of the device.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  lps22hh_: union of registers from FIFO_STATUS2 to
  *
  */
int32_t lps22hh_fifo_src_get(lps22hh_ctx_t *ctx, lps22hh_fifo_status2_t *val)
{
  return lps22hh_read_reg(ctx, LPS22HH_FIFO_STATUS2, (uint8_t*) val, 1);
}

/**
  * @brief  fifo_full_flag: [get]  Smart FIFO full status.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of f_full in reg FIFO_STATUS2
  *
  */
int32_t lps22hh_fifo_full_flag_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_FIFO_STATUS2, &reg.byte, 1);
  *val = reg.fifo_status2.f_full;

  return mm_error;
}

/**
  * @brief  fifo_ovr_flag: [get]  FIFO overrun status.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ovr in reg FIFO_STATUS2
  *
  */
int32_t lps22hh_fifo_ovr_flag_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_FIFO_STATUS2, &reg.byte, 1);
  *val = reg.fifo_status2.ovr;

  return mm_error;
}

/**
  * @brief  fifo_wtm_flag: [get]  FIFO watermark status.
  *
  * @param  lps22hh_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fth_fifo in reg FIFO_STATUS2
  *
  */
int32_t lps22hh_fifo_wtm_flag_get(lps22hh_ctx_t *ctx, uint8_t *val)
{
  lps22hh_reg_t reg;
  int32_t mm_error;

  mm_error = lps22hh_read_reg(ctx, LPS22HH_FIFO_STATUS2, &reg.byte, 1);
  *val = reg.fifo_status2.fth_fifo;

  return mm_error;
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/