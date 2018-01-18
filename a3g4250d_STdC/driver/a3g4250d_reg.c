/*
 ******************************************************************************
 * @file    a3g4250d_reg.c
 * @author  MEMS Software Solution Team
 * @date    20-December-2017
 * @brief   A3G4250D driver file
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

#include "a3g4250d_reg.h"

/**
  * @addtogroup  a3g4250d
  * @brief  This file provides a set of functions needed to drive the
  *         a3g4250d enanced inertial module.
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
  * @param  a3g4250d_ctx_t* ctx: read / write interface definitions
  * @param  uint8_t reg: register to read
  * @param  uint8_t* data: pointer to buffer that store the data read
  * @param  uint16_t len: number of consecutive register to read
  *
  */
int32_t a3g4250d_read_reg(a3g4250d_ctx_t* ctx, uint8_t reg, uint8_t* data,
                          uint16_t len)
{
  return ctx->read_reg(ctx->handle, reg, data, len);
}

/**
  * @brief  Write generic device register
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t reg: register to write
  * @param  uint8_t* data: pointer to data to write in register reg
  * @param  uint16_t len: number of consecutive register to write
  *
*/
int32_t a3g4250d_write_reg(a3g4250d_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
  * @brief  data_rate: [set]  Accelerometer data rate selection.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_dr_t: change the values of dr in reg CTRL_REG1
  *
  */
int32_t a3g4250d_data_rate_set(a3g4250d_ctx_t *ctx, a3g4250d_dr_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.dr = (val & 0x30) >> 4;
  reg.ctrl_reg1.pd = (val & 0x0F);
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG1, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  data_rate: [get]  Accelerometer data rate selection.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_dr_t: Get the values of dr in reg CTRL_REG1
  *
  */
int32_t a3g4250d_data_rate_get(a3g4250d_ctx_t *ctx, a3g4250d_dr_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG1, &reg.byte, 1);
  *val = (a3g4250d_dr_t) ( ( reg.ctrl_reg1.dr  << 4 ) + reg.ctrl_reg1.pd );

  return mm_error;
}

/**
  * @brief  status_reg: [get] The STATUS_REG register is read by the
  *                           primary interface
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_status_reg_t: registers STATUS_REG
  *
  */
int32_t a3g4250d_status_reg_get(a3g4250d_ctx_t *ctx,
                                a3g4250d_status_reg_t *val)
{
  return a3g4250d_read_reg(ctx, A3G4250D_STATUS_REG, (uint8_t*) val, 1);
}

/**
  * @brief  flag_data_ready: [get]  Accelerometer new data available.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of zyxda in reg STATUS_REG
  *
  */
int32_t a3g4250d_flag_data_ready_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_STATUS_REG, &reg.byte, 1);
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
  * @brief  temperature_raw: [get] Temperaturedata.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t a3g4250d_temperature_raw_get(a3g4250d_ctx_t *ctx, uint8_t *buff)
{
  return a3g4250d_read_reg(ctx, A3G4250D_OUT_TEMP, buff, 1);
}

/**
  * @brief  angular_rate_raw: [get] Angular rate sensor. The value is
  *                                 expressed as a 16-bit word in two’s
  *                                 complement.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t a3g4250d_angular_rate_raw_get(a3g4250d_ctx_t *ctx, uint8_t *buff)
{
  return a3g4250d_read_reg(ctx, A3G4250D_OUT_X_L, buff, 6);
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
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t a3g4250d_device_id_get(a3g4250d_ctx_t *ctx, uint8_t *buff)
{
  return a3g4250d_read_reg(ctx, A3G4250D_WHO_AM_I, buff, 1);
}

/**
  * @brief  self_test: [set]  Angular rate sensor self-test enable.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_st_t: change the values of st in reg CTRL_REG4
  *
  */
int32_t a3g4250d_self_test_set(a3g4250d_ctx_t *ctx, a3g4250d_st_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.st = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  self_test: [get]  Angular rate sensor self-test enable.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_st_t: Get the values of st in reg CTRL_REG4
  *
  */
int32_t a3g4250d_self_test_get(a3g4250d_ctx_t *ctx, a3g4250d_st_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG4, &reg.byte, 1);
  *val = (a3g4250d_st_t) reg.ctrl_reg4.st;

  return mm_error;
}

/**
  * @brief  data_format: [set] Big/Little Endian Data selection on
  *                            aux interface.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_ble_t: change the values of ble in reg CTRL_REG4
  *
  */
int32_t a3g4250d_data_format_set(a3g4250d_ctx_t *ctx, a3g4250d_ble_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.ble = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  data_format: [get] Big/Little Endian Data selection on
  *                            aux interface.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_ble_t: Get the values of ble in reg CTRL_REG4
  *
  */
int32_t a3g4250d_data_format_get(a3g4250d_ctx_t *ctx, a3g4250d_ble_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG4, &reg.byte, 1);
  *val = (a3g4250d_ble_t) reg.ctrl_reg4.ble;

  return mm_error;
}

/**
  * @brief  boot: [set] Reboot memory content. Reload the
  *                     calibration parameters.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of boot in reg CTRL_REG5
  *
  */
int32_t a3g4250d_boot_set(a3g4250d_ctx_t *ctx, uint8_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5, &reg.byte, 1);
  reg.ctrl_reg5.boot = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG5, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  boot: [get] Reboot memory content. Reload the
  *                     calibration parameters.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of boot in reg CTRL_REG5
  *
  */
int32_t a3g4250d_boot_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5, &reg.byte, 1);
  *val = reg.ctrl_reg5.boot;

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
  * @brief  lp_bandwidth: [set]  Bandwidth selection.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_bw_t: change the values of bw in reg CTRL_REG1
  *
  */
int32_t a3g4250d_lp_bandwidth_set(a3g4250d_ctx_t *ctx, a3g4250d_bw_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.bw = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG1, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  lp_bandwidth: [get]  Bandwidth selection.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_bw_t: Get the values of bw in reg CTRL_REG1
  *
  */
int32_t a3g4250d_lp_bandwidth_get(a3g4250d_ctx_t *ctx, a3g4250d_bw_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG1, &reg.byte, 1);
  *val = (a3g4250d_bw_t) reg.ctrl_reg1.bw;

  return mm_error;
}

/**
  * @brief  hp_bandwidth: [set]
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_hpcf_t: change the values of hpcf in reg CTRL_REG2
  *
  */
int32_t a3g4250d_hp_bandwidth_set(a3g4250d_ctx_t *ctx, a3g4250d_hpcf_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.hpcf = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  hp_bandwidth: [get]
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_hpcf_t: Get the values of hpcf in reg CTRL_REG2
  *
  */
int32_t a3g4250d_hp_bandwidth_get(a3g4250d_ctx_t *ctx, a3g4250d_hpcf_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG2, &reg.byte, 1);
  *val = (a3g4250d_hpcf_t) reg.ctrl_reg2.hpcf;

  return mm_error;
}

/**
  * @brief  hp_mode: [set]  High-pass filter mode selection.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_hpm_t: change the values of hpm in reg CTRL_REG2
  *
  */
int32_t a3g4250d_hp_mode_set(a3g4250d_ctx_t *ctx, a3g4250d_hpm_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.hpm = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  hp_mode: [get]  High-pass filter mode selection.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_hpm_t: Get the values of hpm in reg CTRL_REG2
  *
  */
int32_t a3g4250d_hp_mode_get(a3g4250d_ctx_t *ctx, a3g4250d_hpm_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG2, &reg.byte, 1);
  *val = (a3g4250d_hpm_t) reg.ctrl_reg2.hpm;

  return mm_error;
}

/**
  * @brief  filter_path: [set]  Out/FIFO selection path.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_out_sel_t: change the values of out_sel in reg CTRL_REG5
  *
  */
int32_t a3g4250d_filter_path_set(a3g4250d_ctx_t *ctx, a3g4250d_out_sel_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5, &reg.byte, 1);
  reg.ctrl_reg5.out_sel = val & 0x03;
  reg.ctrl_reg5.hpen = ( val & 0x04 ) >> 2;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG5, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  filter_path: [get]  Out/FIFO selection path.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_out_sel_t: Get the values of out_sel in reg CTRL_REG5
  *
  */
int32_t a3g4250d_filter_path_get(a3g4250d_ctx_t *ctx, a3g4250d_out_sel_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5, &reg.byte, 1);
  *val = (a3g4250d_out_sel_t) ( ( reg.ctrl_reg5.hpen << 2 ) +
                               reg.ctrl_reg5.out_sel );

  return mm_error;
}

/**
  * @brief   filter_path_internal: [set]  Interrupt generator selection path.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_int1_sel_t: change the values of int1_sel in
  *                              reg CTRL_REG5
  *
  */
int32_t a3g4250d_filter_path_internal_set(a3g4250d_ctx_t *ctx,
                                          a3g4250d_int1_sel_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5, &reg.byte, 1);
  reg.ctrl_reg5.int1_sel = val & 0x03;
  reg.ctrl_reg5.hpen = ( val & 0x04 ) >> 2;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG5, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   filter_path_internal: [get]  Interrupt generator selection path.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_int1_sel_t: Get the values of int1_sel in reg CTRL_REG5
  *
  */
int32_t a3g4250d_filter_path_internal_get(a3g4250d_ctx_t *ctx,
                                          a3g4250d_int1_sel_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5, &reg.byte, 1);
  *val = (a3g4250d_int1_sel_t) ( ( reg.ctrl_reg5.hpen << 2 ) +
                                reg.ctrl_reg5.int1_sel );

  return mm_error;
}

/**
  * @brief  hp_reference_value: [set]  Reference value for high-pass filter.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of ref in reg REFERENCE
  *
  */
int32_t a3g4250d_hp_reference_value_set(a3g4250d_ctx_t *ctx, uint8_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_REFERENCE, &reg.byte, 1);
  reg.reference.ref = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_REFERENCE, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  hp_reference_value: [get]  Reference value for high-pass filter.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ref in reg REFERENCE
  *
  */
int32_t a3g4250d_hp_reference_value_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_REFERENCE, &reg.byte, 1);
  *val = reg.reference.ref;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  serial_interface
  * @brief   This section groups all the functions concerning main serial
  *          interface management.
  * @{
  */

/**
  * @brief  spi_mode: [set]   SPI Serial Interface Mode selection.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_sim_t: change the values of sim in reg CTRL_REG4
  *
  */
int32_t a3g4250d_spi_mode_set(a3g4250d_ctx_t *ctx, a3g4250d_sim_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG4, &reg.byte, 1);
  reg.ctrl_reg4.sim = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG4, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  spi_mode: [get]   SPI Serial Interface Mode selection.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_sim_t: Get the values of sim in reg CTRL_REG4
  *
  */
int32_t a3g4250d_spi_mode_get(a3g4250d_ctx_t *ctx, a3g4250d_sim_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG4, &reg.byte, 1);
  *val = (a3g4250d_sim_t) reg.ctrl_reg4.sim;

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
  * @brief  pin_int1_route: [set] Select the signal that need to route on
  *                               int1 pad.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_int1_route_t: configure CTRL_REG3 int1 pad
  *
  */
int32_t a3g4250d_pin_int1_route_set(a3g4250d_ctx_t *ctx,
                                    a3g4250d_int1_route_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3, &reg.byte, 1);
  reg.ctrl_reg3.i1_int1         = val.i1_int1;
  reg.ctrl_reg3.i1_boot         = val.i1_boot;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_int1_route: [get] Select the signal that need to
  *                               route on int1 pad.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_int1_route_t: read CTRL_REG3 int1 pad
  *
  */
int32_t a3g4250d_pin_int1_route_get(a3g4250d_ctx_t *ctx,
                                    a3g4250d_int1_route_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3, &reg.byte, 1);
  val->i1_int1       = reg.ctrl_reg3.i1_int1;
  val->i1_boot       = reg.ctrl_reg3.i1_boot;

  return mm_error;
}
/**
  * @brief  pin_int2_route: [set] Select the signal that need to
  *                               route on int2 pad.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_int2_route_t: configure CTRL_REG3 int2 pad
  *
  */
int32_t a3g4250d_pin_int2_route_set(a3g4250d_ctx_t *ctx,
                                    a3g4250d_int2_route_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3, &reg.byte, 1);
  reg.ctrl_reg3.i2_empty         = val.i2_empty;
  reg.ctrl_reg3.i2_orun          = val.i2_orun;
  reg.ctrl_reg3.i2_wtm           = val.i2_wtm;
  reg.ctrl_reg3.i2_drdy          = val.i2_drdy;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_int2_route: [get] Select the signal that need to route
  *                               on int2 pad.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_int2_route_t:  read CTRL_REG3 int2 pad
  *
  */
int32_t a3g4250d_pin_int2_route_get(a3g4250d_ctx_t *ctx,
                                    a3g4250d_int2_route_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3, &reg.byte, 1);
  val->i2_empty       = reg.ctrl_reg3.i2_empty;
  val->i2_orun        = reg.ctrl_reg3.i2_orun;
  val->i2_wtm         = reg.ctrl_reg3.i2_wtm;
  val->i2_drdy        = reg.ctrl_reg3.i2_drdy;

  return mm_error;
}
/**
  * @brief  pin_mode: [set]  Push-pull/open drain selection on interrupt pads.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_pp_od_t: change the values of pp_od in reg CTRL_REG3
  *
  */
int32_t a3g4250d_pin_mode_set(a3g4250d_ctx_t *ctx, a3g4250d_pp_od_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3, &reg.byte, 1);
  reg.ctrl_reg3.pp_od = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_mode: [get]  Push-pull/open drain selection on interrupt pads.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_pp_od_t: Get the values of pp_od in reg CTRL_REG3
  *
  */
int32_t a3g4250d_pin_mode_get(a3g4250d_ctx_t *ctx, a3g4250d_pp_od_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3, &reg.byte, 1);
  *val = (a3g4250d_pp_od_t) reg.ctrl_reg3.pp_od;

  return mm_error;
}

/**
  * @brief  pin_polarity: [set]  Pin active-high/low.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_h_lactive_t: change the values of h_lactive in
  *                               reg CTRL_REG3
  *
  */
int32_t a3g4250d_pin_polarity_set(a3g4250d_ctx_t *ctx,
                                  a3g4250d_h_lactive_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3, &reg.byte, 1);
  reg.ctrl_reg3.h_lactive = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_polarity: [get]  Pin active-high/low.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_h_lactive_t: Get the values of h_lactive in
  *                               reg CTRL_REG3
  *
  */
int32_t a3g4250d_pin_polarity_get(a3g4250d_ctx_t *ctx,
                                  a3g4250d_h_lactive_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3, &reg.byte, 1);
  *val = (a3g4250d_h_lactive_t) reg.ctrl_reg3.h_lactive;

  return mm_error;
}

/**
  * @brief  int_notification: [set]  Latched/pulsed interrupt.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_lir_t: change the values of lir in reg INT1_CFG
  *
  */
int32_t a3g4250d_int_notification_set(a3g4250d_ctx_t *ctx,
                                      a3g4250d_lir_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_INT1_CFG, &reg.byte, 1);
  reg.int1_cfg.lir = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_INT1_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int_notification: [get]  Latched/pulsed interrupt.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_lir_t: Get the values of lir in reg INT1_CFG
  *
  */
int32_t a3g4250d_int_notification_get(a3g4250d_ctx_t *ctx,
                                      a3g4250d_lir_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_INT1_CFG, &reg.byte, 1);
  *val = (a3g4250d_lir_t) reg.int1_cfg.lir;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup   interrupt_on_threshold
  * @brief   This section groups all the functions that manage the event
  *          generation on threshold.
  * @{
  */

/**
  * @brief   int_on_threshold_conf: [set]  Configure the interrupt
  *          threshold sign.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_: union of registers from INT1_CFG to
  *
  */
int32_t a3g4250d_int_on_threshold_conf_set(a3g4250d_ctx_t *ctx,
                                           a3g4250d_int1_cfg_t *val)
{
  return a3g4250d_write_reg(ctx, A3G4250D_INT1_CFG, (uint8_t*) val, 1);
}

/**
  * @brief   int_on_threshold_conf: [get] Configure the interrupt
  *                                       threshold sign.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_: union of registers from INT1_CFG to
  *
  */
int32_t a3g4250d_int_on_threshold_conf_get(a3g4250d_ctx_t *ctx,
                                           a3g4250d_int1_cfg_t *val)
{
  return a3g4250d_read_reg(ctx, A3G4250D_INT1_CFG, (uint8_t*) val, 1);
}
/**
  * @brief   int_on_threshold_mode: [set] AND/OR combination of
  *                                       interrupt events.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_and_or_t: change the values of and_or in reg INT1_CFG
  *
  */
int32_t a3g4250d_int_on_threshold_mode_set(a3g4250d_ctx_t *ctx,
                                           a3g4250d_and_or_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_INT1_CFG, &reg.byte, 1);
  reg.int1_cfg.and_or = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_INT1_CFG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   int_on_threshold_mode: [get] AND/OR combination of
  *                                       interrupt events.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_and_or_t: Get the values of and_or in reg INT1_CFG
  *
  */
int32_t a3g4250d_int_on_threshold_mode_get(a3g4250d_ctx_t *ctx,
                                           a3g4250d_and_or_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_INT1_CFG, &reg.byte, 1);
  *val = (a3g4250d_and_or_t) reg.int1_cfg.and_or;

  return mm_error;
}

/**
  * @brief   int_on_threshold_src: [get]
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  a3g4250d_: union of registers from INT1_SRC to
  *
  */
int32_t a3g4250d_int_on_threshold_src_get(a3g4250d_ctx_t *ctx,
                                          a3g4250d_int1_src_t *val)
{
  return a3g4250d_read_reg(ctx, A3G4250D_INT1_SRC, (uint8_t*) val, 1);
}
/**
  * @brief  int_x_treshold: [set]  Interrupt threshold on X.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of thsx in reg INT1_TSH_XH
  *
  */
int32_t a3g4250d_int_x_treshold_set(a3g4250d_ctx_t *ctx, uint8_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_XH, &reg.byte, 1);
  reg.int1_tsh_xh.thsx = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_INT1_TSH_XH, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int_x_treshold: [get]  Interrupt threshold on X.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of thsx in reg INT1_TSH_XH
  *
  */
int32_t a3g4250d_int_x_treshold_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_XH, &reg.byte, 1);
  *val = reg.int1_tsh_xh.thsx;

  return mm_error;
}

/**
  * @brief  int_y_treshold: [set]  Interrupt threshold on Y.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of thsy in reg INT1_TSH_YH
  *
  */
int32_t a3g4250d_int_y_treshold_set(a3g4250d_ctx_t *ctx, uint8_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_YH, &reg.byte, 1);
  reg.int1_tsh_yh.thsy = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_INT1_TSH_YH, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int_y_treshold: [get]  Interrupt threshold on Y.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of thsy in reg INT1_TSH_YH
  *
  */
int32_t a3g4250d_int_y_treshold_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_YH, &reg.byte, 1);
  *val = reg.int1_tsh_yh.thsy;

  return mm_error;
}

/**
  * @brief  int_z_treshold: [set]  Interrupt threshold on Z.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of thsz in reg INT1_TSH_ZH
  *
  */
int32_t a3g4250d_int_z_treshold_set(a3g4250d_ctx_t *ctx, uint8_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_ZH, &reg.byte, 1);
  reg.int1_tsh_zh.thsz = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_INT1_TSH_ZH, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int_z_treshold: [get]  Interrupt threshold on Z.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of thsz in reg INT1_TSH_ZH
  *
  */
int32_t a3g4250d_int_z_treshold_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_ZH, &reg.byte, 1);
  *val = reg.int1_tsh_zh.thsz;

  return mm_error;
}

/**
  * @brief   int_on_threshold_dur: [set] Durationvalue.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of d in reg INT1_DURATION
  *
  */
int32_t a3g4250d_int_on_threshold_dur_set(a3g4250d_ctx_t *ctx, uint8_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_INT1_DURATION, &reg.byte, 1);
  reg.int1_duration.d = val;
  if (val){
    reg.int1_duration.wait = PROPERTY_ENABLE;
  }
  else{
    reg.int1_duration.wait = PROPERTY_DISABLE;
  }
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_INT1_DURATION, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief   int_on_threshold_dur: [get] Durationvalue.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of d in reg INT1_DURATION
  *
  */
int32_t a3g4250d_int_on_threshold_dur_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_INT1_DURATION, &reg.byte, 1);
  *val = reg.int1_duration.d;

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
  * @brief  fifo_enable: [set] FIFOenable.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of fifo_en in reg CTRL_REG5
  *
  */
int32_t a3g4250d_fifo_enable_set(a3g4250d_ctx_t *ctx, uint8_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5, &reg.byte, 1);
  reg.ctrl_reg5.fifo_en = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG5, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  fifo_enable: [get] FIFOenable.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fifo_en in reg CTRL_REG5
  *
  */
int32_t a3g4250d_fifo_enable_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5, &reg.byte, 1);
  *val = reg.ctrl_reg5.fifo_en;

  return mm_error;
}

/**
  * @brief  fifo_watermark: [set]  FIFO watermark level selection.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of wtm in reg FIFO_CTRL_REG
  *
  */
int32_t a3g4250d_fifo_watermark_set(a3g4250d_ctx_t *ctx, uint8_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_FIFO_CTRL_REG, &reg.byte, 1);
  reg.fifo_ctrl_reg.wtm = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_FIFO_CTRL_REG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  fifo_watermark: [get]  FIFO watermark level selection.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of wtm in reg FIFO_CTRL_REG
  *
  */
int32_t a3g4250d_fifo_watermark_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_FIFO_CTRL_REG, &reg.byte, 1);
  *val = reg.fifo_ctrl_reg.wtm;

  return mm_error;
}

/**
  * @brief  fifo_mode: [set]  FIFO mode selection.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of fm in reg FIFO_CTRL_REG
  *
  */
int32_t a3g4250d_fifo_mode_set(a3g4250d_ctx_t *ctx, uint8_t val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_FIFO_CTRL_REG, &reg.byte, 1);
  reg.fifo_ctrl_reg.fm = val;
  mm_error = a3g4250d_write_reg(ctx, A3G4250D_FIFO_CTRL_REG, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  fifo_mode: [get]  FIFO mode selection.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fm in reg FIFO_CTRL_REG
  *
  */
int32_t a3g4250d_fifo_mode_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_FIFO_CTRL_REG, &reg.byte, 1);
  *val = reg.fifo_ctrl_reg.fm;

  return mm_error;
}

/**
  * @brief  fifo_data_level: [get]  FIFO stored data level
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of fss in reg FIFO_SRC_REG
  *
  */
int32_t a3g4250d_fifo_data_level_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_FIFO_SRC_REG, &reg.byte, 1);
  *val = reg.fifo_src_reg.fss;

  return mm_error;
}
/**
  * @brief  fifo_empty_flag: [get] FIFOemptybit.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of empty in reg FIFO_SRC_REG
  *
  */
int32_t a3g4250d_fifo_empty_flag_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_FIFO_SRC_REG, &reg.byte, 1);
  *val = reg.fifo_src_reg.empty;

  return mm_error;
}
/**
  * @brief  fifo_ovr_flag: [get]  Overrun bit status.
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of ovrn in reg FIFO_SRC_REG
  *
  */
int32_t a3g4250d_fifo_ovr_flag_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_FIFO_SRC_REG, &reg.byte, 1);
  *val = reg.fifo_src_reg.ovrn;

  return mm_error;
}
/**
  * @brief  fifo_wtm_flag: [get] Watermark status. (0: FIFO filling is
  *                              lower than WTM level; 1: FIFO filling
  *                              is equal or higher than WTM level)
  *
  * @param  a3g4250d_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of wtm in reg FIFO_SRC_REG
  *
  */
int32_t a3g4250d_fifo_wtm_flag_get(a3g4250d_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_reg_t reg;
  int32_t mm_error;

  mm_error = a3g4250d_read_reg(ctx, A3G4250D_FIFO_SRC_REG, &reg.byte, 1);
  *val = reg.fifo_src_reg.wtm;

  return mm_error;
}
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/