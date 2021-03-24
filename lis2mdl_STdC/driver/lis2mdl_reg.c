/*
 ******************************************************************************
 * @file    lis2mdl_reg.c
 * @author  Sensors Software Solution Team
 * @brief   LIS2MDL driver file
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

#include "lis2mdl_reg.h"

/**
  * @defgroup    LIS2MDL
  * @brief       This file provides a set of functions needed to drive the
  *              lis2mdl enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup    LIS2MDL_Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
                         uint8_t *data,
                         uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);
  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                          uint8_t *data,
                          uint16_t len)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len);
  return ret;
}

/**
  * @}
  *
  */

/**
* @defgroup    LIS2MDL_Sensitivity
* @brief       These functions convert raw-data into engineering units.
* @{
*
*/
float_t lis2mdl_from_lsb_to_mgauss(int16_t lsb)
{
  return ((float_t)lsb * 1.5f);
}

float_t lis2mdl_from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb / 8.0f) + 25.0f);
}

/**
  * @}
  *
  */

/**
  * @defgroup    LIS2MDL_data_generation
  * @brief       This section group all the functions concerning
  *              data generation
  * @{
  *
  */

/**
  * @brief  These registers comprise a 3 group of 16-bit number and represent
  *         hard-iron offset in order to compensate environmental effects.
  *         Data format is the same of output data raw: two’s complement
  *         with 1LSb = 1.5mG. These values act on the magnetic output data
  *         value in order to delete the environmental offset.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  buffer that contains data to write
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_mag_user_offset_set(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;
  buff[1] = (uint8_t) ( (uint16_t)val[0] / 256U);
  buff[0] = (uint8_t) ( (uint16_t)val[0] - (buff[1] * 256U));
  buff[3] = (uint8_t) ( (uint16_t)val[1] / 256U);
  buff[2] = (uint8_t) ( (uint16_t)val[1] - (buff[1] * 256U));
  buff[5] = (uint8_t) ( (uint16_t)val[2] / 256U);
  buff[4] = (uint8_t) ( (uint16_t)val[2] - (buff[1] * 256U));
  ret = lis2mdl_write_reg(ctx, LIS2MDL_OFFSET_X_REG_L, buff, 6);
  return ret;
}

/**
  * @brief  These registers comprise a 3 group of 16-bit number and represent
  *         hard-iron offset in order to compensate environmental effects.
  *         Data format is the same of output data raw: two’s complement
  *         with 1LSb = 1.5mG. These values act on the magnetic output data
  *         value in order to delete the environmental offset.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  that stores data read
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_mag_user_offset_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_OFFSET_X_REG_L, buff, 6);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) +  (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) +  (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) +  (int16_t)buff[4];
  return ret;
}

/**
  * @brief  Operating mode selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of md in reg CFG_REG_A
  * @retval        interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_operating_mode_set(stmdev_ctx_t *ctx,
                                   lis2mdl_md_t val)
{
  lis2mdl_cfg_reg_a_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.md = (uint8_t)val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Operating mode selection.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of md in reg CFG_REG_A.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_operating_mode_get(stmdev_ctx_t *ctx,
                                   lis2mdl_md_t *val)
{
  lis2mdl_cfg_reg_a_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);

  switch (reg.md) {
    case LIS2MDL_POWER_DOWN:
      *val = LIS2MDL_POWER_DOWN;
      break;

    case LIS2MDL_CONTINUOUS_MODE:
      *val = LIS2MDL_CONTINUOUS_MODE;
      break;

    case LIS2MDL_SINGLE_TRIGGER:
      *val = LIS2MDL_SINGLE_TRIGGER;
      break;

    default:
      *val = LIS2MDL_POWER_DOWN;
      break;
  }

  return ret;
}

/**
  * @brief  Output data rate selection.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of odr in reg CFG_REG_A
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_data_rate_set(stmdev_ctx_t *ctx, lis2mdl_odr_t val)
{
  lis2mdl_cfg_reg_a_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.odr = (uint8_t)val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Output data rate selection.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of odr in reg CFG_REG_A.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_data_rate_get(stmdev_ctx_t *ctx, lis2mdl_odr_t *val)
{
  lis2mdl_cfg_reg_a_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);

  switch (reg.odr) {
    case LIS2MDL_ODR_10Hz:
      *val = LIS2MDL_ODR_10Hz;
      break;

    case LIS2MDL_ODR_20Hz:
      *val = LIS2MDL_ODR_20Hz;
      break;

    case LIS2MDL_ODR_50Hz:
      *val = LIS2MDL_ODR_50Hz;
      break;

    case LIS2MDL_ODR_100Hz:
      *val = LIS2MDL_ODR_100Hz;
      break;

    default:
      *val = LIS2MDL_ODR_10Hz;
      break;
  }

  return ret;
}

/**
  * @brief  Enables high-resolution/low-power mode.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of lp in reg CFG_REG_A
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_power_mode_set(stmdev_ctx_t *ctx, lis2mdl_lp_t val)
{
  lis2mdl_cfg_reg_a_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.lp = (uint8_t)val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Enables high-resolution/low-power mode.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of lp in reg CFG_REG_A.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_power_mode_get(stmdev_ctx_t *ctx, lis2mdl_lp_t *val)
{
  lis2mdl_cfg_reg_a_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);

  switch (reg.lp) {
    case LIS2MDL_HIGH_RESOLUTION:
      *val = LIS2MDL_HIGH_RESOLUTION;
      break;

    case LIS2MDL_LOW_POWER:
      *val = LIS2MDL_LOW_POWER;
      break;

    default:
      *val = LIS2MDL_HIGH_RESOLUTION;
      break;
  }

  return ret;
}

/**
  * @brief  Enables the magnetometer temperature compensation.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of comp_temp_en in reg CFG_REG_A
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_offset_temp_comp_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2mdl_cfg_reg_a_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.comp_temp_en = val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Enables the magnetometer temperature compensation.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of comp_temp_en in reg CFG_REG_A.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_offset_temp_comp_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2mdl_cfg_reg_a_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);
  *val = reg.comp_temp_en;
  return ret;
}

/**
  * @brief  Low-pass bandwidth selection.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of lpf in reg CFG_REG_B
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_low_pass_bandwidth_set(stmdev_ctx_t *ctx,
                                       lis2mdl_lpf_t val)
{
  lis2mdl_cfg_reg_b_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.lpf = (uint8_t)val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Low-pass bandwidth selection.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of lpf in reg CFG_REG_B.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_low_pass_bandwidth_get(stmdev_ctx_t *ctx,
                                       lis2mdl_lpf_t *val)
{
  lis2mdl_cfg_reg_b_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t *)&reg, 1);

  switch (reg.lpf) {
    case LIS2MDL_ODR_DIV_2:
      *val = LIS2MDL_ODR_DIV_2;
      break;

    case LIS2MDL_ODR_DIV_4:
      *val = LIS2MDL_ODR_DIV_4;
      break;

    default:
      *val = LIS2MDL_ODR_DIV_2;
      break;
  }

  return ret;
}

/**
  * @brief  Reset mode.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of set_rst in reg CFG_REG_B
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_set_rst_mode_set(stmdev_ctx_t *ctx,
                                 lis2mdl_set_rst_t val)
{
  lis2mdl_cfg_reg_b_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.set_rst = (uint8_t)val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Reset mode.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of set_rst in reg CFG_REG_B.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_set_rst_mode_get(stmdev_ctx_t *ctx,
                                 lis2mdl_set_rst_t *val)
{
  lis2mdl_cfg_reg_b_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t *)&reg, 1);

  switch (reg.set_rst) {
    case LIS2MDL_SET_SENS_ODR_DIV_63:
      *val = LIS2MDL_SET_SENS_ODR_DIV_63;
      break;

    case LIS2MDL_SENS_OFF_CANC_EVERY_ODR:
      *val = LIS2MDL_SENS_OFF_CANC_EVERY_ODR;
      break;

    case LIS2MDL_SET_SENS_ONLY_AT_POWER_ON:
      *val = LIS2MDL_SET_SENS_ONLY_AT_POWER_ON;
      break;

    default:
      *val = LIS2MDL_SET_SENS_ODR_DIV_63;
      break;
  }

  return ret;
}

/**
  * @brief  Enables offset cancellation in single measurement mode.
  *         The OFF_CANC bit must be set to 1 when enabling offset
  *         cancellation in single measurement mode this means a
  *         call function: set_rst_mode(SENS_OFF_CANC_EVERY_ODR)
  *         is need.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of off_canc_one_shot in reg CFG_REG_B
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_set_rst_sensor_single_set(stmdev_ctx_t *ctx,
                                          uint8_t val)
{
  lis2mdl_cfg_reg_b_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.off_canc_one_shot = val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Enables offset cancellation in single measurement mode.
  *         The OFF_CANC bit must be set to 1 when enabling offset
  *         cancellation in single measurement mode this means a
  *         call function: set_rst_mode(SENS_OFF_CANC_EVERY_ODR)
  *         is need.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of off_canc_one_shot in reg CFG_REG_B.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_set_rst_sensor_single_get(stmdev_ctx_t *ctx,
                                          uint8_t *val)
{
  lis2mdl_cfg_reg_b_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t *)&reg, 1);
  *val = reg.off_canc_one_shot;
  return ret;
}

/**
  * @brief  Blockdataupdate.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of bdu in reg CFG_REG_C
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.bdu = val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Blockdataupdate.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of bdu in reg CFG_REG_C.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);
  *val = reg.bdu;
  return ret;
}

/**
  * @brief  Magnetic set of data available.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of zyxda in reg STATUS_REG.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_mag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2mdl_status_reg_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_STATUS_REG, (uint8_t *)&reg, 1);
  *val = reg.zyxda;
  return ret;
}

/**
  * @brief  Magnetic set of data overrun.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of zyxor in reg STATUS_REG.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_mag_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2mdl_status_reg_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_STATUS_REG, (uint8_t *)&reg, 1);
  *val = reg.zyxor;
  return ret;
}

/**
  * @brief  Magnetic output value.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  that stores data read
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_magnetic_raw_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_OUTX_L_REG, buff, 6);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) +  (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) +  (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) +  (int16_t)buff[4];
  return ret;
}

/**
  * @brief  Temperature output value.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  that stores data read
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_temperature_raw_get(stmdev_ctx_t *ctx,  int16_t *val)
{
  uint8_t buff[2];
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_TEMP_OUT_L_REG, buff, 2);
  *val = (int16_t)buff[1];
  *val = (*val * 256) +  (int16_t)buff[0];
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LIS2MDL_common
  * @brief       This section group common useful functions
  * @{
  *
  */

/**
  * @brief  DeviceWhoamI.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  that stores data read
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_WHO_AM_I, buff, 1);
  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of soft_rst in reg CFG_REG_A
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_reset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2mdl_cfg_reg_a_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.soft_rst = val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of soft_rst in reg CFG_REG_A.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2mdl_cfg_reg_a_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);
  *val = reg.soft_rst;
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of reboot in reg CFG_REG_A
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_boot_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2mdl_cfg_reg_a_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.reboot = val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of reboot in reg CFG_REG_A.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_boot_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2mdl_cfg_reg_a_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t *)&reg, 1);
  *val = reg.reboot;
  return ret;
}

/**
  * @brief  Selftest.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of self_test in reg CFG_REG_C
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_self_test_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.self_test = val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Selftest.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of self_test in reg CFG_REG_C.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_self_test_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);
  *val = reg.self_test;
  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of ble in reg CFG_REG_C
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_data_format_set(stmdev_ctx_t *ctx, lis2mdl_ble_t val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.ble = (uint8_t)val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of ble in reg CFG_REG_C.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_data_format_get(stmdev_ctx_t *ctx, lis2mdl_ble_t *val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);

  switch (reg.ble) {
    case LIS2MDL_LSB_AT_LOW_ADD:
      *val = LIS2MDL_LSB_AT_LOW_ADD;
      break;

    case LIS2MDL_MSB_AT_LOW_ADD:
      *val = LIS2MDL_MSB_AT_LOW_ADD;
      break;

    default:
      *val = LIS2MDL_LSB_AT_LOW_ADD;
      break;
  }

  return ret;
}

/**
  * @brief  Info about device status.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   registers STATUS_REG.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_status_get(stmdev_ctx_t *ctx,
                           lis2mdl_status_reg_t *val)
{
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_STATUS_REG, (uint8_t *) val, 1);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LIS2MDL_interrupts
  * @brief       This section group all the functions that manage interrupts
  * @{
  *
  */

/**
  * @brief  The interrupt block recognition checks data after/before the
  *         hard-iron correction to discover the interrupt.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of int_on_dataoff in reg CFG_REG_B
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_offset_int_conf_set(stmdev_ctx_t *ctx,
                                    lis2mdl_int_on_dataoff_t val)
{
  lis2mdl_cfg_reg_b_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.int_on_dataoff = (uint8_t)val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  The interrupt block recognition checks data after/before the
  *         hard-iron correction to discover the interrupt.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of int_on_dataoff in reg CFG_REG_B.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_offset_int_conf_get(stmdev_ctx_t *ctx,
                                    lis2mdl_int_on_dataoff_t *val)
{
  lis2mdl_cfg_reg_b_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t *)&reg, 1);

  switch (reg.int_on_dataoff) {
    case LIS2MDL_CHECK_BEFORE:
      *val = LIS2MDL_CHECK_BEFORE;
      break;

    case LIS2MDL_CHECK_AFTER:
      *val = LIS2MDL_CHECK_AFTER;
      break;

    default:
      *val = LIS2MDL_CHECK_BEFORE;
      break;
  }

  return ret;
}

/**
  * @brief  Data-ready signal on INT_DRDY pin.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of drdy_on_pin in reg CFG_REG_C
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_drdy_on_pin_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.drdy_on_pin = val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Data-ready signal on INT_DRDY pin.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of drdy_on_pin in reg CFG_REG_C.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_drdy_on_pin_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);
  *val = reg.drdy_on_pin;
  return ret;
}

/**
  * @brief  Interrupt signal on INT_DRDY pin.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of int_on_pin in reg CFG_REG_C
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_on_pin_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.int_on_pin = val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Interrupt signal on INT_DRDY pin.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of int_on_pin in reg CFG_REG_C.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_on_pin_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);
  *val = reg.int_on_pin;
  return ret;
}

/**
  * @brief  Interrupt generator configuration register.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   registers INT_CRTL_REG.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_gen_conf_set(stmdev_ctx_t *ctx,
                                 lis2mdl_int_crtl_reg_t *val)
{
  int32_t ret;
  ret = lis2mdl_write_reg(ctx, LIS2MDL_INT_CRTL_REG, (uint8_t *) val,
                          1);
  return ret;
}

/**
  * @brief  Interrupt generator configuration register.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   registers INT_CRTL_REG.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_gen_conf_get(stmdev_ctx_t *ctx,
                                 lis2mdl_int_crtl_reg_t *val)
{
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_INT_CRTL_REG, (uint8_t *) val, 1);
  return ret;
}

/**
  * @brief  Interrupt generator source register.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   registers INT_SOURCE_REG.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_gen_source_get(stmdev_ctx_t *ctx,
                                   lis2mdl_int_source_reg_t *val)
{
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_INT_SOURCE_REG, (uint8_t *) val,
                         1);
  return ret;
}

/**
  * @brief  User-defined threshold value for xl interrupt event on generator.
  *         Data format is the same of output data raw: two’s complement with
  *         1LSb = 1.5mG.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  that contains data to write
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_gen_treshold_set(stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t buff[2];
  int32_t ret;
  buff[1] = (uint8_t) (val / 256U);
  buff[0] = (uint8_t) (val - (buff[1] * 256U));
  ret = lis2mdl_write_reg(ctx, LIS2MDL_INT_THS_L_REG, buff, 2);
  return ret;
}

/**
  * @brief  User-defined threshold value for xl interrupt event on generator.
  *         Data format is the same of output data raw: two’s complement with
  *         1LSb = 1.5mG.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  that stores data read
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_gen_treshold_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buff[2];
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_INT_THS_L_REG, buff, 2);
  *val = buff[1];
  *val = (*val * 256U) +  buff[0];
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LIS2MDL_serial_interface
  * @brief       This section group all the functions concerning serial
  *              interface management
  * @{
  *
  */

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of 4wspi in reg CFG_REG_C
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_spi_mode_set(stmdev_ctx_t *ctx, lis2mdl_sim_t val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg._4wspi = (uint8_t)val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of 4wspi in reg CFG_REG_C
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_spi_mode_get(stmdev_ctx_t *ctx, lis2mdl_sim_t *val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);

  switch (reg._4wspi) {
    case LIS2MDL_SPI_4_WIRE:
      *val = LIS2MDL_SPI_4_WIRE;
      break;

    case LIS2MDL_SPI_3_WIRE:
      *val = LIS2MDL_SPI_3_WIRE;
      break;

    default:
      *val = LIS2MDL_SPI_3_WIRE;
      break;
  }

  return ret;
}

/**
  * @brief  Enable/Disable I2C interface.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of i2c_dis in reg CFG_REG_C
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_i2c_interface_set(stmdev_ctx_t *ctx,
                                  lis2mdl_i2c_dis_t val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);

  if (ret == 0) {
    reg.i2c_dis = (uint8_t)val;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Enable/Disable I2C interface.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of i2c_dis in reg CFG_REG_C.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_i2c_interface_get(stmdev_ctx_t *ctx,
                                  lis2mdl_i2c_dis_t *val)
{
  lis2mdl_cfg_reg_c_t reg;
  int32_t ret;
  ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t *)&reg, 1);

  switch (reg.i2c_dis) {
    case LIS2MDL_I2C_ENABLE:
      *val = LIS2MDL_I2C_ENABLE;
      break;

    case LIS2MDL_I2C_DISABLE:
      *val = LIS2MDL_I2C_DISABLE;
      break;

    default:
      *val = LIS2MDL_I2C_ENABLE;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
