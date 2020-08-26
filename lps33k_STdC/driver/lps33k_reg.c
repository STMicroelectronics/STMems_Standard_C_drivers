/*
 ******************************************************************************
 * @file    lps33k_reg.c
 * @author  Sensors Software Solution Team
 * @brief   LPS33K driver file
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

#include "lps33k_reg.h"

/**
  * @defgroup    LPS33K
  * @brief       This file provides a set of functions needed to drive the
  *              ultra-compact piezoresistive absolute pressure sensor.
  * @{
  *
  */

/**
  * @defgroup    LPS33K_Interfaces_functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
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
int32_t lps33k_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data,
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
int32_t lps33k_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
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
  * @defgroup    LPS33K_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t lps33k_from_lsb_to_hpa(int32_t lsb)
{
  return ( (float_t)lsb / 4096.0f );
}

float_t lps33k_from_lsb_to_degc(int16_t lsb)
{
  return ( (float_t)lsb / 100.0f );
}

/**
  * @}
  *
  */

/**
  * @defgroup    LPS33K_data_generation_c
  * @brief       This section group all the functions concerning data
  *              generation
  * @{
  *
  */

/**
  * @brief  Block data update.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of bdu in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33k_ctrl_reg1_t ctrl_reg1;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG1, (uint8_t *)&ctrl_reg1,
                        1);

  if (ret == 0) {
    ctrl_reg1.bdu = val;
    ret = lps33k_write_reg(ctx, LPS33K_CTRL_REG1, (uint8_t *)&ctrl_reg1,
                           1);
  }

  return ret;
}

/**
  * @brief  Block data update.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of bdu in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33k_ctrl_reg1_t ctrl_reg1;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG1, (uint8_t *)&ctrl_reg1,
                        1);
  *val = ctrl_reg1.bdu;
  return ret;
}

/**
  * @brief  Low-pass bandwidth selection.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of lpfp in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_low_pass_filter_mode_set(stmdev_ctx_t *ctx,
                                        lps33k_lpfp_t val)
{
  lps33k_ctrl_reg1_t ctrl_reg1;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG1, (uint8_t *)&ctrl_reg1,
                        1);

  if (ret == 0) {
    ctrl_reg1.lpfp = (uint8_t)val;
    ret = lps33k_write_reg(ctx, LPS33K_CTRL_REG1, (uint8_t *)&ctrl_reg1,
                           1);
  }

  return ret;
}

/**
  * @brief   Low-pass bandwidth selection.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of lpfp in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_low_pass_filter_mode_get(stmdev_ctx_t *ctx,
                                        lps33k_lpfp_t *val)
{
  lps33k_ctrl_reg1_t ctrl_reg1;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG1, (uint8_t *)&ctrl_reg1,
                        1);

  switch (ctrl_reg1.lpfp) {
    case LPS33K_LPF_ODR_DIV_2:
      *val = LPS33K_LPF_ODR_DIV_2;
      break;

    case LPS33K_LPF_ODR_DIV_9:
      *val = LPS33K_LPF_ODR_DIV_9;
      break;

    case LPS33K_LPF_ODR_DIV_20:
      *val = LPS33K_LPF_ODR_DIV_20;
      break;

    default:
      *val = LPS33K_LPF_ODR_DIV_2;
      break;
  }

  return ret;
}

/**
  * @brief  Output data rate selection.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of odr in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_data_rate_set(stmdev_ctx_t *ctx, lps33k_odr_t val)
{
  lps33k_ctrl_reg1_t ctrl_reg1;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG1, (uint8_t *)&ctrl_reg1,
                        1);

  if (ret == 0) {
    ctrl_reg1.odr = (uint8_t)val;
    ret = lps33k_write_reg(ctx, LPS33K_CTRL_REG1, (uint8_t *)&ctrl_reg1,
                           1);
  }

  return ret;
}

/**
  * @brief  Output data rate selection.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of odr in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_data_rate_get(stmdev_ctx_t *ctx, lps33k_odr_t *val)
{
  lps33k_ctrl_reg1_t ctrl_reg1;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG1, (uint8_t *)&ctrl_reg1,
                        1);

  switch (ctrl_reg1.odr) {
    case LPS33K_POWER_DOWN:
      *val = LPS33K_POWER_DOWN;
      break;

    case LPS33K_ODR_1_Hz:
      *val = LPS33K_ODR_1_Hz;
      break;

    case LPS33K_ODR_10_Hz:
      *val = LPS33K_ODR_10_Hz;
      break;

    case LPS33K_ODR_25_Hz:
      *val = LPS33K_ODR_25_Hz;
      break;

    case LPS33K_ODR_50_Hz:
      *val = LPS33K_ODR_50_Hz;
      break;

    case LPS33K_ODR_75_Hz:
      *val = LPS33K_ODR_75_Hz;
      break;

    default:
      *val = LPS33K_ODR_1_Hz;
      break;
  }

  return ret;
}

/**
  * @brief  One-shot mode. Device perform a single measure.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of one_shot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_one_shoot_trigger_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33k_ctrl_reg2_t ctrl_reg2;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG2, (uint8_t *)&ctrl_reg2,
                        1);

  if (ret == 0) {
    ctrl_reg2.one_shot = val;
    ret = lps33k_write_reg(ctx, LPS33K_CTRL_REG2, (uint8_t *)&ctrl_reg2,
                           1);
  }

  return ret;
}

/**
  * @brief  One-shot mode. Device perform a single measure.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of one_shot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_one_shoot_trigger_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33k_ctrl_reg2_t ctrl_reg2;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG2, (uint8_t *)&ctrl_reg2,
                        1);
  *val = ctrl_reg2.one_shot;
  return ret;
}

/**
  * @brief  The pressure offset value is 16-bit data that can be used to
  *         implement one-point calibration (OPC) after soldering.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_pressure_offset_set(stmdev_ctx_t *ctx, int16_t val)
{
  uint8_t buff[2];
  int32_t ret;
  buff[1] = (uint8_t) ((uint16_t)val / 256U);
  buff[0] = (uint8_t) ((uint16_t)val - (buff[1] * 256U));
  ret =  lps33k_write_reg(ctx, LPS33K_RPDS_L, buff, 2);
  return ret;
}

/**
  * @brief  The pressure offset value is 16-bit data that can be used to
  *         implement one-point calibration (OPC) after soldering.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_pressure_offset_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[2];
  int32_t ret;
  ret =  lps33k_read_reg(ctx, LPS33K_RPDS_L, buff, 2);
  *val = (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];
  return ret;
}

/**
  * @brief  Pressure data available.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of p_da in reg STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_press_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33k_status_t status;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_STATUS, (uint8_t *)&status, 1);
  *val = status.p_da;
  return ret;
}

/**
  * @brief  Temperature data available.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of t_da in reg STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_temp_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33k_status_t status;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_STATUS, (uint8_t *)&status, 1);
  *val = status.t_da;
  return ret;
}

/**
  * @brief  Pressure data overrun.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of p_or in reg STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_press_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33k_status_t status;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_STATUS, (uint8_t *)&status, 1);
  *val = status.p_or;
  return ret;
}

/**
  * @brief  Temperature data overrun.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of t_or in reg STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_temp_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33k_status_t status;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_STATUS, (uint8_t *)&status, 1);
  *val = status.t_or;
  return ret;
}

/**
  * @brief  Pressure output value[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_pressure_raw_get(stmdev_ctx_t *ctx, uint32_t *buff)
{
  uint8_t reg[3];
  int32_t ret;
  ret =  lps33k_read_reg(ctx, LPS33K_PRESS_OUT_XL, reg, 3);
  *buff = reg[2];
  *buff = (*buff * 256) + reg[1];
  *buff = (*buff * 256) + reg[0];
  *buff *= 256;
  return ret;
}

/**
  * @brief  temperature_raw:   Temperature output value[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_temperature_raw_get(stmdev_ctx_t *ctx, int16_t *buff)
{
  uint8_t reg[2];
  int32_t ret;
  ret =  lps33k_read_reg(ctx, LPS33K_TEMP_OUT_L, (uint8_t *) reg, 2);
  *buff = reg[1];
  *buff = (*buff * 256) + reg[0];
  return ret;
}

/**
  * @brief  Low-pass filter reset register. If the LPFP is active, in
  *         order to avoid the transitory phase, the filter can be
  *         reset by reading this register before generating pressure
  *         measurements.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_low_pass_rst_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret =  lps33k_read_reg(ctx, LPS33K_LPFP_RES, (uint8_t *) buff, 1);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LPS33K_common
  * @brief       This section group common useful functions
  * @{
  *
  */

/**
  * @brief  Device Who am I[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret =  lps33k_read_reg(ctx, LPS33K_WHO_AM_I, (uint8_t *) buff, 1);
  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of swreset in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_reset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33k_ctrl_reg2_t ctrl_reg2;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG2, (uint8_t *)&ctrl_reg2,
                        1);

  if (ret == 0) {
    ctrl_reg2.swreset = val;
    ret = lps33k_write_reg(ctx, LPS33K_CTRL_REG2, (uint8_t *)&ctrl_reg2,
                           1);
  }

  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of swreset in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33k_ctrl_reg2_t ctrl_reg2;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG2, (uint8_t *)&ctrl_reg2,
                        1);
  *val = ctrl_reg2.swreset;
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of boot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_boot_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33k_ctrl_reg2_t ctrl_reg2;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG2, (uint8_t *)&ctrl_reg2,
                        1);

  if (ret == 0) {
    ctrl_reg2.boot = val;
    ret = lps33k_write_reg(ctx, LPS33K_CTRL_REG2, (uint8_t *)&ctrl_reg2,
                           1);
  }

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of boot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_boot_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33k_ctrl_reg2_t ctrl_reg2;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG2, (uint8_t *)&ctrl_reg2,
                        1);
  *val = ctrl_reg2.boot;
  return ret;
}

/**
  * @brief  Low current mode.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of lc_en in reg RES_CONF
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_low_power_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33k_res_conf_t res_conf;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_RES_CONF, (uint8_t *)&res_conf, 1);

  if (ret == 0) {
    res_conf.lc_en = val;
    ret = lps33k_write_reg(ctx, LPS33K_RES_CONF, (uint8_t *)&res_conf, 1);
  }

  return ret;
}

/**
  * @brief  Low current mode.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of lc_en in reg RES_CONF
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_low_power_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33k_res_conf_t res_conf;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_RES_CONF, (uint8_t *)&res_conf, 1);
  *val = res_conf.lc_en;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LPS33K_serial_interface
  * @brief       This section group all the functions concerning serial
  *              interface management
  * @{
  *
  */

/**
  * @brief  Register address automatically incremented during a
  *         multiple byte access with a serial interface (I2C or SPI).[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of if_add_inc in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_auto_add_inc_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33k_ctrl_reg2_t ctrl_reg2;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG2, (uint8_t *)&ctrl_reg2,
                        1);

  if (ret == 0) {
    ctrl_reg2.if_add_inc = val;
    ret = lps33k_write_reg(ctx, LPS33K_CTRL_REG2, (uint8_t *)&ctrl_reg2,
                           1);
  }

  return ret;
}

/**
  * @brief  Register address automatically incremented during a
  *         multiple byte access with a serial interface (I2C or SPI).[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of if_add_inc in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33k_auto_add_inc_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33k_ctrl_reg2_t ctrl_reg2;
  int32_t ret;
  ret = lps33k_read_reg(ctx, LPS33K_CTRL_REG2, (uint8_t *)&ctrl_reg2,
                        1);
  *val = ctrl_reg2.if_add_inc;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @}
  *
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
