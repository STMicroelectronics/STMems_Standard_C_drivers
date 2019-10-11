/*
 ******************************************************************************
 * @file    lps25hb_reg.c
 * @author  Sensors Software Solution Team
 * @brief   LPS25HB driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "lps25hb_reg.h"

/**
  * @defgroup    LPS25HB
  * @brief       This file provides a set of functions needed to drive the
  *              ultra-compact piezoresistive absolute pressure sensor.
  * @{
  *
  */

/**
  * @defgroup    LPS25HB_Interfaces_functions
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
int32_t lps25hb_read_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
int32_t lps25hb_write_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
  * @defgroup    LPS25HB_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t lps25hb_from_lsb_to_hpa(uint32_t lsb)
{
  return ( (float_t)lsb / 4096.0f );
}

float_t lps25hb_from_lsb_to_degc(int16_t lsb)
{
  return ( (float_t)lsb / 480.0f ) + 42.5f ;
}

/**
  * @}
  *
  */

/**
  * @defgroup   LPS25HB_data_generation_c
  * @brief      This section group all the functions concerning data generation
  * @{
  *
  */

/**
  * @brief  The Reference pressure value is a 24-bit data expressed as 2’s
  *         complement. The value is used when AUTOZERO or AUTORIFP function
  *         is enabled.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_pressure_ref_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lps25hb_read_reg(ctx, LPS25HB_REF_P_XL,  buff, 3);
  return ret;
}

/**
  * @brief  The Reference pressure value is a 24-bit data expressed as 2’s
  *         complement. The value is used when AUTOZERO or AUTORIFP function
  *         is enabled.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_pressure_ref_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lps25hb_read_reg(ctx, LPS25HB_REF_P_XL,  buff, 3);
  return ret;
}

/**
  * @brief  Pressure internal average configuration.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of avgp in reg RES_CONF
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_pressure_avg_set(stmdev_ctx_t *ctx, lps25hb_avgp_t val)
{
  lps25hb_res_conf_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_RES_CONF, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.avgp = (uint8_t)val;
    ret = lps25hb_write_reg(ctx, LPS25HB_RES_CONF, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Pressure internal average configuration.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of avgp in reg RES_CONF.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_pressure_avg_get(stmdev_ctx_t *ctx, lps25hb_avgp_t *val)
{
  lps25hb_res_conf_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_RES_CONF, (uint8_t*)&reg, 1);
  switch (reg.avgp){
    case LPS25HB_P_AVG_8:
      *val = LPS25HB_P_AVG_8;
      break;
    case LPS25HB_P_AVG_16:
      *val = LPS25HB_P_AVG_16;
      break;
    case LPS25HB_P_AVG_32:
      *val = LPS25HB_P_AVG_32;
      break;
    case LPS25HB_P_AVG_64:
      *val = LPS25HB_P_AVG_64;
      break;
    default:
      *val = LPS25HB_P_AVG_8;
      break;
  }
  return ret;
}

/**
  * @brief  Temperature internal average configuration.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of avgt in reg RES_CONF
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_temperature_avg_set(stmdev_ctx_t *ctx, lps25hb_avgt_t val)
{
  lps25hb_res_conf_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_RES_CONF, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.avgt = (uint8_t)val;
    ret = lps25hb_write_reg(ctx, LPS25HB_RES_CONF, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Temperature internal average configuration.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of avgt in reg RES_CONF.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_temperature_avg_get(stmdev_ctx_t *ctx, lps25hb_avgt_t *val)
{
  lps25hb_res_conf_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_RES_CONF, (uint8_t*)&reg, 1);
  switch (reg.avgt){
    case LPS25HB_T_AVG_8:
      *val = LPS25HB_T_AVG_8;
      break;
    case LPS25HB_T_AVG_16:
      *val = LPS25HB_T_AVG_16;
      break;
    case LPS25HB_T_AVG_32:
      *val = LPS25HB_T_AVG_32;
      break;
    case LPS25HB_T_AVG_64:
      *val = LPS25HB_T_AVG_64;
      break;
    default:
      *val = LPS25HB_T_AVG_8;
      break;
  }
  return ret;
}

/**
  * @brief  Reset Autozero function. [set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of reset_az in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_autozero_rst_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg1_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.reset_az = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Reset Autozero function.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of reset_az in reg CTRL_REG1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_autozero_rst_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg1_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  *val = reg.reset_az;

  return ret;
}

/**
  * @brief  Blockdataupdate.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of bdu in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg1_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.bdu = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief Blockdataupdate. [get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of bdu in reg CTRL_REG1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg1_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  *val = reg.bdu;

  return ret;
}

/**
  * @brief  Output data rate selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of odr in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_data_rate_set(stmdev_ctx_t *ctx, lps25hb_odr_t val)
{
  lps25hb_ctrl_reg1_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.odr = (uint8_t)val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Output data rate selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of odr in reg CTRL_REG1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_data_rate_get(stmdev_ctx_t *ctx, lps25hb_odr_t *val)
{
  lps25hb_ctrl_reg1_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  switch (reg.odr){
    case LPS25HB_POWER_DOWN:
      *val = LPS25HB_POWER_DOWN;
      break;
    case LPS25HB_ODR_1Hz:
      *val = LPS25HB_ODR_1Hz;
      break;
    case LPS25HB_ODR_7Hz:
      *val = LPS25HB_ODR_7Hz;
      break;
    case LPS25HB_ODR_12Hz5:
      *val = LPS25HB_ODR_12Hz5;
      break;
    case LPS25HB_ODR_25Hz:
      *val = LPS25HB_ODR_25Hz;
      break;
    case LPS25HB_ONE_SHOT:
      *val = LPS25HB_ONE_SHOT;
      break;
    default:
      *val = LPS25HB_POWER_DOWN;
      break;
  }
  return ret;
}

/**
  * @brief  One-shot mode. Device perform a single measure.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of one_shot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_one_shoot_trigger_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.one_shot = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  One-shot mode. Device perform a single measure.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of one_shot in reg CTRL_REG2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_one_shoot_trigger_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  *val = reg.one_shot;

  return ret;
}

/**
  * @brief  Enable Autozero function.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of autozero in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_autozero_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.autozero = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enable Autozero function.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of autozero in reg CTRL_REG2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_autozero_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  *val = reg.autozero;

  return ret;
}

/**
  * @brief  Enable to decimate the output pressure to 1Hz
  *         with FIFO Mean mode.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fifo_mean_dec in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_mean_decimator_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.fifo_mean_dec = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enable to decimate the output pressure to 1Hz
  *         with FIFO Mean mode.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fifo_mean_dec in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_mean_decimator_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  *val = reg.fifo_mean_dec;

  return ret;
}

/**
  * @brief  Pressure data available.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of p_da in reg STATUS_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_press_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_status_reg_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_STATUS_REG, (uint8_t*)&reg, 1);
  *val = reg.p_da;

  return ret;
}

/**
  * @brief  Temperature data available.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of t_da in reg STATUS_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_temp_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_status_reg_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_STATUS_REG, (uint8_t*)&reg, 1);
  *val = reg.t_da;

  return ret;
}

/**
  * @brief  Temperature data overrun.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of t_or in reg STATUS_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_temp_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_status_reg_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_STATUS_REG, (uint8_t*)&reg, 1);
  *val = reg.t_or;

  return ret;
}

/**
  * @brief  Pressure data overrun.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of p_or in reg STATUS_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_press_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_status_reg_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_STATUS_REG, (uint8_t*)&reg, 1);
  *val = reg.p_or;

  return ret;
}

/**
  * @brief  Pressure output value.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_pressure_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lps25hb_read_reg(ctx, LPS25HB_PRESS_OUT_XL,  buff, 3);
  return ret;
}

/**
  * @brief  Temperature output value.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lps25hb_read_reg(ctx, LPS25HB_TEMP_OUT_L,  buff, 2);
  return ret;
}

/**
  * @brief  The pressure offset value is 16-bit data that can be used to
  *         implement one-point calibration (OPC) after soldering.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_pressure_offset_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lps25hb_read_reg(ctx, LPS25HB_RPDS_L,  buff, 2);
  return ret;
}

/**
  * @brief  The pressure offset value is 16-bit data that can be used to
  *         implement one-point calibration (OPC) after soldering.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_pressure_offset_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lps25hb_read_reg(ctx, LPS25HB_RPDS_L,  buff, 2);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   LPS25HB_common
  * @brief      This section group common usefull functions
  * @{
  *
  */

/**
  * @brief  DeviceWhoamI.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lps25hb_read_reg(ctx, LPS25HB_WHO_AM_I,  buff, 1);
  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of swreset in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_reset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.swreset = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of swreset in reg CTRL_REG2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  *val = reg.swreset;

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of boot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_boot_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.boot = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of boot in reg CTRL_REG2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_boot_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  *val = reg.boot;

  return ret;
}

/**
  * @brief  Status: [get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get registers STATUS_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_status_get(stmdev_ctx_t *ctx, lps25hb_status_reg_t *val)
{
  int32_t ret;
  ret = lps25hb_read_reg(ctx, LPS25HB_STATUS_REG, (uint8_t*) val, 1);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   LPS25HB_interrupts
  * @brief      This section group all the functions that manage interrupts
  * @{
  *
  */

/**
  * @brief  Enable interrupt generation.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of diff_en in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_int_generation_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg1_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.diff_en = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enable interrupt generation.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of diff_en in reg CTRL_REG1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_int_generation_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg1_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  *val = reg.diff_en;

  return ret;
}

/**
  * @brief  Data signal on INT_DRDY pin control bits.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of int_s in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_int_pin_mode_set(stmdev_ctx_t *ctx, lps25hb_int_s_t val)
{
  lps25hb_ctrl_reg3_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG3, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.int_s = (uint8_t)val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG3, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Data signal on INT_DRDY pin control bits.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of int_s in reg CTRL_REG3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_int_pin_mode_get(stmdev_ctx_t *ctx, lps25hb_int_s_t *val)
{
  lps25hb_ctrl_reg3_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG3, (uint8_t*)&reg, 1);
  switch (reg.int_s){
    case LPS25HB_DRDY_OR_FIFO_FLAGS:
      *val = LPS25HB_DRDY_OR_FIFO_FLAGS;
      break;
    case LPS25HB_HIGH_PRES_INT:
      *val = LPS25HB_HIGH_PRES_INT;
      break;
    case LPS25HB_LOW_PRES_INT:
      *val = LPS25HB_LOW_PRES_INT;
      break;
    case LPS25HB_EVERY_PRES_INT:
      *val = LPS25HB_EVERY_PRES_INT;
      break;
    default:
      *val = LPS25HB_DRDY_OR_FIFO_FLAGS;
      break;
  }
  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of pp_od in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_pin_mode_set(stmdev_ctx_t *ctx, lps25hb_pp_od_t val)
{
  lps25hb_ctrl_reg3_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG3, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.pp_od = (uint8_t)val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG3, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of pp_od in reg CTRL_REG3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_pin_mode_get(stmdev_ctx_t *ctx, lps25hb_pp_od_t *val)
{
  lps25hb_ctrl_reg3_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG3, (uint8_t*)&reg, 1);
  switch (reg.pp_od){
    case LPS25HB_PUSH_PULL:
      *val = LPS25HB_PUSH_PULL;
      break;
    case LPS25HB_OPEN_DRAIN:
      *val = LPS25HB_OPEN_DRAIN;
      break;
    default:
      *val = LPS25HB_PUSH_PULL;
      break;
  }
  return ret;
}

/**
  * @brief  Interrupt active-high/low.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of int_h_l in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_int_polarity_set(stmdev_ctx_t *ctx, lps25hb_int_h_l_t val)
{
  lps25hb_ctrl_reg3_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG3, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.int_h_l = (uint8_t)val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG3, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Interrupt active-high/low.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of int_h_l in reg CTRL_REG3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_int_polarity_get(stmdev_ctx_t *ctx, lps25hb_int_h_l_t *val)
{
  lps25hb_ctrl_reg3_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG3, (uint8_t*)&reg, 1);
  switch (reg.int_h_l){
    case LPS25HB_ACTIVE_HIGH:
      *val = LPS25HB_ACTIVE_HIGH;
      break;
    case LPS25HB_ACTIVE_LOW:
      *val = LPS25HB_ACTIVE_LOW;
      break;
    default:
      *val = LPS25HB_ACTIVE_HIGH;
      break;
  }
  return ret;
}

/**
  * @brief  Data-ready signal on INT_DRDY pin.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of drdy in reg CTRL_REG4
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_drdy_on_int_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg4_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG4, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.drdy = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG4, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Data-ready signal on INT_DRDY pin.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of drdy in reg CTRL_REG4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_drdy_on_int_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg4_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG4, (uint8_t*)&reg, 1);
  *val = reg.drdy;

  return ret;
}

/**
  * @brief  FIFO overrun interrupt on INT_DRDY pin.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of f_ovr in reg CTRL_REG4
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_ovr_on_int_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg4_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG4, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.f_ovr = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG4, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  FIFO overrun interrupt on INT_DRDY pin.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of f_ovr in reg CTRL_REG4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_ovr_on_int_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg4_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG4, (uint8_t*)&reg, 1);
  *val = reg.f_ovr;

  return ret;
}

/**
  * @brief  FIFO watermark status on INT_DRDY pin.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of f_fth in reg CTRL_REG4
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_threshold_on_int_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg4_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG4, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.f_fth = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG4, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  FIFO watermark status on INT_DRDY pin.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of f_fth in reg CTRL_REG4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_threshold_on_int_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg4_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG4, (uint8_t*)&reg, 1);
  *val = reg.f_fth;

  return ret;
}

/**
  * @brief  FIFO empty flag on INT_DRDY pin.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of f_empty in reg CTRL_REG4
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_empty_on_int_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg4_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG4, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.f_empty = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG4, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  FIFO empty flag on INT_DRDY pin.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of f_empty in reg CTRL_REG4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_empty_on_int_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg4_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG4, (uint8_t*)&reg, 1);
  *val = reg.f_empty;

  return ret;
}

/**
  * @brief  Enable interrupt generation on pressure low/high event.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of pe in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_sign_of_int_threshold_set(stmdev_ctx_t *ctx,
                                          lps25hb_pe_t val)
{
  lps25hb_interrupt_cfg_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_INTERRUPT_CFG, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.pe = (uint8_t)val;
    ret = lps25hb_write_reg(ctx, LPS25HB_INTERRUPT_CFG, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enable interrupt generation on pressure low/high event.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of pe in reg INTERRUPT_CFG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_sign_of_int_threshold_get(stmdev_ctx_t *ctx,
                                          lps25hb_pe_t *val)
{
  lps25hb_interrupt_cfg_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_INTERRUPT_CFG, (uint8_t*)&reg, 1);
  switch (reg.pe){
    case LPS25HB_NO_THRESHOLD:
      *val = LPS25HB_NO_THRESHOLD;
      break;
    case LPS25HB_POSITIVE:
      *val = LPS25HB_POSITIVE;
      break;
    case LPS25HB_NEGATIVE:
      *val = LPS25HB_NEGATIVE;
      break;
    case LPS25HB_BOTH:
      *val = LPS25HB_BOTH;
      break;
    default:
      *val = LPS25HB_NO_THRESHOLD;
      break;
  }
  return ret;
}

/**
  * @brief  Interrupt request to the INT_SOURCE (25h) register mode.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of lir in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_int_notification_mode_set(stmdev_ctx_t *ctx,
                                          lps25hb_lir_t val)
{
  lps25hb_interrupt_cfg_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_INTERRUPT_CFG, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.lir = (uint8_t)val;
    ret = lps25hb_write_reg(ctx, LPS25HB_INTERRUPT_CFG, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Interrupt request to the  INT_SOURCE (25h) register mode.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of lir in reg INTERRUPT_CFG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_int_notification_mode_get(stmdev_ctx_t *ctx,
                                          lps25hb_lir_t *val)
{
  lps25hb_interrupt_cfg_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_INTERRUPT_CFG, (uint8_t*)&reg, 1);
  switch (reg.lir){
    case LPS25HB_INT_PULSED:
      *val = LPS25HB_INT_PULSED;
      break;
    case LPS25HB_INT_LATCHED:
      *val = LPS25HB_INT_LATCHED;
      break;
    default:
      *val = LPS25HB_INT_PULSED;
      break;
  }
  return ret;
}

/**
  * @brief  Interrupt source register[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get registers INT_SOURCE.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_int_source_get(stmdev_ctx_t *ctx, lps25hb_int_source_t *val)
{
  int32_t ret;
  ret = lps25hb_read_reg(ctx, LPS25HB_INT_SOURCE, (uint8_t*) val, 1);
  return ret;
}

/**
  * @brief  Differential pressure high interrupt flag.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of ph in reg INT_SOURCE.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_int_on_press_high_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_int_source_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_INT_SOURCE, (uint8_t*)&reg, 1);
  *val = reg.ph;

  return ret;
}

/**
  * @brief  Differential pressure low interrupt flag.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of pl in reg INT_SOURCE.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_int_on_press_low_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_int_source_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_INT_SOURCE, (uint8_t*)&reg, 1);
  *val = reg.pl;

  return ret;
}

/**
  * @brief  Interrupt active flag.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of ia in reg INT_SOURCE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_interrupt_event_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_int_source_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_INT_SOURCE, (uint8_t*)&reg, 1);
  *val = reg.ia;

  return ret;
}

/**
  * @brief  User-defined threshold value for pressure interrupt event[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_int_threshold_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lps25hb_read_reg(ctx, LPS25HB_THS_P_L,  buff, 2);
  return ret;
}

/**
  * @brief  User-defined threshold value for pressure interrupt event[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_int_threshold_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lps25hb_read_reg(ctx, LPS25HB_THS_P_L,  buff, 2);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   LPS25HB_fifo
  * @brief   This section group all the functions concerning the fifo usage
  * @{
  *
  */

/**
  * @brief  Stop on FIFO watermark. Enable FIFO watermark level use.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of stop_on_fth in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_stop_on_fifo_threshold_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.stop_on_fth = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Stop on FIFO watermark. Enable FIFO watermark level use.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of stop_on_fth in reg CTRL_REG2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_stop_on_fifo_threshold_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  *val = reg.stop_on_fth;

  return ret;
}

/**
  * @brief  FIFOenable.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fifo_en in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.fifo_en = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  FIFOenable.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fifo_en in reg CTRL_REG2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  *val = reg.fifo_en;

  return ret;
}

/**
  * @brief  FIFO watermark level selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of wtm_point in reg FIFO_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps25hb_fifo_ctrl_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_FIFO_CTRL, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.wtm_point = val;
    ret = lps25hb_write_reg(ctx, LPS25HB_FIFO_CTRL, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  FIFO watermark level selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of wtm_point in reg FIFO_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_fifo_ctrl_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_FIFO_CTRL, (uint8_t*)&reg, 1);
  *val = reg.wtm_point;

  return ret;
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of f_mode in reg FIFO_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_mode_set(stmdev_ctx_t *ctx, lps25hb_f_mode_t val)
{
  lps25hb_fifo_ctrl_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_FIFO_CTRL, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.f_mode = (uint8_t)val;
    ret = lps25hb_write_reg(ctx, LPS25HB_FIFO_CTRL, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of f_mode in reg FIFO_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_mode_get(stmdev_ctx_t *ctx, lps25hb_f_mode_t *val)
{
  lps25hb_fifo_ctrl_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_FIFO_CTRL, (uint8_t*)&reg, 1);
  switch (reg.f_mode){
    case LPS25HB_BYPASS_MODE:
      *val = LPS25HB_BYPASS_MODE;
      break;
    case LPS25HB_FIFO_MODE:
      *val = LPS25HB_FIFO_MODE;
      break;
    case LPS25HB_STREAM_MODE:
      *val = LPS25HB_STREAM_MODE;
      break;
    case LPS25HB_Stream_to_FIFO_mode:
      *val = LPS25HB_Stream_to_FIFO_mode;
      break;
    case LPS25HB_BYPASS_TO_STREAM_MODE:
      *val = LPS25HB_BYPASS_TO_STREAM_MODE;
      break;
    case LPS25HB_MEAN_MODE:
      *val = LPS25HB_MEAN_MODE;
      break;
    case LPS25HB_BYPASS_TO_FIFO_MODE:
      *val = LPS25HB_BYPASS_TO_FIFO_MODE;
      break;
    default:
      *val = LPS25HB_BYPASS_MODE;
      break;
  }
  return ret;
}

/**
  * @brief  FIFO status register. [get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  lps25hb_: registers FIFO_STATUS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_status_get(stmdev_ctx_t *ctx,
                                lps25hb_fifo_status_t *val)
{
  int32_t ret;
  ret = lps25hb_read_reg(ctx, LPS25HB_FIFO_STATUS, (uint8_t*) val, 1);
  return ret;
}

/**
  * @brief  FIFO stored data level.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fss in reg FIFO_STATUS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_data_level_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_fifo_status_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_FIFO_STATUS, (uint8_t*)&reg, 1);
  *val = reg.fss;

  return ret;
}

/**
  * @brief  Empty FIFO status flag.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of empty_fifo in reg FIFO_STATUS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_empty_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_fifo_status_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_FIFO_STATUS, (uint8_t*)&reg, 1);
  *val = reg.empty_fifo;

  return ret;
}

/**
  * @brief  FIFO overrun status flag.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of ovr in reg FIFO_STATUS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_fifo_status_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_FIFO_STATUS, (uint8_t*)&reg, 1);
  *val = reg.ovr;

  return ret;
}

/**
  * @brief  FIFO watermark status.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fth_fifo in reg FIFO_STATUS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_fifo_fth_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps25hb_fifo_status_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_FIFO_STATUS, (uint8_t*)&reg, 1);
  *val = reg.fth_fifo;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   LPS25HB_serial_interface
  * @brief      This section group all the functions concerning serial
  *             interface management
  * @{
  *
  */

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of sim in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_spi_mode_set(stmdev_ctx_t *ctx, lps25hb_sim_t val)
{
  lps25hb_ctrl_reg1_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.sim = (uint8_t)val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of sim in reg CTRL_REG1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_spi_mode_get(stmdev_ctx_t *ctx, lps25hb_sim_t *val)
{
  lps25hb_ctrl_reg1_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG1, (uint8_t*)&reg, 1);
  switch (reg.sim){
    case LPS25HB_SPI_4_WIRE:
      *val = LPS25HB_SPI_4_WIRE;
      break;
    case LPS25HB_SPI_3_WIRE:
      *val = LPS25HB_SPI_3_WIRE;
      break;
    default:
      *val = LPS25HB_SPI_4_WIRE;
      break;
  }
  return ret;
}

/**
  * @brief  Disable I2C interface.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of i2c_dis in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_i2c_interface_set(stmdev_ctx_t *ctx, lps25hb_i2c_dis_t val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  if(ret == 0){
    reg.i2c_dis = (uint8_t)val;
    ret = lps25hb_write_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Disable I2C interface.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of i2c_dis in reg CTRL_REG2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps25hb_i2c_interface_get(stmdev_ctx_t *ctx, lps25hb_i2c_dis_t *val)
{
  lps25hb_ctrl_reg2_t reg;
  int32_t ret;

  ret = lps25hb_read_reg(ctx, LPS25HB_CTRL_REG2, (uint8_t*)&reg, 1);
  switch (reg.i2c_dis){
    case LPS25HB_I2C_ENABLE:
      *val = LPS25HB_I2C_ENABLE;
      break;
    case LPS25HB_I2C_DISABLE:
      *val = LPS25HB_I2C_DISABLE;
      break;
    default:
      *val = LPS25HB_I2C_ENABLE;
      break;
  }
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
