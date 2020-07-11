/*
 ******************************************************************************
 * @file    l3gd20h_reg.c
 * @author  Sensors Software Solution Team
 * @brief   L3GD20H driver file
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
#include "l3gd20h_reg.h"

/**
  * @defgroup    L3GD20H
  * @brief      This file provides a set of functions needed to drive the
  *              l3gd20h enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup    L3GD20H_Interfaces_Functions
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
int32_t l3gd20h_read_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
int32_t l3gd20h_write_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
  * @defgroup    L3GD20H_Sensitivity
  * @brief      These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t l3gd20h_from_fs245_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 8.75f);
}

float_t l3gd20h_from_fs500_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 17.50f);
}

float_t l3gd20h_from_fs2000_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 70.0f);
}

float_t l3gd20h_from_lsb_to_celsius(int16_t lsb)
{
  return ((float_t)lsb +25.0f);
}
/**
  * @}
  *
  */

/**
  * @defgroup    L3GD20H_Datageneration
  * @brief      This section groups all the functions concerning data generation.
  * @{
  *
  */

/**
  * @brief  Enable gyroscope axis.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val     Gyroscope’s pitch axis (X) output enable..
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_axis_set(stmdev_ctx_t *ctx, l3gd20h_gy_axis_t val)
{
  l3gd20h_ctrl1_t ctrl1;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL1, (uint8_t*)&ctrl1, 1);
  if(ret == 0) {
    ctrl1.xen  = val.xen;
    ctrl1.yen  = val.yen;
    ctrl1.zen  = val.zen;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL1, (uint8_t*)&ctrl1, 1);
  }
  return ret;
}

/**
  * @brief  Enable gyroscope axis.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val     Gyroscope’s pitch axis (X) output enable..(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_axis_get(stmdev_ctx_t *ctx, l3gd20h_gy_axis_t *val)
{
  l3gd20h_ctrl1_t ctrl1;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL1, (uint8_t*)&ctrl1, 1);
  val->xen = ctrl1.xen;
  val->yen = ctrl1.yen;
  val->zen = ctrl1.zen;

  return ret;
}

/**
  * @brief  Gyroscope data rate selection..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "pd" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_data_rate_set(stmdev_ctx_t *ctx,
                                 l3gd20h_gy_data_rate_t val)
{
  l3gd20h_low_odr_t low_odr;
  l3gd20h_ctrl1_t ctrl1;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL1, (uint8_t*)&ctrl1, 1);
  if(ret == 0){
    ctrl1.pd = ( (uint8_t)val & 0x80U ) >> 7;
    ctrl1.dr = (uint8_t)val & 0x07U;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL1, (uint8_t*)&ctrl1, 1);
  }
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_LOW_ODR, (uint8_t*)&low_odr, 1);
  }
  if(ret == 0){
    low_odr.low_odr = ( (uint8_t)val & 0x10U ) >> 4;
    ret = l3gd20h_write_reg(ctx, L3GD20H_LOW_ODR, (uint8_t*)&low_odr, 1);
  }
  return ret;
}

/**
  * @brief  Gyroscope data rate selection..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of pd in reg CTRL1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_data_rate_get(stmdev_ctx_t *ctx,
                                 l3gd20h_gy_data_rate_t *val)
{
  l3gd20h_low_odr_t low_odr;
  l3gd20h_ctrl1_t ctrl1;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL1, (uint8_t*)&ctrl1, 1);
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_LOW_ODR, (uint8_t*)&low_odr, 1);
  }
  switch ((ctrl1.pd << 7) + (low_odr.low_odr << 4) + ctrl1.dr){
    case L3GD20H_POWER_DOWN:
      *val = L3GD20H_POWER_DOWN;
      break;
    case L3GD20H_12Hz5:
      *val = L3GD20H_12Hz5;
      break;
    case L3GD20H_25Hz:
      *val = L3GD20H_25Hz;
      break;
    case L3GD20H_50Hz:
      *val = L3GD20H_50Hz;
      break;
    case L3GD20H_100Hz:
      *val = L3GD20H_100Hz;
      break;
    case L3GD20H_200Hz:
      *val = L3GD20H_200Hz;
      break;
    case L3GD20H_400Hz:
      *val = L3GD20H_400Hz;
      break;
    case L3GD20H_800Hz:
      *val = L3GD20H_800Hz;
      break;
    default:
      *val = L3GD20H_POWER_DOWN;
      break;
  }
  return ret;
}

/**
  * @brief   Gyroscope full-scale selection..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "fs" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_full_scale_set(stmdev_ctx_t *ctx, l3gd20h_gy_fs_t val)
{
  l3gd20h_ctrl4_t ctrl4;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  if(ret == 0){
    ctrl4.fs = (uint8_t)val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  }
  return ret;
}

/**
  * @brief   Gyroscope full-scale selection..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fs in reg CTRL4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_full_scale_get(stmdev_ctx_t *ctx, l3gd20h_gy_fs_t *val)
{
  l3gd20h_ctrl4_t ctrl4;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  switch (ctrl4.fs){
    case L3GD20H_245dps:
      *val = L3GD20H_245dps;
      break;
    case L3GD20H_500dps:
      *val = L3GD20H_500dps;
      break;
    default:
      *val = L3GD20H_245dps;
      break;
  }
  return ret;
}
/**
  * @brief  Block data update.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of bdu in reg CTRL4.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val)
{
  l3gd20h_ctrl4_t ctrl4;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  if(ret == 0){
    ctrl4.bdu = val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  }
  return ret;
}

/**
  * @brief  Blockdataupdate.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of bdu in reg CTRL4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l3gd20h_ctrl4_t ctrl4;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  *val = ctrl4.bdu;

  return ret;
}
/**
  * @brief  Gyroscope new data available..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "zyxda" in reg STATUS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l3gd20h_status_t status;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_STATUS, (uint8_t*)&status, 1);
  *val = status.zyxda;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L3GD20H_Dataoutput
  * @brief       This section groups all the data output functions.
  * @{
  *
  */

/**
  * @brief  Temperature data output register (r). L and H registers together
  *          express a 16-bit word in two’s complement..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = l3gd20h_read_reg(ctx, L3GD20H_OUT_TEMP, buff, 1);
  return ret;
}

/**
  * @brief  Angular rate sensor. The value is expressed as a 16-bit
  *          word in two’s complement..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_angular_rate_raw_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_OUT_X_L, buff, 6);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) +  (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) +  (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) +  (int16_t)buff[4];

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L3GD20H_Common
  * @brief       This section groups common usefull functions.
  * @{
  *
  */

/**
  * @brief  DeviceWhoamI..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_dev_id_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = l3gd20h_read_reg(ctx, L3GD20H_WHO_AM_I, buff, 1);
  return ret;
}

/**
  * @brief  Big/Little Endian data selection..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "ble" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_dev_data_format_set(stmdev_ctx_t *ctx, l3gd20h_ble_t val)
{
  l3gd20h_ctrl4_t ctrl4;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  if(ret == 0){
    ctrl4.ble = (uint8_t)val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  }
  return ret;
}

/**
  * @brief  Big/Little Endian data selection..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of ble in reg CTRL4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_dev_data_format_get(stmdev_ctx_t *ctx, l3gd20h_ble_t *val)
{
  l3gd20h_ctrl4_t ctrl4;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  switch (ctrl4.ble){
    case L3GD20H_LSB_LOW_ADDRESS:
      *val = L3GD20H_LSB_LOW_ADDRESS;
      break;
    case L3GD20H_MSB_LOW_ADDRESS:
      *val = L3GD20H_MSB_LOW_ADDRESS;
      break;
    default:
      *val = L3GD20H_LSB_LOW_ADDRESS;
      break;
  }
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of boot in reg CTRL5.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_dev_boot_set(stmdev_ctx_t *ctx, uint8_t val)
{
  l3gd20h_ctrl5_t ctrl5;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  if(ret == 0){
    ctrl5.boot = val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of boot in reg CTRL5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_dev_boot_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l3gd20h_ctrl5_t ctrl5;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  *val = ctrl5.boot;

  return ret;
}
/**
  * @brief  Device status register.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val     X axis new data available..(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_dev_status_get(stmdev_ctx_t *ctx, l3gd20h_status_reg_t *val)
{
  l3gd20h_status_t status;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_STATUS, (uint8_t*)&status, 1);
  val->xda   = status.xda;
  val->yda   = status.yda;
  val->zda   = status.zda;
  val->zyxda = status.zyxda;
  val->_xor  = status._xor;
  val->yor   = status.yor;
  val->zor   = status.zor;
  val->zyxor = status.zyxor;

  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of sw_res in reg LOW_ODR.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_dev_reset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  l3gd20h_low_odr_t low_odr;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_LOW_ODR, (uint8_t*)&low_odr, 1);
  if(ret == 0){
    low_odr.sw_res = val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_LOW_ODR, (uint8_t*)&low_odr, 1);
  }
  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of sw_res in reg LOW_ODR.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_dev_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l3gd20h_low_odr_t low_odr;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_LOW_ODR, (uint8_t*)&low_odr, 1);
  *val = low_odr.sw_res;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L3GD20H_Filters
  * @brief       This section group all the functions concerning the
  *              filters configuration.
  * @{
  *
  */

/**
  * @brief  Low pass filter cutoff frequency.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "bw" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_filter_lp_bandwidth_set(stmdev_ctx_t *ctx,
                                           l3gd20h_lpbw_t val)
{
  l3gd20h_ctrl1_t ctrl1;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL1, (uint8_t*)&ctrl1, 1);
  if(ret == 0){
    ctrl1.bw = (uint8_t)val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL1, (uint8_t*)&ctrl1, 1);
  }
  return ret;
}

/**
  * @brief  Low pass filter cutoff frequency.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of bw in reg CTRL1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_filter_lp_bandwidth_get(stmdev_ctx_t *ctx,
                                           l3gd20h_lpbw_t *val)
{
  l3gd20h_low_odr_t low_odr;
  l3gd20h_ctrl1_t ctrl1;
  int32_t ret;
  
  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL1, (uint8_t*)&ctrl1, 1);
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_LOW_ODR, (uint8_t*)&low_odr, 1);
  }
  switch ((low_odr.low_odr << 7) + (ctrl1.dr << 4) + ctrl1.bw){
    case L3GD20H_16Hz6_USE_ODR_50Hz:
      *val = L3GD20H_16Hz6_USE_ODR_50Hz;
      break;
    case L3GD20H_12Hz5_USE_ODR_100Hz:
      *val = L3GD20H_12Hz5_USE_ODR_100Hz;
      break;
    case L3GD20H_25Hz_USE_ODR_100Hz:
      *val = L3GD20H_25Hz_USE_ODR_100Hz;
      break;
    case L3GD20H_12Hz5_USE_ODR_200Hz:
      *val = L3GD20H_12Hz5_USE_ODR_200Hz;
      break;
    case L3GD20H_70Hz_USE_ODR_200Hz:
      *val = L3GD20H_70Hz_USE_ODR_200Hz;
      break;
    case L3GD20H_20Hz_USE_ODR_400Hz:
      *val = L3GD20H_20Hz_USE_ODR_400Hz;
      break;
    case L3GD20H_25Hz_USE_ODR_400Hz:
      *val = L3GD20H_25Hz_USE_ODR_400Hz;
      break;
    case L3GD20H_50Hz_USE_ODR_400Hz:
      *val = L3GD20H_50Hz_USE_ODR_400Hz;
      break;
    case L3GD20H_110Hz_USE_ODR_400Hz:
      *val = L3GD20H_110Hz_USE_ODR_400Hz;
      break;
    case L3GD20H_30Hz_USE_ODR_800Hz:
      *val = L3GD20H_30Hz_USE_ODR_800Hz;
      break;
    case L3GD20H_35Hz_USE_ODR_800Hz:
      *val = L3GD20H_35Hz_USE_ODR_800Hz;
      break;
    case L3GD20H_100Hz_USE_ODR_800Hz:
      *val = L3GD20H_100Hz_USE_ODR_800Hz;
      break;
    default:
      *val = L3GD20H_16Hz6_USE_ODR_50Hz;
      break;
  }
  return ret;
}

/**
  * @brief  Gyroscope high-pass filter bandwidth selection..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "hpcf" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_filter_hp_bandwidth_set(stmdev_ctx_t *ctx,
                                            l3gd20h_gy_hp_bw_t val)
{
  l3gd20h_ctrl2_t ctrl2;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL2, (uint8_t*)&ctrl2, 1);
  if(ret == 0){
    ctrl2.hpcf = (uint8_t)val & 0x03U;
    ctrl2.hpm = ((uint8_t)val & 0x30U) >> 4;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  return ret;
}

/**
  * @brief  Gyroscope high-pass filter bandwidth selection..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of hpcf in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_filter_hp_bandwidth_get(stmdev_ctx_t *ctx,
                                            l3gd20h_gy_hp_bw_t *val)
{
  l3gd20h_ctrl2_t ctrl2;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL2, (uint8_t*)&ctrl2, 1);
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  switch ( (ctrl2.hpm << 4) + ctrl2.hpcf){
    case L3GD20H_NORMAL_MODE_LIGHT:
      *val = L3GD20H_NORMAL_MODE_LIGHT;
      break;
    case L3GD20H_NORMAL_MODE_NORMAL:
      *val = L3GD20H_NORMAL_MODE_NORMAL;
      break;
    case L3GD20H_NORMAL_MODE_STRONG:
      *val = L3GD20H_NORMAL_MODE_STRONG;
      break;
    case L3GD20H_NORMAL_MODE_EXTREME:
      *val = L3GD20H_NORMAL_MODE_EXTREME;
      break;
    case L3GD20H_USE_REFERENCE_LIGHT:
      *val = L3GD20H_USE_REFERENCE_LIGHT;
      break;
    case L3GD20H_USE_REFERENCE_NORMAL:
      *val = L3GD20H_USE_REFERENCE_NORMAL;
      break;
    case L3GD20H_USE_REFERENCE_STRONG:
      *val = L3GD20H_USE_REFERENCE_STRONG;
      break;
    case L3GD20H_USE_REFERENCE_EXTREME:
      *val = L3GD20H_USE_REFERENCE_EXTREME;
      break;
    case L3GD20H_AUTORESET_LIGHT:
      *val = L3GD20H_AUTORESET_LIGHT;
      break;
    case L3GD20H_AUTORESET_NORMAL:
      *val = L3GD20H_AUTORESET_NORMAL;
      break;
    case L3GD20H_AUTORESET_STRONG:
      *val = L3GD20H_AUTORESET_STRONG;
      break;
    case L3GD20H_AUTORESET_EXTREME:
      *val = L3GD20H_AUTORESET_EXTREME;
      break;
    default:
      *val = L3GD20H_NORMAL_MODE_LIGHT;
      break;
  }
  return ret;
}

/**
  * @brief  Gyro output filter path configuration..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "out_sel" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_filter_out_path_set(stmdev_ctx_t *ctx,
                                       l3gd20h_gy_out_path_t val)
{
  l3gd20h_ctrl5_t ctrl5;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  if(ret == 0){
    ctrl5.out_sel = (uint8_t)val & 0x03U;
    ctrl5.hpen = ( (uint8_t)val & 0x10U ) >> 4;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  return ret;
}

/**
  * @brief  Gyro output filter path configuration..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of out_sel in reg CTRL5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_filter_out_path_get(stmdev_ctx_t *ctx,
                                       l3gd20h_gy_out_path_t *val)
{
  l3gd20h_ctrl5_t ctrl5;
  int32_t ret;
  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  switch (( ctrl5.hpen << 4 ) + ctrl5.out_sel ){
    case L3GD20H_LPF1_OUT:
      *val = L3GD20H_LPF1_OUT;
      break;
    case L3GD20H_LPF1_HPF_OUT:
      *val = L3GD20H_LPF1_HPF_OUT;
      break;
    case L3GD20H_LPF1_LPF2_OUT:
      *val = L3GD20H_LPF1_LPF2_OUT;
      break;
    case L3GD20H_LPF1_HPF_LPF2_OUT:
      *val = L3GD20H_LPF1_HPF_LPF2_OUT;
      break;
    default:
      *val = L3GD20H_LPF1_OUT;
      break;
  }
  return ret;
}

/**
  * @brief  Gyro interrupt filter path configuration..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "ig_sel" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_filter_int_path_set(stmdev_ctx_t *ctx,
                                       l3gd20h_gy_int_path_t val)
{
  l3gd20h_ctrl5_t ctrl5;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  if(ret == 0){
    ctrl5.ig_sel = (uint8_t)val & 0x03U;
    ctrl5.hpen = ( (uint8_t)val & 0x10U ) >> 4;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  return ret;
}

/**
  * @brief  Gyro interrupt filter path configuration..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of ig_sel in reg CTRL5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_filter_int_path_get(stmdev_ctx_t *ctx,
                                       l3gd20h_gy_int_path_t *val)
{
  l3gd20h_ctrl5_t ctrl5;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL5,
                            (uint8_t*)&ctrl5, 1);
  switch (( ctrl5.hpen << 4 ) + ctrl5.ig_sel ){
    case L3GD20H_LPF1_INT:
      *val = L3GD20H_LPF1_INT;
      break;
    case L3GD20H_LPF1_HPF_INT:
      *val = L3GD20H_LPF1_HPF_INT;
      break;
    case L3GD20H_LPF1_LPF2_INT:
      *val = L3GD20H_LPF1_LPF2_INT;
      break;
    case L3GD20H_LPF1_HPF_LPF2_INT:
      *val = L3GD20H_LPF1_HPF_LPF2_INT;
      break;
    default:
      *val = L3GD20H_LPF1_INT;
      break;
  }
  return ret;
}

/**
  * @brief  Reference value for gyroscope’s digital high-pass filter.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data to be write.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_filter_reference_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = l3gd20h_write_reg(ctx, L3GD20H_REFERENCE, buff, 1);
  return ret;
}

/**
  * @brief  Reference value for gyroscope’s digital high-pass filter.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_filter_reference_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = l3gd20h_read_reg(ctx, L3GD20H_REFERENCE, buff, 1);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L3GD20H_Serialinterface
  * @brief       This section groups all the functions concerning main
  *              serial interface management (not auxiliary)
  * @{
  *
  */

/**
  * @brief  SPI Serial Interface Mode selection..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "sim" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_spi_mode_set(stmdev_ctx_t *ctx, l3gd20h_sim_t val)
{
  l3gd20h_ctrl4_t ctrl4;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  if(ret == 0){
    ctrl4.sim = (uint8_t)val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  }
  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of sim in reg CTRL4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_spi_mode_get(stmdev_ctx_t *ctx, l3gd20h_sim_t *val)
{
  l3gd20h_ctrl4_t ctrl4;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  switch (ctrl4.sim){
    case L3GD20H_SPI_4_WIRE:
      *val = L3GD20H_SPI_4_WIRE;
      break;
    case L3GD20H_SPI_3_WIRE:
      *val = L3GD20H_SPI_3_WIRE;
      break;
    default:
      *val = L3GD20H_SPI_4_WIRE;
      break;
  }
  return ret;
}

/**
  * @brief  Enable / Disable I2C interface..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "i2c_dis" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_i2c_interface_set(stmdev_ctx_t *ctx, l3gd20h_i2c_dis_t val)
{
  l3gd20h_low_odr_t low_odr;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_LOW_ODR, (uint8_t*)&low_odr, 1);
  if(ret == 0){
    low_odr.i2c_dis = (uint8_t)val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_LOW_ODR, (uint8_t*)&low_odr, 1);
  }
  return ret;
}

/**
  * @brief  Enable / Disable I2C interface..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of i2c_dis in reg LOW_ODR.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_i2c_interface_get(stmdev_ctx_t *ctx, l3gd20h_i2c_dis_t *val)
{
  l3gd20h_low_odr_t low_odr;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_LOW_ODR, (uint8_t*)&low_odr, 1);
  switch (low_odr.i2c_dis){
    case L3GD20H_I2C_ENABLE:
      *val = L3GD20H_I2C_ENABLE;
      break;
    case L3GD20H_I2C_DISABLE:
      *val = L3GD20H_I2C_DISABLE;
      break;
    default:
      *val = L3GD20H_I2C_ENABLE;
      break;
  }
  return ret;
}
/**
  * @}
  *
  */

/**
  * @defgroup    L3GD20H_Interruptpins
  * @brief       This section groups all the functions that manage interrupt pins
  * @{
  *
  */

/**
  * @brief  Route a signal on INT 2_A/G pin..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val     FIFO Empty interrupt on DRDY/INT2 pin..
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_pin_int2_route_set(stmdev_ctx_t *ctx,
                                   l3gd20h_pin_int2_rt_t val)
{
  l3gd20h_ctrl3_t ctrl3;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL3, (uint8_t*)&ctrl3, 1);
  if(ret == 0) {
    ctrl3.int2_empty = val.int2_empty;
    ctrl3.int2_fth = val.int2_fth;
    ctrl3.int2_orun = val.int2_orun;
    ctrl3.int2_drdy = val.int2_drdy;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

/**
  * @brief  Route a signal on INT 2_A/G pin..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val     FIFO Empty interrupt on DRDY/INT2 pin..(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_pin_int2_route_get(stmdev_ctx_t *ctx,
                                   l3gd20h_pin_int2_rt_t *val)
{
  l3gd20h_ctrl3_t ctrl3;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL3, (uint8_t*)&ctrl3, 1);
  val->int2_empty = ctrl3.int2_empty;
  val->int2_orun = ctrl3.int2_orun;
  val->int2_fth = ctrl3.int2_fth;
  val->int2_drdy = ctrl3.int2_drdy;

  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "pp_od" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_pin_mode_set(stmdev_ctx_t *ctx, l3gd20h_pp_od_t val)
{
  l3gd20h_ctrl3_t ctrl3;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL3, (uint8_t*)&ctrl3, 1);
  if(ret == 0){
    ctrl3.pp_od = (uint8_t)val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of pp_od in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_pin_mode_get(stmdev_ctx_t *ctx, l3gd20h_pp_od_t *val)
{
  l3gd20h_ctrl3_t ctrl3;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL3, (uint8_t*)&ctrl3, 1);
  switch (ctrl3.pp_od){
    case L3GD20H_PUSH_PULL:
      *val = L3GD20H_PUSH_PULL;
      break;
    case L3GD20H_OPEN_DRAIN:
      *val = L3GD20H_OPEN_DRAIN;
      break;
    default:
      *val = L3GD20H_PUSH_PULL;
      break;
  }
  return ret;
}

/**
  * @brief  Interrupt active-high/low.Interrupt active-high/low..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "h_lactive" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_pin_polarity_set(stmdev_ctx_t *ctx, l3gd20h_pin_pol_t val)
{
  l3gd20h_low_odr_t low_odr;
  l3gd20h_ctrl3_t ctrl3;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_LOW_ODR, (uint8_t*)&low_odr, 1);
  if(ret == 0){
    low_odr.drdy_hl = (uint8_t)val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_LOW_ODR, (uint8_t*)&low_odr, 1);
  }
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  if(ret == 0){
    ctrl3.h_lactive = (uint8_t)val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

/**
  * @brief  Interrupt active-high/low.Interrupt active-high/low..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of h_lactive in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_pin_polarity_get(stmdev_ctx_t *ctx, l3gd20h_pin_pol_t *val)
{
  l3gd20h_ctrl3_t ctrl3;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL3, (uint8_t*)&ctrl3, 1);
  switch (ctrl3.h_lactive){
    case L3GD20H_ACTIVE_HIGH:
      *val = L3GD20H_ACTIVE_HIGH;
      break;
    case L3GD20H_ACTIVE_LOW:
      *val = L3GD20H_ACTIVE_LOW;
      break;
    default:
      *val = L3GD20H_ACTIVE_HIGH;
      break;
  }
  return ret;
}

/**
  * @brief  Route a signal on INT1 pin.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val     Boot status available on INT1 pin..
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_pin_int1_route_set(stmdev_ctx_t *ctx,
                                   l3gd20h_pin_int1_rt_t val)
{
  l3gd20h_ctrl3_t ctrl3;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL3, (uint8_t*)&ctrl3, 1);
  if(ret == 0) {
    ctrl3.int1_boot  = val.int1_boot;
    ctrl3.int1_ig  = val.int1_ig;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

/**
  * @brief  Route a signal on INT1 pin.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val     Boot status available on INT1 pin..(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_pin_int1_route_get(stmdev_ctx_t *ctx,
                                   l3gd20h_pin_int1_rt_t *val)
{
  l3gd20h_ctrl3_t ctrl3;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL3, (uint8_t*)&ctrl3, 1);
  val->int1_boot = ctrl3.int1_boot;
  val->int1_ig = ctrl3.int1_ig;

  return ret;
}

/**
  * @brief  Latched/pulsed interrupt..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "lir" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_pin_notification_set(stmdev_ctx_t *ctx, l3gd20h_lir_t val)
{
  l3gd20h_ig_cfg_t ig_cfg;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_CFG, (uint8_t*)&ig_cfg, 1);
  if(ret == 0){
    ig_cfg.lir = (uint8_t)val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_IG_CFG, (uint8_t*)&ig_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Latched/pulsed interrupt..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of lir in reg IG_CFG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_pin_notification_get(stmdev_ctx_t *ctx, l3gd20h_lir_t *val)
{
  l3gd20h_ig_cfg_t ig_cfg;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_CFG,
                            (uint8_t*)&ig_cfg, 1);
  switch (ig_cfg.lir){
    case L3GD20H_INT_PULSED:
      *val = L3GD20H_INT_PULSED;
      break;
    case L3GD20H_INT_LATCHED:
      *val = L3GD20H_INT_LATCHED;
      break;
    default:
      *val = L3GD20H_INT_PULSED;
      break;
  }
  return ret;
}

/**
  * @brief  AND/OR combination of accelerometer’s interrupt events..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "and_or" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_pin_logic_set(stmdev_ctx_t *ctx, l3gd20h_pin_logic_t val)
{
  l3gd20h_ig_cfg_t ig_cfg;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_CFG, (uint8_t*)&ig_cfg, 1);
  if(ret == 0){
    ig_cfg.and_or = (uint8_t)val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_IG_CFG, (uint8_t*)&ig_cfg, 1);
  }
  return ret;
}

/**
  * @brief  AND/OR combination of accelerometer’s interrupt events..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of and_or in reg IG_CFG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_pin_logic_get(stmdev_ctx_t *ctx, l3gd20h_pin_logic_t *val)
{
  l3gd20h_ig_cfg_t ig_cfg;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_CFG, (uint8_t*)&ig_cfg, 1);
  switch (ig_cfg.and_or){
    case L3GD20H_LOGIC_OR:
      *val = L3GD20H_LOGIC_OR;
      break;
    case L3GD20H_LOGIC_AND:
      *val = L3GD20H_LOGIC_AND;
      break;
    default:
      *val = L3GD20H_LOGIC_OR;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L3GD20H_Interrupt on threshold
  * @brief       This section group all the functions concerning the
  *              interrupt on threshold configuration.
  * @{
  *
  */

/**
  * @brief  Enable interrupt generation on threshold event..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Enable interrupt generation on gyroscope’s pitch (X)
  *                axis low event..
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_trshld_axis_set(stmdev_ctx_t *ctx,
                                   l3gd20h_gy_trshld_en_t val)
{
  l3gd20h_ig_cfg_t ig_cfg;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_CFG, (uint8_t*)&ig_cfg, 1);
  if(ret == 0) {
    ig_cfg.xlie  = val.xlie;
    ig_cfg.xhie  = val.xhie;
    ig_cfg.ylie  = val.ylie;
    ig_cfg.yhie  = val.yhie;
    ig_cfg.zlie  = val.zlie;
    ig_cfg.zhie  = val.zhie;
    ret = l3gd20h_write_reg(ctx, L3GD20H_IG_CFG, (uint8_t*)&ig_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Enable interrupt generation on threshold event..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Enable interrupt generation on gyroscope’s pitch (X)
  *                axis low event..(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_trshld_axis_get(stmdev_ctx_t *ctx,
                                   l3gd20h_gy_trshld_en_t *val)
{
  l3gd20h_ig_cfg_t ig_cfg;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_CFG,
                            (uint8_t*)&ig_cfg, 1);
  val->xlie = ig_cfg.xlie;
  val->xhie = ig_cfg.xhie;
  val->ylie = ig_cfg.ylie;
  val->yhie = ig_cfg.yhie;
  val->zlie = ig_cfg.zlie;
  val->zhie = ig_cfg.zhie;

  return ret;
}

/**
  * @brief  Angular rate sensor interrupt on threshold source..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Pitch(X)low..(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_trshld_src_get(stmdev_ctx_t *ctx,
                                  l3gd20h_gy_trshld_src_t *val)
{
  l3gd20h_ig_src_t ig_src;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_SRC, (uint8_t*)&ig_src, 1);
  val->xl = ig_src.xl;
  val->xh = ig_src.xh;
  val->yl = ig_src.yl;
  val->yh = ig_src.yh;
  val->zl = ig_src.zl;
  val->zh = ig_src.zh;
  val->ia = ig_src.ia;

  return ret;
}

/**
  * @brief  Angular rate sensor interrupt threshold on pitch (X) axis..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of thsx in reg IG_THS_XH.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_trshld_x_set(stmdev_ctx_t *ctx, uint16_t val)
{
  l3gd20h_ig_ths_xl_t ig_ths_xl;
  l3gd20h_ig_ths_xh_t ig_ths_xh;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_XH, (uint8_t*)&ig_ths_xh, 1);
  if(ret == 0){
    ig_ths_xh.thsx = (uint8_t)( val / 256U ) & 0x7FU;
    ret = l3gd20h_write_reg(ctx, L3GD20H_IG_THS_XH, (uint8_t*)&ig_ths_xh, 1);
  }
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_XL, (uint8_t*)&ig_ths_xl, 1);
  }
  if(ret == 0){
    ig_ths_xl.thsx = (uint8_t) (val - (ig_ths_xh.thsx * 256U));
    ret = l3gd20h_write_reg(ctx, L3GD20H_IG_THS_XL, (uint8_t*)&ig_ths_xl, 1);
  }

  return ret;
}

/**
  * @brief  Angular rate sensor interrupt threshold on pitch (X) axis..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of thsx in reg IG_THS_XH.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_trshld_x_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  l3gd20h_ig_ths_xl_t ig_ths_xl;
  l3gd20h_ig_ths_xh_t ig_ths_xh;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_XL, (uint8_t*)&ig_ths_xl, 1);
  if(ret == 0){
      ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_XH, (uint8_t*)&ig_ths_xh, 1);
      *val = ig_ths_xh.thsx;
      *val = *val / 256U;
      *val += ig_ths_xl.thsx;
  }


  return ret;
}
/**
  * @brief  Decrement or reset counter mode selection..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "dcrm" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_trshld_mode_set(stmdev_ctx_t *ctx, l3gd20h_dcrm_g_t val)
{
  l3gd20h_ig_ths_xh_t ig_ths_xh;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_XH, (uint8_t*)&ig_ths_xh, 1);
  if(ret == 0){
    ig_ths_xh.dcrm = (uint8_t)val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_IG_THS_XH, (uint8_t*)&ig_ths_xh, 1);
  }
  return ret;
}

/**
  * @brief  Decrement or reset counter mode selection..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of dcrm in reg IG_THS_XH.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_trshld_mode_get(stmdev_ctx_t *ctx, l3gd20h_dcrm_g_t *val)
{
  l3gd20h_ig_ths_xh_t ig_ths_xh;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_XH, (uint8_t*)&ig_ths_xh, 1);
  switch (ig_ths_xh.dcrm){
    case L3GD20H_RESET_MODE:
      *val = L3GD20H_RESET_MODE;
      break;
    case L3GD20H_DECREMENT_MODE:
      *val = L3GD20H_DECREMENT_MODE;
      break;
    default:
      *val = L3GD20H_RESET_MODE;
      break;
  }
  return ret;
}

/**
  * @brief  Angular rate sensor interrupt threshold on roll (Y) axis.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of thsy in reg IG_THS_YH.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_trshld_y_set(stmdev_ctx_t *ctx, uint16_t val)
{
  l3gd20h_ig_ths_yh_t ig_ths_yh;
  l3gd20h_ig_ths_yl_t ig_ths_yl;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_YH, (uint8_t*)&ig_ths_yh, 1);
  if(ret == 0){
    ig_ths_yh.thsy = (uint8_t)( val / 256U ) & 0x7FU;
    ret = l3gd20h_write_reg(ctx, L3GD20H_IG_THS_YH, (uint8_t*)&ig_ths_yh, 1);
  }
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_YL, (uint8_t*)&ig_ths_yl, 1);
  }
  if(ret == 0){
    ig_ths_yl.thsy = (uint8_t) (val - (ig_ths_yh.thsy * 256U));
    ret = l3gd20h_write_reg(ctx, L3GD20H_IG_THS_YL, (uint8_t*)&ig_ths_yl, 1);
  }

  return ret;
}

/**
  * @brief  Angular rate sensor interrupt threshold on roll (Y) axis.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of thsy in reg IG_THS_YH.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_trshld_y_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  l3gd20h_ig_ths_yh_t ig_ths_yh;
  l3gd20h_ig_ths_yl_t ig_ths_yl;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_YL, (uint8_t*)&ig_ths_yl, 1);
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_YH, (uint8_t*)&ig_ths_yh, 1);
    *val = ig_ths_yh.thsy;
    *val = *val / 256U;
    *val += ig_ths_yl.thsy;
  }
  return ret;
}

/**
  * @brief  Angular rate sensor interrupt threshold on roll (Z) axis.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of thsz in reg IG_THS_ZH.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_trshld_z_set(stmdev_ctx_t *ctx, uint16_t val)
{
  l3gd20h_ig_ths_zh_t ig_ths_zh;
  l3gd20h_ig_ths_zl_t ig_ths_zl;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_ZH, (uint8_t*)&ig_ths_zh, 1);
  if(ret == 0){
    ig_ths_zh.thsz = (uint8_t)( val / 256U ) & 0x7FU;
    ret = l3gd20h_write_reg(ctx, L3GD20H_IG_THS_ZH, (uint8_t*)&ig_ths_zh, 1);
  }
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_ZL, (uint8_t*)&ig_ths_zl, 1);
  }
  if(ret == 0){
    ig_ths_zl.thsz = (uint8_t) (val - (ig_ths_zh.thsz * 256U));
    ret = l3gd20h_write_reg(ctx, L3GD20H_IG_THS_ZL, (uint8_t*)&ig_ths_zl, 1);
  }

  return ret;
}

/**
  * @brief  Angular rate sensor interrupt threshold on roll (Z) axis.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of thsz in reg IG_THS_ZH.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_trshld_z_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  l3gd20h_ig_ths_zh_t ig_ths_zh;
  l3gd20h_ig_ths_zl_t ig_ths_zl;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_ZL, (uint8_t*)&ig_ths_zl, 1);
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_IG_THS_ZH, (uint8_t*)&ig_ths_zh, 1);
    *val = ig_ths_zh.thsz;
    *val = *val / 256U;
    *val += ig_ths_zh.thsz;
  }
  return ret;
}

/**
  * @brief  Enter/exit interrupt duration value..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of d in reg IG_DURATION.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_trshld_min_sample_set(stmdev_ctx_t *ctx, uint8_t val)
{
  l3gd20h_ig_duration_t ig_duration;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_DURATION, (uint8_t*)&ig_duration, 1);
  if(ret == 0){
    ig_duration.d = val;
    if (val != 0x00U){
      ig_duration.wait = PROPERTY_ENABLE;
    }
    else{
      ig_duration.wait = PROPERTY_DISABLE;
    }
    ret = l3gd20h_write_reg(ctx, L3GD20H_IG_DURATION,
                            (uint8_t*)&ig_duration, 1);
  }
  return ret;
}

/**
  * @brief  Enter/exit interrupt duration value..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of d in reg IG_DURATION.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_trshld_min_sample_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l3gd20h_ig_duration_t ig_duration;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_IG_DURATION, (uint8_t*)&ig_duration, 1);
  *val = ig_duration.d;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L3GD20H_Fifo
  * @brief       This section group all the functions concerning the fifo usage.
  * @{
  *
  */

/**
  * @brief  Sensing chain FIFO stop values memorization at threshold level.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of stoponfth in reg CTRL5.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_fifo_stop_on_wtm_set(stmdev_ctx_t *ctx, uint8_t val)
{
  l3gd20h_ctrl5_t ctrl5;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  if(ret == 0){
    ctrl5.stoponfth = val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  return ret;
}

/**
  * @brief  Sensing chain FIFO stop values memorization at threshold level.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of stoponfth in reg CTRL5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_fifo_stop_on_wtm_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l3gd20h_ctrl5_t ctrl5;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  *val = ctrl5.stoponfth;

  return ret;
}
/**
  * @brief  FIFO mode selection..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "fifo_en" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_fifo_mode_set(stmdev_ctx_t *ctx, l3gd20h_fifo_m_t val)
{
  l3gd20h_ctrl5_t ctrl5;
  l3gd20h_fifo_ctrl_t fifo_ctrl;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  if(ret == 0){
    fifo_ctrl.fm = ( (uint8_t)val & 0x07U );
    ret = l3gd20h_write_reg(ctx, L3GD20H_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  }
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  if(ret == 0){
    ctrl5.fifo_en = ( ( (uint8_t)val & 0x10U ) >> 4);
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  return ret;
}

/**
  * @brief  FIFO mode selection..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fifo_en in reg CTRL5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_fifo_mode_get(stmdev_ctx_t *ctx, l3gd20h_fifo_m_t *val)
{
  l3gd20h_ctrl5_t ctrl5;
  l3gd20h_fifo_ctrl_t fifo_ctrl;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  switch ((ctrl5.fifo_en << 4) + fifo_ctrl.fm) {
    case L3GD20H_BYPASS_MODE:
      *val = L3GD20H_BYPASS_MODE;
      break;
    case L3GD20H_FIFO_MODE:
      *val = L3GD20H_FIFO_MODE;
      break;
    case L3GD20H_STREAM_MODE:
      *val = L3GD20H_STREAM_MODE;
      break;
    case L3GD20H_STREAM_TO_FIFO_MODE:
      *val = L3GD20H_STREAM_TO_FIFO_MODE;
      break;
    case L3GD20H_BYPASS_TO_STREAM_MODE:
      *val = L3GD20H_BYPASS_TO_STREAM_MODE;
      break;
    case L3GD20H_BYPASS_TO_FIFO_MODE:
      *val = L3GD20H_BYPASS_TO_FIFO_MODE;
      break;
    default:
      *val = L3GD20H_BYPASS_MODE;
      break;
  }
  return ret;
}
/**
  * @brief  FIFO watermark level selection..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fth in reg FIFO_CTRL.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val)
{
  l3gd20h_fifo_ctrl_t fifo_ctrl;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  if(ret == 0){
    fifo_ctrl.fth = val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  }
  return ret;
}

/**
  * @brief  FIFO watermark level selection..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fth in reg FIFO_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l3gd20h_fifo_ctrl_t fifo_ctrl;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  *val = fifo_ctrl.fth;

  return ret;
}

/**
  * @brief  FIFO source register..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val     FIFO stored data level of the unread samples..(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_fifo_src_get(stmdev_ctx_t *ctx, l3gd20h_fifo_srs_t *val)
{
  l3gd20h_fifo_src_t fifo_src;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_FIFO_SRC, (uint8_t*)&fifo_src, 1);
  val->fss = fifo_src.fss;
  val->empty = fifo_src.empty;
  val->ovrn = fifo_src.ovrn;
  val->fth = fifo_src.fth;

  return ret;
}

/**
  * @brief  FIFO stored data level of the unread samples..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "fss" in reg FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_fifo_data_level_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l3gd20h_fifo_src_t fifo_src;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_FIFO_SRC, (uint8_t*)&fifo_src, 1);
  *val = fifo_src.fss;

  return ret;
}

/**
  * @brief  FIFOfullstatus.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "fss" in reg FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_fifo_full_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l3gd20h_fifo_src_t fifo_src;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_FIFO_SRC, (uint8_t*)&fifo_src, 1);
  *val = fifo_src.fss;

  return ret;
}

/**
  * @brief  FIFO watermark status..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "fth" in reg FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l3gd20h_fifo_src_t fifo_src;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_FIFO_SRC, (uint8_t*)&fifo_src, 1);
  *val = fifo_src.fth;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L3GD20H_DENPin
  * @brief       This section group all the functions concerning DEN pin usage.
  * @{
  *
  */

/**
  * @brief  DEN pin mode..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "lvlen" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_den_mode_set(stmdev_ctx_t *ctx, l3gd20h_den_md_t val)
{
  l3gd20h_ctrl2_t ctrl2;
  l3gd20h_ctrl4_t ctrl4;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  if(ret == 0){
    ctrl4.impen = (uint8_t)val & 0x01U;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  }
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  if(ret == 0){
    ctrl2.lvlen = ((uint8_t)val & 0x04U) >> 2;
    ctrl2.extren = ((uint8_t)val & 0x02U) >> 1;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  return ret;
}

/**
  * @brief  DEN pin mode..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of lvlen in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_den_mode_get(stmdev_ctx_t *ctx, l3gd20h_den_md_t *val)
{
  l3gd20h_ctrl2_t ctrl2;
  l3gd20h_ctrl4_t ctrl4;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  if(ret == 0){
    ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  switch ( (ctrl2.lvlen << 2) + (ctrl2.extren << 1) + ctrl4.impen ){
    case L3GD20H_DEN_DISABLE:
      *val = L3GD20H_DEN_DISABLE;
      break;
    case L3GD20H_DEN_ON_LEVEL_TRIGGER:
      *val = L3GD20H_DEN_ON_LEVEL_TRIGGER;
      break;
    case L3GD20H_DEN_ON_EDGE_TRIGGER:
      *val = L3GD20H_DEN_ON_EDGE_TRIGGER;
      break;
    case L3GD20H_DEN_IMPULSE_TRIGGER:
      *val = L3GD20H_DEN_IMPULSE_TRIGGER;
      break;
    default:
      *val = L3GD20H_DEN_DISABLE;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L3GD20H_Selftest
  * @brief       This section groups all the functions that manage self
  *              test configuration
  * @{
  *
  */

/**
  * @brief  Enable/disable self-test mode for gyroscope..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "st" in reg L3GD20H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_self_test_set(stmdev_ctx_t *ctx, l3gd20h_st_t val)
{
  l3gd20h_ctrl4_t ctrl4;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  if(ret == 0){
    ctrl4.st = (uint8_t)val;
    ret = l3gd20h_write_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  }
  return ret;
}

/**
  * @brief  Enable/disable self-test mode for gyroscope..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of st in reg CTRL4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l3gd20h_gy_self_test_get(stmdev_ctx_t *ctx, l3gd20h_st_t *val)
{
  l3gd20h_ctrl4_t ctrl4;
  int32_t ret;

  ret = l3gd20h_read_reg(ctx, L3GD20H_CTRL4, (uint8_t*)&ctrl4, 1);
  switch (ctrl4.st){
    case L3GD20H_ST_DISABLE:
      *val = L3GD20H_ST_DISABLE;
      break;
    case L3GD20H_ST_POSITIVE:
      *val = L3GD20H_ST_POSITIVE;
      break;
    case L3GD20H_ST_NEGATIVE:
      *val = L3GD20H_ST_NEGATIVE;
      break;
    default:
      *val = L3GD20H_ST_DISABLE;
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
