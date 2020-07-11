/*
 ******************************************************************************
 * @file    lis2ds12_reg.c
 * @author  Sensors Software Solution Team
 * @brief   LIS2DS12 driver file
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

#include "lis2ds12_reg.h"

/**
  * @defgroup  LIS2DS12
  * @brief     This file provides a set of functions needed to drive the
  *            lis2ds12 enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup  LIS2DS12_Interfaces_Functions
  * @brief     This section provide a set of functions used to read and
  *            write a generic register of the device.
  *            MANDATORY: return 0 -> no Error.
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
int32_t lis2ds12_read_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
int32_t lis2ds12_write_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
  * @defgroup    LIS2DS12_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t lis2ds12_from_fs2g_to_mg(int16_t lsb)
{
  return ((float_t)lsb *0.061f);
}

float_t lis2ds12_from_fs4g_to_mg(int16_t lsb)
{
  return ((float_t)lsb *0.122f);
}

float_t lis2ds12_from_fs8g_to_mg(int16_t lsb)
{
  return ((float_t)lsb *0.244f);
}

float_t lis2ds12_from_fs16g_to_mg(int16_t lsb)
{
  return ((float_t)lsb *0.488f);
}

float_t lis2ds12_from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb / 256.0f) + 25.0f);
}

/**
  * @}
  *
  */

/**
  * @defgroup  Data generation
  * @brief     This section groups all the functions concerning
  *            data generation
  * @{
  *
  */

/**
  * @brief  Read all the interrupt/status flag of the device.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get FIFO_SRC, STATUS_DUP, WAKE_UP_SRC, TAP_SRC,
  *                6D_SRC, FUNC_CK_GATE, FUNC_SRC.(ptr)
  *
  */
int32_t lis2ds12_all_sources_get(stmdev_ctx_t *ctx,
                                 lis2ds12_all_sources_t *val)
{
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_SRC,
                          (uint8_t*)&(val->fifo_src), 1);
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_STATUS_DUP,
                            (uint8_t*)&(val->status_dup), 1);
  }
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_SRC,
                            (uint8_t*)&(val->wake_up_src), 1);
  }
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_TAP_SRC,
                            (uint8_t*)&(val->tap_src), 1);
  }
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_6D_SRC,
                            (uint8_t*)&(val->_6d_src), 1);
  }
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CK_GATE,
                            (uint8_t*)&(val->func_ck_gate), 1);
  }
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_SRC,
                            (uint8_t*)&(val->func_src), 1);
  }
  return ret;
}

/**
  * @brief  Blockdataupdate.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of bdu in reg CTRL1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_ctrl1_t ctrl1;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL1, (uint8_t*)&ctrl1, 1);
  if(ret == 0){
    ctrl1.bdu = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL1, (uint8_t*)&ctrl1, 1);
  }
  return ret;
}

/**
  * @brief  Blockdataupdate.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    get the values of bdu in reg CTRL1(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_ctrl1_t ctrl1;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL1, (uint8_t*)&ctrl1, 1);
  *val = ctrl1.bdu;

  return ret;
}

/**
  * @brief  Accelerometer full-scale selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of fs in reg CTRL1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_xl_full_scale_set(stmdev_ctx_t *ctx, lis2ds12_fs_t val)
{
  lis2ds12_ctrl1_t ctrl1;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL1, (uint8_t*)&ctrl1, 1);
  if(ret == 0){
    ctrl1.fs = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL1, (uint8_t*)&ctrl1, 1);
  }
  return ret;
}

/**
  * @brief  Accelerometer full-scale selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of fs in reg CTRL1(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_xl_full_scale_get(stmdev_ctx_t *ctx, lis2ds12_fs_t *val)
{
  lis2ds12_ctrl1_t ctrl1;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL1, (uint8_t*)&ctrl1, 1);
  switch (ctrl1.fs){
    case LIS2DS12_2g:
      *val = LIS2DS12_2g;
      break;
    case LIS2DS12_16g:
      *val = LIS2DS12_16g;
      break;
    case LIS2DS12_4g:
      *val = LIS2DS12_4g;
      break;
    case LIS2DS12_8g:
      *val = LIS2DS12_8g;
      break;
    default:
      *val = LIS2DS12_2g;
      break;
  }
  return ret;
}

/**
  * @brief  Accelerometer data rate selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of odr in reg CTRL1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_xl_data_rate_set(stmdev_ctx_t *ctx, lis2ds12_odr_t val)
{
  lis2ds12_ctrl1_t ctrl1;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL1, (uint8_t*)&ctrl1, 1);
  if(ret == 0){
    ctrl1.odr = (uint8_t)val & 0x0FU;
    ctrl1.hf_odr = ((uint8_t)val & 0x10U) >> 4;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL1, (uint8_t*)&ctrl1, 1);
  }
  return ret;
}

/**
  * @brief  Accelerometer data rate selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of odr in reg CTRL1(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_xl_data_rate_get(stmdev_ctx_t *ctx, lis2ds12_odr_t *val)
{
  lis2ds12_ctrl1_t ctrl1;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL1, (uint8_t*)&ctrl1, 1);
  switch ((ctrl1.hf_odr << 4) + ctrl1.odr){
    case LIS2DS12_XL_ODR_OFF:
      *val = LIS2DS12_XL_ODR_OFF;
      break;
    case LIS2DS12_XL_ODR_1Hz_LP:
      *val = LIS2DS12_XL_ODR_1Hz_LP;
      break;
    case LIS2DS12_XL_ODR_12Hz5_LP:
      *val = LIS2DS12_XL_ODR_12Hz5_LP;
      break;
    case LIS2DS12_XL_ODR_25Hz_LP:
      *val = LIS2DS12_XL_ODR_25Hz_LP;
      break;
    case LIS2DS12_XL_ODR_50Hz_LP:
      *val = LIS2DS12_XL_ODR_50Hz_LP;
      break;
    case LIS2DS12_XL_ODR_100Hz_LP:
      *val = LIS2DS12_XL_ODR_100Hz_LP;
      break;
    case LIS2DS12_XL_ODR_200Hz_LP:
      *val = LIS2DS12_XL_ODR_200Hz_LP;
      break;
    case LIS2DS12_XL_ODR_400Hz_LP:
      *val = LIS2DS12_XL_ODR_400Hz_LP;
      break;
    case LIS2DS12_XL_ODR_800Hz_LP:
      *val = LIS2DS12_XL_ODR_800Hz_LP;
      break;
    case LIS2DS12_XL_ODR_12Hz5_HR:
      *val = LIS2DS12_XL_ODR_12Hz5_HR;
      break;
    case LIS2DS12_XL_ODR_25Hz_HR:
      *val = LIS2DS12_XL_ODR_25Hz_HR;
      break;
    case LIS2DS12_XL_ODR_50Hz_HR:
      *val = LIS2DS12_XL_ODR_50Hz_HR;
      break;
    case LIS2DS12_XL_ODR_100Hz_HR:
      *val = LIS2DS12_XL_ODR_100Hz_HR;
      break;
    case LIS2DS12_XL_ODR_200Hz_HR:
      *val = LIS2DS12_XL_ODR_200Hz_HR;
      break;
    case LIS2DS12_XL_ODR_400Hz_HR:
      *val = LIS2DS12_XL_ODR_400Hz_HR;
      break;
    case LIS2DS12_XL_ODR_800Hz_HR:
      *val = LIS2DS12_XL_ODR_800Hz_HR;
      break;
    case LIS2DS12_XL_ODR_1k6Hz_HF:
      *val = LIS2DS12_XL_ODR_1k6Hz_HF;
      break;
    case LIS2DS12_XL_ODR_3k2Hz_HF:
      *val = LIS2DS12_XL_ODR_3k2Hz_HF;
      break;
    case LIS2DS12_XL_ODR_6k4Hz_HF:
      *val = LIS2DS12_XL_ODR_6k4Hz_HF;
      break;
    default:
      *val = LIS2DS12_XL_ODR_OFF;
      break;
  }
  return ret;
}

/**
  * @brief  The STATUS_REG register.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get registers STATUS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_status_reg_get(stmdev_ctx_t *ctx, lis2ds12_status_t *val)
{
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_STATUS, (uint8_t*) val, 1);
  return ret;
}

/**
  * @brief  Accelerometer new data available.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    get the values of drdy in reg STATUS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_xl_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_status_t status;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_STATUS, (uint8_t*)&status, 1);
  *val = status.drdy;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Data output
  * @brief     This section groups all the data output functions.
  * @{
  *
  */

/**
  * @brief  Module output value (8-bit).[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_acceleration_module_raw_get(stmdev_ctx_t *ctx,
                                             uint8_t *buff)
{
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_MODULE_8BIT, buff, 1);
  return ret;
}

/**
  * @brief  Temperature data output register (r). L and H registers together
  *         express a 16-bit word in two’s complement.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_OUT_T, buff, 1);
  return ret;
}

/**
  * @brief  Linear acceleration output register. The value is expressed as a
  *         16-bit word in two’s complement.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_acceleration_raw_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_OUT_X_L, buff, 6);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) +  (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) +  (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) +  (int16_t)buff[4];

  return ret;
}

/**
  * @brief  Number of steps detected by step counter routine.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_number_of_steps_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_STEP_COUNTER_L, buff, 2);
  *val = (int16_t)buff[1];
  *val = (*val * 256) +  (int16_t)buff[0];

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Common
  * @brief     This section groups common usefull functions.
  * @{
  *
  */

/**
  * @brief  DeviceWhoamI.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_WHO_AM_I, buff, 1);
  return ret;
}

/**
  * @brief  Register address automatically incremented during a multiple byte
  *         access with a serial interface.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of if_add_inc in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_auto_increment_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_ctrl2_t ctrl2;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  if(ret == 0){
    ctrl2.if_add_inc = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  return ret;
}

/**
  * @brief  Register address automatically incremented during a multiple byte
  *         access with a serial interface.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of if_add_inc in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_auto_increment_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_ctrl2_t ctrl2;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  *val = ctrl2.if_add_inc;
  return ret;
}

/**
  * @brief  Enable access to the embedded functions/sensor hub configuration
  *         registers.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of func_cfg_en in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_mem_bank_set(stmdev_ctx_t *ctx, lis2ds12_func_cfg_en_t val)
{
  lis2ds12_ctrl2_t ctrl2;
  lis2ds12_ctrl2_adv_t ctrl2_adv;
  int32_t ret;

  if (val == LIS2DS12_ADV_BANK){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
    if(ret == 0){
      ctrl2.func_cfg_en = (uint8_t)val;
      ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
    }
  }
  else {
    ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2_ADV,
                            (uint8_t*)&ctrl2_adv, 1);
    if(ret == 0){
      ctrl2_adv.func_cfg_en = (uint8_t)val;
      ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL2_ADV,
                               (uint8_t*)&ctrl2_adv, 1);
    }
  }
  return ret;
}

/**
  * @brief  Software reset. Restore the default values in
  *         user registers.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of soft_reset in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_reset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_ctrl2_t ctrl2;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  if(ret == 0){
    ctrl2.soft_reset = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  return ret;
}

/**
  * @brief  Software reset. Restore the default values
  *         in user registers.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    get the values of soft_reset in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_ctrl2_t ctrl2;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  *val = ctrl2.soft_reset;
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of boot in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_boot_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_ctrl2_t ctrl2;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  if(ret == 0){
    ctrl2.boot = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    get the values of boot in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_boot_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_ctrl2_t ctrl2;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  *val = ctrl2.boot;
  return ret;
}

/**
  * @brief  xl_self_test: [set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of st in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_xl_self_test_set(stmdev_ctx_t *ctx, lis2ds12_st_t val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  if(ret == 0){
    ctrl3.st = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

/**
  * @brief  xl_self_test [get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of st in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_xl_self_test_get(stmdev_ctx_t *ctx, lis2ds12_st_t *val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  switch (ctrl3.st){
    case LIS2DS12_XL_ST_DISABLE:
      *val = LIS2DS12_XL_ST_DISABLE;
      break;
    case LIS2DS12_XL_ST_POSITIVE:
      *val = LIS2DS12_XL_ST_POSITIVE;
      break;
    case LIS2DS12_XL_ST_NEGATIVE:
      *val = LIS2DS12_XL_ST_NEGATIVE;
      break;
    default:
      *val = LIS2DS12_XL_ST_DISABLE;
      break;
  }

  return ret;
}

/**
  * @brief  data_ready_mode [set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of drdy_pulsed in reg CTRL5
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_data_ready_mode_set(stmdev_ctx_t *ctx,
                                     lis2ds12_drdy_pulsed_t val)
{
  lis2ds12_ctrl5_t ctrl5;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL5, (uint8_t*)&ctrl5, 1);
  if(ret == 0){
    ctrl5.drdy_pulsed = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  return ret;
}

/**
  * @brief  data_ready_mode: [get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of drdy_pulsed in reg CTRL5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_data_ready_mode_get(stmdev_ctx_t *ctx,
                                     lis2ds12_drdy_pulsed_t *val)
{
  lis2ds12_ctrl5_t ctrl5;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL5, (uint8_t*)&ctrl5, 1);
  switch (ctrl5.drdy_pulsed){
    case LIS2DS12_DRDY_LATCHED:
      *val = LIS2DS12_DRDY_LATCHED;
      break;
    case LIS2DS12_DRDY_PULSED:
      *val = LIS2DS12_DRDY_PULSED;
      break;
    default:
      *val = LIS2DS12_DRDY_LATCHED;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Filters
  * @brief     This section group all the functions concerning the filters
  *            configuration.
  * @{
  *
  */

/**
  * @brief  High-pass filter data selection on output register and FIFO.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of fds_slope in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_xl_hp_path_set(stmdev_ctx_t *ctx, lis2ds12_fds_slope_t val)
{
  lis2ds12_ctrl2_t ctrl2;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  if(ret == 0){
    ctrl2.fds_slope = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  return ret;
}

/**
  * @brief  High-pass filter data selection on output register and FIFO.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of fds_slope in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_xl_hp_path_get(stmdev_ctx_t *ctx,
                                lis2ds12_fds_slope_t *val)
{
  lis2ds12_ctrl2_t ctrl2;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  switch (ctrl2.fds_slope){
    case LIS2DS12_HP_INTERNAL_ONLY:
      *val = LIS2DS12_HP_INTERNAL_ONLY;
      break;
    case LIS2DS12_HP_ON_OUTPUTS:
      *val = LIS2DS12_HP_ON_OUTPUTS;
      break;
    default:
      *val = LIS2DS12_HP_INTERNAL_ONLY;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   Auxiliary_interface
  * @brief      This section groups all the functions concerning auxiliary
  *             interface.
  * @{
  *
  */

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of sim in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_spi_mode_set(stmdev_ctx_t *ctx, lis2ds12_sim_t val)
{
  lis2ds12_ctrl2_t ctrl2;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  if(ret == 0){
    ctrl2.sim = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sim in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_spi_mode_get(stmdev_ctx_t *ctx, lis2ds12_sim_t *val)
{
  lis2ds12_ctrl2_t ctrl2;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  switch (ctrl2.sim){
    case LIS2DS12_SPI_4_WIRE:
      *val = LIS2DS12_SPI_4_WIRE;
      break;
    case LIS2DS12_SPI_3_WIRE:
      *val = LIS2DS12_SPI_3_WIRE;
      break;
    default:
      *val = LIS2DS12_SPI_4_WIRE;
      break;
  }

  return ret;
}

/**
  * @brief  Disable / Enable I2C interface.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of i2c_disable in reg CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_i2c_interface_set(stmdev_ctx_t *ctx,
                                   lis2ds12_i2c_disable_t val)
{
  lis2ds12_ctrl2_t ctrl2;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  if(ret == 0){
    ctrl2.i2c_disable = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  }
  return ret;
}

/**
  * @brief  Disable / Enable I2C interface.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of i2c_disable in reg CTRL2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_i2c_interface_get(stmdev_ctx_t *ctx,
                                   lis2ds12_i2c_disable_t *val)
{
  lis2ds12_ctrl2_t ctrl2;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL2, (uint8_t*)&ctrl2, 1);
  switch (ctrl2.i2c_disable){
    case LIS2DS12_I2C_ENABLE:
      *val = LIS2DS12_I2C_ENABLE;
      break;
    case LIS2DS12_I2C_DISABLE:
      *val = LIS2DS12_I2C_DISABLE;
      break;
    default:
      *val = LIS2DS12_I2C_ENABLE;
      break;
  }

  return ret;
}

/**
  * @brief  Connect/Disconnects pull-up in if_cs pad.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of if_cs_pu_dis in reg FIFO_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_cs_mode_set(stmdev_ctx_t *ctx,
                             lis2ds12_if_cs_pu_dis_t val)
{
  lis2ds12_fifo_ctrl_t fifo_ctrl;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  if(ret == 0){
    fifo_ctrl.if_cs_pu_dis = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_FIFO_CTRL,
                             (uint8_t*)&fifo_ctrl, 1);
  }
  return ret;
}

/**
  * @brief  Connect/Disconnects pull-up in if_cs pad.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of if_cs_pu_dis in reg FIFO_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_cs_mode_get(stmdev_ctx_t *ctx,
                             lis2ds12_if_cs_pu_dis_t *val)
{
  lis2ds12_fifo_ctrl_t fifo_ctrl;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_CTRL,
                          (uint8_t*)&fifo_ctrl, 1);
  switch (fifo_ctrl.if_cs_pu_dis){
    case LIS2DS12_PULL_UP_CONNECTED:
      *val = LIS2DS12_PULL_UP_CONNECTED;
      break;
    case LIS2DS12_PULL_UP_DISCONNECTED:
      *val = LIS2DS12_PULL_UP_DISCONNECTED;
      break;
    default:
      *val = LIS2DS12_PULL_UP_CONNECTED;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   Serial Interface
  * @brief      This section groups all the functions concerning main serial
  *             interface management (not auxiliary)
  * @{
  *
  */

/**
  * @brief  Push-pull/open-drain selection on interrupt pad.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of pp_od in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pin_mode_set(stmdev_ctx_t *ctx, lis2ds12_pp_od_t val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  if(ret == 0){
    ctrl3.pp_od = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

/**
  * @brief  Push-pull/open-drain selection on interrupt pad.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of pp_od in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pin_mode_get(stmdev_ctx_t *ctx, lis2ds12_pp_od_t *val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  switch (ctrl3.pp_od){
    case LIS2DS12_PUSH_PULL:
      *val = LIS2DS12_PUSH_PULL;
      break;
    case LIS2DS12_OPEN_DRAIN:
      *val = LIS2DS12_OPEN_DRAIN;
      break;
    default:
      *val = LIS2DS12_PUSH_PULL;
      break;
  }

  return ret;
}

/**
  * @brief  pin_polarity:   Interrupt active-high/low.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of h_lactive in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pin_polarity_set(stmdev_ctx_t *ctx,
                                  lis2ds12_h_lactive_t val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  if(ret == 0){
    ctrl3.h_lactive = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

/**
  * @brief  Interrupt active-high/low.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of h_lactive in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pin_polarity_get(stmdev_ctx_t *ctx,
                                  lis2ds12_h_lactive_t *val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  switch (ctrl3.h_lactive){
    case LIS2DS12_ACTIVE_HIGH:
      *val = LIS2DS12_ACTIVE_HIGH;
      break;
    case LIS2DS12_ACTIVE_LOW:
      *val = LIS2DS12_ACTIVE_LOW;
      break;
    default:
      *val = LIS2DS12_ACTIVE_HIGH;
      break;
  }

  return ret;
}

/**
  * @brief  Latched/pulsed interrupt.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of lir in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_int_notification_set(stmdev_ctx_t *ctx,
                                      lis2ds12_lir_t val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  if(ret == 0){
    ctrl3.lir = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

/**
  * @brief  Latched/pulsed interrupt.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of lir in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_int_notification_get(stmdev_ctx_t *ctx,
                                      lis2ds12_lir_t *val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  switch (ctrl3.lir){
    case LIS2DS12_INT_PULSED:
      *val = LIS2DS12_INT_PULSED;
      break;
    case LIS2DS12_INT_LATCHED:
      *val = LIS2DS12_INT_LATCHED;
      break;
    default:
      *val = LIS2DS12_INT_PULSED;
      break;
  }

  return ret;
}

/**
  * @brief  Select the signal that need to route on int1 pad.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change union of registers from CTRL4 to
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pin_int1_route_set(stmdev_ctx_t *ctx,
                                    lis2ds12_pin_int1_route_t val)
{
  lis2ds12_wake_up_dur_t wake_up_dur;
  lis2ds12_ctrl4_t ctrl4;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL4, (uint8_t*)&ctrl4, 1);
  if(ret == 0){
    ctrl4.int1_drdy         = (uint8_t)val.int1_drdy;
    ctrl4.int1_fth          = (uint8_t)val.int1_fth;
    ctrl4.int1_6d           = (uint8_t)val.int1_6d;
    ctrl4.int1_tap          = (uint8_t)val.int1_tap;
    ctrl4.int1_ff           = (uint8_t)val.int1_ff;
    ctrl4.int1_wu           = (uint8_t)val.int1_wu;
    ctrl4.int1_s_tap        = (uint8_t)val.int1_s_tap;
    ctrl4.int1_master_drdy  = (uint8_t)val.int1_master_drdy;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL4, (uint8_t*)&ctrl4, 1);
  }
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_DUR,
                            (uint8_t*)&wake_up_dur, 1);
  }
  if(ret == 0){
    wake_up_dur.int1_fss7   = (uint8_t)val.int1_fss7;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_WAKE_UP_DUR,
                             (uint8_t*)&wake_up_dur, 1);
  }
  return ret;
}

/**
  * @brief  Select the signal that need to route on int1 pad.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get union of registers from CTRL4 to.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pin_int1_route_get(stmdev_ctx_t *ctx,
                                    lis2ds12_pin_int1_route_t *val)
{
  lis2ds12_wake_up_dur_t wake_up_dur;
  lis2ds12_ctrl4_t ctrl4;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL4, (uint8_t*)&ctrl4, 1);
  if(ret == 0){
    val->int1_drdy          = ctrl4.int1_drdy;
    val->int1_fth           = ctrl4.int1_fth;
    val->int1_6d            = ctrl4.int1_6d;
    val->int1_tap           = ctrl4.int1_tap;
    val->int1_ff            = ctrl4.int1_ff;
    val->int1_wu            = ctrl4.int1_wu;
    val->int1_s_tap         = ctrl4.int1_s_tap;
    val->int1_master_drdy   = ctrl4.int1_master_drdy;
    ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_DUR,
                            (uint8_t*)&wake_up_dur, 1);
    val->int1_fss7 = wake_up_dur.int1_fss7;
  }
  return ret;
}

/**
  * @brief  Select the signal that need to route on int2 pad.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change union of registers from CTRL5 to
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pin_int2_route_set(stmdev_ctx_t *ctx,
                                    lis2ds12_pin_int2_route_t val)
{
  lis2ds12_ctrl5_t ctrl5;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL5, (uint8_t*)&ctrl5, 1);
  if(ret == 0){
    ctrl5.int2_boot       = val.int2_boot;
    ctrl5.int2_tilt       = val.int2_tilt;
    ctrl5.int2_sig_mot    = val.int2_sig_mot;
    ctrl5.int2_step_det   = val.int2_step_det;
    ctrl5.int2_fth        = val.int2_fth;
    ctrl5.int2_drdy       = val.int2_drdy;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  return ret;
}

/**
  * @brief  Select the signal that need to route on int2 pad.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get union of registers from CTRL5 to.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pin_int2_route_get(stmdev_ctx_t *ctx,
                                    lis2ds12_pin_int2_route_t *val)
{
  lis2ds12_ctrl5_t ctrl5;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL5, (uint8_t*)&ctrl5, 1);
  val->int2_boot     = ctrl5.int2_boot;
  val->int2_tilt     = ctrl5.int2_tilt;
  val->int2_sig_mot  = ctrl5.int2_sig_mot;
  val->int2_step_det = ctrl5.int2_step_det;
  val->int2_fth      = ctrl5.int2_fth;
  val->int2_drdy     = ctrl5.int2_drdy;

  return ret;
}

/**
  * @brief  All interrupt signals become available on INT1 pin.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of int2_on_int1 in reg CTRL5
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_all_on_int1_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_ctrl5_t ctrl5;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL5, (uint8_t*)&ctrl5, 1);
  if(ret == 0){
    ctrl5.int2_on_int1 = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL5, (uint8_t*)&ctrl5, 1);
  }
  return ret;
}

/**
  * @brief  All interrupt signals become available on INT1 pin.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of int2_on_int1 in reg CTRL5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_all_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_ctrl5_t ctrl5;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL5, (uint8_t*)&ctrl5, 1);
  *val = ctrl5.int2_on_int1;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Interrupt Pins
  * @brief     This section groups all the functions that manage interrup pins.
  * @{
  *
  */

  /**
  * @brief  Connect / Disconnect pull-up on auxiliary I2C line.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of tud_en in reg FUNC_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_sh_pin_mode_set(stmdev_ctx_t *ctx, lis2ds12_tud_en_t val)
{
  lis2ds12_func_ctrl_t func_ctrl;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  if(ret == 0){
    func_ctrl.tud_en = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  }
  return ret;
}

/**
  * @brief  Connect / Disconnect pull-up on auxiliary I2C line.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of tud_en in reg FUNC_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_sh_pin_mode_get(stmdev_ctx_t *ctx, lis2ds12_tud_en_t *val)
{
  lis2ds12_func_ctrl_t func_ctrl;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  switch (func_ctrl.tud_en){
    case LIS2DS12_EXT_PULL_UP:
      *val = LIS2DS12_EXT_PULL_UP;
      break;
    case LIS2DS12_INTERNAL_PULL_UP:
      *val = LIS2DS12_INTERNAL_PULL_UP;
      break;
    default:
      *val = LIS2DS12_EXT_PULL_UP;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Wake_Up_event
  * @brief     This section groups all the functions that manage the Wake Up
  *            event generation.
  * @{
  *
  */

  /**
  * @brief  Threshold for wakeup [1 LSb = FS_XL / 64].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of wu_ths in reg WAKE_UP_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_wkup_threshold_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_THS,
                          (uint8_t*)&wake_up_ths, 1);
  if(ret == 0){
    wake_up_ths.wu_ths = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_WAKE_UP_THS,
                             (uint8_t*)&wake_up_ths, 1);
  }
  return ret;
}

/**
  * @brief  Threshold for wakeup [1 LSb = FS_XL / 64].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of wu_ths in reg WAKE_UP_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_wkup_threshold_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_THS,
                          (uint8_t*)&wake_up_ths, 1);
  *val = wake_up_ths.wu_ths;

  return ret;
}

/**
  * @brief  Wakeup duration [1 LSb = 1 / ODR].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of wu_dur in reg WAKE_UP_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_wkup_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_DUR,
                          (uint8_t*)&wake_up_dur, 1);
  if(ret == 0){
    wake_up_dur.wu_dur = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_WAKE_UP_DUR,
                             (uint8_t*)&wake_up_dur, 1);
  }
  return ret;
}

/**
  * @brief  Wakeup duration [1 LSb = 1 / ODR].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of wu_dur in reg WAKE_UP_DUR.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_wkup_dur_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_wake_up_dur_t wake_up_dur;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_DUR,
                          (uint8_t*)&wake_up_dur, 1);
  *val = wake_up_dur.wu_dur;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   Activity/Inactivity detection
  * @brief      This section groups all the functions concerning
  *             activity/inactivity detection.
  * @{
  *
  */

/**
  * @brief  Enables Sleep mode.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of sleep_on in reg WAKE_UP_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_sleep_mode_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_THS,
                          (uint8_t*)&wake_up_ths, 1);
  if(ret == 0){
    wake_up_ths.sleep_on = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_WAKE_UP_THS,
                             (uint8_t*)&wake_up_ths, 1);
  }
  return ret;
}

/**
  * @brief  Enables Sleep mode.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sleep_on in reg WAKE_UP_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_sleep_mode_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_wake_up_ths_t wake_up_ths;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_THS,
                          (uint8_t*)&wake_up_ths, 1);
  *val = wake_up_ths.sleep_on;
  return ret;
}

/**
  * @brief  Duration to go in sleep mode [1 LSb = 512 / ODR].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of sleep_dur in reg WAKE_UP_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_act_sleep_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_DUR,
                          (uint8_t*)&wake_up_dur, 1);
  if(ret == 0){
    wake_up_dur.sleep_dur = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_WAKE_UP_DUR,
                             (uint8_t*)&wake_up_dur, 1);
  }
  return ret;
}

/**
  * @brief  Duration to go in sleep mode [1 LSb = 512 / ODR].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sleep_dur in reg WAKE_UP_DUR.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_act_sleep_dur_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_wake_up_dur_t wake_up_dur;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_DUR,
                          (uint8_t*)&wake_up_dur, 1);
  *val = wake_up_dur.sleep_dur;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Tap Generator
  * @brief     This section groups all the functions that manage the tap and
  *            double tap event generation.
  * @{
  *
  */

/**
  * @brief  Enable Z direction in tap recognition.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of tap_z_en in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_detection_on_z_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  if(ret == 0){
    ctrl3.tap_z_en = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

/**
  * @brief  Enable Z direction in tap recognition.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of tap_z_en in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_detection_on_z_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  *val = ctrl3.tap_z_en;
  return ret;
}

/**
  * @brief  Enable Y direction in tap recognition.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of tap_y_en in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_detection_on_y_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  if(ret == 0){
    ctrl3.tap_y_en = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

/**
  * @brief  Enable Y direction in tap recognition.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of tap_y_en in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_detection_on_y_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  *val = ctrl3.tap_y_en;
  return ret;
}

/**
  * @brief  Enable X direction in tap recognition.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of tap_x_en in reg CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_detection_on_x_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  if(ret == 0){
    ctrl3.tap_x_en = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  }
  return ret;
}

/**
  * @brief  Enable X direction in tap recognition.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of tap_x_en in reg CTRL3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_detection_on_x_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_ctrl3_t ctrl3;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_CTRL3, (uint8_t*)&ctrl3, 1);
  *val = ctrl3.tap_x_en;
  return ret;
}

/**
  * @brief  Threshold for tap recognition [1 LSb = FS/32].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of tap_ths in reg TAP_6D_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_threshold_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_tap_6d_ths_t tap_6d_ths;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_TAP_6D_THS,
                          (uint8_t*)&tap_6d_ths, 1);
  if(ret == 0){
    tap_6d_ths.tap_ths = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_TAP_6D_THS,
                             (uint8_t*)&tap_6d_ths, 1);
  }
  return ret;
}

/**
  * @brief  Threshold for tap recognition [1 LSb = FS/32].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of tap_ths in reg TAP_6D_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_threshold_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_tap_6d_ths_t tap_6d_ths;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_TAP_6D_THS,
                          (uint8_t*)&tap_6d_ths, 1);
  *val = tap_6d_ths.tap_ths;
  return ret;
}

/**
  * @brief  Maximum duration is the maximum time of an overthreshold signal
  *         detection to be recognized as a tap event. The default value of
  *         these bits is 00b which corresponds to 4*ODR_XL time.
  *         If the SHOCK[1:0] bits are set to a different value, 1LSB
  *         corresponds to 8*ODR_XL time.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of shock in reg INT_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_shock_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_int_dur_t int_dur;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_INT_DUR, (uint8_t*)&int_dur, 1);
  if(ret == 0){
    int_dur.shock = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_INT_DUR, (uint8_t*)&int_dur, 1);
  }
  return ret;
}

/**
  * @brief  Maximum duration is the maximum time of an overthreshold signal
  *         detection to be recognized as a tap event. The default value of
  *         these bits is 00b which corresponds to 4*ODR_XL time.
  *         If the SHOCK[1:0] bits are set to a different value, 1LSB
  *         corresponds to 8*ODR_XL time.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of shock in reg INT_DUR.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_shock_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_int_dur_t int_dur;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_INT_DUR, (uint8_t*)&int_dur, 1);
  *val = int_dur.shock;

  return ret;
}

/**
  * @brief  Quiet time is the time after the first detected tap in which there
  *         must not be any overthreshold event. The default value of these
  *         bits is 00b which corresponds to 2*ODR_XL time.
  *         If the QUIET[1:0] bits are set to a different value, 1LSB
  *         corresponds to 4*ODR_XL time.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of quiet in reg INT_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_quiet_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_int_dur_t int_dur;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_INT_DUR, (uint8_t*)&int_dur, 1);
  if(ret == 0){
    int_dur.quiet = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_INT_DUR, (uint8_t*)&int_dur, 1);
  }
  return ret;
}

/**
  * @brief  Quiet time is the time after the first detected tap in which there
  *         must not be any overthreshold event. The default value of these
  *         bits is 00b which corresponds to 2*ODR_XL time.
  *         If the QUIET[1:0] bits are set to a different value, 1LSB
  *         corresponds to 4*ODR_XL time.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of quiet in reg INT_DUR.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_quiet_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_int_dur_t int_dur;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_INT_DUR, (uint8_t*)&int_dur, 1);
  *val = int_dur.quiet;
  return ret;
}

/**
  * @brief  When double tap recognition is enabled, this register expresses the
  *         maximum time between two consecutive detected taps to determine a
  *         double tap event. The default value of these bits is 0000b which
  *         corresponds to 16*ODR_XL time. If the DUR[3:0] bits are set to a
  *         different value, 1LSB corresponds to 32*ODR_XL time.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of lat in reg INT_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_int_dur_t int_dur;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_INT_DUR, (uint8_t*)&int_dur, 1);
  if(ret == 0){
    int_dur.lat = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_INT_DUR, (uint8_t*)&int_dur, 1);
  }
  return ret;
}

/**
  * @brief  When double tap recognition is enabled, this register expresses the
  *         maximum time between two consecutive detected taps to determine a
  *         double tap event. The default value of these bits is 0000b which
  *         corresponds to 16*ODR_XL time. If the DUR[3:0] bits are set to a
  *         different value, 1LSB corresponds to 32*ODR_XL time.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of lat in reg INT_DUR.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_dur_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_int_dur_t int_dur;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_INT_DUR, (uint8_t*)&int_dur, 1);
  *val = int_dur.lat;
  return ret;
}

/**
  * @brief  Single/double-tap event enable/disable.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of single_double_tap in regWAKE_UP_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_mode_set(stmdev_ctx_t *ctx,
                              lis2ds12_single_double_tap_t val)
{
  lis2ds12_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_THS,
                          (uint8_t*)&wake_up_ths, 1);
  if(ret == 0){
    wake_up_ths.single_double_tap = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_WAKE_UP_THS,
                             (uint8_t*)&wake_up_ths, 1);
  }
  return ret;
}

/**
  * @brief  Single/double-tap event enable/disable.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of single_double_tap in reg WAKE_UP_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_mode_get(stmdev_ctx_t *ctx,
                              lis2ds12_single_double_tap_t *val)
{
  lis2ds12_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_THS,
                          (uint8_t*)&wake_up_ths, 1);
  switch (wake_up_ths.single_double_tap){
    case LIS2DS12_ONLY_SINGLE:
      *val = LIS2DS12_ONLY_SINGLE;
      break;
    case LIS2DS12_ONLY_DOUBLE:
      *val = LIS2DS12_ONLY_DOUBLE;
      break;
    default:
      *val = LIS2DS12_ONLY_SINGLE;
      break;
  }

  return ret;
}

/**
  * @brief  TAP source register[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get registers TAP_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tap_src_get(stmdev_ctx_t *ctx, lis2ds12_tap_src_t *val)
{
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_TAP_SRC, (uint8_t*) val, 1);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   Six_position_detection(6D/4D)
  * @brief      This section groups all the functions concerning six
  *             position detection (6D).
  * @{
  *
  */

/**
  * @brief  Threshold for 4D/6D function.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of 6d_ths in reg TAP_6D_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_6d_threshold_set(stmdev_ctx_t *ctx, lis2ds12_6d_ths_t val)
{
  lis2ds12_tap_6d_ths_t tap_6d_ths;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_TAP_6D_THS, (uint8_t*)&tap_6d_ths, 1);
  if(ret == 0){
    tap_6d_ths._6d_ths = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_TAP_6D_THS,
                             (uint8_t*)&tap_6d_ths, 1);
  }
  return ret;
}

/**
  * @brief  Threshold for 4D/6D function.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of 6d_ths in reg TAP_6D_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_6d_threshold_get(stmdev_ctx_t *ctx, lis2ds12_6d_ths_t *val)
{
  lis2ds12_tap_6d_ths_t tap_6d_ths;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_TAP_6D_THS, (uint8_t*)&tap_6d_ths, 1);
  switch (tap_6d_ths._6d_ths){
    case LIS2DS12_DEG_80:
      *val = LIS2DS12_DEG_80;
      break;
    case LIS2DS12_DEG_70:
      *val = LIS2DS12_DEG_70;
      break;
    case LIS2DS12_DEG_60:
      *val = LIS2DS12_DEG_60;
      break;
    case LIS2DS12_DEG_50:
      *val = LIS2DS12_DEG_50;
      break;
    default:
      *val = LIS2DS12_DEG_80;
      break;
  }

  return ret;
}

/**
  * @brief  4D orientation detection enable.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of 4d_en in reg TAP_6D_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_4d_mode_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_tap_6d_ths_t tap_6d_ths;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_TAP_6D_THS,
                          (uint8_t*)& tap_6d_ths, 1);
  if(ret == 0){
    tap_6d_ths._4d_en = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_TAP_6D_THS,
                             (uint8_t*)& tap_6d_ths, 1);
  }
  return ret;
}

/**
  * @brief  4D orientation detection enable.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of 4d_en in reg TAP_6D_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_4d_mode_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_tap_6d_ths_t tap_6d_ths;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_TAP_6D_THS,
                          (uint8_t*)&tap_6d_ths, 1);
  *val = tap_6d_ths._4d_en;
  return ret;
}

/**
  * @brief  6D source register.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get union of registers from 6D_SRC to.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_6d_src_get(stmdev_ctx_t *ctx, lis2ds12_6d_src_t *val)
{
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_6D_SRC, (uint8_t*) val, 1);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  free_fall
  * @brief     This section group all the functions concerning the
  *            free fall detection.
  * @{
  *
  */

/**
  * @brief  Free-fall duration [1 LSb = 1 / ODR].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of ff_dur in reg WAKE_UP_DUR/FREE_FALL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_ff_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_free_fall_t free_fall;
  lis2ds12_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FREE_FALL, (uint8_t*)&free_fall, 1);
  if(ret == 0){
    free_fall.ff_dur =  val & 0x1FU;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_FREE_FALL,
                             (uint8_t*)&free_fall, 1);
  }
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_DUR,
                            (uint8_t*)&wake_up_dur, 1);
  }
  if(ret == 0){
    wake_up_dur.ff_dur = (val & 0x20U) >> 5;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_WAKE_UP_DUR,
                             (uint8_t*)&wake_up_dur, 1);
  }
  return ret;
}

/**
  * @brief  Free-fall duration [1 LSb = 1 / ODR].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of ff_dur in reg WAKE_UP_DUR/FREE_FALL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_ff_dur_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_free_fall_t free_fall;
  lis2ds12_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FREE_FALL, (uint8_t*)&free_fall, 1);
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_WAKE_UP_DUR,
                            (uint8_t*)&wake_up_dur, 1);
    *val = (wake_up_dur.ff_dur << 5) + free_fall.ff_dur;
  }
  return ret;
}

/**
  * @brief  Free-fall threshold [1 LSB = 31.25 mg].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of ff_ths in reg FREE_FALL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_ff_threshold_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_free_fall_t free_fall;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FREE_FALL, (uint8_t*)&free_fall, 1);
  if(ret == 0){
    free_fall.ff_ths = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_FREE_FALL, (uint8_t*)&free_fall, 1);
  }
  return ret;
}

/**
  * @brief  Free-fall threshold [1 LSB = 31.25 mg].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of ff_ths in reg FREE_FALL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_ff_threshold_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_free_fall_t free_fall;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FREE_FALL, (uint8_t*)&free_fall, 1);
  *val = free_fall.ff_ths;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Fifo
  * @brief     This section group all the functions concerning the fifo usage
  * @{
  *
  */

/**
  * @brief  Module routine result is send to FIFO instead of X,Y,Z
  *         acceleration data[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of module_to_fifo in reg FIFO_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_fifo_xl_module_batch_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_fifo_ctrl_t fifo_ctrl;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  if(ret == 0){
    fifo_ctrl.module_to_fifo = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_FIFO_CTRL,
                             (uint8_t*)&fifo_ctrl, 1);
  }
  return ret;
}

/**
  * @brief   Module routine result is send to FIFO instead of X,Y,Z
  *          acceleration data[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of module_to_fifo in reg FIFO_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_fifo_xl_module_batch_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_fifo_ctrl_t fifo_ctrl;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  *val = fifo_ctrl.module_to_fifo;
  return ret;
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of fmode in reg FIFO_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_fifo_mode_set(stmdev_ctx_t *ctx, lis2ds12_fmode_t val)
{
  lis2ds12_fifo_ctrl_t fifo_ctrl;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  if(ret == 0){
    fifo_ctrl.fmode = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_FIFO_CTRL,
                             (uint8_t*)&fifo_ctrl, 1);
  }
  return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of fmode in reg FIFO_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_fifo_mode_get(stmdev_ctx_t *ctx, lis2ds12_fmode_t *val)
{
  lis2ds12_fifo_ctrl_t fifo_ctrl;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_CTRL, (uint8_t*)&fifo_ctrl, 1);
  switch (fifo_ctrl.fmode){
    case LIS2DS12_BYPASS_MODE:
      *val = LIS2DS12_BYPASS_MODE;
      break;
    case LIS2DS12_FIFO_MODE:
      *val = LIS2DS12_FIFO_MODE;
      break;
    case LIS2DS12_STREAM_TO_FIFO_MODE:
      *val = LIS2DS12_STREAM_TO_FIFO_MODE;
      break;
    case LIS2DS12_BYPASS_TO_STREAM_MODE:
      *val = LIS2DS12_BYPASS_TO_STREAM_MODE;
      break;

    case LIS2DS12_STREAM_MODE:
      *val = LIS2DS12_STREAM_MODE;
      break;
    default:
      *val = LIS2DS12_BYPASS_MODE;
      break;
  }

  return ret;
}

/**
  * @brief  FIFO watermark level selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of fifo_watermark in reg FIFO_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val)
{
  int32_t ret;
  ret = lis2ds12_write_reg(ctx, LIS2DS12_FIFO_THS, (uint8_t*)&val, 1);
  return ret;
}

/**
  * @brief  FIFO watermark level selection.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of fifo_watermark in reg FIFO_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_THS, val, 1);
  return ret;
}

/**
  * @brief  FIFO full, 256 unread samples.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of diff in reg FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_fifo_full_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_fifo_src_t fifo_src;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_SRC, (uint8_t*)&fifo_src, 1);
  *val = fifo_src.diff;
  return ret;
}

/**
  * @brief  FIFO overrun status.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of fifo_ovr in reg FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_fifo_src_t fifo_src;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_SRC, (uint8_t*)&fifo_src, 1);
  *val = fifo_src.fifo_ovr;
  return ret;
}

/**
  * @brief  FIFO threshold status.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of fth in reg FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_fifo_src_t fifo_src;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_SRC, (uint8_t*)&fifo_src, 1);
  *val = fifo_src.fth;
  return ret;
}

/**
  * @brief  The number of unread samples stored in FIFO.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of diff in reg FIFO_SAMPLES.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_fifo_data_level_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  lis2ds12_fifo_ths_t fifo_ths;
  lis2ds12_fifo_src_t fifo_src;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_THS, (uint8_t*)&fifo_ths, 1);
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_SRC, (uint8_t*)&fifo_src, 1);
    *val = fifo_src.diff;
    *val = (*val * 256U) +  fifo_ths.fth;
  }
  return ret;
}

/**
  * @brief  FIFO_SRCregister.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get registers FIFO_SRC.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_fifo_src_get(stmdev_ctx_t *ctx, lis2ds12_fifo_src_t *val)
{
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FIFO_SRC, (uint8_t*) val, 1);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Pedometer
  * @brief     This section groups all the functions that manage pedometer.
  * @{
  *
  */

/**
  * @brief  Minimum threshold value for step counter routine.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of sc_mths in reg STEP_COUNTER_MINTHS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_threshold_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_step_counter_minths_t step_counter_minths;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_STEP_COUNTER_MINTHS,
                          (uint8_t*)&step_counter_minths, 1);
  if(ret == 0){
    step_counter_minths.sc_mths = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_STEP_COUNTER_MINTHS,
                             (uint8_t*)&step_counter_minths, 1);
  }
  return ret;
}

/**
  * @brief  Minimum threshold value for step counter routine.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sc_mths in reg  STEP_COUNTER_MINTHS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_threshold_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_step_counter_minths_t step_counter_minths;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_STEP_COUNTER_MINTHS,
                          (uint8_t*)&step_counter_minths, 1);
  *val = step_counter_minths.sc_mths;
  return ret;
}

/**
  * @brief  Pedometer data range.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of pedo4g in reg STEP_COUNTER_MINTHS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_full_scale_set(stmdev_ctx_t *ctx,
                                     lis2ds12_pedo4g_t val)
{
  lis2ds12_step_counter_minths_t step_counter_minths;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_STEP_COUNTER_MINTHS,
                          (uint8_t*)&step_counter_minths, 1);
  if(ret == 0){
    step_counter_minths.pedo4g = (uint8_t)val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_STEP_COUNTER_MINTHS,
                             (uint8_t*)&step_counter_minths, 1);
  }
  return ret;
}

/**
  * @brief  Pedometer data range.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of pedo4g in reg STEP_COUNTER_MINTHS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_full_scale_get(stmdev_ctx_t *ctx,
                                     lis2ds12_pedo4g_t *val)
{
  lis2ds12_step_counter_minths_t step_counter_minths;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_STEP_COUNTER_MINTHS,
                          (uint8_t*)&step_counter_minths, 1);
  switch (step_counter_minths.pedo4g){
    case LIS2DS12_PEDO_AT_2g:
      *val = LIS2DS12_PEDO_AT_2g;
      break;
    case LIS2DS12_PEDO_AT_4g:
      *val = LIS2DS12_PEDO_AT_4g;
      break;
    default:
      *val = LIS2DS12_PEDO_AT_2g;
      break;
  }

  return ret;
}

/**
  * @brief  Reset pedometer step counter.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of rst_nstep in reg STEP_COUNTER_MINTHS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_step_reset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_step_counter_minths_t step_counter_minths;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_STEP_COUNTER_MINTHS,
                          (uint8_t*)&step_counter_minths, 1);
  if(ret == 0){
    step_counter_minths.rst_nstep = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_STEP_COUNTER_MINTHS,
                             (uint8_t*)&step_counter_minths, 1);
  }
  return ret;
}

/**
  * @brief  Reset pedometer step counter.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of rst_nstep in reg STEP_COUNTER_MINTHS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_step_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_step_counter_minths_t step_counter_minths;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_STEP_COUNTER_MINTHS,
                          (uint8_t*)&step_counter_minths, 1);
  *val = step_counter_minths.rst_nstep;
  return ret;
}

/**
  * @brief  Step detection flag.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of step_detect in reg FUNC_CK_GATE.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_step_detect_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_func_ck_gate_t func_ck_gate;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CK_GATE,
                          (uint8_t*)&func_ck_gate, 1);
  *val = func_ck_gate.step_detect;
  return ret;
}

/**
  * @brief  Enable pedometer algorithm.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of step_cnt_on in reg FUNC_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_sens_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_func_ctrl_t func_ctrl;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  if(ret == 0){
    func_ctrl.step_cnt_on = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_FUNC_CTRL,
                             (uint8_t*)&func_ctrl, 1);
  }
  return ret;
}

/**
  * @brief  Enable pedometer algorithm.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of step_cnt_on in reg FUNC_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_sens_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_func_ctrl_t func_ctrl;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  *val = func_ctrl.step_cnt_on;
  return ret;
}

/**
  * @brief  Minimum number of steps to start the increment step counter.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of deb_step in reg PEDO_DEB_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_debounce_steps_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_pedo_deb_reg_t pedo_deb_reg;
  int32_t ret;

  ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_ADV_BANK);
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_PEDO_DEB_REG,
                            (uint8_t*)&pedo_deb_reg, 1);
  }
  if(ret == 0){
    pedo_deb_reg.deb_step = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_PEDO_DEB_REG,
                             (uint8_t*)&pedo_deb_reg, 1);
  }
  if(ret == 0){
    ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_USER_BANK);
  }
  return ret;
}

/**
  * @brief   Minimum number of steps to start the increment step counter.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of deb_step in reg PEDO_DEB_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_debounce_steps_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_pedo_deb_reg_t pedo_deb_reg;
  int32_t ret;

  ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_ADV_BANK);
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_PEDO_DEB_REG,
                            (uint8_t*)&pedo_deb_reg, 1);
  }
  if(ret == 0){
    *val = pedo_deb_reg.deb_step;
    ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Debounce time. If the time between two consecutive steps is greater
  *         than DEB_TIME*80ms, the debouncer is reactivated.
  *         Default value: 01101[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of deb_time in reg PEDO_DEB_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_timeout_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_pedo_deb_reg_t pedo_deb_reg;
  int32_t ret;

  ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_ADV_BANK);
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_PEDO_DEB_REG,
                            (uint8_t*)&pedo_deb_reg, 1);
  }
  if(ret == 0){
    pedo_deb_reg.deb_time = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_PEDO_DEB_REG,
                             (uint8_t*)&pedo_deb_reg, 1);
  }
  if(ret == 0){
    ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Debounce time. If the time between two consecutive steps is
  *         greater than DEB_TIME*80ms, the debouncer is reactivated.
  *         Default value: 01101[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of deb_time in reg PEDO_DEB_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_timeout_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_pedo_deb_reg_t pedo_deb_reg;
  int32_t ret;

  ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_ADV_BANK);
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_PEDO_DEB_REG,
                            (uint8_t*)&pedo_deb_reg, 1);
  }
  if(ret == 0){
    *val = pedo_deb_reg.deb_time;
    ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Period of time to detect at least one step to generate step
  *         recognition [1 LSb = 1.6384 s].[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that contains data to write.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_steps_period_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_ADV_BANK);
  if(ret == 0){
    ret = lis2ds12_write_reg(ctx, LIS2DS12_STEP_COUNT_DELTA, buff, 1);
  }
  if(ret == 0){
    ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Period of time to detect at least one step to generate step
  *         recognition [1 LSb = 1.6384 s].[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  buff   buffer that stores data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_pedo_steps_period_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_ADV_BANK);
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_STEP_COUNT_DELTA, buff, 1);
  }
  if(ret == 0){
    ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_USER_BANK);
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  significant_motion
  * @brief   This section groups all the functions that manage the
  *          significant motion detection.
  * @{
  *
  */

/**
  * @brief  Significant motion event detection status.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sig_mot_detect in reg FUNC_CK_GATE.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_motion_data_ready_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_func_ck_gate_t func_ck_gate;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CK_GATE,
                          (uint8_t*)&func_ck_gate, 1);
  *val = func_ck_gate.sig_mot_detect;
  return ret;
}

/**
  * @brief  Enable significant motion detection function.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of sign_mot_on in reg FUNC_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_motion_sens_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_func_ctrl_t func_ctrl;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  if(ret == 0){
    func_ctrl.sign_mot_on = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_FUNC_CTRL,
                             (uint8_t*)&func_ctrl, 1);
  }
  return ret;
}

/**
  * @brief  Enable significant motion detection function.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sign_mot_on in reg FUNC_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_motion_sens_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_func_ctrl_t func_ctrl;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  *val = func_ctrl.sign_mot_on;
  return ret;
}

/**
  * @brief  These bits define the threshold value which corresponds to the
  *         number of steps to be performed by the user upon a change of
  *         location before the significant motion interrupt is generated.
  *         It is expressed as an 8-bit unsigned value.
  *         The default value of this field is equal to 6 (= 00000110b).[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of sm_ths in reg SM_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_motion_threshold_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_sm_ths_t sm_ths;
  int32_t ret;

  ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_ADV_BANK);
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_SM_THS, (uint8_t*)&sm_ths, 1);
  }
  if(ret == 0){
    sm_ths.sm_ths = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_SM_THS, (uint8_t*)&sm_ths, 1);
  }
  if(ret == 0){
    ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_USER_BANK);
  }
  return ret;
}

/**
  * @brief  These bits define the threshold value which corresponds to the
  *         number of steps to be performed by the user upon a change of
  *         location before the significant motion interrupt is generated.
  *         It is expressed as an 8-bit unsigned value.
  *         The default value of this field is equal to 6 (= 00000110b).[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sm_ths in reg SM_THS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_motion_threshold_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_sm_ths_t sm_ths;
  int32_t ret;

  ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_ADV_BANK);
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_SM_THS, (uint8_t*)&sm_ths, 1);
  }
  if(ret == 0){
    *val = sm_ths.sm_ths;
    ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_USER_BANK);
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  tilt_detection
  * @brief   This section groups all the functions that manage the tilt
  *          event detection.
  * @{
  *
  */

/**
  * @brief  Tilt event detection status.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of tilt_int in reg FUNC_CK_GATE.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tilt_data_ready_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_func_ck_gate_t func_ck_gate;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CK_GATE,
                          (uint8_t*)&func_ck_gate, 1);
  *val = func_ck_gate.tilt_int;
  return ret;
}

/**
  * @brief  Enable tilt calculation.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param val     change the values of tilt_on in reg FUNC_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tilt_sens_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_func_ctrl_t func_ctrl;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  if(ret == 0){
    func_ctrl.tilt_on = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  }
  return ret;
}

/**
  * @brief  Enable tilt calculation.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of tilt_on in reg FUNC_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_tilt_sens_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_func_ctrl_t func_ctrl;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  *val = func_ctrl.tilt_on;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  module
  * @brief   This section groups all the functions that manage
  *          module calculation
  * @{
  *
  */

/**
  * @brief  Module processing enable.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of module_on in reg FUNC_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_module_sens_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_func_ctrl_t func_ctrl;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  if(ret == 0){
    func_ctrl.module_on = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  }
  return ret;
}

/**
  * @brief  Module processing enable.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of module_on in reg FUNC_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_module_sens_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_func_ctrl_t func_ctrl;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  *val = func_ctrl.module_on;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Sensor_hub
  * @brief   This section groups all the functions that manage the sensor
  *          hub functionality.
  * @{
  *
  */

/**
  * @brief  Sensor hub output registers.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get registers from SENSORHUB1_REG to SENSORHUB6_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_sh_read_data_raw_get(stmdev_ctx_t *ctx,
                                      lis2ds12_sh_read_data_raw_t *val)
{
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_SENSORHUB1_REG, (uint8_t*) val, 6);
  return ret;
}

/**
  * @brief  Sensor hub I2C master enable.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of master_on in reg FUNC_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_sh_master_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lis2ds12_func_ctrl_t func_ctrl;
  int32_t ret;

  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  if(ret == 0){
    func_ctrl.master_on = val;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  }
  return ret;
}

/**
  * @brief  Sensor hub I2C master enable.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of master_on in reg FUNC_CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_sh_master_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_func_ctrl_t func_ctrl;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_CTRL, (uint8_t*)&func_ctrl, 1);
  *val = func_ctrl.master_on;
  return ret;
}

/**
  * @brief  Configure slave to perform a write.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change a structure:
  *                      - uint8_t slv_add;    8 bit i2c device address
  *                      - uint8_t slv_subadd; 8 bit register device address
  *                      - uint8_t slv_data;   8 bit data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_sh_cfg_write(stmdev_ctx_t *ctx,
                              lis2ds12_sh_cfg_write_t *val)
{
  lis2ds12_slv0_add_t slv0_add;
  int32_t ret;

  ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_ADV_BANK);
  if(ret == 0){
    slv0_add.slave0_add = (val->slv_add & 0xFEU) >> 1; 
    slv0_add.rw_0 = 0;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_SLV0_ADD,
                             (uint8_t*)&slv0_add, 1);
  }
  if(ret == 0){
    ret = lis2ds12_write_reg(ctx, LIS2DS12_SLV0_SUBADD,
                               &(val->slv_subadd), 1);
  }
  if(ret == 0){
    ret = lis2ds12_write_reg(ctx, LIS2DS12_DATAWRITE_SLV0,
                              &(val->slv_data), 1);
  }
  if(ret == 0){
    ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Configure slave 0 for perform a write/read.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get a structure(ptr):
  *                      - uint8_t slv_add;    8 bit i2c device address
  *                      - uint8_t slv_subadd; 8 bit register device address
  *                      - uint8_t slv_len;    num of bit to read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_sh_slv_cfg_read(stmdev_ctx_t *ctx,
                                 lis2ds12_sh_cfg_read_t *val)
{
  lis2ds12_slv0_add_t slv0_add;
  lis2ds12_slv0_config_t slv0_config;

  int32_t ret;

  ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_ADV_BANK);
  if(ret == 0){
    slv0_add.slave0_add = (val->slv_add & 0xFEU) >> 1; 
    slv0_add.rw_0 = 1;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_SLV0_ADD, 
                             (uint8_t*)&slv0_add, 1);
  }
  if(ret == 0){
    ret = lis2ds12_write_reg(ctx, LIS2DS12_SLV0_SUBADD,
                             &(val->slv_subadd), 1);
  }
  if(ret == 0){
    ret = lis2ds12_read_reg(ctx, LIS2DS12_SLV0_CONFIG,
                            (uint8_t*)&slv0_config, 1);
  }
  if(ret == 0){
    slv0_config.slave0_numop = val->slv_len;
    ret = lis2ds12_write_reg(ctx, LIS2DS12_SLV0_CONFIG,
                             (uint8_t*)&slv0_config, 1);
  }
  if(ret == 0){
    ret = lis2ds12_mem_bank_set(ctx, LIS2DS12_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Sensor hub communication status.[get]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    Get the values of sensorhub_end_op.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lis2ds12_sh_end_op_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lis2ds12_func_src_t func_src;
  int32_t ret;
  ret = lis2ds12_read_reg(ctx, LIS2DS12_FUNC_SRC, (uint8_t*)&func_src, 1);
  *val = func_src.sensorhub_end_op;
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
