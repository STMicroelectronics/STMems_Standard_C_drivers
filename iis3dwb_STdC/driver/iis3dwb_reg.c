/*
 ******************************************************************************
 * @file    iis3dwb_reg.c
 * @author  Sensors Software Solution Team
 * @brief   IIS3DWB driver file
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

#include "iis3dwb_reg.h"

/**
  * @defgroup    IIS3DWB
  * @brief       This file provides a set of functions needed to drive the
  *              iis3dwb enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup    IIS3DWB_Interfaces_Functions
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
int32_t iis3dwb_read_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
int32_t iis3dwb_write_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
  * @defgroup    IIS3DWB_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t iis3dwb_from_fs2g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.061f);
}

float_t iis3dwb_from_fs4g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.122f);
}

float_t iis3dwb_from_fs8g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.244f);
}

float_t iis3dwb_from_fs16g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.488f);
}

float_t iis3dwb_from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb / 256.0f) + 25.0f);
}

float_t iis3dwb_from_lsb_to_nsec(int32_t lsb)
{
  return ((float_t)lsb * 25000.0f);
}

/**
  * @}
  *
  */

/**
  * @defgroup   LSM9DS1_Data_generation
  * @brief      This section groups all the functions concerning data
  *             generation
  * @{
  *
  */

/**
  * @brief  Accelerometer full-scale selection[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fs_xl in reg CTRL1_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_full_scale_set(stmdev_ctx_t *ctx,
                                    iis3dwb_fs_xl_t val)
{
  iis3dwb_ctrl1_xl_t ctrl1_xl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL1_XL, (uint8_t*)&ctrl1_xl, 1);
  if(ret == 0){
    ctrl1_xl.fs_xl = (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL1_XL,
                              (uint8_t*)&ctrl1_xl, 1);
  }
  return ret;
}

/**
  * @brief  Accelerometer full-scale selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fs_xl in reg CTRL1_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_full_scale_get(stmdev_ctx_t *ctx,
                                    iis3dwb_fs_xl_t *val)
{
  iis3dwb_ctrl1_xl_t ctrl1_xl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL1_XL, (uint8_t*)&ctrl1_xl, 1);
  switch (ctrl1_xl.fs_xl){
    case IIS3DWB_2g:
      *val = IIS3DWB_2g;
      break;
    case IIS3DWB_16g:
      *val = IIS3DWB_16g;
      break;
    case IIS3DWB_4g:
      *val = IIS3DWB_4g;
      break;
    case IIS3DWB_8g:
      *val = IIS3DWB_8g;
      break;
    default:
      *val = IIS3DWB_2g;
      break;
  }
  return ret;
}

/**
  * @brief  Accelerometer UI data rate selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of xl_en in reg CTRL1_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_data_rate_set(stmdev_ctx_t *ctx,
                                   iis3dwb_odr_xl_t val)
{
  iis3dwb_ctrl1_xl_t ctrl1_xl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL1_XL, (uint8_t*)&ctrl1_xl, 1);
  if(ret == 0){
    ctrl1_xl.xl_en= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL1_XL,
                              (uint8_t*)&ctrl1_xl, 1);
  }
  return ret;
}

/**
  * @brief  Accelerometer UI data rate selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of odr_xl in reg CTRL1_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_data_rate_get(stmdev_ctx_t *ctx,
                                   iis3dwb_odr_xl_t *val)
{
  iis3dwb_ctrl1_xl_t ctrl1_xl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL1_XL, (uint8_t*)&ctrl1_xl, 1);
  switch (ctrl1_xl.xl_en){
    case IIS3DWB_XL_ODR_OFF:
      *val = IIS3DWB_XL_ODR_OFF;
      break;
    case IIS3DWB_XL_ODR_26k7Hz:
      *val = IIS3DWB_XL_ODR_26k7Hz;
      break;
    default:
      *val = IIS3DWB_XL_ODR_OFF;
      break;
  }
  return ret;
}

/**
  * @brief  Block data update.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of bdu in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.bdu= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}

/**
  * @brief  Block data update.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of bdu in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  *val = ctrl3_c.bdu;

  return ret;
}

/**
  * @brief  Weight of XL user offset bits of registers X_OFS_USR (73h),
  *         Y_OFS_USR (74h), Z_OFS_USR (75h).[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of usr_off_w in reg CTRL6_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_offset_weight_set(stmdev_ctx_t *ctx,
                                     iis3dwb_usr_off_w_t val)
{
  iis3dwb_ctrl6_c_t ctrl6_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL6_C, (uint8_t*)&ctrl6_c, 1);
  if(ret == 0){
    ctrl6_c.usr_off_w= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL6_C, (uint8_t*)&ctrl6_c, 1);
  }
  return ret;
}

/**
  * @brief  Weight of XL user offset bits of registers X_OFS_USR (73h),
  *         Y_OFS_USR (74h), Z_OFS_USR (75h).[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of usr_off_w in reg CTRL6_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_offset_weight_get(stmdev_ctx_t *ctx,
                                       iis3dwb_usr_off_w_t *val)
{
  iis3dwb_ctrl6_c_t ctrl6_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL6_C, (uint8_t*)&ctrl6_c, 1);

  switch (ctrl6_c.usr_off_w){
    case IIS3DWB_LSb_1mg:
      *val = IIS3DWB_LSb_1mg;
      break;
    case IIS3DWB_LSb_16mg:
      *val = IIS3DWB_LSb_16mg;
      break;
    default:
      *val = IIS3DWB_LSb_1mg;
      break;
  }
  return ret;
}

/**
  * @brief  select accelerometer axis.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of xl_axis_sel in reg CTRL6_C and
  *                the values of _1ax_to_3regout in reg CTRL4_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_axis_selection_set(stmdev_ctx_t *ctx,
                                     iis3dwb_xl_axis_sel_t val)
{
  iis3dwb_ctrl4_c_t ctrl4_c;
  iis3dwb_ctrl6_c_t ctrl6_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  if(ret == 0){
    ctrl4_c._1ax_to_3regout = ( (uint8_t)val & 0x10U ) >> 4;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  }
  if(ret == 0){
    ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL6_C, (uint8_t*)&ctrl6_c, 1);
  }
  if(ret == 0){
    ctrl6_c.xl_axis_sel = (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL6_C, (uint8_t*)&ctrl6_c, 1);
  }
  return ret;
}

/**
  * @brief  select accelerometer axis.[get] 
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of xl_axis_sel in reg CTRL6_C and
  *                the values of _1ax_to_3regout in reg CTRL4_C.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_axis_selection_get(stmdev_ctx_t *ctx,
                                     iis3dwb_xl_axis_sel_t *val)
{
  iis3dwb_ctrl4_c_t ctrl4_c;
  iis3dwb_ctrl6_c_t ctrl6_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  if(ret == 0){
    ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL6_C, (uint8_t*)&ctrl6_c, 1);
  }

  switch ( ( ctrl4_c._1ax_to_3regout << 4 ) + ctrl6_c.xl_axis_sel ){
    case IIS3DWB_ENABLE_ALL:
      *val = IIS3DWB_ENABLE_ALL;
      break;
    case IIS3DWB_ONLY_X_ON_ONE_OUT_REG:
      *val = IIS3DWB_ONLY_X_ON_ONE_OUT_REG;
      break;
    case IIS3DWB_ONLY_Y_ON_ONE_OUT_REG:
      *val = IIS3DWB_ONLY_Y_ON_ONE_OUT_REG;
      break;
    case IIS3DWB_ONLY_Z_ON_ONE_OUT_REG:
      *val = IIS3DWB_ONLY_Z_ON_ONE_OUT_REG;
      break;
    case IIS3DWB_ONLY_X_ON_ALL_OUT_REG:
      *val = IIS3DWB_ONLY_X_ON_ALL_OUT_REG;
      break;
    case IIS3DWB_ONLY_Y_ON_ALL_OUT_REG:
      *val = IIS3DWB_ONLY_Y_ON_ALL_OUT_REG;
      break;
    case IIS3DWB_ONLY_Z_ON_ALL_OUT_REG:
      *val = IIS3DWB_ONLY_Z_ON_ALL_OUT_REG;
      break;
    default:
      *val = IIS3DWB_ENABLE_ALL;
      break;
  }
  return ret;
}

/**
  * @brief  Read all the interrupt flag of the device.[get]
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get registers ALL_INT_SRC; WAKE_UP_SRC;
  *                              TAP_SRC; D6D_SRC; STATUS_REG;
  *                              EMB_FUNC_STATUS; FSM_STATUS_A/B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_all_sources_get(stmdev_ctx_t *ctx,
                                  iis3dwb_all_sources_t *val)
{
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_ALL_INT_SRC,
                           (uint8_t*)&val->all_int_src, 1);
  if(ret == 0){
    ret = iis3dwb_read_reg(ctx, IIS3DWB_WAKE_UP_SRC,
                             (uint8_t*)&val->wake_up_src, 1);
  }
  if(ret == 0){
  ret = iis3dwb_read_reg(ctx, IIS3DWB_STATUS_REG,
                           (uint8_t*)&val->status_reg, 1);
  }

  return ret;
}

/**
  * @brief  The STATUS_REG register is read by the primary interface.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get register STATUS_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_status_reg_get(stmdev_ctx_t *ctx,
                                 iis3dwb_status_reg_t *val)
{
  int32_t ret;
  ret = iis3dwb_read_reg(ctx, IIS3DWB_STATUS_REG, (uint8_t*) val, 1);
  return ret;
}

/**
  * @brief  Accelerometer new data available.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of xlda in reg STATUS_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_status_reg_t status_reg;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_STATUS_REG,
                           (uint8_t*)&status_reg, 1);
  *val = status_reg.xlda;

  return ret;
}

/**
  * @brief  Temperature new data available.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of tda in reg STATUS_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_temp_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_status_reg_t status_reg;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_STATUS_REG,
                           (uint8_t*)&status_reg, 1);
  *val = status_reg.tda;

  return ret;
}

/**
  * @brief  Accelerometer X-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_usr_offset_x_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis3dwb_write_reg(ctx, IIS3DWB_X_OFS_USR, buff, 1);
  return ret;
}

/**
  * @brief  Accelerometer X-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_usr_offset_x_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis3dwb_read_reg(ctx, IIS3DWB_X_OFS_USR, buff, 1);
  return ret;
}

/**
  * @brief  Accelerometer Y-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_usr_offset_y_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis3dwb_write_reg(ctx, IIS3DWB_Y_OFS_USR, buff, 1);
  return ret;
}

/**
  * @brief  Accelerometer Y-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_usr_offset_y_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis3dwb_read_reg(ctx, IIS3DWB_Y_OFS_USR, buff, 1);
  return ret;
}

/**
  * @brief  Accelerometer Z-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_usr_offset_z_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis3dwb_write_reg(ctx, IIS3DWB_Z_OFS_USR, buff, 1);
  return ret;
}

/**
  * @brief  Accelerometer X-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_usr_offset_z_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis3dwb_read_reg(ctx, IIS3DWB_Z_OFS_USR, buff, 1);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS3DWB_Timestamp
  * @brief      This section groups all the functions that manage the
  *             timestamp generation.
  * @{
  *
  */

/**
  * @brief  Enables timestamp counter.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of timestamp_en in reg CTRL10_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_timestamp_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL10_C, (uint8_t*)&ctrl10_c, 1);
  if(ret == 0){
    ctrl10_c.timestamp_en= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL10_C,
                              (uint8_t*)&ctrl10_c, 1);
  }
  return ret;
}

/**
  * @brief  Enables timestamp counter.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of timestamp_en in reg CTRL10_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_timestamp_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL10_C, (uint8_t*)&ctrl10_c, 1);
  *val = ctrl10_c.timestamp_en;

  return ret;
}

/**
  * @brief  Timestamp first data output register (r).
  *         The value is expressed as a 32-bit word and the bit resolution
  *         is 25 μs.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_timestamp_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis3dwb_read_reg(ctx, IIS3DWB_TIMESTAMP0, buff, 4);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS3DWB_Data output
  * @brief      This section groups all the data output functions.
  * @{
  *
  */

/**
  * @brief  Circular burst-mode (rounding) read of the output registers.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of rounding in reg CTRL5_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_rounding_mode_set(stmdev_ctx_t *ctx,
                                    iis3dwb_rounding_t val)
{
  iis3dwb_ctrl5_c_t ctrl5_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL5_C, (uint8_t*)&ctrl5_c, 1);
  if(ret == 0){
    ctrl5_c.rounding= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL5_C, (uint8_t*)&ctrl5_c, 1);
  }
  return ret;
}

/**
  * @brief  Gyroscope UI chain full-scale selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of rounding in reg CTRL5_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_rounding_mode_get(stmdev_ctx_t *ctx,
                                    iis3dwb_rounding_t *val)
{
  iis3dwb_ctrl5_c_t ctrl5_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL5_C, (uint8_t*)&ctrl5_c, 1);
  switch (ctrl5_c.rounding){
    case IIS3DWB_NO_ROUND:
      *val = IIS3DWB_NO_ROUND;
      break;
    case IIS3DWB_ROUND:
      *val = IIS3DWB_ROUND;
      break;
    default:
      *val = IIS3DWB_NO_ROUND;
      break;
  }
  return ret;
}

/**
  * @brief  Temperature data output register (r).
  *         L and H registers together express a 16-bit word in two’s
  *         complement.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis3dwb_read_reg(ctx, IIS3DWB_OUT_TEMP_L, buff, 2);
  return ret;
}

/**
  * @brief  Linear acceleration output register. The value is expressed as a
  *         16-bit word in two’s complement.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_acceleration_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis3dwb_read_reg(ctx, IIS3DWB_OUTX_L_A, buff, 6);
  return ret;
}

/**
  * @brief  FIFO data output.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_out_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_DATA_OUT_X_L, buff, 6);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS3DWB_common
  * @brief      This section groups common usefull functions.
  * @{
  *
  */

/**
  * @brief  Difference in percentage of the effective ODR (and timestamp rate)
  *         with respect to the typical.[set]
  *         Step:  0.15%. 8-bit format, 2's complement.
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of freq_fine in reg INTERNAL_FREQ_FINE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_odr_cal_reg_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_internal_freq_fine_t internal_freq_fine;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_INTERNAL_FREQ_FINE,
                           (uint8_t*)&internal_freq_fine, 1);
  if(ret == 0){
    internal_freq_fine.freq_fine= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_INTERNAL_FREQ_FINE,
                              (uint8_t*)&internal_freq_fine, 1);
  }
  return ret;
}

/**
  * @brief  Difference in percentage of the effective ODR (and timestamp rate)
  *         with respect to the typical.[get]
  *         Step:  0.15%. 8-bit format, 2's complement.
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of freq_fine in reg INTERNAL_FREQ_FINE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_odr_cal_reg_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_internal_freq_fine_t internal_freq_fine;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_INTERNAL_FREQ_FINE,
                           (uint8_t*)&internal_freq_fine, 1);
  *val = internal_freq_fine.freq_fine;

  return ret;
}

/**
  * @brief  Data-ready pulsed / letched mode.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of dataready_pulsed in
  *                reg COUNTER_BDR_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_data_ready_mode_set(stmdev_ctx_t *ctx,
                                      iis3dwb_dataready_pulsed_t val)
{
  iis3dwb_counter_bdr_reg1_t counter_bdr_reg1;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_COUNTER_BDR_REG1,
                           (uint8_t*)&counter_bdr_reg1, 1);
  if(ret == 0){
    counter_bdr_reg1.dataready_pulsed= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_COUNTER_BDR_REG1,
                              (uint8_t*)&counter_bdr_reg1, 1);
  }
  return ret;
}

/**
  * @brief  Data-ready pulsed / letched mode.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of dataready_pulsed in
  *                reg COUNTER_BDR_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_data_ready_mode_get(stmdev_ctx_t *ctx,
                                      iis3dwb_dataready_pulsed_t *val)
{
  iis3dwb_counter_bdr_reg1_t counter_bdr_reg1;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_COUNTER_BDR_REG1,
                           (uint8_t*)&counter_bdr_reg1, 1);
  switch (counter_bdr_reg1.dataready_pulsed){
    case IIS3DWB_DRDY_LATCHED:
      *val = IIS3DWB_DRDY_LATCHED;
      break;
    case IIS3DWB_DRDY_PULSED:
      *val = IIS3DWB_DRDY_PULSED;
      break;
    default:
      *val = IIS3DWB_DRDY_LATCHED;
      break;
  }
  return ret;
}

/**
  * @brief  Device Who am I.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis3dwb_read_reg(ctx, IIS3DWB_WHO_AM_I, buff, 1);
  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of sw_reset in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_reset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.sw_reset= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of sw_reset in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  *val = ctrl3_c.sw_reset;

  return ret;
}

/**
  * @brief  Register address automatically incremented during a multiple byte
  *         access with a serial interface.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of if_inc in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_auto_increment_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.if_inc= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}

/**
  * @brief  Register address automatically incremented during a multiple byte
  *         access with a serial interface.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of if_inc in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_auto_increment_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  *val = ctrl3_c.if_inc;

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of boot in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_boot_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.boot= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of boot in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_boot_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  *val = ctrl3_c.boot;

  return ret;
}



/**
  * @brief  Linear acceleration sensor self-test enable.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of st_xl in reg CTRL5_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_self_test_set(stmdev_ctx_t *ctx,
                                   iis3dwb_st_xl_t val)
{
  iis3dwb_ctrl5_c_t ctrl5_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL5_C, (uint8_t*)&ctrl5_c, 1);
  if(ret == 0){
    ctrl5_c.st_xl= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL5_C, (uint8_t*)&ctrl5_c, 1);
  }
  return ret;
}

/**
  * @brief  Linear acceleration sensor self-test enable.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of st_xl in reg CTRL5_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_self_test_get(stmdev_ctx_t *ctx,
                                   iis3dwb_st_xl_t *val)
{
  iis3dwb_ctrl5_c_t ctrl5_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL5_C, (uint8_t*)&ctrl5_c, 1);

  switch (ctrl5_c.st_xl){
    case IIS3DWB_XL_ST_DISABLE:
      *val = IIS3DWB_XL_ST_DISABLE;
      break;
    case IIS3DWB_XL_ST_POSITIVE:
      *val = IIS3DWB_XL_ST_POSITIVE;
      break;
    case IIS3DWB_XL_ST_NEGATIVE:
      *val = IIS3DWB_XL_ST_NEGATIVE;
      break;
    default:
      *val = IIS3DWB_XL_ST_DISABLE;
      break;
  }
  return ret;
}



/**
  * @}
  *
  */

/**
  * @defgroup   IIS3DWB_filters
  * @brief      This section group all the functions concerning the
  *             filters configuration
  * @{
  *
  */

/**
  * @brief  Accelerometer output from LPF2 filtering stage selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of lpf2_xl_en in reg CTRL1_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_filter_lp2_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_ctrl1_xl_t ctrl1_xl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL1_XL, (uint8_t*)&ctrl1_xl, 1);
  if(ret == 0){
    ctrl1_xl.lpf2_xl_en= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL1_XL,
                              (uint8_t*)&ctrl1_xl, 1);
  }
  return ret;
}

/**
  * @brief  Accelerometer output from LPF2 filtering stage selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of lpf2_xl_en in reg CTRL1_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_filter_lp2_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_ctrl1_xl_t ctrl1_xl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL1_XL, (uint8_t*)&ctrl1_xl, 1);
  *val = ctrl1_xl.lpf2_xl_en;

  return ret;
}

/**
  * @brief  Mask DRDY on pin (both XL & Gyro) until filter settling ends
  *         (XL and Gyro independently masked).[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of drdy_mask in reg CTRL4_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_filter_settling_mask_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  if(ret == 0){
    ctrl4_c.drdy_mask= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  }
  return ret;
}

/**
  * @brief  Mask DRDY on pin (both XL & Gyro) until filter settling ends
  *         (XL and Gyro independently masked).[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of drdy_mask in reg CTRL4_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_filter_settling_mask_get(stmdev_ctx_t *ctx,
                                           uint8_t *val)
{
  iis3dwb_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  *val = ctrl4_c.drdy_mask;

  return ret;
}

/**
  * @brief  Accelerometer slope filter / high-pass filter selection
  *         on output.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of hp_slope_xl_en in reg CTRL8_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_hp_path_on_out_set(stmdev_ctx_t *ctx,
                                        iis3dwb_hp_slope_xl_en_t val)
{
  iis3dwb_ctrl1_xl_t ctrl1_xl;
  iis3dwb_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL1_XL, (uint8_t*)&ctrl1_xl, 1);
  if(ret == 0){
    ctrl1_xl.lpf2_xl_en = ((uint8_t)val & 0x80U) >> 7;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL1_XL, (uint8_t*)&ctrl1_xl, 1);
  }
  if(ret == 0){
    ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL8_XL, (uint8_t*)&ctrl8_xl, 1);
  }
  if(ret == 0){
    ctrl8_xl.fds = ((uint8_t)val & 0x10U) >> 4;
    ctrl8_xl.hp_ref_mode_xl = ((uint8_t)val & 0x20U) >> 5;
    ctrl8_xl.hpcf_xl = (uint8_t)val & 0x07U;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL8_XL, (uint8_t*)&ctrl8_xl, 1);
  }
  return ret;
}

/**
  * @brief  Accelerometer slope filter / high-pass filter selection on
  *         output.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of hp_slope_xl_en in reg CTRL8_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_hp_path_on_out_get(stmdev_ctx_t *ctx,
                                        iis3dwb_hp_slope_xl_en_t *val)
{
  iis3dwb_ctrl1_xl_t ctrl1_xl;
  iis3dwb_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL1_XL, (uint8_t*)&ctrl1_xl, 1);
  if(ret == 0){
    ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL8_XL, (uint8_t*)&ctrl8_xl, 1);
  }

  switch ( (ctrl1_xl.lpf2_xl_en << 7) + (ctrl8_xl.hp_ref_mode_xl << 5) +
           (ctrl8_xl.fds << 4) + ctrl8_xl.hpcf_xl ){
    case IIS3DWB_SLOPE_ODR_DIV_4:
      *val = IIS3DWB_SLOPE_ODR_DIV_4;
      break;
    case IIS3DWB_HP_ODR_DIV_10:
      *val = IIS3DWB_HP_ODR_DIV_10;
      break;
    case IIS3DWB_HP_ODR_DIV_20:
      *val = IIS3DWB_HP_ODR_DIV_20;
      break;
    case IIS3DWB_HP_ODR_DIV_45:
      *val = IIS3DWB_HP_ODR_DIV_45;
      break;
    case IIS3DWB_HP_ODR_DIV_100:
      *val = IIS3DWB_HP_ODR_DIV_100;
      break;
    case IIS3DWB_HP_ODR_DIV_200:
      *val = IIS3DWB_HP_ODR_DIV_200;
      break;
    case IIS3DWB_HP_ODR_DIV_400:
      *val = IIS3DWB_HP_ODR_DIV_400;
      break;
    case IIS3DWB_HP_ODR_DIV_800:
      *val = IIS3DWB_HP_ODR_DIV_800;
      break;
    case IIS3DWB_LP_ODR_DIV_4:
      *val = IIS3DWB_LP_ODR_DIV_4;
      break;
    case IIS3DWB_LP_5kHz:
      *val = IIS3DWB_LP_5kHz;
      break;
    case IIS3DWB_LP_ODR_DIV_10:
      *val = IIS3DWB_LP_ODR_DIV_10;
      break;
    case IIS3DWB_LP_ODR_DIV_20:
      *val = IIS3DWB_LP_ODR_DIV_20;
      break;
    case IIS3DWB_LP_ODR_DIV_45:
      *val = IIS3DWB_LP_ODR_DIV_45;
      break;
    case IIS3DWB_LP_ODR_DIV_100:
      *val = IIS3DWB_LP_ODR_DIV_100;
      break;
    case IIS3DWB_LP_ODR_DIV_200:
      *val = IIS3DWB_LP_ODR_DIV_200;
      break;
    case IIS3DWB_LP_ODR_DIV_400:
      *val = IIS3DWB_LP_ODR_DIV_400;
      break;
    case IIS3DWB_LP_ODR_DIV_800:
      *val = IIS3DWB_LP_ODR_DIV_800;
      break;
    default:
      *val = IIS3DWB_SLOPE_ODR_DIV_4;
      break;
  }
  return ret;
}

/**
  * @brief  Enables accelerometer LPF2 and HPF fast-settling mode.
  *         The filter sets the second samples after writing this bit.
  *         Active only during device exit from powerdown mode.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fastsettl_mode_xl in reg CTRL8_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_fast_settling_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL8_XL, (uint8_t*)&ctrl8_xl, 1);
  if(ret == 0){
    ctrl8_xl.fastsettl_mode_xl= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL8_XL,
                              (uint8_t*)&ctrl8_xl, 1);
  }
  return ret;
}

/**
  * @brief  Enables accelerometer LPF2 and HPF fast-settling mode.
  *         The filter sets the second samples after writing
  *         this bit. Active only during device exit from powerdown mode.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fastsettl_mode_xl in reg CTRL8_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_fast_settling_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL8_XL, (uint8_t*)&ctrl8_xl, 1);
  *val = ctrl8_xl.fastsettl_mode_xl;

  return ret;
}

/**
  * @brief  HPF or SLOPE filter selection on wake-up and Activity/Inactivity
  *         functions.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of slope_fds in reg TAP_CFG0
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_hp_path_internal_set(stmdev_ctx_t *ctx,
                                          iis3dwb_slope_fds_t val)
{
  iis3dwb_slope_en_t int_cfg0;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_SLOPE_EN, (uint8_t*)&int_cfg0, 1);
  if(ret == 0){
    int_cfg0.slope_fds= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_SLOPE_EN,
                              (uint8_t*)&int_cfg0, 1);
  }
  return ret;
}

/**
  * @brief  HPF or SLOPE filter selection on wake-up and Activity/Inactivity
  *         functions.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of slope_fds in reg TAP_CFG0
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_hp_path_internal_get(stmdev_ctx_t *ctx,
                                          iis3dwb_slope_fds_t *val)
{
  iis3dwb_slope_en_t int_cfg0;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_SLOPE_EN, (uint8_t*)&int_cfg0, 1);
  switch (int_cfg0.slope_fds){
    case IIS3DWB_USE_SLOPE:
      *val = IIS3DWB_USE_SLOPE;
      break;
    case IIS3DWB_USE_HPF:
      *val = IIS3DWB_USE_HPF;
      break;
    default:
      *val = IIS3DWB_USE_SLOPE;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS3DWB_ main_serial_interface
  * @brief      This section groups all the functions concerning main
  *             serial interface management (not auxiliary)
  * @{
  *
  */

/**
  * @brief  Connect/Disconnect SDO/SA0 internal pull-up.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of sdo_pu_en in reg PIN_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_sdo_sa0_mode_set(stmdev_ctx_t *ctx,
                                   iis3dwb_sdo_pu_en_t val)
{
  iis3dwb_pin_ctrl_t pin_ctrl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_PIN_CTRL, (uint8_t*)&pin_ctrl, 1);
  if(ret == 0){
    pin_ctrl.sdo_pu_en= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_PIN_CTRL, (uint8_t*)&pin_ctrl, 1);
  }
  return ret;
}

/**
  * @brief  Connect/Disconnect SDO/SA0 internal pull-up.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of sdo_pu_en in reg PIN_CTRL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_sdo_sa0_mode_get(stmdev_ctx_t *ctx,
                                   iis3dwb_sdo_pu_en_t *val)
{
  iis3dwb_pin_ctrl_t pin_ctrl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_PIN_CTRL, (uint8_t*)&pin_ctrl, 1);

  switch (pin_ctrl.sdo_pu_en){
    case IIS3DWB_PULL_UP_DISC:
      *val = IIS3DWB_PULL_UP_DISC;
      break;
    case IIS3DWB_PULL_UP_CONNECT:
      *val = IIS3DWB_PULL_UP_CONNECT;
      break;
    default:
      *val = IIS3DWB_PULL_UP_DISC;
      break;
  }
  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of sim in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_spi_mode_set(stmdev_ctx_t *ctx, iis3dwb_sim_t val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.sim= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of sim in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_spi_mode_get(stmdev_ctx_t *ctx, iis3dwb_sim_t *val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);

  switch (ctrl3_c.sim){
    case IIS3DWB_SPI_4_WIRE:
      *val = IIS3DWB_SPI_4_WIRE;
      break;
    case IIS3DWB_SPI_3_WIRE:
      *val = IIS3DWB_SPI_3_WIRE;
      break;
    default:
      *val = IIS3DWB_SPI_4_WIRE;
      break;
  }
  return ret;
}

/**
  * @brief  Disable / Enable I2C interface.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of i2c_disable in reg CTRL4_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_i2c_interface_set(stmdev_ctx_t *ctx,
                                    iis3dwb_i2c_disable_t val)
{
  iis3dwb_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  if(ret == 0){
    ctrl4_c.i2c_disable= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  }
  return ret;
}

/**
  * @brief  Disable / Enable I2C interface.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of i2c reg CTRL4_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_i2c_interface_get(stmdev_ctx_t *ctx,
                                    iis3dwb_i2c_disable_t *val)
{
  iis3dwb_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL4_C, (uint8_t*)&ctrl4_c, 1);

  switch (ctrl4_c.i2c_disable){
    case IIS3DWB_I2C_ENABLE:
      *val = IIS3DWB_I2C_ENABLE;
      break;
    case IIS3DWB_I2C_DISABLE:
      *val = IIS3DWB_I2C_DISABLE;
      break;
    default:
      *val = IIS3DWB_I2C_ENABLE;
      break;
  }
  return ret;
}

/**
  * @brief  I3C Enable/Disable communication protocol.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of i3c_disable in reg CTRL9_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_i3c_disable_set(stmdev_ctx_t *ctx, iis3dwb_i3c_disable_t val)
{
  iis3dwb_ctrl9_xl_t ctrl9_xl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL9_XL, (uint8_t*)&ctrl9_xl, 1);
  if(ret == 0){
    ctrl9_xl.i3c_disable = ((uint8_t)val & 0x80U) >> 7;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL9_XL, (uint8_t*)&ctrl9_xl, 1);
  }

  return ret;
}

/**
  * @brief  I3C Enable/Disable communication protocol[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of i3c_disable in reg CTRL9_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_i3c_disable_get(stmdev_ctx_t *ctx, iis3dwb_i3c_disable_t *val)
{
  iis3dwb_ctrl9_xl_t ctrl9_xl;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL9_XL, (uint8_t*)&ctrl9_xl, 1);

  switch (ctrl9_xl.i3c_disable){
    case IIS3DWB_I3C_DISABLE:
      *val = IIS3DWB_I3C_DISABLE;
      break;
    case IIS3DWB_I3C_ENABLE:
      *val = IIS3DWB_I3C_ENABLE;
      break;
    default:
      *val = IIS3DWB_I3C_ENABLE;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS3DWB_interrupt_pins
  * @brief      This section groups all the functions that manage
  *             interrup pins
  * @{
  *
  */

/**
  * @brief   Select the signal that need to route on int1 pad[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure of registers: INT1_CTRL,MD1_CFG,
  *                EMB_FUNC_INT1, FSM_INT1_A, FSM_INT1_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_pin_int1_route_set(stmdev_ctx_t *ctx,
                                     iis3dwb_pin_int1_route_t *val)
{
  int32_t ret;
  
  ret = iis3dwb_write_reg(ctx, IIS3DWB_INT1_CTRL,
                            (uint8_t*)&val->int1_ctrl, 1);
  if(ret == 0){
    ret = iis3dwb_write_reg(ctx, IIS3DWB_MD1_CFG,
                              (uint8_t*)&val->md1_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Select the signal that need to route on int1 pad.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure of registers: INT1_CTRL, MD1_CFG,
  *                EMB_FUNC_INT1, FSM_INT1_A, FSM_INT1_B.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_pin_int1_route_get(stmdev_ctx_t *ctx,
                                     iis3dwb_pin_int1_route_t *val)
{
  int32_t ret;
  
  ret = iis3dwb_read_reg(ctx, IIS3DWB_INT1_CTRL,
                           (uint8_t*)&val->int1_ctrl, 1);
  if(ret == 0){
    ret = iis3dwb_read_reg(ctx, IIS3DWB_MD1_CFG,
                             (uint8_t*)&val->md1_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Select the signal that need to route on int2 pad[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure of registers INT2_CTRL,  MD2_CFG,
  *                EMB_FUNC_INT2, FSM_INT2_A, FSM_INT2_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_pin_int2_route_set(stmdev_ctx_t *ctx,
                                     iis3dwb_pin_int2_route_t *val)
{
  int32_t ret;


  ret = iis3dwb_write_reg(ctx, IIS3DWB_INT2_CTRL,
                            (uint8_t*)&val->int2_ctrl, 1);

  if(ret == 0){
    ret = iis3dwb_write_reg(ctx, IIS3DWB_MD2_CFG,
                              (uint8_t*)&val->md2_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Select the signal that need to route on int2 pad.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure of registers INT2_CTRL,  MD2_CFG,
  *                EMB_FUNC_INT2, FSM_INT2_A, FSM_INT2_B.[get]
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_pin_int2_route_get(stmdev_ctx_t *ctx,
                                     iis3dwb_pin_int2_route_t *val)
{
  int32_t ret;

    ret = iis3dwb_read_reg(ctx, IIS3DWB_INT2_CTRL,
                             (uint8_t*)&val->int2_ctrl, 1);
  if(ret == 0){
    ret = iis3dwb_read_reg(ctx, IIS3DWB_MD2_CFG,
                             (uint8_t*)&val->md2_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of pp_od in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_pin_mode_set(stmdev_ctx_t *ctx, iis3dwb_pp_od_t val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.pp_od= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of pp_od in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_pin_mode_get(stmdev_ctx_t *ctx, iis3dwb_pp_od_t *val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);

  switch (ctrl3_c.pp_od){
    case IIS3DWB_PUSH_PULL:
      *val = IIS3DWB_PUSH_PULL;
      break;
    case IIS3DWB_OPEN_DRAIN:
      *val = IIS3DWB_OPEN_DRAIN;
      break;
    default:
      *val = IIS3DWB_PUSH_PULL;
      break;
  }
  return ret;
}

/**
  * @brief  Interrupt active-high/low.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of h_lactive in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_pin_polarity_set(stmdev_ctx_t *ctx,
                                   iis3dwb_h_lactive_t val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.h_lactive= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}

/**
  * @brief  Interrupt active-high/low.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of h_lactive in reg CTRL3_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_pin_polarity_get(stmdev_ctx_t *ctx,
                                   iis3dwb_h_lactive_t *val)
{
  iis3dwb_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL3_C, (uint8_t*)&ctrl3_c, 1);

  switch (ctrl3_c.h_lactive){
    case IIS3DWB_ACTIVE_HIGH:
      *val = IIS3DWB_ACTIVE_HIGH;
      break;
    case IIS3DWB_ACTIVE_LOW:
      *val = IIS3DWB_ACTIVE_LOW;
      break;
    default:
      *val = IIS3DWB_ACTIVE_HIGH;
      break;
  }
  return ret;
}

/**
  * @brief  All interrupt signals become available on INT1 pin.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of int2_on_int1 in reg CTRL4_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_all_on_int1_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  if(ret == 0){
    ctrl4_c.int2_on_int1= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  }
  return ret;
}

/**
  * @brief  All interrupt signals become available on INT1 pin.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of int2_on_int1 in reg CTRL4_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_all_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  *val = ctrl4_c.int2_on_int1;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS3DWB_Wake_Up_event
  * @brief      This section groups all the functions that manage the
  *             Wake Up event generation.
  * @{
  *
  */

/**
  * @brief  Weight of 1 LSB of wakeup threshold.[set]
  *         0: 1 LSB =FS_XL  /  64
  *         1: 1 LSB = FS_XL / 256
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of wake_ths_w in reg WAKE_UP_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_wkup_ths_weight_set(stmdev_ctx_t *ctx,
                                      iis3dwb_wake_ths_w_t val)
{
  iis3dwb_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_WAKE_UP_DUR,
                           (uint8_t*)&wake_up_dur, 1);
  if(ret == 0){
    wake_up_dur.wake_ths_w= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_WAKE_UP_DUR,
                              (uint8_t*)&wake_up_dur, 1);
  }
  return ret;
}

/**
  * @brief  Weight of 1 LSB of wakeup threshold.[get]
  *         0: 1 LSB =FS_XL  /  64
  *         1: 1 LSB = FS_XL / 256
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of wake_ths_w in reg WAKE_UP_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_wkup_ths_weight_get(stmdev_ctx_t *ctx,
                                      iis3dwb_wake_ths_w_t *val)
{
  iis3dwb_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_WAKE_UP_DUR,
                           (uint8_t*)&wake_up_dur, 1);

  switch (wake_up_dur.wake_ths_w){
    case IIS3DWB_LSb_FS_DIV_64:
      *val = IIS3DWB_LSb_FS_DIV_64;
      break;
    case IIS3DWB_LSb_FS_DIV_256:
      *val = IIS3DWB_LSb_FS_DIV_256;
      break;
    default:
      *val = IIS3DWB_LSb_FS_DIV_64;
      break;
  }
  return ret;
}

/**
  * @brief  Threshold for wakeup: 1 LSB weight depends on WAKE_THS_W in
  *         WAKE_UP_DUR.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of wk_ths in reg WAKE_UP_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_wkup_threshold_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_WAKE_UP_THS,
                           (uint8_t*)&wake_up_ths, 1);
  if(ret == 0){
    wake_up_ths.wk_ths= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_WAKE_UP_THS,
                              (uint8_t*)&wake_up_ths, 1);
  }
  return ret;
}

/**
  * @brief  Threshold for wakeup: 1 LSB weight depends on WAKE_THS_W in
  *         WAKE_UP_DUR.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of wk_ths in reg WAKE_UP_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_wkup_threshold_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_WAKE_UP_THS,
                           (uint8_t*)&wake_up_ths, 1);
  *val = wake_up_ths.wk_ths;

  return ret;
}

/**
  * @brief  Wake up duration event( 1LSb = 1 / ODR ).[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of usr_off_on_wu in reg WAKE_UP_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_usr_offset_on_wkup_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_WAKE_UP_THS,
                           (uint8_t*)&wake_up_ths, 1);
  if(ret == 0){
    wake_up_ths.usr_off_on_wu= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_WAKE_UP_THS,
                              (uint8_t*)&wake_up_ths, 1);
  }
  return ret;
}

/**
  * @brief  Wake up duration event( 1LSb = 1 / ODR ).[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of usr_off_on_wu in reg WAKE_UP_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_xl_usr_offset_on_wkup_get(stmdev_ctx_t *ctx,
                                            uint8_t *val)
{
  iis3dwb_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_WAKE_UP_THS,
                           (uint8_t*)&wake_up_ths, 1);
  *val = wake_up_ths.usr_off_on_wu;

  return ret;
}

/**
  * @brief  Wake up duration event(1LSb = 1 / ODR).[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of wake_dur in reg WAKE_UP_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_wkup_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_WAKE_UP_DUR,
                           (uint8_t*)&wake_up_dur, 1);
  if(ret == 0){
    wake_up_dur.wake_dur= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_WAKE_UP_DUR,
                              (uint8_t*)&wake_up_dur, 1);
  }
  return ret;
}

/**
  * @brief  Wake up duration event(1LSb = 1 / ODR).[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of wake_dur in reg WAKE_UP_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_wkup_dur_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_WAKE_UP_DUR,
                           (uint8_t*)&wake_up_dur, 1);
  *val = wake_up_dur.wake_dur;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS3DWB_ Activity/Inactivity_detection
  * @brief      This section groups all the functions concerning
  *             activity/inactivity detection.
  * @{
  *
  */

/**
  * @brief  Duration to go in sleep mode (1 LSb = 512 / ODR).[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of sleep_dur in reg WAKE_UP_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_act_sleep_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_WAKE_UP_DUR,
                           (uint8_t*)&wake_up_dur, 1);
  if(ret == 0){
    wake_up_dur.sleep_dur= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_WAKE_UP_DUR,
                              (uint8_t*)&wake_up_dur, 1);
  }
  return ret;
}

/**
  * @brief  Duration to go in sleep mode.(1 LSb = 512 / ODR).[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of sleep_dur in reg WAKE_UP_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_act_sleep_dur_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_WAKE_UP_DUR,
                           (uint8_t*)&wake_up_dur, 1);
  *val = wake_up_dur.sleep_dur;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS3DWB_fifo
  * @brief      This section group all the functions concerning
  *             the fifo usage
  * @{
  *
  */

/**
  * @brief  FIFO watermark level selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of wtm in reg FIFO_CTRL1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_watermark_set(stmdev_ctx_t *ctx, uint16_t val)
{
  iis3dwb_fifo_ctrl1_t fifo_ctrl1;
  iis3dwb_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_CTRL2,
                           (uint8_t*)&fifo_ctrl2, 1);
  if(ret == 0){
    fifo_ctrl1.wtm = (uint8_t)(0x00FFU & val);
    ret = iis3dwb_write_reg(ctx, IIS3DWB_FIFO_CTRL1,
                              (uint8_t*)&fifo_ctrl1, 1);
  }
  if(ret == 0){
    fifo_ctrl2.wtm = (uint8_t)(( 0x0100U & val ) >> 8);
    ret = iis3dwb_write_reg(ctx, IIS3DWB_FIFO_CTRL2,
                              (uint8_t*)&fifo_ctrl2, 1);
  }
  return ret;
}

/**
  * @brief  FIFO watermark level selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of wtm in reg FIFO_CTRL1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_watermark_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  iis3dwb_fifo_ctrl1_t fifo_ctrl1;
  iis3dwb_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_CTRL2,
                           (uint8_t*)&fifo_ctrl2, 1);
  if(ret == 0){
    ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_CTRL1,
                             (uint8_t*)&fifo_ctrl1, 1);
  }
  *val = fifo_ctrl2.wtm;
  *val = *val << 8;
  *val += fifo_ctrl1.wtm;
  return ret;
}

/**
  * @brief  Sensing chain FIFO stop values memorization at threshold
  *         level.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of stop_on_wtm in reg FIFO_CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_stop_on_wtm_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_CTRL2,
                           (uint8_t*)&fifo_ctrl2, 1);
  if(ret == 0){
    fifo_ctrl2.stop_on_wtm= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_FIFO_CTRL2,
                              (uint8_t*)&fifo_ctrl2, 1);
  }
  return ret;
}

/**
  * @brief  Sensing chain FIFO stop values memorization at threshold
  *         level.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of stop_on_wtm in reg FIFO_CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_stop_on_wtm_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_CTRL2,
                           (uint8_t*)&fifo_ctrl2, 1);
  *val = fifo_ctrl2.stop_on_wtm;

  return ret;
}

/**
  * @brief  Selects Batching Data Rate (writing frequency in FIFO)
  *         for accelerometer data.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of bdr_xl in reg FIFO_CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_xl_batch_set(stmdev_ctx_t *ctx,
                                    iis3dwb_bdr_xl_t val)
{
  iis3dwb_fifo_ctrl3_t fifo_ctrl3;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_CTRL3,
                           (uint8_t*)&fifo_ctrl3, 1);
  if(ret == 0){
    fifo_ctrl3.bdr_xl= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_FIFO_CTRL3,
                              (uint8_t*)&fifo_ctrl3, 1);
  }
  return ret;
}

/**
  * @brief  Selects Batching Data Rate (writing frequency in FIFO)
  *         for accelerometer data.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of bdr_xl in reg FIFO_CTRL3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_xl_batch_get(stmdev_ctx_t *ctx,
                                    iis3dwb_bdr_xl_t *val)
{
  iis3dwb_fifo_ctrl3_t fifo_ctrl3;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_CTRL3,
                           (uint8_t*)&fifo_ctrl3, 1);

  switch (fifo_ctrl3.bdr_xl){
    case IIS3DWB_XL_NOT_BATCHED:
      *val = IIS3DWB_XL_NOT_BATCHED;
      break;
    case IIS3DWB_XL_BATCHED_AT_26k7Hz:
      *val = IIS3DWB_XL_BATCHED_AT_26k7Hz;
      break;
    default:
      *val = IIS3DWB_XL_NOT_BATCHED;
      break;
  }
  return ret;
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fifo_mode in reg FIFO_CTRL4
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_mode_set(stmdev_ctx_t *ctx,
                                iis3dwb_fifo_mode_t val)
{
  iis3dwb_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_CTRL4,
                           (uint8_t*)&fifo_ctrl4, 1);
  if(ret == 0){
    fifo_ctrl4.fifo_mode= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_FIFO_CTRL4,
                              (uint8_t*)&fifo_ctrl4, 1);
  }
  return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fifo_mode in reg FIFO_CTRL4
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_mode_get(stmdev_ctx_t *ctx,
                                iis3dwb_fifo_mode_t *val)
{
  iis3dwb_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_CTRL4,
                           (uint8_t*)&fifo_ctrl4, 1);

  switch (fifo_ctrl4.fifo_mode){
    case IIS3DWB_BYPASS_MODE:
      *val = IIS3DWB_BYPASS_MODE;
      break;
    case IIS3DWB_FIFO_MODE:
      *val = IIS3DWB_FIFO_MODE;
      break;
    case IIS3DWB_STREAM_TO_FIFO_MODE:
      *val = IIS3DWB_STREAM_TO_FIFO_MODE;
      break;
    case IIS3DWB_BYPASS_TO_STREAM_MODE:
      *val = IIS3DWB_BYPASS_TO_STREAM_MODE;
      break;
    case IIS3DWB_STREAM_MODE:
      *val = IIS3DWB_STREAM_MODE;
      break;
    case IIS3DWB_BYPASS_TO_FIFO_MODE:
      *val = IIS3DWB_BYPASS_TO_FIFO_MODE;
      break;
    default:
      *val = IIS3DWB_BYPASS_MODE;
      break;
  }
  return ret;
}

/**
  * @brief  Selects Batching Data Rate (writing frequency in FIFO)
  *         for temperature data.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of odr_t_batch in reg FIFO_CTRL4
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_temp_batch_set(stmdev_ctx_t *ctx,
                                      iis3dwb_odr_t_batch_t val)
{
  iis3dwb_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_CTRL4,
                           (uint8_t*)&fifo_ctrl4, 1);
  if(ret == 0){
    fifo_ctrl4.odr_t_batch= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_FIFO_CTRL4,
                              (uint8_t*)&fifo_ctrl4, 1);
  }
  return ret;
}

/**
  * @brief  Selects Batching Data Rate (writing frequency in FIFO)
  *         for temperature data.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of odr_t_batch in reg FIFO_CTRL4
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_temp_batch_get(stmdev_ctx_t *ctx,
                                      iis3dwb_odr_t_batch_t *val)
{
  iis3dwb_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_CTRL4,
                           (uint8_t*)&fifo_ctrl4, 1);

  switch (fifo_ctrl4.odr_t_batch){
    case IIS3DWB_TEMP_NOT_BATCHED:
      *val = IIS3DWB_TEMP_NOT_BATCHED;
      break;
    case IIS3DWB_TEMP_BATCHED_AT_104Hz:
      *val = IIS3DWB_TEMP_BATCHED_AT_104Hz;
      break;
    default:
      *val = IIS3DWB_TEMP_NOT_BATCHED;
      break;
  }
  return ret;
}

/**
  * @brief  Selects decimation for timestamp batching in FIFO.
  *         Writing rate will be the maximum rate between XL and
  *         GYRO BDR divided by decimation decoder.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of odr_ts_batch in reg FIFO_CTRL4
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_timestamp_decimation_set(stmdev_ctx_t *ctx,
                                                iis3dwb_odr_ts_batch_t val)
{
  iis3dwb_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_CTRL4,
                           (uint8_t*)&fifo_ctrl4, 1);
  if(ret == 0){
    fifo_ctrl4.odr_ts_batch= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_FIFO_CTRL4,
                              (uint8_t*)&fifo_ctrl4, 1);
  }
  return ret;
}

/**
  * @brief  Selects decimation for timestamp batching in FIFO.
  *         Writing rate will be the maximum rate between XL and
  *         GYRO BDR divided by decimation decoder.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of odr_ts_batch in reg
  *                                 FIFO_CTRL4
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_timestamp_decimation_get(stmdev_ctx_t *ctx,
                                                iis3dwb_odr_ts_batch_t *val)
{
  iis3dwb_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_CTRL4,
                           (uint8_t*)&fifo_ctrl4, 1);

  switch (fifo_ctrl4.odr_ts_batch){
    case IIS3DWB_NO_DECIMATION:
      *val = IIS3DWB_NO_DECIMATION;
      break;
    case IIS3DWB_DEC_1:
      *val = IIS3DWB_DEC_1;
      break;
    case IIS3DWB_DEC_8:
      *val = IIS3DWB_DEC_8;
      break;
    case IIS3DWB_DEC_32:
      *val = IIS3DWB_DEC_32;
      break;
    default:
      *val = IIS3DWB_NO_DECIMATION;
      break;
  }
  return ret;
}

/**
  * @brief  Resets the internal counter of batching events for a single sensor.
  *         This bit is automatically reset to zero if it was set to ‘1’.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of rst_counter_bdr in reg COUNTER_BDR_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_rst_batch_counter_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb_counter_bdr_reg1_t counter_bdr_reg1;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_COUNTER_BDR_REG1,
                           (uint8_t*)&counter_bdr_reg1, 1);
  if(ret == 0){
    counter_bdr_reg1.rst_counter_bdr= (uint8_t)val;
    ret = iis3dwb_write_reg(ctx, IIS3DWB_COUNTER_BDR_REG1,
                              (uint8_t*)&counter_bdr_reg1, 1);
  }
  return ret;
}

/**
  * @brief  Resets the internal counter of batching events for a single sensor.
  *         This bit is automatically reset to zero if it was set to ‘1’.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of rst_counter_bdr in reg COUNTER_BDR_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_rst_batch_counter_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_counter_bdr_reg1_t counter_bdr_reg1;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_COUNTER_BDR_REG1,
                           (uint8_t*)&counter_bdr_reg1, 1);
  *val = counter_bdr_reg1.rst_counter_bdr;

  return ret;
}

/**
  * @brief  Batch data rate counter.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of cnt_bdr_th in reg COUNTER_BDR_REG2
  *                and COUNTER_BDR_REG1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_batch_counter_threshold_set(stmdev_ctx_t *ctx,
                                              uint16_t val)
{
  iis3dwb_counter_bdr_reg2_t counter_bdr_reg1;
  iis3dwb_counter_bdr_reg2_t counter_bdr_reg2;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_COUNTER_BDR_REG1,
                           (uint8_t*)&counter_bdr_reg1, 1);
  if (ret == 0){
    counter_bdr_reg1.cnt_bdr_th = (uint8_t)((0x0700U & val) >> 8);
    ret = iis3dwb_write_reg(ctx, IIS3DWB_COUNTER_BDR_REG1, (uint8_t*)&counter_bdr_reg1, 1);
  }
  if (ret == 0){
    counter_bdr_reg2.cnt_bdr_th = (uint8_t)(0x00FFU & val);
    ret = iis3dwb_write_reg(ctx, IIS3DWB_COUNTER_BDR_REG2,
                              (uint8_t*)&counter_bdr_reg2, 1);
  }
  return ret;
}

/**
  * @brief  Batch data rate counter.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of cnt_bdr_th in reg COUNTER_BDR_REG2
  *                and COUNTER_BDR_REG1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_batch_counter_threshold_get(stmdev_ctx_t *ctx,
                                              uint16_t *val)
{
  iis3dwb_counter_bdr_reg1_t counter_bdr_reg1;
  iis3dwb_counter_bdr_reg2_t counter_bdr_reg2;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_COUNTER_BDR_REG1,
                           (uint8_t*)&counter_bdr_reg1, 1);
  if (ret == 0){
    ret = iis3dwb_read_reg(ctx, IIS3DWB_COUNTER_BDR_REG2,
                             (uint8_t*)&counter_bdr_reg2, 1);
  }

  *val = counter_bdr_reg1.cnt_bdr_th;
  *val = *val << 8;
  *val += counter_bdr_reg2.cnt_bdr_th;
  return ret;
}

/**
  * @brief  Number of unread sensor data (TAG + 6 bytes) stored in FIFO.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of diff_fifo in reg FIFO_STATUS1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_data_level_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  iis3dwb_fifo_status1_t fifo_status1;
  iis3dwb_fifo_status2_t fifo_status2;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_STATUS1,
                           (uint8_t*)&fifo_status1, 1);
  if (ret == 0){
    ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_STATUS2,
                             (uint8_t*)&fifo_status2, 1);
    *val = fifo_status2.diff_fifo;
    *val = *val << 8;
    *val += fifo_status1.diff_fifo;
  }
  return ret;
}

/**
  * @brief  Smart FIFO status.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Registers FIFO_STATUS2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_status_get(stmdev_ctx_t *ctx,
                                  iis3dwb_fifo_status2_t *val)
{
  int32_t ret;
  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_STATUS2, (uint8_t*)val, 1);
  return ret;
}

/**
  * @brief  Smart FIFO full status.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fifo_full_ia in reg FIFO_STATUS2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_full_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_fifo_status2_t fifo_status2;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_STATUS2,
                           (uint8_t*)&fifo_status2, 1);
  *val = fifo_status2.fifo_full_ia;

  return ret;
}

/**
  * @brief  FIFO overrun status.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of  fifo_over_run_latched in
  *                reg FIFO_STATUS2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_fifo_status2_t fifo_status2;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_STATUS2,
                           (uint8_t*)&fifo_status2, 1);
  *val = fifo_status2. fifo_ovr_ia;

  return ret;
}

/**
  * @brief  FIFO watermark status.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fifo_wtm_ia in reg FIFO_STATUS2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb_fifo_status2_t fifo_status2;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_STATUS2,
                           (uint8_t*)&fifo_status2, 1);
  *val = fifo_status2.fifo_wtm_ia;

  return ret;
}

/**
  * @brief  Identifies the sensor in FIFO_DATA_OUT.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of tag_sensor in reg FIFO_DATA_OUT_TAG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb_fifo_sensor_tag_get(stmdev_ctx_t *ctx,
                                      iis3dwb_fifo_tag_t *val)
{
  iis3dwb_fifo_data_out_tag_t fifo_data_out_tag;
  int32_t ret;

  ret = iis3dwb_read_reg(ctx, IIS3DWB_FIFO_DATA_OUT_TAG,
                           (uint8_t*)&fifo_data_out_tag, 1);

  switch (fifo_data_out_tag.tag_sensor){
    case IIS3DWB_XL_NC_TAG:
      *val = IIS3DWB_XL_NC_TAG;
      break;
    case IIS3DWB_TEMPERATURE_TAG:
      *val = IIS3DWB_TEMPERATURE_TAG;
      break;
    case IIS3DWB_TIMESTAMP_TAG:
      *val = IIS3DWB_TIMESTAMP_TAG;
      break;
    case IIS3DWB_CFG_CHANGE_TAG:
      *val = IIS3DWB_CFG_CHANGE_TAG;
      break;
    default:
      *val = IIS3DWB_CFG_CHANGE_TAG;
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
