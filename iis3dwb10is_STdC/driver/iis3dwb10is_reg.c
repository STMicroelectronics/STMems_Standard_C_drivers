/*
 ******************************************************************************
 * @file    iis3dwb10is_reg.c
 * @author  Sensors Software Solution Team
 * @brief   IIS3DWB10IS driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "iis3dwb10is_reg.h"

/**
  * @defgroup    IIS3DWB10IS
  * @brief       This file provides a set of functions needed to drive the
  *              iis3dwb10is sensor.
  * @{
  *
  */

/**
  * @defgroup    IIS3DWB10IS_Interfaces_Functions
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
int32_t __weak iis3dwb10is_read_reg(const stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
                                    uint16_t len)
{
  if (ctx == NULL) { return -1; }

  return ctx->read_reg(ctx->handle, reg, data, len);
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
int32_t __weak iis3dwb10is_write_reg(const stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
                                     uint16_t len)
{
  if (ctx == NULL) { return -1; }

  return ctx->write_reg(ctx->handle, reg, data, len);
}

/**
  * @}
  *
  */
uint64_t iis3dwb10is_from_lsb_to_us(uint64_t lsb)
{
  return (lsb * 25U);
}

float_t iis3dwb10is_from_lsb_to_celsius(int16_t lsb)
{
  return ((float_t)lsb / 200.0f);
}

float_t iis3dwb10is_16b_from_fs50g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.38f);
}

float_t iis3dwb10is_16b_from_fs100g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.764f);
}

float_t iis3dwb10is_16b_from_fs200g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 1.524f);
}

float_t iis3dwb10is_from_fs50g_to_mg(int32_t lsb)
{
  return ((float_t)lsb * 0.095f);
}

float_t iis3dwb10is_from_fs100g_to_mg(int32_t lsb)
{
  return ((float_t)lsb * 0.191f);
}

float_t iis3dwb10is_from_fs200g_to_mg(int32_t lsb)
{
  return ((float_t)lsb * 0.381f);
}

/**
  * @brief  Device Who am I.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  id     Pointer to id buffer
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb10is_device_id_get(const stmdev_ctx_t *ctx, uint8_t *id)
{
  return iis3dwb10is_read_reg(ctx, IIS3DWB10IS_WHO_AM_I, id, 1);
}

/**
  * @brief  Change memory bank.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      MAIN_MEM_BANK, ISPU_MEM_BANK,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_mem_bank_set(const stmdev_ctx_t *ctx, iis3dwb10is_mem_bank_t val)
{
  iis3dwb10is_ram_access_t ram_access;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_RAM_ACCESS, (uint8_t *)&ram_access, 1);
  ram_access.page_sel = (val == IIS3DWB10IS_ISPU_MEM_BANK) ? 0x1U : 0x0U;
  ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_RAM_ACCESS, (uint8_t *)&ram_access, 1);

  return ret;
}

/**
  * @brief  Change memory bank.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Pointer to MAIN_MEM_BANK, ISPU_MEM_BANK,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_mem_bank_get(const stmdev_ctx_t *ctx, iis3dwb10is_mem_bank_t *val)
{
  iis3dwb10is_ram_access_t ram_access;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_RAM_ACCESS, (uint8_t *)&ram_access, 1);

  if (ram_access.page_sel == 1U)
  {
    *val = IIS3DWB10IS_ISPU_MEM_BANK;
  }
  else
  {
    *val = IIS3DWB10IS_MAIN_MEM_BANK;
  }

  return ret;
}

/**
  * @brief  reset device.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      enum iis3dwb10is_reset_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_reset_set(const stmdev_ctx_t *ctx, iis3dwb10is_reset_t val)
{
  iis3dwb10is_ctrl3_t ctrl3;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL3, (uint8_t *)&ctrl3, 1);
  ctrl3.sw_reset = (uint8_t)val.sw_rst;
  ctrl3.boot = (uint8_t)val.boot;
  ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_CTRL3, (uint8_t *)&ctrl3, 1);

  return ret;
}

/**
  * @brief  reset device.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      pointer to enum iis3dwb10is_reset_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_reset_get(const stmdev_ctx_t *ctx, iis3dwb10is_reset_t *val)
{
  iis3dwb10is_ctrl3_t ctrl3;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL3, (uint8_t *)&ctrl3, 1);
  val->sw_rst = ctrl3.sw_reset;
  val->boot = ctrl3.boot;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @brief  ODR and burst values.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      enum iis3dwb10is_data_rate_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_xl_data_rate_set(const stmdev_ctx_t *ctx, iis3dwb10is_data_rate_t val)
{
  iis3dwb10is_ctrl1_t ctrl1;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL1, (uint8_t *)&ctrl1, 1);

  /* write burst when odr is 0 */
  if (val.burst != IIS3DWB10IS_CONTINUOS_MODE)
  {
    ctrl1.burst_cfg = (uint8_t)val.burst;
    ctrl1.odr_xl = 0U;
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_CTRL1, (uint8_t *)&ctrl1, 1);
  }

  ctrl1.odr_xl = (uint8_t)val.odr;
  ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_CTRL1, (uint8_t *)&ctrl1, 1);

  return ret;
}

/**
  * @brief  ODR and burst values.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      pointer to enum iis3dwb10is_data_rate_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_xl_data_rate_get(const stmdev_ctx_t *ctx, iis3dwb10is_data_rate_t *val)
{
  iis3dwb10is_ctrl1_t ctrl1;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL1, (uint8_t *)&ctrl1, 1);

  switch (ctrl1.burst_cfg)
  {
    case IIS3DWB10IS_CONTINUOS_MODE:
      val->burst = IIS3DWB10IS_CONTINUOS_MODE;
      break;

    case IIS3DWB10IS_TON_UI_TOFF_ISPU:
      val->burst = IIS3DWB10IS_TON_UI_TOFF_ISPU;
      break;

    case IIS3DWB10IS_TON_UI_TOFF_FIFO:
      val->burst = IIS3DWB10IS_TON_UI_TOFF_FIFO;
      break;

    case IIS3DWB10IS_TON_STC_TOFF_UI:
      val->burst = IIS3DWB10IS_TON_STC_TOFF_UI;
      break;

    case IIS3DWB10IS_TON_STC_TOFF_ISPU:
      val->burst = IIS3DWB10IS_TON_STC_TOFF_ISPU;
      break;

    case IIS3DWB10IS_TON_STC_TOFF_FIFO:
      val->burst = IIS3DWB10IS_TON_STC_TOFF_FIFO;
      break;

    case IIS3DWB10IS_TON_EXT_TOFF_UI:
      val->burst = IIS3DWB10IS_TON_EXT_TOFF_UI;
      break;

    case IIS3DWB10IS_TON_EXT_TOFF_ISPU:
      val->burst = IIS3DWB10IS_TON_EXT_TOFF_ISPU;
      break;

    case IIS3DWB10IS_TON_EXT_TOFF_FIFO:
      val->burst = IIS3DWB10IS_TON_EXT_TOFF_FIFO;
      break;

    default:
      val->burst = IIS3DWB10IS_CONTINUOS_MODE;
      break;
  }

  switch(ctrl1.odr_xl)
  {
    case IIS3DWB10IS_ODR_IDLE:
      val->odr = IIS3DWB10IS_ODR_IDLE;
      break;

    case IIS3DWB10IS_ODR_2KHz5:
      val->odr = IIS3DWB10IS_ODR_2KHz5;
      break;

    case IIS3DWB10IS_ODR_5KHz:
      val->odr = IIS3DWB10IS_ODR_5KHz;
      break;

    case IIS3DWB10IS_ODR_10KHz:
      val->odr = IIS3DWB10IS_ODR_10KHz;
      break;

    case IIS3DWB10IS_ODR_20KHz:
      val->odr = IIS3DWB10IS_ODR_20KHz;
      break;

    case IIS3DWB10IS_ODR_40KHz:
      val->odr = IIS3DWB10IS_ODR_40KHz;
      break;

    default:
      val->odr = IIS3DWB10IS_ODR_IDLE;
      break;
  }

  return ret;
}

/**
  * @brief  ODR and burst values.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      enum iis3dwb10is_data_rate_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_xl_full_scale_set(const stmdev_ctx_t *ctx, iis3dwb10is_fs_xl_t val)
{
  iis3dwb10is_ctrl2_t ctrl2;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL2, (uint8_t *)&ctrl2, 1);
  ctrl2.fs = (uint8_t)val;
  ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_CTRL2, (uint8_t *)&ctrl2, 1);

  return ret;
}

/**
  * @brief  ODR and burst values.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      enum iis3dwb10is_data_rate_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_xl_full_scale_get(const stmdev_ctx_t *ctx, iis3dwb10is_fs_xl_t *val)
{
  iis3dwb10is_ctrl2_t ctrl2;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL2, (uint8_t *)&ctrl2, 1);
  switch (ctrl2.fs)
  {
    case IIS3DWB_50g:
      *val = IIS3DWB_50g;
      break;

    case IIS3DWB_100g:
      *val = IIS3DWB_100g;
      break;

    case IIS3DWB_200g:
      *val = IIS3DWB_200g;
      break;

    default:
      *val = IIS3DWB_50g;
      break;
  }

  return ret;
}

/**
  * @brief  Block data update.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      0: bdu off, 1: bdu on
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_block_data_update_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb10is_ctrl3_t ctrl3;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL3, (uint8_t *)&ctrl3, 1);
  ctrl3.bdu = (uint8_t)val;
  ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_CTRL3, (uint8_t *)&ctrl3, 1);

  return ret;
}

/**
  * @brief  Block data update.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      0: bdu off, 1: bdu on
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_block_data_update_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb10is_ctrl3_t ctrl3;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL3, (uint8_t *)&ctrl3, 1);
  *val = ctrl3.bdu;

  return ret;
}

/**
  * @brief  XL axis configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_xl_data_cfg_t enum
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_xl_data_config_set(const stmdev_ctx_t *ctx, iis3dwb10is_xl_data_cfg_t val)
{
  iis3dwb10is_ctrl4_t ctrl4;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL4, (uint8_t *)&ctrl4, 1);
  ctrl4.x_axis_enable_us = val.x_axis_en;
  ctrl4.y_axis_enable_us = val.y_axis_en;
  ctrl4.z_axis_enable_us = val.z_axis_en;
  ctrl4.rounding = (uint8_t)val.rounding;
  ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_CTRL4, (uint8_t *)&ctrl4, 1);

  return ret;
}

/**
  * @brief  XL axis configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      pointer to iis3dwb10is_xl_data_cfg_t enum
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_xl_data_config_get(const stmdev_ctx_t *ctx, iis3dwb10is_xl_data_cfg_t *val)
{
  iis3dwb10is_ctrl4_t ctrl4;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL4, (uint8_t *)&ctrl4, 1);
  val->x_axis_en = ctrl4.x_axis_enable_us;
  val->y_axis_en = ctrl4.y_axis_enable_us;
  val->z_axis_en = ctrl4.z_axis_enable_us;

  switch(ctrl4.rounding)
  {
    case IIS3DWB10IS_WRAPAROUND_DISABLED:
      val->rounding = IIS3DWB10IS_WRAPAROUND_DISABLED;
      break;

    case IIS3DWB10IS_WRAPAROUND_1_EN:
      val->rounding = IIS3DWB10IS_WRAPAROUND_1_EN;
      break;

    case IIS3DWB10IS_WRAPAROUND_2_EN:
      val->rounding = IIS3DWB10IS_WRAPAROUND_2_EN;
      break;

    default:
      val->rounding = IIS3DWB10IS_WRAPAROUND_DISABLED;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @brief  Configure INT1 and INT2 pins.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      enum iis3dwb10is_int_pin_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_interrupt_pin_mode_set(const stmdev_ctx_t *ctx, iis3dwb10is_int_pin_t val)
{
  iis3dwb10is_pad_ctrl_t pad;
  iis3dwb10is_if_cfg_t if_cfg;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PAD_CTRL, (uint8_t *)&pad, 1);
  pad.io_pad_strength = (uint8_t)val.strength;
  pad.pd_dis_int = (uint8_t)val.pd_dis;
  ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_PAD_CTRL, (uint8_t *)&pad, 1);
  if (ret != 0) { return -1; }

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_IF_CFG, (uint8_t *)&if_cfg, 1);
  if_cfg.int_pp_od = (uint8_t)val.pp_od;
  if_cfg.int_active_level = val.int_active_level;
  ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_IF_CFG, (uint8_t *)&if_cfg, 1);


  return ret;
}

/**
  * @brief  Configure INT1 and INT2 pins.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      pointer to enum iis3dwb10is_int_pin_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_interrupt_pin_mode_get(const stmdev_ctx_t *ctx, iis3dwb10is_int_pin_t *val)
{
  iis3dwb10is_pad_ctrl_t pad;
  iis3dwb10is_if_cfg_t if_cfg;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PAD_CTRL, (uint8_t *)&pad, 1);
  if (ret != 0) { return -1; }

  switch (pad.io_pad_strength)
  {
    case IIS3DWB10IS_PAD_STRENGTH_LOWER:
      val->strength = IIS3DWB10IS_PAD_STRENGTH_LOWER;
      break;

    case IIS3DWB10IS_PAD_STRENGTH_INTERMEDIATE:
      val->strength = IIS3DWB10IS_PAD_STRENGTH_INTERMEDIATE;
      break;

    case IIS3DWB10IS_PAD_STRENGTH_HIGHEST:
      val->strength = IIS3DWB10IS_PAD_STRENGTH_HIGHEST;
      break;

    default:
      val->strength = IIS3DWB10IS_PAD_STRENGTH_LOWER;
      break;
  }


  switch (pad.pd_dis_int)
  {
    case IIS3DWB10IS_PD_INT1_ON_INT2_ON:
      val->pd_dis = IIS3DWB10IS_PD_INT1_ON_INT2_ON;
      break;

    case IIS3DWB10IS_PD_INT1_OFF_INT2_ON:
      val->pd_dis = IIS3DWB10IS_PD_INT1_OFF_INT2_ON;
      break;

    case IIS3DWB10IS_PD_INT1_ON_INT2_OFF:
      val->pd_dis = IIS3DWB10IS_PD_INT1_ON_INT2_OFF;
      break;

    case IIS3DWB10IS_PD_INT1_OFF_INT2_OFF:
      val->pd_dis = IIS3DWB10IS_PD_INT1_OFF_INT2_OFF;
      break;

    default:
      val->pd_dis = IIS3DWB10IS_PD_INT1_ON_INT2_ON;
      break;
  }

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_IF_CFG, (uint8_t *)&if_cfg, 1);
  if (ret != 0) { return -1; }

  val->int_active_level = if_cfg.int_active_level;

  switch (if_cfg.int_pp_od)
  {
    case IIS3DWB10IS_PUSH_PULL:
      val->pp_od = IIS3DWB10IS_PUSH_PULL;
      break;

    case IIS3DWB10IS_OPEN_DRAIN:
      val->pp_od = IIS3DWB10IS_OPEN_DRAIN;
      break;

    default:
      val->pp_od = IIS3DWB10IS_PUSH_PULL;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

 /**
  * @brief  I3C configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_i3c_cfg_t enum.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_i3c_configure_set(const stmdev_ctx_t *ctx, iis3dwb10is_i3c_cfg_t val)
{
  iis3dwb10is_if_cfg_t if_cfg;
  iis3dwb10is_i3c_ctrl_t i3c_ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_IF_CFG, (uint8_t *)&if_cfg, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_I3C_CTRL, (uint8_t *)&i3c_ctrl, 1);

  if (ret == 0)
  {
    if_cfg.i3c_disable = val.i3c_disable;
    if_cfg.sda_pu_en = val.sda_pu_en;
    i3c_ctrl.int_enable_i3c = val.i3c_int_en;
    i3c_ctrl.bus_act_sel = (uint8_t)val.bus_act_sel;
    ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_IF_CFG, (uint8_t *)&if_cfg, 1);
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_I3C_CTRL, (uint8_t *)&i3c_ctrl, 1);
  }

  return ret;
}

 /**
  * @brief  I3C configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      poniter to iis3dwb10is_i3c_cfg_t enum.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_i3c_configure_get(const stmdev_ctx_t *ctx, iis3dwb10is_i3c_cfg_t *val)
{
  iis3dwb10is_if_cfg_t if_cfg;
  iis3dwb10is_i3c_ctrl_t i3c_ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_IF_CFG, (uint8_t *)&if_cfg, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_I3C_CTRL, (uint8_t *)&i3c_ctrl, 1);

  if (ret == 0)
  {
    val->i3c_disable = if_cfg.i3c_disable;
    val->sda_pu_en = if_cfg.sda_pu_en;
    val->i3c_int_en = i3c_ctrl.int_enable_i3c;

    switch(i3c_ctrl.bus_act_sel)
    {
     case IIS3DWB10IS_I3C_BUS_AVAIL_TIME_50US:
      val->bus_act_sel = IIS3DWB10IS_I3C_BUS_AVAIL_TIME_50US;
      break;

     case IIS3DWB10IS_I3C_BUS_AVAIL_TIME_2US:
      val->bus_act_sel = IIS3DWB10IS_I3C_BUS_AVAIL_TIME_2US;
      break;

     default:
      val->bus_act_sel = IIS3DWB10IS_I3C_BUS_AVAIL_TIME_50US;
      break;
   }
  }

  return ret;
}

 /**
  * @brief  SPI mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_spi_mode enum.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_spi_mode_set(const stmdev_ctx_t *ctx, iis3dwb10is_spi_mode_t val)
{
  iis3dwb10is_spi_ctrl_t spi_ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_SPI_CTRL, (uint8_t *)&spi_ctrl, 1);

  if (ret == 0)
  {
    spi_ctrl.sim = (uint8_t)val;
    ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_SPI_CTRL, (uint8_t *)&spi_ctrl, 1);
  }

  return ret;
}

 /**
  * @brief  SPI mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      pointer to iis3dwb10is_spi_mode enum.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_spi_mode_get(const stmdev_ctx_t *ctx, iis3dwb10is_spi_mode_t *val)
{
  iis3dwb10is_spi_ctrl_t spi_ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_SPI_CTRL, (uint8_t *)&spi_ctrl, 1);

  if (ret == 0)
  {
    switch(spi_ctrl.sim)
    {
     case IIS3DWB10IS_SPI_4_WIRE:
      *val = IIS3DWB10IS_SPI_4_WIRE;
      break;

     case IIS3DWB10IS_SPI_3_WIRE:
      *val = IIS3DWB10IS_SPI_3_WIRE;
      break;

     default:
      *val = IIS3DWB10IS_SPI_4_WIRE;
      break;
   }
  }

  return ret;
}

/**
  * @}
  *
  */

 /**
  * @brief  FIFO watermark threshold (1 LSb = TAG (1 Byte) + 1 sensor (9 Bytes) written in FIFO).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      FIFO watermark threshold.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_fifo_watermark_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t buf[2];
  iis3dwb10is_fifo_ctrl1_t *fifo_ctrl1;
  iis3dwb10is_fifo_ctrl2_t *fifo_ctrl2;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_FIFO_CTRL1, buf, 2);

  if (ret == 0)
  {
    fifo_ctrl1 = (iis3dwb10is_fifo_ctrl1_t *)&buf[0];
    fifo_ctrl2 = (iis3dwb10is_fifo_ctrl2_t *)&buf[1];

    fifo_ctrl1->wtm = (uint8_t)(val & 0xffU);
    fifo_ctrl2->wtm = (uint8_t)((val >> 8U) & 0xfU);
    ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_FIFO_CTRL1, buf, 2);
  }

  return ret;
}

/**
  * @brief  FIFO watermark threshold (1 LSb = TAG (1 Byte) + 1 sensor (9 Bytes) written in FIFO).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      pointer to FIFO watermark threshold.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_fifo_watermark_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buf[2];
  iis3dwb10is_fifo_ctrl1_t *fifo_ctrl1;
  iis3dwb10is_fifo_ctrl2_t *fifo_ctrl2;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_FIFO_CTRL1, buf, 2);

  if (ret == 0)
  {
    fifo_ctrl1 = (iis3dwb10is_fifo_ctrl1_t *)&buf[0];
    fifo_ctrl2 = (iis3dwb10is_fifo_ctrl2_t *)&buf[1];

    *val = (fifo_ctrl2->wtm * 256U) | (uint16_t)fifo_ctrl1->wtm;
  }

  return ret;
}

/**
  * @brief  Stop FIFO recording when watermark is reached.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      1: stop enabled, 0: stop disabled.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_fifo_stop_on_wtm_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb10is_fifo_ctrl3_t fifo_ctrl3;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  if (ret == 0)
  {
    fifo_ctrl3.stop_on_wtm = val;
    ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  }

  return ret;
}

/**
  * @brief  Stop FIFO recording when watermark is reached.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      1: stop enabled, 0: stop disabled.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_fifo_stop_on_wtm_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb10is_fifo_ctrl3_t fifo_ctrl3;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  *val = fifo_ctrl3.stop_on_wtm;

  return ret;
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_fifo_mode_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_fifo_mode_set(const stmdev_ctx_t *ctx, iis3dwb10is_fifo_mode_t val)
{
  iis3dwb10is_fifo_ctrl3_t fifo_ctrl3;
  iis3dwb10is_ctrl3_t ctrl3;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL3, (uint8_t *)&ctrl3, 1);
  if (ret == 0)
  {
    fifo_ctrl3.fifo_mode = (uint8_t)val & 0x07U;
    ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);

    ctrl3.fifo_en = (val == IIS3DWB10IS_FIFO_OFF) ? 0U : 1U;
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_CTRL3, (uint8_t *)&ctrl3, 1);
  }

  return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_fifo_mode_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_fifo_mode_get(const stmdev_ctx_t *ctx, iis3dwb10is_fifo_mode_t *val)
{
  iis3dwb10is_fifo_ctrl3_t fifo_ctrl3;
  iis3dwb10is_ctrl3_t ctrl3;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL3, (uint8_t *)&ctrl3, 1);
  if (ret != 0) { return ret; }

  if (ctrl3.fifo_en == 0U)
  {
    *val = IIS3DWB10IS_FIFO_OFF;
    return 0;
  }

  switch (fifo_ctrl3.fifo_mode)
  {
    case IIS3DWB10IS_BYPASS_MODE:
      *val = IIS3DWB10IS_BYPASS_MODE;
      break;

    case IIS3DWB10IS_FIFO_MODE:
      *val = IIS3DWB10IS_FIFO_MODE;
      break;

    case IIS3DWB10IS_STREAM_MODE:
      *val = IIS3DWB10IS_STREAM_MODE;
      break;

    case IIS3DWB10IS_STREAM_TO_FIFO_MODE:
      *val = IIS3DWB10IS_STREAM_TO_FIFO_MODE;
      break;

    case IIS3DWB10IS_BYPASS_TO_STREAM_MODE:
      *val = IIS3DWB10IS_BYPASS_TO_STREAM_MODE;
      break;

    case IIS3DWB10IS_BYPASS_TO_FIFO_MODE:
      *val = IIS3DWB10IS_BYPASS_TO_FIFO_MODE;
      break;

    default:
      *val = IIS3DWB10IS_BYPASS_MODE;
      break;
  }
  return ret;
}

/**
  * @brief  Batching in FIFO buffer of sensor data.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      1: enable, 0: disable (sensors: XL, temp, QVAR, ISPU).
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_fifo_batch_set(const stmdev_ctx_t *ctx, iis3dwb10is_fifo_sensor_batch_t val)
{
  uint8_t buf[2];
  iis3dwb10is_fifo_ctrl2_t *fifo_ctrl2;
  iis3dwb10is_fifo_ctrl3_t *fifo_ctrl3;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_FIFO_CTRL2, buf, 2);
  if (ret == 0)
  {
    fifo_ctrl2 = (iis3dwb10is_fifo_ctrl2_t *)&buf[0];
    fifo_ctrl3 = (iis3dwb10is_fifo_ctrl3_t *)&buf[1];

    fifo_ctrl2->dec_ts_batch = (uint8_t)val.batch_ts;
    fifo_ctrl3->xl_batch   = val.batch_xl;
    fifo_ctrl3->t_batch    = val.batch_temp;
    fifo_ctrl3->qvar_batch = val.batch_qvar;
    fifo_ctrl3->ispu_batch = val.batch_ispu;
    ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_FIFO_CTRL2, buf, 2);
  }

  return ret;
}

/**
  * @brief  Batching in FIFO buffer of sensor data.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      1: enable, 0: disable (sensors: XL, temp, QVAR, ISPU).
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_fifo_batch_get(const stmdev_ctx_t *ctx, iis3dwb10is_fifo_sensor_batch_t *val)
{
  uint8_t buf[2];
  iis3dwb10is_fifo_ctrl2_t *fifo_ctrl2;
  iis3dwb10is_fifo_ctrl3_t *fifo_ctrl3;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_FIFO_CTRL2, buf, 2);
  if (ret == 0)
  {
    fifo_ctrl2 = (iis3dwb10is_fifo_ctrl2_t *)&buf[0];
    fifo_ctrl3 = (iis3dwb10is_fifo_ctrl3_t *)&buf[1];

    switch (fifo_ctrl2->dec_ts_batch)
    {
      case IIS3DWB10IS_TMSTMP_NOT_BATCHED:
        val->batch_ts = IIS3DWB10IS_TMSTMP_NOT_BATCHED;
        break;

      case IIS3DWB10IS_TMSTMP_DEC_1:
        val->batch_ts = IIS3DWB10IS_TMSTMP_DEC_1;
        break;

      case IIS3DWB10IS_TMSTMP_DEC_8:
        val->batch_ts = IIS3DWB10IS_TMSTMP_DEC_8;
        break;

      case IIS3DWB10IS_TMSTMP_DEC_32:
        val->batch_ts = IIS3DWB10IS_TMSTMP_DEC_32;
        break;

      default:
        val->batch_ts = IIS3DWB10IS_TMSTMP_NOT_BATCHED;
        break;
    }

    val->batch_xl   = fifo_ctrl3->xl_batch;
    val->batch_temp = fifo_ctrl3->t_batch;
    val->batch_qvar = fifo_ctrl3->qvar_batch;
    val->batch_ispu = fifo_ctrl3->ispu_batch;
  }

  return ret;
}

/**
  * @brief  Configure FIFO ISPU control.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_fifo_ispu_ctrl_batch_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_fifo_ispu_ctrl_set(const stmdev_ctx_t *ctx,
                                       iis3dwb10is_fifo_ispu_ctrl_batch_t val)
{
  iis3dwb10is_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  if (ret == 0)
  {
    fifo_ctrl2.fifo_read_from_ispu   = val.read_from_ispu;
    fifo_ctrl2.fifo_trigger_cfg      = (uint8_t)val.trigger;
    ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  }

  return ret;
}

/**
  * @brief  Configure FIFO ISPU control.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_fifo_ispu_ctrl_batch_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_fifo_ispu_ctrl_get(const stmdev_ctx_t *ctx,
                                       iis3dwb10is_fifo_ispu_ctrl_batch_t *val)
{
  iis3dwb10is_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  if (ret != 0) { return ret; }

  val->read_from_ispu = fifo_ctrl2.fifo_read_from_ispu;
  switch (fifo_ctrl2.fifo_trigger_cfg)
  {
    case IIS3DWB10IS_FIFO_TRIGGER_ISPU:
      val->trigger = IIS3DWB10IS_FIFO_TRIGGER_ISPU;
      break;

    case IIS3DWB10IS_FIFO_TRIGGER_INT2:
      val->trigger = IIS3DWB10IS_FIFO_TRIGGER_INT2;
      break;

    default:
      val->trigger = IIS3DWB10IS_FIFO_TRIGGER_ISPU;
      break;
  }

  return ret;
}

int32_t iis3dwb10is_fifo_status_get(const stmdev_ctx_t *ctx, iis3dwb10is_fifo_status_t *val)
{
  uint8_t buff[2];
  iis3dwb10is_fifo_status2_t *status;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_FIFO_STATUS1, buff, 2);
  if (ret != 0)
  {
    return ret;
  }

  status = (iis3dwb10is_fifo_status2_t *)&buff[1];

  val->fifo_ovr  = status->fifo_ovr_ia;
  val->fifo_full = status->fifo_full_ia;
  val->fifo_th   = status->fifo_wtm_ia;

  val->fifo_level = (uint16_t)buff[1] & 0x0fU;
  val->fifo_level = (val->fifo_level * 256U) + buff[0];

  return ret;
}

/**
  * @brief  FIFO data output[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_fifo_out_raw_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_fifo_out_raw_get(const stmdev_ctx_t *ctx,
                                     iis3dwb10is_fifo_out_raw_t *val)
{
  iis3dwb10is_fifo_data_out_tag_t *fifo_tag;
  uint8_t buff[10];
  uint8_t *datap;
  int32_t ret;
  uint8_t i;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_FIFO_DATA_OUT_TAG, buff, 10);
  if (ret != 0)
  {
    return ret;
  }

  fifo_tag = (iis3dwb10is_fifo_data_out_tag_t *)&buff[0];
  datap = &buff[1];

  switch (fifo_tag->tag_sensor)
  {
    case IIS3DWB10IS_TAG_EMPTY:
      val->tag = IIS3DWB10IS_TAG_EMPTY;
      break;

    case IIS3DWB10IS_TAG_QVAR:
      val->tag = IIS3DWB10IS_TAG_QVAR;

      val->qvar = (int16_t)datap[2];
      val->qvar = (val->qvar * 256) + (int16_t)datap[1];
      break;

    case IIS3DWB10IS_TAG_XL:
      val->tag = IIS3DWB10IS_TAG_XL;

      val->xl.x_raw = (int32_t)datap[2];
      val->xl.x_raw = (val->xl.x_raw * 256) + (int32_t)datap[1];
      val->xl.x_raw = (val->xl.x_raw * 256) + (int32_t)datap[0];
      val->xl.y_raw = (int32_t)datap[5];
      val->xl.y_raw = (val->xl.y_raw * 256) + (int32_t)datap[4];
      val->xl.y_raw = (val->xl.y_raw * 256) + (int32_t)datap[3];
      val->xl.z_raw = (int32_t)datap[8];
      val->xl.z_raw = (val->xl.z_raw * 256) + (int32_t)datap[7];
      val->xl.z_raw = (val->xl.z_raw * 256) + (int32_t)datap[6];
      break;

    case IIS3DWB10IS_TAG_TEMP:
      val->tag = IIS3DWB10IS_TAG_TEMP;

      val->temp_raw = (int16_t)datap[2];
      val->temp_raw = (val->temp_raw * 256) + (int16_t)datap[1];
      break;

    case IIS3DWB10IS_TAG_TS:
      val->tag = IIS3DWB10IS_TAG_TS;

      val->ts_raw = (uint64_t)datap[4];
      val->ts_raw = (val->ts_raw * 256U) + (uint64_t)datap[3];
      val->ts_raw = (val->ts_raw * 256U) + (uint64_t)datap[2];
      val->ts_raw = (val->ts_raw * 256U) + (uint64_t)datap[1];
      val->ts_raw = (val->ts_raw * 256U) + (uint64_t)datap[0];
      break;

    case IIS3DWB10IS_TAG_TEMP_QVAR:
      val->tag = IIS3DWB10IS_TAG_TEMP_QVAR;

      val->temp_raw = (int16_t)datap[2];
      val->temp_raw = (val->temp_raw * 256) + (int16_t)datap[1];

      val->qvar = (int16_t)datap[5];
      val->qvar = (val->qvar * 256) + (int16_t)datap[4];
      break;

    default:
      /* unknown tag */
      break;
  }

  /* always return register raw data */
  for (i = 0U; i < 9U; i++)
  {
    val->raw[i] = datap[i];
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @brief  QVAR configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_qvar_mode_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_qvar_mode_set(const stmdev_ctx_t *ctx, iis3dwb10is_qvar_mode_t val)
{
  iis3dwb10is_ctrl4_t ctrl4;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL4, (uint8_t *)&ctrl4, 1);
  if (ret == 0)
  {
    ctrl4.qvar_enable  = val.qvar_en;
    ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_CTRL4, (uint8_t *)&ctrl4, 1);
  }

  return ret;
}

/**
  * @brief  QVAR configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      pointer to iis3dwb10is_qvar_mode_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_qvar_mode_get(const stmdev_ctx_t *ctx, iis3dwb10is_qvar_mode_t *val)
{
}

/**
  * @}
  *
  */

/**
  * @brief  Configure PLL control.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_pll_ctrl_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_pll_ctrl_set(const stmdev_ctx_t *ctx, iis3dwb10is_pll_ctrl_t val)
{
  iis3dwb10is_pll_ctrl1_t pll_ctrl1;
  iis3dwb10is_pll_ctrl2_t pll_ctrl2;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PLL_CTRL1, (uint8_t *)&pll_ctrl1, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PLL_CTRL2, (uint8_t *)&pll_ctrl2, 1);
  if (ret == 0)
  {
    pll_ctrl1.osc_ext_sel  = (uint8_t)val.osc_ext_sel;
    pll_ctrl1.ref_div      = (uint8_t)val.ref_div;
    pll_ctrl2.pll_div      = val.pll_div;
    ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_PLL_CTRL1, (uint8_t *)&pll_ctrl1, 1);
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_PLL_CTRL2, (uint8_t *)&pll_ctrl2, 1);
  }

  return ret;
}

/**
  * @brief  Configure PLL control.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      pointer to iis3dwb10is_pll_ctrl_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_pll_ctrl_get(const stmdev_ctx_t *ctx, iis3dwb10is_pll_ctrl_t *val)
{
  iis3dwb10is_pll_ctrl1_t pll_ctrl1;
  iis3dwb10is_pll_ctrl2_t pll_ctrl2;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PLL_CTRL1, (uint8_t *)&pll_ctrl1, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PLL_CTRL2, (uint8_t *)&pll_ctrl2, 1);
  if (ret != 0) { return ret; }

  val->pll_div = pll_ctrl2.pll_div;

  switch (pll_ctrl1.ref_div)
  {
    case IIS3DWB10IS_PLL_NO_DIVIDER:
      val->ref_div = IIS3DWB10IS_PLL_NO_DIVIDER;
      break;

    case IIS3DWB10IS_PLL_DIV_2:
      val->ref_div = IIS3DWB10IS_PLL_DIV_2;
      break;

    case IIS3DWB10IS_PLL_DIV_4:
      val->ref_div = IIS3DWB10IS_PLL_DIV_4;
      break;

    case IIS3DWB10IS_PLL_DIV_8:
      val->ref_div = IIS3DWB10IS_PLL_DIV_8;
      break;

    case IIS3DWB10IS_PLL_DIV_16:
      val->ref_div = IIS3DWB10IS_PLL_DIV_16;
      break;

    case IIS3DWB10IS_PLL_DIV_32:
      val->ref_div = IIS3DWB10IS_PLL_DIV_32;
      break;

    case IIS3DWB10IS_PLL_DIV_64:
      val->ref_div = IIS3DWB10IS_PLL_DIV_64;
      break;

    case IIS3DWB10IS_PLL_DIV_128:
      val->ref_div = IIS3DWB10IS_PLL_DIV_128;
      break;

    default:
      val->ref_div = IIS3DWB10IS_PLL_NO_DIVIDER;
      break;
  }

  switch (pll_ctrl1.osc_ext_sel)
  {
    case IIS3DWB10IS_PLL_INTERNAL_CLOCK:
      val->osc_ext_sel = IIS3DWB10IS_PLL_INTERNAL_CLOCK;
      break;

    case IIS3DWB10IS_PLL_EXTERNAL_CLOCK:
      val->osc_ext_sel = IIS3DWB10IS_PLL_EXTERNAL_CLOCK;
      break;

    default:
      val->osc_ext_sel = IIS3DWB10IS_PLL_INTERNAL_CLOCK;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @brief  Configure interrupt routing.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_pin_int_route_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_pin_int_route_set(const stmdev_ctx_t *ctx,
                                      iis3dwb10is_pin_int_route_t val)
{
  iis3dwb10is_int_ctrl0_t int_ctrl0;
  iis3dwb10is_int_ctrl1_t int_ctrl1;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_INT_CTRL0, (uint8_t *)&int_ctrl0, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_INT_CTRL1, (uint8_t *)&int_ctrl1, 1);
  if (ret == 0)
  {
    int_ctrl0.int1_boot        = val.int1_boot;
    int_ctrl0.int1_drdy_temp   = val.int1_drdy_temp;
    int_ctrl0.int2_drdy_temp   = val.int2_drdy_temp;
    int_ctrl0.int2_on_int1     = val.int2_on_int1;
    int_ctrl0.int2_sleep_ispu  = val.int2_sleep_ispu;
    int_ctrl1.int1_drdy_qvar   = val.int1_drdy_qvar;
    int_ctrl1.int1_drdy_xl     = val.int1_drdy_xl;
    int_ctrl1.int1_ext_trig    = val.int1_ext_trig;
    int_ctrl1.int1_fifo_ovr    = val.int1_fifo_ovr;
    int_ctrl1.int1_fifo_full   = val.int1_fifo_full;
    int_ctrl1.int1_fifo_th         = val.int1_fifo_th;
    int_ctrl1.int1_sleepcnt    = val.int1_sleep_cnt;
    ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_INT_CTRL0, (uint8_t *)&int_ctrl0, 1);
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_INT_CTRL1, (uint8_t *)&int_ctrl1, 1);
  }

  return ret;
}

/**
  * @brief  Configure interrupt routing.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      pinter to iis3dwb10is_pin_int_route_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_pin_int_route_get(const stmdev_ctx_t *ctx,
                                      iis3dwb10is_pin_int_route_t *val)
{
  iis3dwb10is_int_ctrl0_t int_ctrl0;
  iis3dwb10is_int_ctrl1_t int_ctrl1;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_INT_CTRL0, (uint8_t *)&int_ctrl0, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_INT_CTRL1, (uint8_t *)&int_ctrl1, 1);
  if (ret == 0)
  {
    val->int1_boot       = int_ctrl0.int1_boot;
    val->int1_drdy_temp  = int_ctrl0.int1_drdy_temp;
    val->int2_drdy_temp  = int_ctrl0.int2_drdy_temp;
    val->int2_on_int1    = int_ctrl0.int2_on_int1;
    val->int2_sleep_ispu = int_ctrl0.int2_sleep_ispu;
    val->int1_drdy_qvar  = int_ctrl1.int1_drdy_qvar;
    val->int1_drdy_xl    = int_ctrl1.int1_drdy_xl;
    val->int1_ext_trig   = int_ctrl1.int1_ext_trig;
    val->int1_fifo_ovr   = int_ctrl1.int1_fifo_ovr;
    val->int1_fifo_full  = int_ctrl1.int1_fifo_full;
    val->int1_fifo_th    = int_ctrl1.int1_fifo_th;
    val->int1_sleep_cnt  = int_ctrl1.int1_sleepcnt;
  }

  return ret;
}

/**
  * @brief  return data ready status flag.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      pinter to iis3dwb10is_data_ready_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_flag_data_ready_get(const stmdev_ctx_t *ctx, iis3dwb10is_data_ready_t *val)
{
  iis3dwb10is_status_reg_t status;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_STATUS_REG, (uint8_t *)&status, 1);
  val->drdy_xl   = status.xlda;
  val->drdy_temp = status.tda;
  val->drdy_qvar = status.qvarda;

  return ret;
}

/**
  * @brief  Temperature data output register (r).
  *         L and H registers together express a 16-bit word in two’s
  *         complement.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb10is_temperature_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[2];

  const int32_t ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_OUT_TEMP_L, buff, 2);
  *val = (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  return ret;
}

/**
  * @brief  Linear acceleration output register. The value is expressed as a
  *         16-bit word in two's complement.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb10is_acceleration_16b_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[12];
  uint8_t i;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_OUTX_L_A, buff, 12);
  for (i = 0U; i < 3U; i++)
  {
    val[i] = (int16_t)buff[4U*i + 1U];
    val[i] = (val[i] * 256) + (int16_t)buff[4U*i];
  }

  return ret;
}

/**
  * @brief  Linear acceleration output register. The value is expressed as a
  *         20-bit word in two's complement.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis3dwb10is_acceleration_raw_get(const stmdev_ctx_t *ctx, int32_t *val)
{
  uint8_t buff[12];
  uint8_t i;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_OUTX_L_A, buff, 12);
  for (i = 0U; i < 3U; i++)
  {
    val[i] = (int32_t)buff[4U*i + 3U];
    val[i] = (val[i] * 256) + (int32_t)buff[4U*i + 2U];
    val[i] = (val[i] * 256) + (int32_t)buff[4U*i + 1U];
    val[i] = (val[i] * 256) + (int32_t)buff[4U*i];
  }

  return ret;
}

/**
  * @}
  *
  */
