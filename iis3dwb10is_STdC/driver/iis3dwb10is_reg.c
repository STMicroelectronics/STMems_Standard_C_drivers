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
int32_t __weak iis3dwb10is_read_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data,
                                    uint16_t len)
{
  if (ctx == NULL)
  {
    return -1;
  }

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
int32_t __weak iis3dwb10is_write_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data,
                                     uint16_t len)
{
  if (ctx == NULL)
  {
    return -1;
  }

  return ctx->write_reg(ctx->handle, reg, data, len);
}

/**
  * @}
  *
  */

/**
  * @defgroup  Private_functions
  * @brief     Section collect all the utility functions needed by APIs.
  * @{
  *
  */

static void bytecpy(uint8_t *target, uint8_t *source)
{
  if ((target != NULL) && (source != NULL))
  {
    *target = *source;
  }
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
  return ((float_t)lsb * 1.526f);
}

float_t iis3dwb10is_16b_from_fs100g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 3.052);
}

float_t iis3dwb10is_16b_from_fs200g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 6.104f);
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
  int32_t ret;

  if (val.sw_por) {
    iis3dwb10is_ram_access_t ram_access;

    ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_RAM_ACCESS, (uint8_t *)&ram_access, 1);
    ram_access.sw_por = (uint8_t)val.sw_por;
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_RAM_ACCESS, (uint8_t *)&ram_access, 1);
  } else {
    iis3dwb10is_ctrl3_t ctrl3;

    ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL3, (uint8_t *)&ctrl3, 1);
    ctrl3.sw_reset = (uint8_t)val.sw_rst;
    ctrl3.boot = (uint8_t)val.boot;
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_CTRL3, (uint8_t *)&ctrl3, 1);
  }

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
    case 0x0:
      val->burst = IIS3DWB10IS_CONTINUOS_MODE;
      break;

    case 0x1:
      val->burst = IIS3DWB10IS_TON_UI_TOFF_ISPU;
      break;

    case 0x2:
      val->burst = IIS3DWB10IS_TON_UI_TOFF_FIFO;
      break;

    case 0x4:
      val->burst = IIS3DWB10IS_TON_STC_TOFF_UI;
      break;

    case 0x5:
      val->burst = IIS3DWB10IS_TON_STC_TOFF_ISPU;
      break;

    case 0x6:
      val->burst = IIS3DWB10IS_TON_STC_TOFF_FIFO;
      break;

    case 0x8:
      val->burst = IIS3DWB10IS_TON_EXT_TOFF_UI;
      break;

    case 0x9:
      val->burst = IIS3DWB10IS_TON_EXT_TOFF_ISPU;
      break;

    case 0xa:
      val->burst = IIS3DWB10IS_TON_EXT_TOFF_FIFO;
      break;

    default:
      val->burst = IIS3DWB10IS_CONTINUOS_MODE;
      break;
  }

  switch (ctrl1.odr_xl)
  {
    case 0:
      val->odr = IIS3DWB10IS_ODR_IDLE;
      break;

    case 2:
      val->odr = IIS3DWB10IS_ODR_2KHz5;
      break;

    case 3:
      val->odr = IIS3DWB10IS_ODR_5KHz;
      break;

    case 4:
      val->odr = IIS3DWB10IS_ODR_10KHz;
      break;

    case 5:
      val->odr = IIS3DWB10IS_ODR_20KHz;
      break;

    case 6:
      val->odr = IIS3DWB10IS_ODR_40KHz;
      break;

    case 7:
      val->odr = IIS3DWB10IS_ODR_80KHz;
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
    case 0:
      *val = IIS3DWB_50g;
      break;

    case 1:
      *val = IIS3DWB_100g;
      break;

    case 2:
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

  switch (ctrl4.rounding)
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
  iis3dwb10is_pin_ctrl_t pin;
  iis3dwb10is_if_cfg_t if_cfg;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PIN_CTRL, (uint8_t *)&pin, 1);
  pin.io_pin_strength = (uint8_t)val.strength;
  pin.pd_dis_int = (uint8_t)val.pd_dis;
  ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_PIN_CTRL, (uint8_t *)&pin, 1);
  if (ret != 0)
  {
    return -1;
  }

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
  iis3dwb10is_pin_ctrl_t pin;
  iis3dwb10is_if_cfg_t if_cfg;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PIN_CTRL, (uint8_t *)&pin, 1);
  if (ret != 0)
  {
    return -1;
  }

  switch (pin.io_pin_strength)
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


  switch (pin.pd_dis_int)
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
  if (ret != 0)
  {
    return -1;
  }

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
    i3c_ctrl.int_en_i3c = val.i3c_int_en;
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
    val->i3c_int_en = i3c_ctrl.int_en_i3c;

    switch (i3c_ctrl.bus_act_sel)
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
    switch (spi_ctrl.sim)
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
  if (ret != 0)
  {
    return ret;
  }

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
  if (ret != 0)
  {
    return ret;
  }

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
  * @brief  FIFO raw data output[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_fifo_out_raw_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_fifo_out_raw_get(const stmdev_ctx_t *ctx, uint8_t *fifo_buf, uint16_t cnt)
{
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_FIFO_DATA_OUT_TAG, fifo_buf, FIFO_ROW_LEN * cnt);

  return ret;
}

/**
  * @brief  FIFO data process[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_fifo_out_raw_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_fifo_process(uint8_t *fifo_buf, iis3dwb10is_fifo_out_raw_t *val)
{
  iis3dwb10is_fifo_data_out_tag_t *fifo_tag;
  uint8_t *datap;
  uint8_t i;

  fifo_tag = (iis3dwb10is_fifo_data_out_tag_t *)&fifo_buf[0];
  datap = &fifo_buf[1];

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

      val->qvar = (int16_t)datap[0];
      val->qvar = (val->qvar * 256) + (int16_t)datap[5];
      break;

    default:
      /* unknown tag */
      return -1;
  }

  /* always return register raw data */
  for (i = 0U; i < (FIFO_ROW_LEN - 1U); i++)
  {
    val->raw[i] = datap[i];
  }

  return 0;
}

/**
  * @brief  COUNTER ODR configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      counter_odr value
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_counter_odr_cfg_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  iis3dwb10is_counter_odr_h_t odr_h;
  iis3dwb10is_counter_odr_l_t odr_l;
  int32_t ret;

  if (val > 4096)
    return -1;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_COUNTER_ODR_H, (uint8_t *)&odr_h, 1);

  if (ret != 0)
  {
    return ret;
  }

  odr_l.counter_odr = val & 0xff;
  odr_h.counter_odr = val / 256;

  ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_COUNTER_ODR_L, (uint8_t *)&odr_l, 1);
  ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_COUNTER_ODR_H, (uint8_t *)&odr_h, 1);

  return ret;
}

/**
  * @brief  COUNTER ODR configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      counter_odr value
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_counter_odr_cfg_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  iis3dwb10is_counter_odr_h_t odr_h;
  iis3dwb10is_counter_odr_l_t odr_l;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_COUNTER_ODR_L, (uint8_t *)&odr_l, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_COUNTER_ODR_H, (uint8_t *)&odr_h, 1);

  if (ret != 0)
  {
    return ret;
  }

  *val = odr_l.counter_odr;
  *val += odr_h.counter_odr * 256;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup ispu
  * @brief    ispu
  * @{/
  *
  */

/**
  * @brief  ISPU reset
  *
  * @param  ctx      read / write interface definitions
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_ispu_reset(const stmdev_ctx_t *ctx)
{
  iis3dwb10is_ispu_ctrl1_t ctrl;
  int32_t ret;

  /* assert reset bit */
  ctrl.sw_reset_ispu = 1;
  ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_ISPU_CTRL1, (uint8_t *)&ctrl, 1);

  /* wait at least 100 usecs */
  ctx->mdelay(1);

  /* de-assert reset bit */
  ctrl.sw_reset_ispu = 0;
  ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_ISPU_CTRL1, (uint8_t *)&ctrl, 1);

  return ret;
}

/**
  * @brief  ispu TIMESTAMP_EN.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      0: disable timestamp, 1: enable timestamp
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_ispu_timestamp_en_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb10is_ispu_ctrl1_t ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_ISPU_CTRL1, (uint8_t *)&ctrl, 1);
  if (ret == 0)
  {
    ctrl.timestamp_en = val;
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_ISPU_CTRL1, (uint8_t *)&ctrl, 1);
  }

  return ret;
}

/**
  * @brief  ispu TIMESTAMP_EN.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      0: disable timestamp, 1: enable timestamp
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_ispu_timestamp_en_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb10is_ispu_ctrl1_t ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_ISPU_CTRL1, (uint8_t *)&ctrl, 1);
  *val = ctrl.timestamp_en;

  return ret;
}

/**
  * @brief  ispu rate.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      rate values (iis3dwb10is_ispu_rate_t)
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_ispu_rate_set(const stmdev_ctx_t *ctx, iis3dwb10is_ispu_rate_t val)
{
  iis3dwb10is_ispu_ctrl0_t ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_ISPU_CTRL0, (uint8_t *)&ctrl, 1);
  if (ret == 0)
  {
    ctrl.ispu_rate = val;
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_ISPU_CTRL0, (uint8_t *)&ctrl, 1);
  }

  return ret;
}

/**
  * @brief  ispu rate.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      rate values (iis3dwb10is_ispu_rate_t)
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_ispu_rate_get(const stmdev_ctx_t *ctx, iis3dwb10is_ispu_rate_t *val)
{
  iis3dwb10is_ispu_ctrl0_t ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_ISPU_CTRL0, (uint8_t *)&ctrl, 1);
  switch (ctrl.ispu_rate)
  {
    case 0:
      *val = IIS3DWB10IS_ISPU_RATE_IDLE;
      break;

    case 2:
      *val = IIS3DWB10IS_ISPU_RATE_2KHz5;
      break;

    case 3:
      *val = IIS3DWB10IS_ISPU_RATE_5KHz;
      break;

    case 4:
      *val = IIS3DWB10IS_ISPU_RATE_10KHz;
      break;

    case 5:
      *val = IIS3DWB10IS_ISPU_RATE_20KHz;
      break;

    case 6:
      *val = IIS3DWB10IS_ISPU_RATE_40KHz;
      break;

    case 7:
      *val = IIS3DWB10IS_ISPU_RATE_80KHz;
      break;

    default:
      *val = IIS3DWB10IS_ISPU_RATE_IDLE;
      break;
  }

  return ret;
}

/**
  * @brief  ispu bdu.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      0: bdu disabled, 1: bdu enabled
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_ispu_bdu_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb10is_ispu_ctrl0_t ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_ISPU_CTRL0, (uint8_t *)&ctrl, 1);
  if (ret == 0)
  {
    ctrl.ispu_bdu = val;
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_ISPU_CTRL0, (uint8_t *)&ctrl, 1);
  }

  return ret;
}

/**
  * @brief  ispu bdu.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      0: bdu disabled, 1: bdu enabled
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_ispu_bdu_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb10is_ispu_ctrl0_t ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_ISPU_CTRL0, (uint8_t *)&ctrl, 1);
  *val = ctrl.ispu_bdu;

  return ret;
}

/**
  * @brief  ispu grant register access.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      0: access disabled, 1: access enabled
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_grant_ispu_regs_access_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb10is_ispu_ctrl0_t ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_ISPU_CTRL0, (uint8_t *)&ctrl, 1);
  if (ret == 0)
  {
    ctrl.reg_access_confirm = val;
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_ISPU_CTRL0, (uint8_t *)&ctrl, 1);
  }

  return ret;
}

/**
  * @brief  ispu register access.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      0: access disabled, 1: access enabled
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_grant_ispu_regs_access_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  iis3dwb10is_ispu_ctrl0_t ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_ISPU_CTRL0, (uint8_t *)&ctrl, 1);
  *val = ctrl.reg_access_confirm;

  return ret;
}

/**
  * @brief  ispu trigger interrupt.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      0: clear interrupt trigger, 1: assert interrupt trigger
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_ispu_trigger_interrupt(const stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb10is_ispu_ctrl0_t ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_ISPU_CTRL0, (uint8_t *)&ctrl, 1);
  if (ret == 0)
  {
    ctrl.loprio_user_trig = val;
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_ISPU_CTRL0, (uint8_t *)&ctrl, 1);
  }

  return ret;
}

/**
  * @brief  ispu status.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_ispu_status_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_ispu_status_get(const stmdev_ctx_t *ctx, iis3dwb10is_ispu_status_t *val)
{
  uint8_t reg[2];
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_STATUS_A_SL, reg, 2);
  if (ret == 0)
  {
    bytecpy(&val->status_a_sl, &reg[0]);
    bytecpy(&val->status_b_sl, &reg[1]);

  }

  return ret;
}

/**
  * @brief  ispu device status.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_ispu_dev_status_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_ispu_dev_status_get(const stmdev_ctx_t *ctx,
                                        iis3dwb10is_ispu_dev_status_t *val)
{
  iis3dwb10is_device_status_t status;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_DEVICE_STATUS, (uint8_t *)&status, 1);
  if (ret == 0)
  {
      val->ctrl_access = status.ispu_ctrl_access;
      val->core_sleep = status.ispu_core_sleep;
  }

  return ret;
}

/**
  * @brief  ispu loprio configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_ispu_loprio_cfg_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_ispu_loprio_cfg_set(const stmdev_ctx_t *ctx,
                                        iis3dwb10is_ispu_loprio_cfg_t val)
{
  iis3dwb10is_ispu_loprio_en_t loprio;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_ISPU_LOPRIO_EN, (uint8_t *)&loprio, 1);
  if (ret == 0)
  {
    loprio.loprio_fifo_en = val.fifo_en;
    loprio.loprio_sleepcnt_en = val.sleepcnt_en;
    loprio.loprio_qvar_en = val.qvar_en;
    loprio.loprio_ext_trg_en = val.ext_trg_en;
    loprio.loprio_user_trg_en = val.user_trg_en;
    loprio.loprio_temp_en = val.temp_en;

    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_ISPU_LOPRIO_EN, (uint8_t *)&loprio, 1);
  }

  return ret;
}

/**
  * @brief  ispu loprio configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_ispu_loprio_cfg_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_ispu_loprio_cfg_get(const stmdev_ctx_t *ctx,
                                        iis3dwb10is_ispu_loprio_cfg_t *val)
{
  iis3dwb10is_ispu_loprio_en_t loprio;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_ISPU_LOPRIO_EN, (uint8_t *)&loprio, 1);
  if (ret == 0)
  {
    val->fifo_en = loprio.loprio_fifo_en;
    val->sleepcnt_en = loprio.loprio_sleepcnt_en;
    val->qvar_en = loprio.loprio_qvar_en;
    val->ext_trg_en = loprio.loprio_ext_trg_en;
    val->user_trg_en = loprio.loprio_user_trg_en;
    val->temp_en = loprio.loprio_temp_en;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @brief  SLEEPCNT configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_slpcnt_cfg_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_sleepcnt_cfg_set(const stmdev_ctx_t *ctx, iis3dwb10is_slpcnt_cfg_t val)
{
  iis3dwb10is_sleepcnt_cfg_t cfg;
  uint8_t preload[2];
  uint8_t threshold[2];
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_SLEEPCNT_CFG, (uint8_t *)&cfg, 1);
  if (ret == 0)
  {
    preload[0] = (uint8_t)(val.preload_val & 0xFFU);
    preload[1] = (uint8_t)(val.preload_val / 256U);
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_SLEEPCNT_PL_L, preload, 2);

    threshold[0] = (uint8_t)(val.threshold_val & 0xFFU);
    threshold[1] = (uint8_t)(val.threshold_val / 256U);
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_SLEEPCNT_TH_L, threshold, 2);

    cfg.pl_sleepcnt = val.preload_en;
    cfg.rst_sleepcnt = val.reset;
    cfg.enable_sleepcnt = val.enable;
    cfg.tick_sel = (uint8_t)val.tick_sel;
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_SLEEPCNT_CFG, (uint8_t *)&cfg, 1);
  }

  return ret;
}

/**
  * @brief  SLEEPCNT configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      pointer to iis3dwb10is_slpcnt_cfg_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_sleepcnt_cfg_get(const stmdev_ctx_t *ctx, iis3dwb10is_slpcnt_cfg_t *val)
{
  iis3dwb10is_sleepcnt_cfg_t cfg;
  uint8_t preload[2];
  uint8_t threshold[2];
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_SLEEPCNT_CFG, (uint8_t *)&cfg, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_SLEEPCNT_PL_L, preload, 2);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_SLEEPCNT_TH_L, threshold, 2);
  if (ret == 0)
  {
    val->preload_val = preload[1];
    val->preload_val = (val->preload_val * 256U) + preload[0];

    val->threshold_val = threshold[1];
    val->threshold_val = (val->threshold_val * 256U) + threshold[0];

    val->preload_en = cfg.pl_sleepcnt;
    val->reset = cfg.rst_sleepcnt;
    val->enable = cfg.enable_sleepcnt;

    switch (cfg.tick_sel)
    {
      case IIS3DWB10IS_SLP_TICK_SLOW:
        val->tick_sel = IIS3DWB10IS_SLP_TICK_SLOW;
        break;

      case IIS3DWB10IS_SLP_TICK_FAST:
        val->tick_sel = IIS3DWB10IS_SLP_TICK_FAST;
        break;

      default:
        val->tick_sel = IIS3DWB10IS_SLP_TICK_SLOW;
        break;
    }
  }

  return ret;
}

/**
  * @brief  SLEEPCNT time.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      tiime (uint16_t)
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_sleepcnt_time_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t *valp = (uint8_t *)&val;
  int32_t ret;

  ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_SLEEPCNT_TIME_L, valp, 2);

  return ret;
}

/**
  * @brief  SLEEPCNT time.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      tiime (uint16_t)
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_sleepcnt_time_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buf[2];
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_SLEEPCNT_TIME_L, buf, 2);

  *val = buf[0] | (buf[1] & 0x7F) * 256U;

  return ret;
}

/**
  * @brief  Set Trigger from UI.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      0: trigger OFF - 1: trigger ON
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_burst_ui_trigger_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  iis3dwb10is_ctrl3_t ctrl3;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL3, (uint8_t *)&ctrl3, 1);
  ctrl3.burst_force_trg = val & 0x1U;
  ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_CTRL3, (uint8_t *)&ctrl3, 1);

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
  iis3dwb10is_qvar_ctrl_t qvar_ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL4, (uint8_t *)&ctrl4, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_QVAR_CTRL, (uint8_t *)&qvar_ctrl, 1);
  if (ret == 0)
  {
    qvar_ctrl.qvar1_en = val.qvar1_pad_en;
    qvar_ctrl.qvar2_en = val.qvar2_pad_en;
    qvar_ctrl.qvar2_en = val.qvar2_pad_en;
    qvar_ctrl.qvar_switch = val.qvar_switch;
    qvar_ctrl.qvar_lpf = val.lpf;
    qvar_ctrl.qvar_hpf = val.hpf;
    qvar_ctrl.qvar_c_zin = (uint8_t)val.c_zin;
    ctrl4.qvar_en  = val.qvar_en;
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
  iis3dwb10is_ctrl4_t ctrl4;
  iis3dwb10is_qvar_ctrl_t qvar_ctrl;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_CTRL4, (uint8_t *)&ctrl4, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_QVAR_CTRL, (uint8_t *)&qvar_ctrl, 1);
  if (ret == 0)
  {
    val->qvar1_pad_en = qvar_ctrl.qvar1_en;
    val->qvar2_pad_en = qvar_ctrl.qvar2_en;
    val->qvar_switch = qvar_ctrl.qvar_switch;
    val->lpf = qvar_ctrl.qvar_lpf;
    val->hpf = qvar_ctrl.qvar_hpf;
    val->qvar_en = ctrl4.qvar_en;

    switch (qvar_ctrl.qvar_c_zin)
    {
      case IIS3DWB10IS_QVAR_GAIN_1X:
        val->c_zin = IIS3DWB10IS_QVAR_GAIN_1X;
        break;

      case IIS3DWB10IS_QVAR_GAIN_2X:
        val->c_zin = IIS3DWB10IS_QVAR_GAIN_2X;
        break;

      default:
        val->c_zin = IIS3DWB10IS_QVAR_GAIN_1X;
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
  * @brief  Configure PLL control.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      iis3dwb10is_pll_ctrl_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_pll_ctrl_set(const stmdev_ctx_t *ctx, iis3dwb10is_pll_ctrl_t val)
{
  iis3dwb10is_pll_ctrl_1_t pll_ctrl1;
  iis3dwb10is_pll_ctrl_2_t pll_ctrl2;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PLL_CTRL_1, (uint8_t *)&pll_ctrl1, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PLL_CTRL_2, (uint8_t *)&pll_ctrl2, 1);
  if (ret == 0)
  {
    pll_ctrl1.osc_ext_sel  = (uint8_t)val.osc_ext_sel;
    pll_ctrl1.ref_div      = (uint8_t)val.ref_div;
    pll_ctrl2.pll_div      = val.pll_div;
    ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_PLL_CTRL_1, (uint8_t *)&pll_ctrl1, 1);
    ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_PLL_CTRL_2, (uint8_t *)&pll_ctrl2, 1);
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
  iis3dwb10is_pll_ctrl_1_t pll_ctrl1;
  iis3dwb10is_pll_ctrl_2_t pll_ctrl2;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PLL_CTRL_1, (uint8_t *)&pll_ctrl1, 1);
  ret += iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PLL_CTRL_2, (uint8_t *)&pll_ctrl2, 1);
  if (ret != 0)
  {
    return ret;
  }

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
  * @brief  Enables pulsed data-ready mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      DRDY_LEVEL, DRDY_PULSED,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_data_ready_mode_set(const stmdev_ctx_t *ctx, iis3dwb10is_data_ready_mode_t val)
{
  iis3dwb10is_int_ctrl0_t ctrl0;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_INT_CTRL0, (uint8_t *)&ctrl0, 1);

  if (ret == 0)
  {
    ctrl0.pulsed_dataready = ((uint8_t)val & 0x1U);
    ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_INT_CTRL0, (uint8_t *)&ctrl0, 1);
  }

  return ret;
}

/**
  * @brief  Enables pulsed data-ready mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      DRDY_LATCHED, DRDY_PULSED,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_data_ready_mode_get(const stmdev_ctx_t *ctx, iis3dwb10is_data_ready_mode_t *val)
{
  iis3dwb10is_int_ctrl0_t ctrl0;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_INT_CTRL0, (uint8_t *)&ctrl0, 1);

  if (ret == 0)
  {
    switch (ctrl0.pulsed_dataready)
    {
      case IIS3DWB10IS_DRDY_LEVEL:
        *val = IIS3DWB10IS_DRDY_LEVEL;
        break;

      case IIS3DWB10IS_DRDY_PULSED:
        *val = IIS3DWB10IS_DRDY_PULSED;
        break;

      default:
        *val = IIS3DWB10IS_DRDY_LEVEL;
        break;
    }
  }

  return ret;
}

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
  uint8_t reg[3];
  iis3dwb10is_int_ctrl0_t int_ctrl0;
  iis3dwb10is_int_ctrl1_t int_ctrl1;
  iis3dwb10is_int_ctrl2_t int_ctrl2;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_INT_CTRL0, reg, 3);
  if (ret == 0)
  {
    bytecpy((uint8_t *)&int_ctrl0, &reg[0]);
    bytecpy((uint8_t *)&int_ctrl1, &reg[1]);
    bytecpy((uint8_t *)&int_ctrl2, &reg[2]);

    int_ctrl0.int1_boot        = val.int1_boot;
    int_ctrl0.int1_drdy_temp   = val.int1_drdy_temp;
    int_ctrl0.int2_drdy_temp   = val.int2_drdy_temp;
    int_ctrl0.int2_on_int1     = val.int2_on_int1;
    int_ctrl0.int2_sleep_ispu  = val.int2_sleep_ispu;
    int_ctrl1.int1_drdy_xl     = val.int1_drdy_xl;
    int_ctrl1.int1_drdy_qvar   = val.int1_drdy_qvar;
    int_ctrl1.int1_ext_trig    = val.int1_ext_trig;
    int_ctrl1.int1_fifo_th     = val.int1_fifo_th;
    int_ctrl1.int1_fifo_ovr    = val.int1_fifo_ovr;
    int_ctrl1.int1_fifo_full   = val.int1_fifo_full;
    int_ctrl2.int2_drdy_xl     = val.int2_drdy_xl;
    int_ctrl2.int2_drdy_qvar   = val.int2_drdy_qvar;
    int_ctrl2.int2_fifo_th     = val.int2_fifo_th;
    int_ctrl2.int2_fifo_ovr    = val.int2_fifo_ovr;
    int_ctrl2.int2_fifo_full   = val.int2_fifo_full;
    int_ctrl1.int1_sleepcnt    = val.int1_sleep_cnt;

    bytecpy(&reg[0], (uint8_t *)&int_ctrl0);
    bytecpy(&reg[1], (uint8_t *)&int_ctrl1);
    bytecpy(&reg[2], (uint8_t *)&int_ctrl2);
    ret = iis3dwb10is_write_reg(ctx, IIS3DWB10IS_INT_CTRL0, reg, 3);
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
  uint8_t reg[3];
  iis3dwb10is_int_ctrl0_t int_ctrl0;
  iis3dwb10is_int_ctrl1_t int_ctrl1;
  iis3dwb10is_int_ctrl2_t int_ctrl2;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_INT_CTRL0, reg, 3);
  if (ret == 0)
  {
    bytecpy((uint8_t *)&int_ctrl0, &reg[0]);
    bytecpy((uint8_t *)&int_ctrl1, &reg[1]);
    bytecpy((uint8_t *)&int_ctrl2, &reg[2]);

    val->int1_boot       = int_ctrl0.int1_boot;
    val->int1_drdy_temp  = int_ctrl0.int1_drdy_temp;
    val->int2_drdy_temp  = int_ctrl0.int2_drdy_temp;
    val->int2_on_int1    = int_ctrl0.int2_on_int1;
    val->int2_sleep_ispu = int_ctrl0.int2_sleep_ispu;
    val->int1_drdy_xl    = int_ctrl1.int1_drdy_xl;
    val->int1_drdy_qvar  = int_ctrl1.int1_drdy_qvar;
    val->int1_ext_trig   = int_ctrl1.int1_ext_trig;
    val->int1_fifo_th    = int_ctrl1.int1_fifo_th;
    val->int1_fifo_ovr   = int_ctrl1.int1_fifo_ovr;
    val->int1_fifo_full  = int_ctrl1.int1_fifo_full;
    val->int2_drdy_xl    = int_ctrl2.int2_drdy_xl;
    val->int2_drdy_qvar  = int_ctrl2.int2_drdy_qvar;
    val->int2_fifo_th    = int_ctrl2.int2_fifo_th;
    val->int2_fifo_ovr   = int_ctrl2.int2_fifo_ovr;
    val->int2_fifo_full  = int_ctrl2.int2_fifo_full;
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
int32_t iis3dwb10is_data_ready_get(const stmdev_ctx_t *ctx, iis3dwb10is_data_ready_t *val)
{
  iis3dwb10is_status_reg_t status;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_STATUS_REG, (uint8_t *)&status, 1);
  val->drdy_xl   = status.xlda;
  val->drdy_temp = status.tda;
  val->drdy_qvar = status.qvarda;
  val->ext_trig_ia = status.ext_trig_ia;
  val->sleepcnt_ia = status.sleepcnt_ia;
  val->timestamp_endcount = status.timestamp_endcount;
  val->ispu_ia = status.ispu_ia;

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
  uint8_t buff[6];
  uint8_t i;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_OUTX_L_A, buff, 6);
  for (i = 0U; i < 3U; i++)
  {
    val[i] = (int16_t)buff[2U * i + 1U];
    val[i] = (val[i] * 256) + (int16_t)buff[2U * i];
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
    val[i] = (int32_t)buff[4U * i + 3U];
    val[i] = (val[i] * 256) + (int32_t)buff[4U * i + 2U];
    val[i] = (val[i] * 256) + (int32_t)buff[4U * i + 1U];
    val[i] = (val[i] * 256) + (int32_t)buff[4U * i];
  }

  return ret;
}

/**
  * @}
  *
  */