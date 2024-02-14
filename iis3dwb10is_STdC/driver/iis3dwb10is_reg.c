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
  * @brief  Pull-Down on INT1 and INT2 pins.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      enum iis3dwb10is_pd_dis_int_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_pd_dis_int_set(const stmdev_ctx_t *ctx, iis3dwb10is_pd_dis_int_t val)
{
  iis3dwb10is_pad_ctrl_t pad;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PAD_CTRL, (uint8_t *)&pad, 1);
  pad.pd_dis_int = (uint8_t)val;
  ret += iis3dwb10is_write_reg(ctx, IIS3DWB10IS_PAD_CTRL, (uint8_t *)&pad, 1);

  return ret;
}

/**
  * @brief  Pull-Down on INT1 and INT2 pins.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      pointer to enum iis3dwb10is_pd_dis_int_t
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t iis3dwb10is_pd_dis_int_get(const stmdev_ctx_t *ctx, iis3dwb10is_pd_dis_int_t *val)
{
  iis3dwb10is_pad_ctrl_t pad;
  int32_t ret;

  ret = iis3dwb10is_read_reg(ctx, IIS3DWB10IS_PAD_CTRL, (uint8_t *)&pad, 1);

  switch (pad.pd_dis_int)
  {
    case IIS3DWB10IS_PD_INT1_ON_INT2_ON:
      *val = IIS3DWB10IS_PD_INT1_ON_INT2_ON;
      break;

    case IIS3DWB10IS_PD_INT1_OFF_INT2_ON:
      *val = IIS3DWB10IS_PD_INT1_OFF_INT2_ON;
      break;

    case IIS3DWB10IS_PD_INT1_ON_INT2_OFF:
      *val = IIS3DWB10IS_PD_INT1_ON_INT2_OFF;
      break;

    case IIS3DWB10IS_PD_INT1_OFF_INT2_OFF:
      *val = IIS3DWB10IS_PD_INT1_OFF_INT2_OFF;
      break;

    default:
      *val = IIS3DWB10IS_PD_INT1_ON_INT2_ON;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */