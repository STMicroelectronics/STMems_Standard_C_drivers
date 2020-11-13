/*
 ******************************************************************************
 * @file    iis2iclx_reg.c
 * @author  Sensors Software Solution Team
 * @brief   IIS2ICLX driver file
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

#include "iis2iclx_reg.h"

/**
  * @defgroup    IIS2ICLX
  * @brief       This file provides a set of functions needed to drive the
  *              iis2iclx enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup    IIS2ICLX_Interfaces_Functions
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
int32_t iis2iclx_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
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
int32_t iis2iclx_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
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
  * @defgroup    IIS2ICLX_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t iis2iclx_from_fs500mg_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.015f);
}

float_t iis2iclx_from_fs1g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.031f);
}

float_t iis2iclx_from_fs2g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.061f);
}

float_t iis2iclx_from_fs3g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.122f);
}

float_t iis2iclx_from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb / 256.0f) + 25.0f);
}

float_t iis2iclx_from_lsb_to_nsec(int32_t lsb)
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
int32_t iis2iclx_xl_full_scale_set(stmdev_ctx_t *ctx,
                                   iis2iclx_fs_xl_t val)
{
  iis2iclx_ctrl1_xl_t ctrl1_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL1_XL, (uint8_t *)&ctrl1_xl,
                          1);

  if (ret == 0) {
    ctrl1_xl.fs_xl = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL1_XL,
                             (uint8_t *)&ctrl1_xl, 1);
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
int32_t iis2iclx_xl_full_scale_get(stmdev_ctx_t *ctx,
                                   iis2iclx_fs_xl_t *val)
{
  iis2iclx_ctrl1_xl_t ctrl1_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL1_XL, (uint8_t *)&ctrl1_xl,
                          1);

  switch (ctrl1_xl.fs_xl) {
    case IIS2ICLX_500mg:
      *val = IIS2ICLX_500mg;
      break;

    case IIS2ICLX_3g:
      *val = IIS2ICLX_3g;
      break;

    case IIS2ICLX_1g:
      *val = IIS2ICLX_1g;
      break;

    case IIS2ICLX_2g:
      *val = IIS2ICLX_2g;
      break;

    default:
      *val = IIS2ICLX_500mg;
      break;
  }

  return ret;
}

/**
  * @brief  Accelerometer UI data rate selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of odr_xl in reg CTRL1_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_xl_data_rate_set(stmdev_ctx_t *ctx,
                                  iis2iclx_odr_xl_t val)
{
  iis2iclx_odr_xl_t odr_xl =  val;
  iis2iclx_emb_fsm_enable_t fsm_enable;
  iis2iclx_emb_func_odr_cfg_b_t emb_func_odr_cfg_b;
  iis2iclx_emb_func_odr_cfg_c_t emb_func_odr_cfg_c;
  iis2iclx_page_sel_t page_sel;
  iis2iclx_fsm_odr_t fsm_odr;
  uint8_t mlc_enable;
  iis2iclx_mlc_odr_t mlc_odr;
  iis2iclx_ctrl1_xl_t ctrl1_xl;
  int32_t ret;
  /* Check the Finite State Machine data rate constraints */
  ret =  iis2iclx_fsm_enable_get(ctx, &fsm_enable);

  if (ret == 0) {
    if ( (fsm_enable.fsm_enable_a.fsm1_en  |
          fsm_enable.fsm_enable_a.fsm2_en  |
          fsm_enable.fsm_enable_a.fsm3_en  |
          fsm_enable.fsm_enable_a.fsm4_en  |
          fsm_enable.fsm_enable_a.fsm5_en  |
          fsm_enable.fsm_enable_a.fsm6_en  |
          fsm_enable.fsm_enable_a.fsm7_en  |
          fsm_enable.fsm_enable_a.fsm8_en  |
          fsm_enable.fsm_enable_b.fsm9_en  |
          fsm_enable.fsm_enable_b.fsm10_en |
          fsm_enable.fsm_enable_b.fsm11_en |
          fsm_enable.fsm_enable_b.fsm12_en |
          fsm_enable.fsm_enable_b.fsm13_en |
          fsm_enable.fsm_enable_b.fsm14_en |
          fsm_enable.fsm_enable_b.fsm15_en |
          fsm_enable.fsm_enable_b.fsm16_en ) == PROPERTY_ENABLE ) {
      ret =  iis2iclx_fsm_data_rate_get(ctx, &fsm_odr);

      if (ret == 0) {
        switch (fsm_odr) {
          case IIS2ICLX_ODR_FSM_12Hz5:
            if (val == IIS2ICLX_XL_ODR_OFF) {
              odr_xl = IIS2ICLX_XL_ODR_12Hz5;
            }

            else {
              odr_xl = val;
            }

            break;

          case IIS2ICLX_ODR_FSM_26Hz:
            if (val == IIS2ICLX_XL_ODR_OFF) {
              odr_xl = IIS2ICLX_XL_ODR_26Hz;
            }

            else if (val == IIS2ICLX_XL_ODR_12Hz5) {
              odr_xl = IIS2ICLX_XL_ODR_26Hz;
            }

            else {
              odr_xl = val;
            }

            break;

          case IIS2ICLX_ODR_FSM_52Hz:
            if (val == IIS2ICLX_XL_ODR_OFF) {
              odr_xl = IIS2ICLX_XL_ODR_52Hz;
            }

            else if (val == IIS2ICLX_XL_ODR_12Hz5) {
              odr_xl = IIS2ICLX_XL_ODR_52Hz;
            }

            else if (val == IIS2ICLX_XL_ODR_26Hz) {
              odr_xl = IIS2ICLX_XL_ODR_52Hz;
            }

            else {
              odr_xl = val;
            }

            break;

          case IIS2ICLX_ODR_FSM_104Hz:
            if (val == IIS2ICLX_XL_ODR_OFF) {
              odr_xl = IIS2ICLX_XL_ODR_104Hz;
            }

            else if (val == IIS2ICLX_XL_ODR_12Hz5) {
              odr_xl = IIS2ICLX_XL_ODR_104Hz;
            }

            else if (val == IIS2ICLX_XL_ODR_26Hz) {
              odr_xl = IIS2ICLX_XL_ODR_104Hz;
            }

            else if (val == IIS2ICLX_XL_ODR_52Hz) {
              odr_xl = IIS2ICLX_XL_ODR_104Hz;
            }

            else {
              odr_xl = val;
            }

            break;

          default:
            odr_xl = val;
            break;
        }
      }
    }
  }

  /* Check the Machine Learning Core data rate constraints */
  mlc_enable = PROPERTY_DISABLE;

  if (ret == 0) {
    ret =  iis2iclx_mlc_get(ctx, &mlc_enable);

    if ( mlc_enable == PROPERTY_ENABLE ) {
      ret =  iis2iclx_mlc_data_rate_get(ctx, &mlc_odr);

      if (ret == 0) {
        switch (mlc_odr) {
          case IIS2ICLX_ODR_PRGS_12Hz5:
            if (val == IIS2ICLX_XL_ODR_OFF) {
              odr_xl = IIS2ICLX_XL_ODR_12Hz5;
            }

            else {
              odr_xl = val;
            }

            break;

          case IIS2ICLX_ODR_PRGS_26Hz:
            if (val == IIS2ICLX_XL_ODR_OFF) {
              odr_xl = IIS2ICLX_XL_ODR_26Hz;
            }

            else if (val == IIS2ICLX_XL_ODR_12Hz5) {
              odr_xl = IIS2ICLX_XL_ODR_26Hz;
            }

            else {
              odr_xl = val;
            }

            break;

          case IIS2ICLX_ODR_PRGS_52Hz:
            if (val == IIS2ICLX_XL_ODR_OFF) {
              odr_xl = IIS2ICLX_XL_ODR_52Hz;
            }

            else if (val == IIS2ICLX_XL_ODR_12Hz5) {
              odr_xl = IIS2ICLX_XL_ODR_52Hz;
            }

            else if (val == IIS2ICLX_XL_ODR_26Hz) {
              odr_xl = IIS2ICLX_XL_ODR_52Hz;
            }

            else {
              odr_xl = val;
            }

            break;

          case IIS2ICLX_ODR_PRGS_104Hz:
            if (val == IIS2ICLX_XL_ODR_OFF) {
              odr_xl = IIS2ICLX_XL_ODR_104Hz;
            }

            else if (val == IIS2ICLX_XL_ODR_12Hz5) {
              odr_xl = IIS2ICLX_XL_ODR_104Hz;
            }

            else if (val == IIS2ICLX_XL_ODR_26Hz) {
              odr_xl = IIS2ICLX_XL_ODR_104Hz;
            }

            else if (val == IIS2ICLX_XL_ODR_52Hz) {
              odr_xl = IIS2ICLX_XL_ODR_104Hz;
            }

            else {
              odr_xl = val;
            }

            break;

          default:
            odr_xl = val;
            break;
        }
      }
    }
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

    if (ret == 0) {
      ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_SEL, (uint8_t *)&page_sel,
                              1);
    }

    if (ret == 0) {
      page_sel.not_used_01 = 0x01U;
      ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_SEL,
                               (uint8_t *)&page_sel, 1);
    }

    if (ret == 0) {
      ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_ODR_CFG_B,
                              (uint8_t *)&emb_func_odr_cfg_b, 1);
    }

    if (ret == 0) {
      emb_func_odr_cfg_b.not_used_01 = 0x03U;
      emb_func_odr_cfg_b.not_used_02 = 0x02U;
      ret = iis2iclx_write_reg(ctx, IIS2ICLX_EMB_FUNC_ODR_CFG_B,
                               (uint8_t *)&emb_func_odr_cfg_b, 1);
    }

    if (ret == 0) {
      ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_ODR_CFG_C,
                              (uint8_t *)&emb_func_odr_cfg_c, 1);
    }

    if (ret == 0) {
      emb_func_odr_cfg_c.not_used_01 = 0x05U;
      ret = iis2iclx_write_reg(ctx, IIS2ICLX_EMB_FUNC_ODR_CFG_C,
                               (uint8_t *)&emb_func_odr_cfg_c, 1);
    }

    if (ret == 0) {
      ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
    }
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL1_XL, (uint8_t *)&ctrl1_xl,
                            1);
  }

  if (ret == 0) {
    ctrl1_xl.odr_xl = (uint8_t)odr_xl;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL1_XL,
                             (uint8_t *)&ctrl1_xl, 1);
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
int32_t iis2iclx_xl_data_rate_get(stmdev_ctx_t *ctx,
                                  iis2iclx_odr_xl_t *val)
{
  iis2iclx_ctrl1_xl_t ctrl1_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL1_XL, (uint8_t *)&ctrl1_xl,
                          1);

  switch (ctrl1_xl.odr_xl) {
    case IIS2ICLX_XL_ODR_OFF:
      *val = IIS2ICLX_XL_ODR_OFF;
      break;

    case IIS2ICLX_XL_ODR_12Hz5:
      *val = IIS2ICLX_XL_ODR_12Hz5;
      break;

    case IIS2ICLX_XL_ODR_26Hz:
      *val = IIS2ICLX_XL_ODR_26Hz;
      break;

    case IIS2ICLX_XL_ODR_52Hz:
      *val = IIS2ICLX_XL_ODR_52Hz;
      break;

    case IIS2ICLX_XL_ODR_104Hz:
      *val = IIS2ICLX_XL_ODR_104Hz;
      break;

    case IIS2ICLX_XL_ODR_208Hz:
      *val = IIS2ICLX_XL_ODR_208Hz;
      break;

    case IIS2ICLX_XL_ODR_416Hz:
      *val = IIS2ICLX_XL_ODR_416Hz;
      break;

    case IIS2ICLX_XL_ODR_833Hz:
      *val = IIS2ICLX_XL_ODR_833Hz;
      break;

    default:
      *val = IIS2ICLX_XL_ODR_OFF;
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
int32_t iis2iclx_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);

  if (ret == 0) {
    ctrl3_c.bdu = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                             1);
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
int32_t iis2iclx_block_data_update_get(stmdev_ctx_t *ctx,
                                       uint8_t *val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);
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
int32_t iis2iclx_xl_offset_weight_set(stmdev_ctx_t *ctx,
                                      iis2iclx_usr_off_w_t val)
{
  iis2iclx_ctrl6_c_t ctrl6_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL6_C, (uint8_t *)&ctrl6_c,
                          1);

  if (ret == 0) {
    ctrl6_c.usr_off_w = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL6_C, (uint8_t *)&ctrl6_c,
                             1);
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
int32_t iis2iclx_xl_offset_weight_get(stmdev_ctx_t *ctx,
                                      iis2iclx_usr_off_w_t *val)
{
  iis2iclx_ctrl6_c_t ctrl6_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL6_C, (uint8_t *)&ctrl6_c,
                          1);

  switch (ctrl6_c.usr_off_w) {
    case IIS2ICLX_LSb_1mg:
      *val = IIS2ICLX_LSb_1mg;
      break;

    case IIS2ICLX_LSb_16mg:
      *val = IIS2ICLX_LSb_16mg;
      break;

    default:
      *val = IIS2ICLX_LSb_1mg;
      break;
  }

  return ret;
}

/**
  * @brief  Read all the interrupt flag of the device.
  *[get]
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get registers ALL_INT_SRC; WAKE_UP_SRC;
  *                              TAP_SRC; D6D_SRC; STATUS_REG;
  *                              EMB_FUNC_STATUS; FSM_STATUS_A/B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_all_sources_get(stmdev_ctx_t *ctx,
                                 iis2iclx_all_sources_t *val)
{
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_ALL_INT_SRC,
                          (uint8_t *)&val->all_int_src, 1);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_WAKE_UP_SRC,
                            (uint8_t *)&val->wake_up_src, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_SRC,
                            (uint8_t *)&val->tap_src, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_DEN_SRC,
                            (uint8_t *)&val->den_src, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_STATUS_REG,
                            (uint8_t *)&val->status_reg, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_STATUS,
                            (uint8_t *)&val->emb_func_status, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FSM_STATUS_A,
                            (uint8_t *)&val->fsm_status_a, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FSM_STATUS_B,
                            (uint8_t *)&val->fsm_status_b, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
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
int32_t iis2iclx_status_reg_get(stmdev_ctx_t *ctx,
                                iis2iclx_status_reg_t *val)
{
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_STATUS_REG, (uint8_t *) val, 1);
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
int32_t iis2iclx_xl_flag_data_ready_get(stmdev_ctx_t *ctx,
                                        uint8_t *val)
{
  iis2iclx_status_reg_t status_reg;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_STATUS_REG,
                          (uint8_t *)&status_reg, 1);
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
int32_t iis2iclx_temp_flag_data_ready_get(stmdev_ctx_t *ctx,
                                          uint8_t *val)
{
  iis2iclx_status_reg_t status_reg;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_STATUS_REG,
                          (uint8_t *)&status_reg, 1);
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
int32_t iis2iclx_xl_usr_offset_x_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis2iclx_write_reg(ctx, IIS2ICLX_X_OFS_USR, buff, 1);
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
int32_t iis2iclx_xl_usr_offset_x_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_X_OFS_USR, buff, 1);
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
int32_t iis2iclx_xl_usr_offset_y_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis2iclx_write_reg(ctx, IIS2ICLX_Y_OFS_USR, buff, 1);
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
int32_t iis2iclx_xl_usr_offset_y_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_Y_OFS_USR, buff, 1);
  return ret;
}

/**
  * @brief  Enables user offset on out.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of usr_off_on_out in reg CTRL7_G
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_xl_usr_offset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_ctrl7_xl_t ctrl7_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL7_XL, (uint8_t *)&ctrl7_xl,
                          1);

  if (ret == 0) {
    ctrl7_xl.usr_off_on_out = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL7_XL, (uint8_t *)&ctrl7_xl,
                             1);
  }

  return ret;
}

/**
  * @brief  Get user offset on out flag.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get values of usr_off_on_out in reg CTRL7_G
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_xl_usr_offset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_ctrl7_xl_t ctrl7_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL7_XL, (uint8_t *)&ctrl7_xl,
                          1);
  *val = ctrl7_xl.usr_off_on_out;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS2ICLX_Timestamp
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
int32_t iis2iclx_timestamp_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_ctrl10_c_t ctrl10_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL10_C, (uint8_t *)&ctrl10_c,
                          1);

  if (ret == 0) {
    ctrl10_c.timestamp_en = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL10_C,
                             (uint8_t *)&ctrl10_c, 1);
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
int32_t iis2iclx_timestamp_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_ctrl10_c_t ctrl10_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL10_C, (uint8_t *)&ctrl10_c,
                          1);
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
int32_t iis2iclx_timestamp_raw_get(stmdev_ctx_t *ctx, int32_t *val)
{
  uint8_t buff[4];
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TIMESTAMP0, buff, 4);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) +  (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS2ICLX_Data output
  * @brief      This section groups all the data output functions.
  * @{
  *
  */

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
int32_t iis2iclx_temperature_raw_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[2];
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_OUT_TEMP_L, buff, 2);
  *val = (int16_t)buff[1];
  *val = (*val * 256) +  (int16_t)buff[0];
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
int32_t iis2iclx_acceleration_raw_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[4];
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_OUTX_L_A, buff, 4);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) +  (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) +  (int16_t)buff[2];
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
int32_t iis2iclx_fifo_out_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_DATA_OUT_X_L, buff, 6);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS2ICLX_common
  * @brief      This section groups common useful functions.
  * @{
  *
  */

/**
  * @brief  DEVICE_CONF bit configuration[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of device_conf in reg CTRL9_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_device_conf_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_ctrl9_xl_t ctrl9_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                          1);

  if (ret == 0) {
    ctrl9_xl.device_conf = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                             1);
  }

  return ret;
}

/**
  * @brief  DEVICE_CONF bit configuration[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of device_conf in reg CTRL9_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_device_conf_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_ctrl9_xl_t ctrl9_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                          1);
  *val = ctrl9_xl.device_conf;
  return ret;
}

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
int32_t iis2iclx_odr_cal_reg_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_internal_freq_fine_t internal_freq_fine;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_INTERNAL_FREQ_FINE,
                          (uint8_t *)&internal_freq_fine, 1);

  if (ret == 0) {
    internal_freq_fine.freq_fine = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_INTERNAL_FREQ_FINE,
                             (uint8_t *)&internal_freq_fine, 1);
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
int32_t iis2iclx_odr_cal_reg_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_internal_freq_fine_t internal_freq_fine;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_INTERNAL_FREQ_FINE,
                          (uint8_t *)&internal_freq_fine, 1);
  *val = internal_freq_fine.freq_fine;
  return ret;
}

/**
  * @brief  Enable access to the embedded functions/sensor hub configuration
  *         registers.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of reg_access in reg FUNC_CFG_ACCESS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_mem_bank_set(stmdev_ctx_t *ctx,
                              iis2iclx_reg_access_t val)
{
  iis2iclx_func_cfg_access_t func_cfg_access;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FUNC_CFG_ACCESS,
                          (uint8_t *)&func_cfg_access, 1);

  if (ret == 0) {
    func_cfg_access.reg_access = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FUNC_CFG_ACCESS,
                             (uint8_t *)&func_cfg_access, 1);
  }

  return ret;
}

/**
  * @brief  Enable access to the embedded functions/sensor hub configuration
  *         registers.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of reg_access in reg FUNC_CFG_ACCESS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_mem_bank_get(stmdev_ctx_t *ctx,
                              iis2iclx_reg_access_t *val)
{
  iis2iclx_func_cfg_access_t func_cfg_access;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FUNC_CFG_ACCESS,
                          (uint8_t *)&func_cfg_access, 1);

  switch (func_cfg_access.reg_access) {
    case IIS2ICLX_USER_BANK:
      *val = IIS2ICLX_USER_BANK;
      break;

    case IIS2ICLX_SENSOR_HUB_BANK:
      *val = IIS2ICLX_SENSOR_HUB_BANK;
      break;

    case IIS2ICLX_EMBEDDED_FUNC_BANK:
      *val = IIS2ICLX_EMBEDDED_FUNC_BANK;
      break;

    default:
      *val = IIS2ICLX_USER_BANK;
      break;
  }

  return ret;
}

/**
  * @brief  Write a line(byte) in a page.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  add    Page line address
  * @param  val    Value to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_ln_pg_write_byte(stmdev_ctx_t *ctx, uint16_t add,
                                  uint8_t *val)
{
  iis2iclx_page_rw_t page_rw;
  iis2iclx_page_sel_t page_sel;
  iis2iclx_page_address_t page_address;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                            1);
  }

  if (ret == 0) {
    page_rw.page_rw = 0x02U; /* page_write enable */
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                             1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_SEL, (uint8_t *)&page_sel,
                            1);
  }

  if (ret == 0) {
    page_sel.page_sel = (uint8_t)((add >> 8) & 0x0FU);
    page_sel.not_used_01 = 1;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_SEL,
                             (uint8_t *)&page_sel, 1);
  }

  if (ret == 0) {
    page_address.page_addr = (uint8_t)(add & 0xFFU);
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_ADDRESS,
                             (uint8_t *)&page_address, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_VALUE, val, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                            1);
  }

  if (ret == 0) {
    page_rw.page_rw = 0x00; /* page_write disable */
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                             1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Write buffer in a page.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buf    Page line address.(ptr)
  * @param  val    Value to write.
  * @param  len    buffer length.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_ln_pg_write(stmdev_ctx_t *ctx, uint16_t add,
                             uint8_t *buf, uint8_t len)
{
  iis2iclx_page_rw_t page_rw;
  iis2iclx_page_sel_t page_sel;
  iis2iclx_page_address_t page_address;
  int32_t ret;
  uint8_t msb, lsb;
  uint8_t i ;
  msb = (uint8_t)((add >> 8) & 0x0FU);
  lsb = (uint8_t)(add & 0xFFU);
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                            1);
  }

  if (ret == 0) {
    page_rw.page_rw = 0x02U; /* page_write enable*/
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                             1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_SEL, (uint8_t *)&page_sel,
                            1);
  }

  if (ret == 0) {
    page_sel.page_sel = msb;
    page_sel.not_used_01 = 1;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_SEL,
                             (uint8_t *)&page_sel, 1);
  }

  if (ret == 0) {
    page_address.page_addr = lsb;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_ADDRESS,
                             (uint8_t *)&page_address, 1);
  }

  for (i = 0; i < len; i++) {
    if (ret == 0) {
      ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_VALUE, &buf[i], 1);

      if (ret == 0) {
        /* Check if page wrap */
        if (lsb == 0x00U) {
          msb++;
          ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_SEL,
                                  (uint8_t *)&page_sel, 1);
        }

        lsb++;
      }

      if (ret == 0) {
        page_sel.page_sel = msb;
        page_sel.not_used_01 = 1;
        ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_SEL,
                                 (uint8_t *)&page_sel, 1);
      }
    }
  }

  if (ret == 0) {
    page_sel.page_sel = 0;
    page_sel.not_used_01 = 1;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_SEL,
                             (uint8_t *)&page_sel, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                            1);
  }

  if (ret == 0) {
    page_rw.page_rw = 0x00U; /* page_write disable */
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                             1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Read a line(byte) in a page.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  add    Page line address.
  * @param  val    Read value.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_ln_pg_read_byte(stmdev_ctx_t *ctx, uint16_t add,
                                 uint8_t *val)
{
  iis2iclx_page_rw_t page_rw;
  iis2iclx_page_sel_t page_sel;
  iis2iclx_page_address_t page_address;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                            1);
  }

  if (ret == 0) {
    page_rw.page_rw = 0x01U; /* page_read enable*/
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                             1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_SEL, (uint8_t *)&page_sel,
                            1);
  }

  if (ret == 0) {
    page_sel.page_sel = (uint8_t)((add >> 8) & 0x0FU);
    page_sel.not_used_01 = 1;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_SEL,
                             (uint8_t *)&page_sel, 1);
  }

  if (ret == 0) {
    page_address.page_addr = (uint8_t)(add & 0x00FFU);
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_ADDRESS,
                             (uint8_t *)&page_address, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_VALUE, val, 2);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                            1);
  }

  if (ret == 0) {
    page_rw.page_rw = 0x00U; /* page_read disable */
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                             1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

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
int32_t iis2iclx_data_ready_mode_set(stmdev_ctx_t *ctx,
                                     iis2iclx_dataready_pulsed_t val)
{
  iis2iclx_counter_bdr_reg1_t counter_bdr_reg1;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_COUNTER_BDR_REG1,
                          (uint8_t *)&counter_bdr_reg1, 1);

  if (ret == 0) {
    counter_bdr_reg1.dataready_pulsed = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_COUNTER_BDR_REG1,
                             (uint8_t *)&counter_bdr_reg1, 1);
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
int32_t iis2iclx_data_ready_mode_get(stmdev_ctx_t *ctx,
                                     iis2iclx_dataready_pulsed_t *val)
{
  iis2iclx_counter_bdr_reg1_t counter_bdr_reg1;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_COUNTER_BDR_REG1,
                          (uint8_t *)&counter_bdr_reg1, 1);

  switch (counter_bdr_reg1.dataready_pulsed) {
    case IIS2ICLX_DRDY_LATCHED:
      *val = IIS2ICLX_DRDY_LATCHED;
      break;

    case IIS2ICLX_DRDY_PULSED:
      *val = IIS2ICLX_DRDY_PULSED;
      break;

    default:
      *val = IIS2ICLX_DRDY_LATCHED;
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
int32_t iis2iclx_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_WHO_AM_I, buff, 1);
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
int32_t iis2iclx_reset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);

  if (ret == 0) {
    ctrl3_c.sw_reset = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                             1);
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
int32_t iis2iclx_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);
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
int32_t iis2iclx_auto_increment_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);

  if (ret == 0) {
    ctrl3_c.if_inc = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                             1);
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
int32_t iis2iclx_auto_increment_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);
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
int32_t iis2iclx_boot_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);

  if (ret == 0) {
    ctrl3_c.boot = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                             1);
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
int32_t iis2iclx_boot_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);
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
int32_t iis2iclx_xl_self_test_set(stmdev_ctx_t *ctx,
                                  iis2iclx_st_xl_t val)
{
  iis2iclx_ctrl5_c_t ctrl5_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL5_C, (uint8_t *)&ctrl5_c,
                          1);

  if (ret == 0) {
    ctrl5_c.st_xl = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL5_C, (uint8_t *)&ctrl5_c,
                             1);
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
int32_t iis2iclx_xl_self_test_get(stmdev_ctx_t *ctx,
                                  iis2iclx_st_xl_t *val)
{
  iis2iclx_ctrl5_c_t ctrl5_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL5_C, (uint8_t *)&ctrl5_c,
                          1);

  switch (ctrl5_c.st_xl) {
    case IIS2ICLX_XL_ST_DISABLE:
      *val = IIS2ICLX_XL_ST_DISABLE;
      break;

    case IIS2ICLX_XL_ST_POSITIVE:
      *val = IIS2ICLX_XL_ST_POSITIVE;
      break;

    case IIS2ICLX_XL_ST_NEGATIVE:
      *val = IIS2ICLX_XL_ST_NEGATIVE;
      break;

    default:
      *val = IIS2ICLX_XL_ST_DISABLE;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS2ICLX_filters
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
int32_t iis2iclx_xl_filter_lp2_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_ctrl1_xl_t ctrl1_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL1_XL, (uint8_t *)&ctrl1_xl,
                          1);

  if (ret == 0) {
    ctrl1_xl.lpf2_xl_en = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL1_XL,
                             (uint8_t *)&ctrl1_xl, 1);
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
int32_t iis2iclx_xl_filter_lp2_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_ctrl1_xl_t ctrl1_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL1_XL, (uint8_t *)&ctrl1_xl,
                          1);
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
int32_t iis2iclx_filter_settling_mask_set(stmdev_ctx_t *ctx,
                                          uint8_t val)
{
  iis2iclx_ctrl4_c_t ctrl4_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL4_C, (uint8_t *)&ctrl4_c,
                          1);

  if (ret == 0) {
    ctrl4_c.drdy_mask = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL4_C, (uint8_t *)&ctrl4_c,
                             1);
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
int32_t iis2iclx_filter_settling_mask_get(stmdev_ctx_t *ctx,
                                          uint8_t *val)
{
  iis2iclx_ctrl4_c_t ctrl4_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL4_C, (uint8_t *)&ctrl4_c,
                          1);
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
int32_t iis2iclx_xl_hp_path_on_out_set(stmdev_ctx_t *ctx,
                                       iis2iclx_hp_slope_xl_en_t val)
{
  iis2iclx_ctrl8_xl_t ctrl8_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL8_XL, (uint8_t *)&ctrl8_xl,
                          1);

  if (ret == 0) {
    ctrl8_xl.hp_slope_xl_en = (((uint8_t)val & 0x10U) >> 4);
    ctrl8_xl.hp_ref_mode_xl = (((uint8_t)val & 0x20U) >> 5);
    ctrl8_xl.hpcf_xl = (uint8_t)val & 0x07U;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL8_XL,
                             (uint8_t *)&ctrl8_xl, 1);
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
int32_t iis2iclx_xl_hp_path_on_out_get(stmdev_ctx_t *ctx,
                                       iis2iclx_hp_slope_xl_en_t *val)
{
  iis2iclx_ctrl8_xl_t ctrl8_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL8_XL, (uint8_t *)&ctrl8_xl,
                          1);

  switch (( (ctrl8_xl.hp_ref_mode_xl << 5) + (ctrl8_xl.hp_slope_xl_en <<
                                              4) +
            ctrl8_xl.hpcf_xl )) {
    case IIS2ICLX_HP_PATH_DISABLE_ON_OUT:
      *val = IIS2ICLX_HP_PATH_DISABLE_ON_OUT;
      break;

    case IIS2ICLX_SLOPE_ODR_DIV_4:
      *val = IIS2ICLX_SLOPE_ODR_DIV_4;
      break;

    case IIS2ICLX_HP_ODR_DIV_10:
      *val = IIS2ICLX_HP_ODR_DIV_10;
      break;

    case IIS2ICLX_HP_ODR_DIV_20:
      *val = IIS2ICLX_HP_ODR_DIV_20;
      break;

    case IIS2ICLX_HP_ODR_DIV_45:
      *val = IIS2ICLX_HP_ODR_DIV_45;
      break;

    case IIS2ICLX_HP_ODR_DIV_100:
      *val = IIS2ICLX_HP_ODR_DIV_100;
      break;

    case IIS2ICLX_HP_ODR_DIV_200:
      *val = IIS2ICLX_HP_ODR_DIV_200;
      break;

    case IIS2ICLX_HP_ODR_DIV_400:
      *val = IIS2ICLX_HP_ODR_DIV_400;
      break;

    case IIS2ICLX_HP_ODR_DIV_800:
      *val = IIS2ICLX_HP_ODR_DIV_800;
      break;

    case IIS2ICLX_HP_REF_MD_ODR_DIV_10:
      *val = IIS2ICLX_HP_REF_MD_ODR_DIV_10;
      break;

    case IIS2ICLX_HP_REF_MD_ODR_DIV_20:
      *val = IIS2ICLX_HP_REF_MD_ODR_DIV_20;
      break;

    case IIS2ICLX_HP_REF_MD_ODR_DIV_45:
      *val = IIS2ICLX_HP_REF_MD_ODR_DIV_45;
      break;

    case IIS2ICLX_HP_REF_MD_ODR_DIV_100:
      *val = IIS2ICLX_HP_REF_MD_ODR_DIV_100;
      break;

    case IIS2ICLX_HP_REF_MD_ODR_DIV_200:
      *val = IIS2ICLX_HP_REF_MD_ODR_DIV_200;
      break;

    case IIS2ICLX_HP_REF_MD_ODR_DIV_400:
      *val = IIS2ICLX_HP_REF_MD_ODR_DIV_400;
      break;

    case IIS2ICLX_HP_REF_MD_ODR_DIV_800:
      *val = IIS2ICLX_HP_REF_MD_ODR_DIV_800;
      break;

    case IIS2ICLX_LP_ODR_DIV_10:
      *val = IIS2ICLX_LP_ODR_DIV_10;
      break;

    case IIS2ICLX_LP_ODR_DIV_20:
      *val = IIS2ICLX_LP_ODR_DIV_20;
      break;

    case IIS2ICLX_LP_ODR_DIV_45:
      *val = IIS2ICLX_LP_ODR_DIV_45;
      break;

    case IIS2ICLX_LP_ODR_DIV_100:
      *val = IIS2ICLX_LP_ODR_DIV_100;
      break;

    case IIS2ICLX_LP_ODR_DIV_200:
      *val = IIS2ICLX_LP_ODR_DIV_200;
      break;

    case IIS2ICLX_LP_ODR_DIV_400:
      *val = IIS2ICLX_LP_ODR_DIV_400;
      break;

    case IIS2ICLX_LP_ODR_DIV_800:
      *val = IIS2ICLX_LP_ODR_DIV_800;
      break;

    default:
      *val = IIS2ICLX_HP_PATH_DISABLE_ON_OUT;
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
int32_t iis2iclx_xl_fast_settling_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_ctrl8_xl_t ctrl8_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL8_XL, (uint8_t *)&ctrl8_xl,
                          1);

  if (ret == 0) {
    ctrl8_xl.fastsettl_mode_xl = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL8_XL,
                             (uint8_t *)&ctrl8_xl, 1);
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
int32_t iis2iclx_xl_fast_settling_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_ctrl8_xl_t ctrl8_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL8_XL, (uint8_t *)&ctrl8_xl,
                          1);
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
int32_t iis2iclx_xl_hp_path_internal_set(stmdev_ctx_t *ctx,
                                         iis2iclx_slope_fds_t val)
{
  iis2iclx_tap_cfg0_t tap_cfg0;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG0, (uint8_t *)&tap_cfg0,
                          1);

  if (ret == 0) {
    tap_cfg0.slope_fds = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_TAP_CFG0,
                             (uint8_t *)&tap_cfg0, 1);
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
int32_t iis2iclx_xl_hp_path_internal_get(stmdev_ctx_t *ctx,
                                         iis2iclx_slope_fds_t *val)
{
  iis2iclx_tap_cfg0_t tap_cfg0;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG0, (uint8_t *)&tap_cfg0,
                          1);

  switch (tap_cfg0.slope_fds) {
    case IIS2ICLX_USE_SLOPE:
      *val = IIS2ICLX_USE_SLOPE;
      break;

    case IIS2ICLX_USE_HPF:
      *val = IIS2ICLX_USE_HPF;
      break;

    default:
      *val = IIS2ICLX_USE_SLOPE;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS2ICLX_ main_serial_interface
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
int32_t iis2iclx_sdo_sa0_mode_set(stmdev_ctx_t *ctx,
                                  iis2iclx_sdo_pu_en_t val)
{
  iis2iclx_pin_ctrl_t pin_ctrl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_PIN_CTRL, (uint8_t *)&pin_ctrl,
                          1);

  if (ret == 0) {
    pin_ctrl.sdo_pu_en = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PIN_CTRL, (uint8_t *)&pin_ctrl,
                             1);
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
int32_t iis2iclx_sdo_sa0_mode_get(stmdev_ctx_t *ctx,
                                  iis2iclx_sdo_pu_en_t *val)
{
  iis2iclx_pin_ctrl_t pin_ctrl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_PIN_CTRL, (uint8_t *)&pin_ctrl,
                          1);

  switch (pin_ctrl.sdo_pu_en) {
    case IIS2ICLX_PULL_UP_DISC:
      *val = IIS2ICLX_PULL_UP_DISC;
      break;

    case IIS2ICLX_PULL_UP_CONNECT:
      *val = IIS2ICLX_PULL_UP_CONNECT;
      break;

    default:
      *val = IIS2ICLX_PULL_UP_DISC;
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
int32_t iis2iclx_spi_mode_set(stmdev_ctx_t *ctx, iis2iclx_sim_t val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);

  if (ret == 0) {
    ctrl3_c.sim = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                             1);
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
int32_t iis2iclx_spi_mode_get(stmdev_ctx_t *ctx, iis2iclx_sim_t *val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);

  switch (ctrl3_c.sim) {
    case IIS2ICLX_SPI_4_WIRE:
      *val = IIS2ICLX_SPI_4_WIRE;
      break;

    case IIS2ICLX_SPI_3_WIRE:
      *val = IIS2ICLX_SPI_3_WIRE;
      break;

    default:
      *val = IIS2ICLX_SPI_4_WIRE;
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
int32_t iis2iclx_i2c_interface_set(stmdev_ctx_t *ctx,
                                   iis2iclx_i2c_disable_t val)
{
  iis2iclx_ctrl4_c_t ctrl4_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL4_C, (uint8_t *)&ctrl4_c,
                          1);

  if (ret == 0) {
    ctrl4_c.i2c_disable = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL4_C, (uint8_t *)&ctrl4_c,
                             1);
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
int32_t iis2iclx_i2c_interface_get(stmdev_ctx_t *ctx,
                                   iis2iclx_i2c_disable_t *val)
{
  iis2iclx_ctrl4_c_t ctrl4_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL4_C, (uint8_t *)&ctrl4_c,
                          1);

  switch (ctrl4_c.i2c_disable) {
    case IIS2ICLX_I2C_ENABLE:
      *val = IIS2ICLX_I2C_ENABLE;
      break;

    case IIS2ICLX_I2C_DISABLE:
      *val = IIS2ICLX_I2C_DISABLE;
      break;

    default:
      *val = IIS2ICLX_I2C_ENABLE;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS2ICLX_interrupt_pins
  * @brief      This section groups all the functions that manage
  *             interrupt pins
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
int32_t iis2iclx_pin_int1_route_set(stmdev_ctx_t *ctx,
                                    iis2iclx_pin_int1_route_t *val)
{
  iis2iclx_pin_int2_route_t pin_int2_route;
  iis2iclx_tap_cfg2_t tap_cfg2;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_MLC_INT1,
                             (uint8_t *)&val->mlc_int1, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_EMB_FUNC_INT1,
                             (uint8_t *)&val->emb_func_int1, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FSM_INT1_A,
                             (uint8_t *)&val->fsm_int1_a, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FSM_INT1_B,
                             (uint8_t *)&val->fsm_int1_b, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  if (ret == 0) {
    if ( ( val->emb_func_int1.int1_fsm_lc
           | val->fsm_int1_a.int1_fsm1
           | val->fsm_int1_a.int1_fsm2
           | val->fsm_int1_a.int1_fsm3
           | val->fsm_int1_a.int1_fsm4
           | val->fsm_int1_a.int1_fsm5
           | val->fsm_int1_a.int1_fsm6
           | val->fsm_int1_a.int1_fsm7
           | val->fsm_int1_a.int1_fsm8
           | val->fsm_int1_b.int1_fsm9
           | val->fsm_int1_b.int1_fsm10
           | val->fsm_int1_b.int1_fsm11
           | val->fsm_int1_b.int1_fsm12
           | val->fsm_int1_b.int1_fsm13
           | val->fsm_int1_b.int1_fsm14
           | val->fsm_int1_b.int1_fsm15
           | val->fsm_int1_b.int1_fsm16
           | val->mlc_int1.int1_mlc1
           | val->mlc_int1.int1_mlc2
           | val->mlc_int1.int1_mlc3
           | val->mlc_int1.int1_mlc4
           | val->mlc_int1.int1_mlc5
           | val->mlc_int1.int1_mlc6
           | val->mlc_int1.int1_mlc7
           | val->mlc_int1.int1_mlc8) != PROPERTY_DISABLE) {
      val->md1_cfg.int1_emb_func = PROPERTY_ENABLE;
    }

    else {
      val->md1_cfg.int1_emb_func = PROPERTY_DISABLE;
    }

    ret = iis2iclx_write_reg(ctx, IIS2ICLX_INT1_CTRL,
                             (uint8_t *)&val->int1_ctrl, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_MD1_CFG,
                             (uint8_t *)&val->md1_cfg, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG2, (uint8_t *) &tap_cfg2,
                            1);
  }

  if (ret == 0) {
    ret = iis2iclx_pin_int2_route_get(ctx, &pin_int2_route);
  }

  if (ret == 0) {
    if ( ( pin_int2_route.int2_ctrl.int2_cnt_bdr
           | pin_int2_route.int2_ctrl.int2_drdy_temp
           | pin_int2_route.int2_ctrl.int2_drdy_xl
           | pin_int2_route.int2_ctrl.int2_fifo_full
           | pin_int2_route.int2_ctrl.int2_fifo_ovr
           | pin_int2_route.int2_ctrl.int2_fifo_th
           | pin_int2_route.md2_cfg.int2_double_tap
           | pin_int2_route.md2_cfg.int2_wu
           | pin_int2_route.md2_cfg.int2_single_tap
           | pin_int2_route.md2_cfg.int2_sleep_change
           | val->int1_ctrl.den_drdy_flag
           | val->int1_ctrl.int1_boot
           | val->int1_ctrl.int1_cnt_bdr
           | val->int1_ctrl.int1_drdy_xl
           | val->int1_ctrl.int1_fifo_full
           | val->int1_ctrl.int1_fifo_ovr
           | val->int1_ctrl.int1_fifo_th
           | val->md1_cfg.int1_shub
           | val->md1_cfg.int1_double_tap
           | val->md1_cfg.int1_wu
           | val->md1_cfg.int1_single_tap
           | val->md1_cfg.int1_sleep_change) != PROPERTY_DISABLE) {
      tap_cfg2.interrupts_enable = PROPERTY_ENABLE;
    }

    else {
      tap_cfg2.interrupts_enable = PROPERTY_DISABLE;
    }

    ret = iis2iclx_write_reg(ctx, IIS2ICLX_TAP_CFG2,
                             (uint8_t *) &tap_cfg2, 1);
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
int32_t iis2iclx_pin_int1_route_get(stmdev_ctx_t *ctx,
                                    iis2iclx_pin_int1_route_t *val)
{
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MLC_INT1,
                            (uint8_t *)&val->mlc_int1, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_INT1,
                            (uint8_t *)&val->emb_func_int1, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FSM_INT1_A,
                            (uint8_t *)&val->fsm_int1_a, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FSM_INT1_B,
                            (uint8_t *)&val->fsm_int1_b, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_INT1_CTRL,
                            (uint8_t *)&val->int1_ctrl, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MD1_CFG,
                            (uint8_t *)&val->md1_cfg, 1);
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
int32_t iis2iclx_pin_int2_route_set(stmdev_ctx_t *ctx,
                                    iis2iclx_pin_int2_route_t *val)
{
  iis2iclx_pin_int1_route_t pin_int1_route;
  iis2iclx_tap_cfg2_t tap_cfg2;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_MLC_INT2,
                             (uint8_t *)&val->mlc_int2, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_EMB_FUNC_INT2,
                             (uint8_t *)&val->emb_func_int2, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FSM_INT2_A,
                             (uint8_t *)&val->fsm_int2_a, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FSM_INT2_B,
                             (uint8_t *)&val->fsm_int2_b, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  if (ret == 0) {
    if (( val->emb_func_int2.int2_fsm_lc
          | val->fsm_int2_a.int2_fsm1
          | val->fsm_int2_a.int2_fsm2
          | val->fsm_int2_a.int2_fsm3
          | val->fsm_int2_a.int2_fsm4
          | val->fsm_int2_a.int2_fsm5
          | val->fsm_int2_a.int2_fsm6
          | val->fsm_int2_a.int2_fsm7
          | val->fsm_int2_a.int2_fsm8
          | val->fsm_int2_b.int2_fsm9
          | val->fsm_int2_b.int2_fsm10
          | val->fsm_int2_b.int2_fsm11
          | val->fsm_int2_b.int2_fsm12
          | val->fsm_int2_b.int2_fsm13
          | val->fsm_int2_b.int2_fsm14
          | val->fsm_int2_b.int2_fsm15
          | val->fsm_int2_b.int2_fsm16
          | val->mlc_int2.int2_mlc1
          | val->mlc_int2.int2_mlc2
          | val->mlc_int2.int2_mlc3
          | val->mlc_int2.int2_mlc4
          | val->mlc_int2.int2_mlc5
          | val->mlc_int2.int2_mlc6
          | val->mlc_int2.int2_mlc7
          | val->mlc_int2.int2_mlc8) != PROPERTY_DISABLE ) {
      val->md2_cfg.int2_emb_func = PROPERTY_ENABLE;
    }

    else {
      val->md2_cfg.int2_emb_func = PROPERTY_DISABLE;
    }

    ret = iis2iclx_write_reg(ctx, IIS2ICLX_INT2_CTRL,
                             (uint8_t *)&val->int2_ctrl, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_MD2_CFG,
                             (uint8_t *)&val->md2_cfg, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG2, (uint8_t *) &tap_cfg2,
                            1);
  }

  if (ret == 0) {
    ret = iis2iclx_pin_int1_route_get(ctx, &pin_int1_route);
  }

  if (ret == 0) {
    if ( ( val->int2_ctrl.int2_cnt_bdr
           | val->int2_ctrl.int2_drdy_temp
           | val->int2_ctrl.int2_drdy_xl
           | val->int2_ctrl.int2_fifo_full
           | val->int2_ctrl.int2_fifo_ovr
           | val->int2_ctrl.int2_fifo_th
           | val->md2_cfg.int2_double_tap
           | val->md2_cfg.int2_wu
           | val->md2_cfg.int2_single_tap
           | val->md2_cfg.int2_sleep_change
           | pin_int1_route.int1_ctrl.den_drdy_flag
           | pin_int1_route.int1_ctrl.int1_boot
           | pin_int1_route.int1_ctrl.int1_cnt_bdr
           | pin_int1_route.int1_ctrl.int1_drdy_xl
           | pin_int1_route.int1_ctrl.int1_fifo_full
           | pin_int1_route.int1_ctrl.int1_fifo_ovr
           | pin_int1_route.int1_ctrl.int1_fifo_th
           | pin_int1_route.md1_cfg.int1_double_tap
           | pin_int1_route.md1_cfg.int1_wu
           | pin_int1_route.md1_cfg.int1_single_tap
           | pin_int1_route.md1_cfg.int1_sleep_change ) != PROPERTY_DISABLE) {
      tap_cfg2.interrupts_enable = PROPERTY_ENABLE;
    }

    else {
      tap_cfg2.interrupts_enable = PROPERTY_DISABLE;
    }

    ret = iis2iclx_write_reg(ctx, IIS2ICLX_TAP_CFG2,
                             (uint8_t *) &tap_cfg2, 1);
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
int32_t iis2iclx_pin_int2_route_get(stmdev_ctx_t *ctx,
                                    iis2iclx_pin_int2_route_t *val)
{
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MLC_INT2,
                            (uint8_t *)&val->mlc_int2, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_INT2,
                            (uint8_t *)&val->emb_func_int2, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FSM_INT2_A,
                            (uint8_t *)&val->fsm_int2_a, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FSM_INT2_B,
                            (uint8_t *)&val->fsm_int2_b, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_INT2_CTRL,
                            (uint8_t *)&val->int2_ctrl, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MD2_CFG,
                            (uint8_t *)&val->md2_cfg, 1);
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
int32_t iis2iclx_pin_mode_set(stmdev_ctx_t *ctx, iis2iclx_pp_od_t val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);

  if (ret == 0) {
    ctrl3_c.pp_od = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                             1);
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
int32_t iis2iclx_pin_mode_get(stmdev_ctx_t *ctx,
                              iis2iclx_pp_od_t *val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);

  switch (ctrl3_c.pp_od) {
    case IIS2ICLX_PUSH_PULL:
      *val = IIS2ICLX_PUSH_PULL;
      break;

    case IIS2ICLX_OPEN_DRAIN:
      *val = IIS2ICLX_OPEN_DRAIN;
      break;

    default:
      *val = IIS2ICLX_PUSH_PULL;
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
int32_t iis2iclx_pin_polarity_set(stmdev_ctx_t *ctx,
                                  iis2iclx_h_lactive_t val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);

  if (ret == 0) {
    ctrl3_c.h_lactive = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                             1);
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
int32_t iis2iclx_pin_polarity_get(stmdev_ctx_t *ctx,
                                  iis2iclx_h_lactive_t *val)
{
  iis2iclx_ctrl3_c_t ctrl3_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);

  switch (ctrl3_c.h_lactive) {
    case IIS2ICLX_ACTIVE_HIGH:
      *val = IIS2ICLX_ACTIVE_HIGH;
      break;

    case IIS2ICLX_ACTIVE_LOW:
      *val = IIS2ICLX_ACTIVE_LOW;
      break;

    default:
      *val = IIS2ICLX_ACTIVE_HIGH;
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
int32_t iis2iclx_all_on_int1_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_ctrl4_c_t ctrl4_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL4_C, (uint8_t *)&ctrl4_c,
                          1);

  if (ret == 0) {
    ctrl4_c.int2_on_int1 = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL4_C, (uint8_t *)&ctrl4_c,
                             1);
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
int32_t iis2iclx_all_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_ctrl4_c_t ctrl4_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL4_C, (uint8_t *)&ctrl4_c,
                          1);
  *val = ctrl4_c.int2_on_int1;
  return ret;
}

/**
  * @brief  All interrupt signals notification mode.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of lir in reg TAP_CFG0
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_int_notification_set(stmdev_ctx_t *ctx,
                                      iis2iclx_lir_t val)
{
  iis2iclx_tap_cfg0_t tap_cfg0;
  iis2iclx_page_rw_t page_rw;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG0, (uint8_t *)&tap_cfg0,
                          1);

  if (ret == 0) {
    tap_cfg0.lir = (uint8_t)val & 0x01U;
    tap_cfg0.int_clr_on_read = (uint8_t)val & 0x01U;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_TAP_CFG0,
                             (uint8_t *)&tap_cfg0, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                            1);
  }

  if (ret == 0) {
    page_rw.emb_func_lir = ((uint8_t)val & 0x02U) >> 1;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                             1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  All interrupt signals notification mode.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of lir in reg TAP_CFG0
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_int_notification_get(stmdev_ctx_t *ctx,
                                      iis2iclx_lir_t *val)
{
  iis2iclx_tap_cfg0_t tap_cfg0;
  iis2iclx_page_rw_t page_rw;
  int32_t ret;
  *val = IIS2ICLX_ALL_INT_PULSED;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG0, (uint8_t *)&tap_cfg0,
                          1);

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_PAGE_RW, (uint8_t *)&page_rw,
                            1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  switch ((page_rw.emb_func_lir << 1) + tap_cfg0.lir) {
    case IIS2ICLX_ALL_INT_PULSED:
      *val = IIS2ICLX_ALL_INT_PULSED;
      break;

    case IIS2ICLX_BASE_LATCHED_EMB_PULSED:
      *val = IIS2ICLX_BASE_LATCHED_EMB_PULSED;
      break;

    case IIS2ICLX_BASE_PULSED_EMB_LATCHED:
      *val = IIS2ICLX_BASE_PULSED_EMB_LATCHED;
      break;

    case IIS2ICLX_ALL_INT_LATCHED:
      *val = IIS2ICLX_ALL_INT_LATCHED;
      break;

    default:
      *val = IIS2ICLX_ALL_INT_PULSED;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS2ICLX_Wake_Up_event
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
int32_t iis2iclx_wkup_ths_weight_set(stmdev_ctx_t *ctx,
                                     iis2iclx_wake_ths_w_t val)
{
  iis2iclx_wake_up_dur_t wake_up_dur;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_WAKE_UP_DUR,
                          (uint8_t *)&wake_up_dur, 1);

  if (ret == 0) {
    wake_up_dur.wake_ths_w = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_WAKE_UP_DUR,
                             (uint8_t *)&wake_up_dur, 1);
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
int32_t iis2iclx_wkup_ths_weight_get(stmdev_ctx_t *ctx,
                                     iis2iclx_wake_ths_w_t *val)
{
  iis2iclx_wake_up_dur_t wake_up_dur;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_WAKE_UP_DUR,
                          (uint8_t *)&wake_up_dur, 1);

  switch (wake_up_dur.wake_ths_w) {
    case IIS2ICLX_LSb_FS_DIV_64:
      *val = IIS2ICLX_LSb_FS_DIV_64;
      break;

    case IIS2ICLX_LSb_FS_DIV_256:
      *val = IIS2ICLX_LSb_FS_DIV_256;
      break;

    default:
      *val = IIS2ICLX_LSb_FS_DIV_64;
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
int32_t iis2iclx_wkup_threshold_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_wake_up_ths_t wake_up_ths;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_WAKE_UP_THS,
                          (uint8_t *)&wake_up_ths, 1);

  if (ret == 0) {
    wake_up_ths.wk_ths = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_WAKE_UP_THS,
                             (uint8_t *)&wake_up_ths, 1);
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
int32_t iis2iclx_wkup_threshold_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_wake_up_ths_t wake_up_ths;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_WAKE_UP_THS,
                          (uint8_t *)&wake_up_ths, 1);
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
int32_t iis2iclx_xl_usr_offset_on_wkup_set(stmdev_ctx_t *ctx,
                                           uint8_t val)
{
  iis2iclx_wake_up_ths_t wake_up_ths;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_WAKE_UP_THS,
                          (uint8_t *)&wake_up_ths, 1);

  if (ret == 0) {
    wake_up_ths.usr_off_on_wu = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_WAKE_UP_THS,
                             (uint8_t *)&wake_up_ths, 1);
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
int32_t iis2iclx_xl_usr_offset_on_wkup_get(stmdev_ctx_t *ctx,
                                           uint8_t *val)
{
  iis2iclx_wake_up_ths_t wake_up_ths;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_WAKE_UP_THS,
                          (uint8_t *)&wake_up_ths, 1);
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
int32_t iis2iclx_wkup_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_wake_up_dur_t wake_up_dur;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_WAKE_UP_DUR,
                          (uint8_t *)&wake_up_dur, 1);

  if (ret == 0) {
    wake_up_dur.wake_dur = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_WAKE_UP_DUR,
                             (uint8_t *)&wake_up_dur, 1);
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
int32_t iis2iclx_wkup_dur_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_wake_up_dur_t wake_up_dur;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_WAKE_UP_DUR,
                          (uint8_t *)&wake_up_dur, 1);
  *val = wake_up_dur.wake_dur;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS2ICLX_ Activity/Inactivity_detection
  * @brief      This section groups all the functions concerning
  *             activity/inactivity detection.
  * @{
  *
  */

/**
  * @brief  Drives the sleep status instead of sleep change on INT pins
  *         (only if INT1_SLEEP_CHANGE or INT2_SLEEP_CHANGE bits
  *         are enabled).[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of sleep_status_on_int in reg TAP_CFG0
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_act_pin_notification_set(stmdev_ctx_t *ctx,
                                          iis2iclx_sleep_status_on_int_t val)
{
  iis2iclx_tap_cfg0_t tap_cfg0;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG0, (uint8_t *)&tap_cfg0,
                          1);

  if (ret == 0) {
    tap_cfg0. sleep_status_on_int = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_TAP_CFG0,
                             (uint8_t *)&tap_cfg0, 1);
  }

  return ret;
}

/**
  * @brief  Drives the sleep status instead of sleep change on INT pins
  *         (only if INT1_SLEEP_CHANGE or INT2_SLEEP_CHANGE bits
  *         are enabled).[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of sleep_status_on_int in reg TAP_CFG0
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_act_pin_notification_get(stmdev_ctx_t *ctx,
                                          iis2iclx_sleep_status_on_int_t *val)
{
  iis2iclx_tap_cfg0_t tap_cfg0;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG0, (uint8_t *)&tap_cfg0,
                          1);

  switch (tap_cfg0. sleep_status_on_int) {
    case IIS2ICLX_DRIVE_SLEEP_CHG_EVENT:
      *val = IIS2ICLX_DRIVE_SLEEP_CHG_EVENT;
      break;

    case IIS2ICLX_DRIVE_SLEEP_STATUS:
      *val = IIS2ICLX_DRIVE_SLEEP_STATUS;
      break;

    default:
      *val = IIS2ICLX_DRIVE_SLEEP_CHG_EVENT;
      break;
  }

  return ret;
}

/**
  * @brief  Duration to go in sleep mode (1 LSb = 512 / ODR).[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of sleep_dur in reg WAKE_UP_DUR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_act_sleep_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_wake_up_dur_t wake_up_dur;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_WAKE_UP_DUR,
                          (uint8_t *)&wake_up_dur, 1);

  if (ret == 0) {
    wake_up_dur.sleep_dur = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_WAKE_UP_DUR,
                             (uint8_t *)&wake_up_dur, 1);
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
int32_t iis2iclx_act_sleep_dur_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_wake_up_dur_t wake_up_dur;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_WAKE_UP_DUR,
                          (uint8_t *)&wake_up_dur, 1);
  *val = wake_up_dur.sleep_dur;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS2ICLX_tap_generator
  * @brief      This section groups all the functions that manage the
  *             tap and double tap event generation.
  * @{
  *
  */

/**
  * @brief  Enable Y direction in tap recognition.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of tap_y_en in reg TAP_CFG0
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_detection_on_y_set(stmdev_ctx_t *ctx,
                                        uint8_t val)
{
  iis2iclx_tap_cfg0_t tap_cfg0;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG0, (uint8_t *)&tap_cfg0,
                          1);

  if (ret == 0) {
    tap_cfg0.tap_y_en = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_TAP_CFG0,
                             (uint8_t *)&tap_cfg0, 1);
  }

  return ret;
}

/**
  * @brief  Enable Y direction in tap recognition.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of tap_y_en in reg TAP_CFG0
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_detection_on_y_get(stmdev_ctx_t *ctx,
                                        uint8_t *val)
{
  iis2iclx_tap_cfg0_t tap_cfg0;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG0, (uint8_t *)&tap_cfg0,
                          1);
  *val = tap_cfg0.tap_y_en;
  return ret;
}

/**
  * @brief  Enable X direction in tap recognition.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of tap_x_en in reg TAP_CFG0
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_detection_on_x_set(stmdev_ctx_t *ctx,
                                        uint8_t val)
{
  iis2iclx_tap_cfg0_t tap_cfg0;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG0, (uint8_t *)&tap_cfg0,
                          1);

  if (ret == 0) {
    tap_cfg0.tap_x_en = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_TAP_CFG0,
                             (uint8_t *)&tap_cfg0, 1);
  }

  return ret;
}

/**
  * @brief  Enable X direction in tap recognition.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of tap_x_en in reg TAP_CFG0
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_detection_on_x_get(stmdev_ctx_t *ctx,
                                        uint8_t *val)
{
  iis2iclx_tap_cfg0_t tap_cfg0;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG0, (uint8_t *)&tap_cfg0,
                          1);
  *val = tap_cfg0.tap_x_en;
  return ret;
}

/**
  * @brief  X-axis tap recognition threshold.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of tap_ths_x in reg TAP_CFG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_threshold_x_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_tap_cfg1_t tap_cfg1;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG1, (uint8_t *)&tap_cfg1,
                          1);

  if (ret == 0) {
    tap_cfg1.tap_ths_x = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_TAP_CFG1,
                             (uint8_t *)&tap_cfg1, 1);
  }

  return ret;
}

/**
  * @brief  X-axis tap recognition threshold.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of tap_ths_x in reg TAP_CFG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_threshold_x_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_tap_cfg1_t tap_cfg1;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG1, (uint8_t *)&tap_cfg1,
                          1);
  *val = tap_cfg1.tap_ths_x;
  return ret;
}

/**
  * @brief  Selection of axis priority for TAP detection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of tap_priority in reg TAP_CFG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_axis_priority_set(stmdev_ctx_t *ctx,
                                       iis2iclx_tap_priority_t val)
{
  iis2iclx_tap_cfg1_t tap_cfg1;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG1, (uint8_t *)&tap_cfg1,
                          1);

  if (ret == 0) {
    tap_cfg1.tap_priority = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_TAP_CFG1, (uint8_t *)&tap_cfg1,
                             1);
  }

  return ret;
}

/**
  * @brief  Selection of axis priority for TAP detection[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of tap_priority in reg TAP_CFG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_axis_priority_get(stmdev_ctx_t *ctx,
                                       iis2iclx_tap_priority_t *val)
{
  iis2iclx_tap_cfg1_t tap_cfg1;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG1, (uint8_t *)&tap_cfg1,
                          1);

  switch (tap_cfg1.tap_priority) {
    case IIS2ICLX_XY:
      *val = IIS2ICLX_XY;
      break;

    case IIS2ICLX_YX:
      *val = IIS2ICLX_YX;
      break;

    default:
      *val = IIS2ICLX_XY;
      break;
  }

  return ret;
}

/**
  * @brief  Y-axis tap recognition threshold.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of tap_ths_y in reg TAP_CFG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_threshold_y_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_tap_cfg2_t tap_cfg2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG2, (uint8_t *)&tap_cfg2,
                          1);

  if (ret == 0) {
    tap_cfg2.tap_ths_y = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_TAP_CFG2,
                             (uint8_t *)&tap_cfg2, 1);
  }

  return ret;
}

/**
  * @brief  Y-axis tap recognition threshold.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of tap_ths_y in reg TAP_CFG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_threshold_y_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_tap_cfg2_t tap_cfg2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_TAP_CFG2, (uint8_t *)&tap_cfg2,
                          1);
  *val = tap_cfg2.tap_ths_y;
  return ret;
}

/**
  * @brief  Maximum duration is the maximum time of an overthreshold signal
  *         detection to be recognized as a tap event. The default value of
  *         these bits is 00b which corresponds to 4*ODR_XL time.
  *         If the SHOCK[1:0] bits are set to a different value, 1LSB
  *         corresponds to 8*ODR_XL time.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of shock in reg INT_DUR2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_shock_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_int_dur2_t int_dur2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_INT_DUR2, (uint8_t *)&int_dur2,
                          1);

  if (ret == 0) {
    int_dur2.shock = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_INT_DUR2,
                             (uint8_t *)&int_dur2, 1);
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
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of shock in reg INT_DUR2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_shock_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_int_dur2_t int_dur2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_INT_DUR2, (uint8_t *)&int_dur2,
                          1);
  *val = int_dur2.shock;
  return ret;
}

/**
  * @brief  Quiet time is the time after the first detected tap in which
  *         there must not be any overthreshold event.
  *         The default value of these bits is 00b which corresponds to
  *         2*ODR_XL time. If the QUIET[1:0] bits are set to a different
  *         value, 1LSB corresponds to 4*ODR_XL time.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of quiet in reg INT_DUR2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_quiet_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_int_dur2_t int_dur2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_INT_DUR2, (uint8_t *)&int_dur2,
                          1);

  if (ret == 0) {
    int_dur2.quiet = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_INT_DUR2,
                             (uint8_t *)&int_dur2, 1);
  }

  return ret;
}

/**
  * @brief  Quiet time is the time after the first detected tap in which
  *         there must not be any overthreshold event.
  *         The default value of these bits is 00b which corresponds to
  *         2*ODR_XL time. If the QUIET[1:0] bits are set to a different
  *         value, 1LSB corresponds to 4*ODR_XL time.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of quiet in reg INT_DUR2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_quiet_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_int_dur2_t int_dur2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_INT_DUR2, (uint8_t *)&int_dur2,
                          1);
  *val = int_dur2.quiet;
  return ret;
}

/**
  * @brief  When double tap recognition is enabled, this register expresses
  *         the maximum time between two consecutive detected taps to
  *         determine a double tap event.
  *         The default value of these bits is 0000b which corresponds to
  *         16*ODR_XL time.
  *         If the DUR[3:0] bits are set to a different value, 1LSB
  *         corresponds to 32*ODR_XL time.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of dur in reg INT_DUR2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_int_dur2_t int_dur2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_INT_DUR2, (uint8_t *)&int_dur2,
                          1);

  if (ret == 0) {
    int_dur2.dur = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_INT_DUR2,
                             (uint8_t *)&int_dur2, 1);
  }

  return ret;
}

/**
  * @brief  When double tap recognition is enabled, this register expresses the
  *         maximum time between two consecutive detected taps to determine
  *         a double tap event. The default value of these bits is 0000b which
  *         corresponds to 16*ODR_XL time. If the DUR[3:0] bits are set to
  *         a different value, 1LSB corresponds to 32*ODR_XL time.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of dur in reg INT_DUR2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_dur_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_int_dur2_t int_dur2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_INT_DUR2, (uint8_t *)&int_dur2,
                          1);
  *val = int_dur2.dur;
  return ret;
}

/**
  * @brief  Single/double-tap event enable.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of single_double_tap in reg WAKE_UP_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_mode_set(stmdev_ctx_t *ctx,
                              iis2iclx_single_double_tap_t val)
{
  iis2iclx_wake_up_ths_t wake_up_ths;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_WAKE_UP_THS,
                          (uint8_t *)&wake_up_ths, 1);

  if (ret == 0) {
    wake_up_ths.single_double_tap = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_WAKE_UP_THS,
                             (uint8_t *)&wake_up_ths, 1);
  }

  return ret;
}

/**
  * @brief  Single/double-tap event enable.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of single_double_tap in reg WAKE_UP_THS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_tap_mode_get(stmdev_ctx_t *ctx,
                              iis2iclx_single_double_tap_t *val)
{
  iis2iclx_wake_up_ths_t wake_up_ths;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_WAKE_UP_THS,
                          (uint8_t *)&wake_up_ths, 1);

  switch (wake_up_ths.single_double_tap) {
    case IIS2ICLX_ONLY_SINGLE:
      *val = IIS2ICLX_ONLY_SINGLE;
      break;

    case IIS2ICLX_BOTH_SINGLE_DOUBLE:
      *val = IIS2ICLX_BOTH_SINGLE_DOUBLE;
      break;

    default:
      *val = IIS2ICLX_ONLY_SINGLE;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS2ICLX_fifo
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
int32_t iis2iclx_fifo_watermark_set(stmdev_ctx_t *ctx, uint16_t val)
{
  iis2iclx_fifo_ctrl1_t fifo_ctrl1;
  iis2iclx_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL2,
                          (uint8_t *)&fifo_ctrl2, 1);

  if (ret == 0) {
    fifo_ctrl1.wtm = (uint8_t)(0x00FFU & val);
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FIFO_CTRL1,
                             (uint8_t *)&fifo_ctrl1, 1);
  }

  if (ret == 0) {
    fifo_ctrl2.wtm = (uint8_t)(( 0x0100U & val ) >> 8);
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FIFO_CTRL2,
                             (uint8_t *)&fifo_ctrl2, 1);
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
int32_t iis2iclx_fifo_watermark_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  iis2iclx_fifo_ctrl1_t fifo_ctrl1;
  iis2iclx_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL2,
                          (uint8_t *)&fifo_ctrl2, 1);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL1,
                            (uint8_t *)&fifo_ctrl1, 1);
  }

  *val = fifo_ctrl2.wtm;
  *val = *val << 8;
  *val += fifo_ctrl1.wtm;
  return ret;
}

/**
  * @brief  Enables ODR CHANGE virtual sensor to be batched in FIFO.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of odrchg_en in reg FIFO_CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_fifo_virtual_sens_odr_chg_set(stmdev_ctx_t *ctx,
                                               uint8_t val)
{
  iis2iclx_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL2,
                          (uint8_t *)&fifo_ctrl2, 1);

  if (ret == 0) {
    fifo_ctrl2.odrchg_en = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FIFO_CTRL2,
                             (uint8_t *)&fifo_ctrl2, 1);
  }

  return ret;
}

/**
  * @brief  Enables ODR CHANGE virtual sensor to be batched in FIFO.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of odrchg_en in reg FIFO_CTRL2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_fifo_virtual_sens_odr_chg_get(stmdev_ctx_t *ctx,
                                               uint8_t *val)
{
  iis2iclx_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL2,
                          (uint8_t *)&fifo_ctrl2, 1);
  *val = fifo_ctrl2.odrchg_en;
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
int32_t iis2iclx_fifo_stop_on_wtm_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL2,
                          (uint8_t *)&fifo_ctrl2, 1);

  if (ret == 0) {
    fifo_ctrl2.stop_on_wtm = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FIFO_CTRL2,
                             (uint8_t *)&fifo_ctrl2, 1);
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
int32_t iis2iclx_fifo_stop_on_wtm_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL2,
                          (uint8_t *)&fifo_ctrl2, 1);
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
int32_t iis2iclx_fifo_xl_batch_set(stmdev_ctx_t *ctx,
                                   iis2iclx_bdr_xl_t val)
{
  iis2iclx_fifo_ctrl3_t fifo_ctrl3;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL3,
                          (uint8_t *)&fifo_ctrl3, 1);

  if (ret == 0) {
    fifo_ctrl3.bdr_xl = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FIFO_CTRL3,
                             (uint8_t *)&fifo_ctrl3, 1);
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
int32_t iis2iclx_fifo_xl_batch_get(stmdev_ctx_t *ctx,
                                   iis2iclx_bdr_xl_t *val)
{
  iis2iclx_fifo_ctrl3_t fifo_ctrl3;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL3,
                          (uint8_t *)&fifo_ctrl3, 1);

  switch (fifo_ctrl3.bdr_xl) {
    case IIS2ICLX_XL_NOT_BATCHED:
      *val = IIS2ICLX_XL_NOT_BATCHED;
      break;

    case IIS2ICLX_XL_BATCHED_AT_1Hz6:
      *val = IIS2ICLX_XL_BATCHED_AT_1Hz6;
      break;

    case IIS2ICLX_XL_BATCHED_AT_12Hz5:
      *val = IIS2ICLX_XL_BATCHED_AT_12Hz5;
      break;

    case IIS2ICLX_XL_BATCHED_AT_26Hz:
      *val = IIS2ICLX_XL_BATCHED_AT_26Hz;
      break;

    case IIS2ICLX_XL_BATCHED_AT_52Hz:
      *val = IIS2ICLX_XL_BATCHED_AT_52Hz;
      break;

    case IIS2ICLX_XL_BATCHED_AT_104Hz:
      *val = IIS2ICLX_XL_BATCHED_AT_104Hz;
      break;

    case IIS2ICLX_XL_BATCHED_AT_208Hz:
      *val = IIS2ICLX_XL_BATCHED_AT_208Hz;
      break;

    case IIS2ICLX_XL_BATCHED_AT_417Hz:
      *val = IIS2ICLX_XL_BATCHED_AT_417Hz;
      break;

    case IIS2ICLX_XL_BATCHED_AT_833Hz:
      *val = IIS2ICLX_XL_BATCHED_AT_833Hz;
      break;

    default:
      *val = IIS2ICLX_XL_NOT_BATCHED;
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
int32_t iis2iclx_fifo_mode_set(stmdev_ctx_t *ctx,
                               iis2iclx_fifo_mode_t val)
{
  iis2iclx_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL4,
                          (uint8_t *)&fifo_ctrl4, 1);

  if (ret == 0) {
    fifo_ctrl4.fifo_mode = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FIFO_CTRL4,
                             (uint8_t *)&fifo_ctrl4, 1);
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
int32_t iis2iclx_fifo_mode_get(stmdev_ctx_t *ctx,
                               iis2iclx_fifo_mode_t *val)
{
  iis2iclx_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL4,
                          (uint8_t *)&fifo_ctrl4, 1);

  switch (fifo_ctrl4.fifo_mode) {
    case IIS2ICLX_BYPASS_MODE:
      *val = IIS2ICLX_BYPASS_MODE;
      break;

    case IIS2ICLX_FIFO_MODE:
      *val = IIS2ICLX_FIFO_MODE;
      break;

    case IIS2ICLX_STREAM_TO_FIFO_MODE:
      *val = IIS2ICLX_STREAM_TO_FIFO_MODE;
      break;

    case IIS2ICLX_BYPASS_TO_STREAM_MODE:
      *val = IIS2ICLX_BYPASS_TO_STREAM_MODE;
      break;

    case IIS2ICLX_STREAM_MODE:
      *val = IIS2ICLX_STREAM_MODE;
      break;

    case IIS2ICLX_BYPASS_TO_FIFO_MODE:
      *val = IIS2ICLX_BYPASS_TO_FIFO_MODE;
      break;

    default:
      *val = IIS2ICLX_BYPASS_MODE;
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
int32_t iis2iclx_fifo_temp_batch_set(stmdev_ctx_t *ctx,
                                     iis2iclx_odr_t_batch_t val)
{
  iis2iclx_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL4,
                          (uint8_t *)&fifo_ctrl4, 1);

  if (ret == 0) {
    fifo_ctrl4.odr_t_batch = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FIFO_CTRL4,
                             (uint8_t *)&fifo_ctrl4, 1);
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
int32_t iis2iclx_fifo_temp_batch_get(stmdev_ctx_t *ctx,
                                     iis2iclx_odr_t_batch_t *val)
{
  iis2iclx_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL4,
                          (uint8_t *)&fifo_ctrl4, 1);

  switch (fifo_ctrl4.odr_t_batch) {
    case IIS2ICLX_TEMP_NOT_BATCHED:
      *val = IIS2ICLX_TEMP_NOT_BATCHED;
      break;

    case IIS2ICLX_TEMP_BATCHED_AT_1Hz6:
      *val = IIS2ICLX_TEMP_BATCHED_AT_1Hz6;
      break;

    case IIS2ICLX_TEMP_BATCHED_AT_52Hz:
      *val = IIS2ICLX_TEMP_BATCHED_AT_52Hz;
      break;

    case IIS2ICLX_TEMP_BATCHED_AT_12Hz5:
      *val = IIS2ICLX_TEMP_BATCHED_AT_12Hz5;
      break;

    default:
      *val = IIS2ICLX_TEMP_NOT_BATCHED;
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
int32_t iis2iclx_fifo_timestamp_decimation_set(stmdev_ctx_t *ctx,
                                               iis2iclx_odr_ts_batch_t val)
{
  iis2iclx_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL4,
                          (uint8_t *)&fifo_ctrl4, 1);

  if (ret == 0) {
    fifo_ctrl4.odr_ts_batch = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FIFO_CTRL4,
                             (uint8_t *)&fifo_ctrl4, 1);
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
int32_t iis2iclx_fifo_timestamp_decimation_get(stmdev_ctx_t *ctx,
                                               iis2iclx_odr_ts_batch_t *val)
{
  iis2iclx_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_CTRL4,
                          (uint8_t *)&fifo_ctrl4, 1);

  switch (fifo_ctrl4.odr_ts_batch) {
    case IIS2ICLX_NO_DECIMATION:
      *val = IIS2ICLX_NO_DECIMATION;
      break;

    case IIS2ICLX_DEC_1:
      *val = IIS2ICLX_DEC_1;
      break;

    case IIS2ICLX_DEC_8:
      *val = IIS2ICLX_DEC_8;
      break;

    case IIS2ICLX_DEC_32:
      *val = IIS2ICLX_DEC_32;
      break;

    default:
      *val = IIS2ICLX_NO_DECIMATION;
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
int32_t iis2iclx_rst_batch_counter_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_counter_bdr_reg1_t counter_bdr_reg1;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_COUNTER_BDR_REG1,
                          (uint8_t *)&counter_bdr_reg1, 1);

  if (ret == 0) {
    counter_bdr_reg1.rst_counter_bdr = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_COUNTER_BDR_REG1,
                             (uint8_t *)&counter_bdr_reg1, 1);
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
int32_t iis2iclx_rst_batch_counter_get(stmdev_ctx_t *ctx,
                                       uint8_t *val)
{
  iis2iclx_counter_bdr_reg1_t counter_bdr_reg1;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_COUNTER_BDR_REG1,
                          (uint8_t *)&counter_bdr_reg1, 1);
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
int32_t iis2iclx_batch_counter_threshold_set(stmdev_ctx_t *ctx,
                                             uint16_t val)
{
  iis2iclx_counter_bdr_reg2_t counter_bdr_reg1;
  iis2iclx_counter_bdr_reg2_t counter_bdr_reg2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_COUNTER_BDR_REG1,
                          (uint8_t *)&counter_bdr_reg1, 1);

  if (ret == 0) {
    counter_bdr_reg1.cnt_bdr_th = (uint8_t)((0x0700U & val) >> 8);
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_COUNTER_BDR_REG1,
                             (uint8_t *)&counter_bdr_reg1, 1);
  }

  if (ret == 0) {
    counter_bdr_reg2.cnt_bdr_th = (uint8_t)(0x00FFU & val);
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_COUNTER_BDR_REG2,
                             (uint8_t *)&counter_bdr_reg2, 1);
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
int32_t iis2iclx_batch_counter_threshold_get(stmdev_ctx_t *ctx,
                                             uint16_t *val)
{
  iis2iclx_counter_bdr_reg1_t counter_bdr_reg1;
  iis2iclx_counter_bdr_reg2_t counter_bdr_reg2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_COUNTER_BDR_REG1,
                          (uint8_t *)&counter_bdr_reg1, 1);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_COUNTER_BDR_REG2,
                            (uint8_t *)&counter_bdr_reg2, 1);
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
int32_t iis2iclx_fifo_data_level_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  iis2iclx_fifo_status1_t fifo_status1;
  iis2iclx_fifo_status2_t fifo_status2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_STATUS1,
                          (uint8_t *)&fifo_status1, 1);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_STATUS2,
                            (uint8_t *)&fifo_status2, 1);
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
int32_t iis2iclx_fifo_status_get(stmdev_ctx_t *ctx,
                                 iis2iclx_fifo_status2_t *val)
{
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_STATUS2, (uint8_t *)val,
                          1);
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
int32_t iis2iclx_fifo_full_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_fifo_status2_t fifo_status2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_STATUS2,
                          (uint8_t *)&fifo_status2, 1);
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
int32_t iis2iclx_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_fifo_status2_t fifo_status2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_STATUS2,
                          (uint8_t *)&fifo_status2, 1);
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
int32_t iis2iclx_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_fifo_status2_t fifo_status2;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_STATUS2,
                          (uint8_t *)&fifo_status2, 1);
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
int32_t iis2iclx_fifo_sensor_tag_get(stmdev_ctx_t *ctx,
                                     iis2iclx_fifo_tag_t *val)
{
  iis2iclx_fifo_data_out_tag_t fifo_data_out_tag;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_FIFO_DATA_OUT_TAG,
                          (uint8_t *)&fifo_data_out_tag, 1);

  switch (fifo_data_out_tag.tag_sensor) {
    case IIS2ICLX_XL_NC_TAG:
      *val = IIS2ICLX_XL_NC_TAG;
      break;

    case IIS2ICLX_TEMPERATURE_TAG:
      *val = IIS2ICLX_TEMPERATURE_TAG;
      break;

    case IIS2ICLX_TIMESTAMP_TAG:
      *val = IIS2ICLX_TIMESTAMP_TAG;
      break;

    case IIS2ICLX_CFG_CHANGE_TAG:
      *val = IIS2ICLX_CFG_CHANGE_TAG;
      break;

    case IIS2ICLX_SENSORHUB_SLAVE0_TAG:
      *val = IIS2ICLX_SENSORHUB_SLAVE0_TAG;
      break;

    case IIS2ICLX_SENSORHUB_SLAVE1_TAG:
      *val = IIS2ICLX_SENSORHUB_SLAVE1_TAG;
      break;

    case IIS2ICLX_SENSORHUB_SLAVE2_TAG:
      *val = IIS2ICLX_SENSORHUB_SLAVE2_TAG;
      break;

    case IIS2ICLX_SENSORHUB_SLAVE3_TAG:
      *val = IIS2ICLX_SENSORHUB_SLAVE3_TAG;
      break;

    case IIS2ICLX_SENSORHUB_NACK_TAG:
      *val = IIS2ICLX_SENSORHUB_NACK_TAG;
      break;

    default:
      *val = IIS2ICLX_SENSORHUB_NACK_TAG;
      break;
  }

  return ret;
}

/**
  * @brief  Enable FIFO batching data of first slave.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of  batch_ext_sens_0_en in reg SLV0_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_batch_slave_0_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_slv0_config_t slv0_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV0_CONFIG,
                            (uint8_t *)&slv0_config, 1);
  }

  if (ret == 0) {
    slv0_config. batch_ext_sens_0_en = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV0_CONFIG,
                             (uint8_t *)&slv0_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Enable FIFO batching data of first slave.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of  batch_ext_sens_0_en in
  *                reg SLV0_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_batch_slave_0_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_slv0_config_t slv0_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV0_CONFIG,
                            (uint8_t *)&slv0_config, 1);
  }

  if (ret == 0) {
    *val = slv0_config. batch_ext_sens_0_en;
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Enable FIFO batching data of second slave.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of  batch_ext_sens_1_en in
  *                reg SLV1_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_batch_slave_1_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_slv1_config_t slv1_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV1_CONFIG,
                            (uint8_t *)&slv1_config, 1);
  }

  if (ret == 0) {
    slv1_config. batch_ext_sens_1_en = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV1_CONFIG,
                             (uint8_t *)&slv1_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Enable FIFO batching data of second slave.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of  batch_ext_sens_1_en in
  *                reg SLV1_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_batch_slave_1_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_slv1_config_t slv1_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV1_CONFIG,
                            (uint8_t *)&slv1_config, 1);
    *val = slv1_config. batch_ext_sens_1_en;
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Enable FIFO batching data of third slave.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of  batch_ext_sens_2_en in
  *                reg SLV2_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_batch_slave_2_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_slv2_config_t slv2_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV2_CONFIG,
                            (uint8_t *)&slv2_config, 1);
  }

  if (ret == 0) {
    slv2_config. batch_ext_sens_2_en = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV2_CONFIG,
                             (uint8_t *)&slv2_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Enable FIFO batching data of third slave.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of  batch_ext_sens_2_en in
  *                reg SLV2_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_batch_slave_2_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_slv2_config_t slv2_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV2_CONFIG,
                            (uint8_t *)&slv2_config, 1);
  }

  if (ret == 0) {
    *val = slv2_config. batch_ext_sens_2_en;
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Enable FIFO batching data of fourth slave.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of  batch_ext_sens_3_en in
  *                reg SLV3_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_batch_slave_3_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_slv3_config_t slv3_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV3_CONFIG,
                            (uint8_t *)&slv3_config, 1);
  }

  if (ret == 0) {
    slv3_config. batch_ext_sens_3_en = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV3_CONFIG,
                             (uint8_t *)&slv3_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Enable FIFO batching data of fourth slave.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of  batch_ext_sens_3_en in
  *                reg SLV3_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_batch_slave_3_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_slv3_config_t slv3_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV3_CONFIG,
                            (uint8_t *)&slv3_config, 1);
    *val = slv3_config. batch_ext_sens_3_en;
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS2ICLX_DEN_functionality
  * @brief      This section groups all the functions concerning
  *             DEN functionality.
  * @{
  *
  */

/**
  * @brief  DEN functionality marking mode.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of den_mode in reg CTRL6_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_den_mode_set(stmdev_ctx_t *ctx,
                              iis2iclx_den_mode_t val)
{
  iis2iclx_ctrl6_c_t ctrl6_c;
  iis2iclx_ctrl9_xl_t ctrl9_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                          1);

  if (ret == 0) {
    ctrl9_xl.den_en = ( (uint8_t)val & 0x70U ) >> 4;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL9_XL,
                             (uint8_t *)&ctrl9_xl, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL6_C, (uint8_t *)&ctrl6_c,
                            1);
  }

  if (ret == 0) {
    ctrl6_c.den_mode = (uint8_t)val & 0x07U;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL6_C, (uint8_t *)&ctrl6_c,
                             1);
  }

  return ret;
}

/**
  * @brief  DEN functionality marking mode.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of den_mode in reg CTRL6_C
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_den_mode_get(stmdev_ctx_t *ctx,
                              iis2iclx_den_mode_t *val)
{
  iis2iclx_ctrl6_c_t ctrl6_c;
  iis2iclx_ctrl9_xl_t ctrl9_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL6_C, (uint8_t *)&ctrl6_c,
                          1);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                            1);
  }

  switch ( (ctrl9_xl.den_en << 4) + ctrl6_c.den_mode) {
    case IIS2ICLX_DEN_DISABLE:
      *val = IIS2ICLX_DEN_DISABLE;
      break;

    case IIS2ICLX_LEVEL_FIFO:
      *val = IIS2ICLX_LEVEL_FIFO;
      break;

    case IIS2ICLX_LEVEL_LETCHED:
      *val = IIS2ICLX_LEVEL_LETCHED;
      break;

    case IIS2ICLX_LEVEL_TRIGGER:
      *val = IIS2ICLX_LEVEL_TRIGGER;
      break;

    case IIS2ICLX_EDGE_TRIGGER:
      *val = IIS2ICLX_EDGE_TRIGGER;
      break;

    default:
      *val = IIS2ICLX_DEN_DISABLE;
      break;
  }

  return ret;
}

/**
  * @brief  DEN active level configuration.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of den_lh in reg CTRL9_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_den_polarity_set(stmdev_ctx_t *ctx,
                                  iis2iclx_den_lh_t val)
{
  iis2iclx_ctrl9_xl_t ctrl9_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                          1);

  if (ret == 0) {
    ctrl9_xl.den_lh = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL9_XL,
                             (uint8_t *)&ctrl9_xl, 1);
  }

  return ret;
}

/**
  * @brief  DEN active level configuration.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of den_lh in reg CTRL9_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_den_polarity_get(stmdev_ctx_t *ctx,
                                  iis2iclx_den_lh_t *val)
{
  iis2iclx_ctrl9_xl_t ctrl9_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                          1);

  switch (ctrl9_xl.den_lh) {
    case IIS2ICLX_DEN_ACT_LOW:
      *val = IIS2ICLX_DEN_ACT_LOW;
      break;

    case IIS2ICLX_DEN_ACT_HIGH:
      *val = IIS2ICLX_DEN_ACT_HIGH;
      break;

    default:
      *val = IIS2ICLX_DEN_ACT_LOW;
      break;
  }

  return ret;
}

/**
  * @brief  DEN value stored in LSB of Y-axis.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of den_y in reg CTRL9_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_den_mark_axis_y_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_ctrl9_xl_t ctrl9_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                          1);

  if (ret == 0) {
    ctrl9_xl.den_y = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL9_XL,
                             (uint8_t *)&ctrl9_xl, 1);
  }

  return ret;
}

/**
  * @brief  DEN value stored in LSB of Y-axis.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of den_y in reg CTRL9_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_den_mark_axis_y_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_ctrl9_xl_t ctrl9_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                          1);
  *val = ctrl9_xl.den_y;
  return ret;
}

/**
  * @brief  DEN value stored in LSB of Z-axis.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of den_x in reg CTRL9_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_den_mark_axis_x_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_ctrl9_xl_t ctrl9_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                          1);

  if (ret == 0) {
    ctrl9_xl.den_x = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                             1);
  }

  return ret;
}

/**
  * @brief  DEN value stored in LSB of Z-axis.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of den_x in reg CTRL9_XL
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_den_mark_axis_x_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_ctrl9_xl_t ctrl9_xl;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                          1);
  *val = ctrl9_xl.den_x;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS2ICLX_finite_state_machine
  * @brief      This section groups all the functions that manage the
  *             state_machine.
  * @{
  *
  */

/**
  * @brief  Interrupt status bit for FSM long counter timeout interrupt
  *         event.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of is_fsm_lc in reg EMB_FUNC_STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_long_cnt_flag_data_ready_get(stmdev_ctx_t *ctx,
                                              uint8_t *val)
{
  iis2iclx_emb_func_status_t emb_func_status;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_STATUS,
                            (uint8_t *)&emb_func_status, 1);
  }

  if (ret == 0) {
    *val = emb_func_status.is_fsm_lc;
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Embedded final state machine functions mode.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fsm_en in reg EMB_FUNC_EN_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_emb_fsm_en_set(stmdev_ctx_t *ctx, uint8_t val)
{
  int32_t ret;
  iis2iclx_emb_func_en_b_t emb_func_en_b;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_EN_B,
                            (uint8_t *)&emb_func_en_b, 1);
  }

  if (ret == 0) {
    emb_func_en_b.fsm_en = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_EMB_FUNC_EN_B,
                             (uint8_t *)&emb_func_en_b, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Embedded final state machine functions mode.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fsm_en in reg EMB_FUNC_EN_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_emb_fsm_en_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  int32_t ret;
  iis2iclx_emb_func_en_b_t emb_func_en_b;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_EN_B,
                            (uint8_t *)&emb_func_en_b, 1);
  }

  if (ret == 0) {
    *val = emb_func_en_b.fsm_en;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_EMB_FUNC_EN_B,
                             (uint8_t *)&emb_func_en_b, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Embedded final state machine functions mode.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure of registers from FSM_ENABLE_A to FSM_ENABLE_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_fsm_enable_set(stmdev_ctx_t *ctx,
                                iis2iclx_emb_fsm_enable_t *val)
{
  iis2iclx_emb_func_en_b_t emb_func_en_b;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FSM_ENABLE_A,
                             (uint8_t *)&val->fsm_enable_a, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FSM_ENABLE_B,
                             (uint8_t *)&val->fsm_enable_b, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_EN_B,
                            (uint8_t *)&emb_func_en_b, 1);
  }

  if (ret == 0) {
    if ( (val->fsm_enable_a.fsm1_en |
          val->fsm_enable_a.fsm2_en |
          val->fsm_enable_a.fsm3_en |
          val->fsm_enable_a.fsm4_en |
          val->fsm_enable_a.fsm5_en |
          val->fsm_enable_a.fsm6_en |
          val->fsm_enable_a.fsm7_en |
          val->fsm_enable_a.fsm8_en |
          val->fsm_enable_b.fsm9_en |
          val->fsm_enable_b.fsm10_en |
          val->fsm_enable_b.fsm11_en |
          val->fsm_enable_b.fsm12_en |
          val->fsm_enable_b.fsm13_en |
          val->fsm_enable_b.fsm14_en |
          val->fsm_enable_b.fsm15_en |
          val->fsm_enable_b.fsm16_en ) != PROPERTY_DISABLE) {
      emb_func_en_b.fsm_en = PROPERTY_ENABLE;
    }

    else {
      emb_func_en_b.fsm_en = PROPERTY_DISABLE;
    }
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_EMB_FUNC_EN_B,
                             (uint8_t *)&emb_func_en_b, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Embedded final state machine functions mode.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure of registers from FSM_ENABLE_A to FSM_ENABLE_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_fsm_enable_get(stmdev_ctx_t *ctx,
                                iis2iclx_emb_fsm_enable_t *val)
{
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FSM_ENABLE_A,
                            (uint8_t *)&val->fsm_enable_a, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FSM_ENABLE_B,
                            (uint8_t *)&val->fsm_enable_b, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  FSM long counter status register. Long counter value is an
  *         unsigned integer value (16-bit format).[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_long_cnt_set(stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t buff[2];
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    buff[1] = (uint8_t) (val / 256U);
    buff[0] = (uint8_t) (val - (buff[1] * 256U));
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FSM_LONG_COUNTER_L, buff, 2);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  FSM long counter status register. Long counter value is an
  *         unsigned integer value (16-bit format).[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_long_cnt_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buff[2];
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FSM_LONG_COUNTER_L, buff, 2);
    *val = buff[1];
    *val = (*val * 256U) + buff[0];
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Clear FSM long counter value.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fsm_lc_clr in reg
  *                FSM_LONG_COUNTER_CLEAR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_long_clr_set(stmdev_ctx_t *ctx,
                              iis2iclx_fsm_lc_clr_t val)
{
  iis2iclx_fsm_long_counter_clear_t fsm_long_counter_clear;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FSM_LONG_COUNTER_CLEAR,
                            (uint8_t *)&fsm_long_counter_clear, 1);
  }

  if (ret == 0) {
    fsm_long_counter_clear.fsm_lc_clr = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_FSM_LONG_COUNTER_CLEAR,
                             (uint8_t *)&fsm_long_counter_clear, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Clear FSM long counter value.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fsm_lc_clr in reg  FSM_LONG_COUNTER_CLEAR
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_long_clr_get(stmdev_ctx_t *ctx,
                              iis2iclx_fsm_lc_clr_t *val)
{
  iis2iclx_fsm_long_counter_clear_t fsm_long_counter_clear;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FSM_LONG_COUNTER_CLEAR,
                            (uint8_t *)&fsm_long_counter_clear, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  switch (fsm_long_counter_clear.fsm_lc_clr) {
    case IIS2ICLX_LC_NORMAL:
      *val = IIS2ICLX_LC_NORMAL;
      break;

    case IIS2ICLX_LC_CLEAR:
      *val = IIS2ICLX_LC_CLEAR;
      break;

    case IIS2ICLX_LC_CLEAR_DONE:
      *val = IIS2ICLX_LC_CLEAR_DONE;
      break;

    default:
      *val = IIS2ICLX_LC_NORMAL;
      break;
  }

  return ret;
}

/**
  * @brief  FSM output registers.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure of registers from FSM_OUTS1 to FSM_OUTS16
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_fsm_out_get(stmdev_ctx_t *ctx,
                             iis2iclx_fsm_out_t *val)
{
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_FSM_OUTS1,
                            (uint8_t *)&val->fsm_outs1, 16);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Finite State Machine ODR configuration.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fsm_odr in reg EMB_FUNC_ODR_CFG_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_fsm_data_rate_set(stmdev_ctx_t *ctx,
                                   iis2iclx_fsm_odr_t val)
{
  iis2iclx_emb_func_odr_cfg_b_t emb_func_odr_cfg_b;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_ODR_CFG_B,
                            (uint8_t *)&emb_func_odr_cfg_b, 1);
  }

  if (ret == 0) {
    emb_func_odr_cfg_b.not_used_01 = 3; /* set default values */
    emb_func_odr_cfg_b.not_used_02 = 1; /* set default values */
    emb_func_odr_cfg_b.fsm_odr = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_EMB_FUNC_ODR_CFG_B,
                             (uint8_t *)&emb_func_odr_cfg_b, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Finite State Machine ODR configuration.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fsm_odr in reg EMB_FUNC_ODR_CFG_B
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_fsm_data_rate_get(stmdev_ctx_t *ctx,
                                   iis2iclx_fsm_odr_t *val)
{
  iis2iclx_emb_func_odr_cfg_b_t emb_func_odr_cfg_b;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_ODR_CFG_B,
                            (uint8_t *)&emb_func_odr_cfg_b, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  switch (emb_func_odr_cfg_b.fsm_odr) {
    case IIS2ICLX_ODR_FSM_12Hz5:
      *val = IIS2ICLX_ODR_FSM_12Hz5;
      break;

    case IIS2ICLX_ODR_FSM_26Hz:
      *val = IIS2ICLX_ODR_FSM_26Hz;
      break;

    case IIS2ICLX_ODR_FSM_52Hz:
      *val = IIS2ICLX_ODR_FSM_52Hz;
      break;

    case IIS2ICLX_ODR_FSM_104Hz:
      *val = IIS2ICLX_ODR_FSM_104Hz;
      break;

    default:
      *val = IIS2ICLX_ODR_FSM_12Hz5;
      break;
  }

  return ret;
}

/**
  * @brief  FSM initialization request.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fsm_init in reg FSM_INIT
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_fsm_init_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_emb_func_init_b_t emb_func_init_b;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_INIT_B,
                            (uint8_t *)&emb_func_init_b, 1);
  }

  if (ret == 0) {
    emb_func_init_b.fsm_init = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_EMB_FUNC_INIT_B,
                             (uint8_t *)&emb_func_init_b, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  FSM initialization request.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fsm_init in reg FSM_INIT
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_fsm_init_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_emb_func_init_b_t emb_func_init_b;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_INIT_B,
                            (uint8_t *)&emb_func_init_b, 1);
  }

  if (ret == 0) {
    *val = emb_func_init_b.fsm_init;
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  FSM long counter timeout register (r/w). The long counter
  *         timeout value is an unsigned integer value (16-bit format).
  *         When the long counter value reached this value, the FSM
  *         generates an interrupt.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_long_cnt_int_value_set(stmdev_ctx_t *ctx,
                                        uint16_t val)
{
  uint8_t buff[2];
  int32_t ret;
  buff[1] = (uint8_t) (val / 256U);
  buff[0] = (uint8_t) (val - (buff[1] * 256U));
  ret = iis2iclx_ln_pg_write_byte(ctx, IIS2ICLX_FSM_LC_TIMEOUT_L,
                                  &buff[0]);

  if (ret == 0) {
    ret = iis2iclx_ln_pg_write_byte(ctx, IIS2ICLX_FSM_LC_TIMEOUT_H,
                                    &buff[1]);
  }

  return ret;
}

/**
  * @brief  FSM long counter timeout register (r/w). The long counter
  *         timeout value is an unsigned integer value (16-bit format).
  *         When the long counter value reached this value, the FSM generates
  *         an interrupt.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_long_cnt_int_value_get(stmdev_ctx_t *ctx,
                                        uint16_t *val)
{
  uint8_t buff[2];
  int32_t ret;
  ret = iis2iclx_ln_pg_read_byte(ctx, IIS2ICLX_FSM_LC_TIMEOUT_L,
                                 &buff[0]);

  if (ret == 0) {
    ret = iis2iclx_ln_pg_read_byte(ctx, IIS2ICLX_FSM_LC_TIMEOUT_H,
                                   &buff[1]);
    *val = buff[1];
    *val = (*val * 256U) + buff[0];
  }

  return ret;
}

/**
  * @brief  FSM number of programs register.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_fsm_number_of_programs_set(stmdev_ctx_t *ctx,
                                            uint8_t *buff)
{
  int32_t ret;
  ret = iis2iclx_ln_pg_write_byte(ctx, IIS2ICLX_FSM_PROGRAMS, buff);

  if (ret == 0) {
    ret = iis2iclx_ln_pg_write_byte(ctx, IIS2ICLX_FSM_PROGRAMS + 0x01U,
                                    buff);
  }

  return ret;
}

/**
  * @brief  FSM number of programs register.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_fsm_number_of_programs_get(stmdev_ctx_t *ctx,
                                            uint8_t *buff)
{
  int32_t ret;
  ret = iis2iclx_ln_pg_read_byte(ctx, IIS2ICLX_FSM_PROGRAMS, buff);
  return ret;
}

/**
  * @brief  FSM start address register (r/w). First available address is
  *         0x033C.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_fsm_start_address_set(stmdev_ctx_t *ctx,
                                       uint16_t val)
{
  uint8_t buff[2];
  int32_t ret;
  buff[1] = (uint8_t) (val / 256U);
  buff[0] = (uint8_t) (val - (buff[1] * 256U));
  ret = iis2iclx_ln_pg_write_byte(ctx, IIS2ICLX_FSM_START_ADD_L,
                                  &buff[0]);

  if (ret == 0) {
    ret = iis2iclx_ln_pg_write_byte(ctx, IIS2ICLX_FSM_START_ADD_H,
                                    &buff[1]);
  }

  return ret;
}

/**
  * @brief  FSM start address register (r/w). First available address
  *         is 0x033C.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_fsm_start_address_get(stmdev_ctx_t *ctx,
                                       uint16_t *val)
{
  uint8_t buff[2];
  int32_t ret;
  ret = iis2iclx_ln_pg_read_byte(ctx, IIS2ICLX_FSM_START_ADD_L,
                                 &buff[0]);

  if (ret == 0) {
    ret = iis2iclx_ln_pg_read_byte(ctx, IIS2ICLX_FSM_START_ADD_H,
                                   &buff[1]);
    *val = buff[1];
    *val = (*val * 256U) +  buff[0];
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @addtogroup  Machine Learning Core
  * @brief   This section group all the functions concerning the
  *          usage of Machine Learning Core
  * @{
  *
  */

/**
  * @brief  Enable Machine Learning Core.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of mlc_en in
  *                  reg EMB_FUNC_EN_B and mlc_init
  *                  in EMB_FUNC_INIT_B
  *
  */
int32_t iis2iclx_mlc_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_emb_func_en_b_t reg;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_EN_B, (uint8_t *)&reg,
                            1);
  }

  if (ret == 0) {
    reg.mlc_en = val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_EMB_FUNC_EN_B, (uint8_t *)&reg,
                             1);
  }

  if ((val != PROPERTY_DISABLE) && (ret == 0)) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_INIT_B,
                            (uint8_t *)&reg, 1);

    if (ret == 0) {
      reg.mlc_en = val;
      ret = iis2iclx_write_reg(ctx, IIS2ICLX_EMB_FUNC_INIT_B,
                               (uint8_t *)&reg, 1);
    }
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Enable Machine Learning Core.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of mlc_en in
  *                  reg EMB_FUNC_EN_B
  *
  */
int32_t iis2iclx_mlc_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_emb_func_en_b_t reg;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_EN_B, (uint8_t *)&reg,
                            1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
    *val  = reg.mlc_en;
  }

  return ret;
}

/**
  * @brief  Machine Learning Core status register[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register MLC_STATUS_MAINPAGE
  *
  */
int32_t iis2iclx_mlc_status_get(stmdev_ctx_t *ctx,
                                iis2iclx_mlc_status_mainpage_t *val)
{
  return iis2iclx_read_reg(ctx, IIS2ICLX_MLC_STATUS_MAINPAGE,
                           (uint8_t *) val, 1);
}

/**
  * @brief  Machine Learning Core data rate selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of mlc_odr in
  *                  reg EMB_FUNC_ODR_CFG_C
  *
  */
int32_t iis2iclx_mlc_data_rate_set(stmdev_ctx_t *ctx,
                                   iis2iclx_mlc_odr_t val)
{
  iis2iclx_emb_func_odr_cfg_c_t reg;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_ODR_CFG_C,
                            (uint8_t *)&reg, 1);
  }

  if (ret == 0) {
    reg.mlc_odr = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_EMB_FUNC_ODR_CFG_C,
                             (uint8_t *)&reg, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Machine Learning Core data rate selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of mlc_odr in
  *                  reg EMB_FUNC_ODR_CFG_C
  *
  */
int32_t iis2iclx_mlc_data_rate_get(stmdev_ctx_t *ctx,
                                   iis2iclx_mlc_odr_t *val)
{
  iis2iclx_emb_func_odr_cfg_c_t reg;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_EMB_FUNC_ODR_CFG_C,
                            (uint8_t *)&reg, 1);
  }

  if (ret == 0) {
    switch (reg.mlc_odr) {
      case IIS2ICLX_ODR_PRGS_12Hz5:
        *val = IIS2ICLX_ODR_PRGS_12Hz5;
        break;

      case IIS2ICLX_ODR_PRGS_26Hz:
        *val = IIS2ICLX_ODR_PRGS_26Hz;
        break;

      case IIS2ICLX_ODR_PRGS_52Hz:
        *val = IIS2ICLX_ODR_PRGS_52Hz;
        break;

      case IIS2ICLX_ODR_PRGS_104Hz:
        *val = IIS2ICLX_ODR_PRGS_104Hz;
        break;

      default:
        *val = IIS2ICLX_ODR_PRGS_12Hz5;
        break;
    }

    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  prgsens_out: [get] Output value of all MLCx decision trees.
  *
  * @param  ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t iis2iclx_mlc_out_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MLC0_SRC, buff, 8);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   IIS2ICLX_Sensor_hub
  * @brief      This section groups all the functions that manage the
  *             sensor hub.
  * @{
  *
  */

/**
  * @brief  Sensor hub output registers.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure of registers from SENSOR_HUB_1 to SENSOR_HUB_18
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_read_data_raw_get(stmdev_ctx_t *ctx,
                                      iis2iclx_emb_sh_read_t *val)
{
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SENSOR_HUB_1, (uint8_t *)val,
                            18);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Number of external sensors to be read by the sensor hub.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of aux_sens_on in reg MASTER_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_slave_connected_set(stmdev_ctx_t *ctx,
                                        iis2iclx_aux_sens_on_t val)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    master_config.aux_sens_on = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                             (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Number of external sensors to be read by the sensor hub.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of aux_sens_on in reg MASTER_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_slave_connected_get(stmdev_ctx_t *ctx,
                                        iis2iclx_aux_sens_on_t *val)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  switch (master_config.aux_sens_on) {
    case IIS2ICLX_SLV_0:
      *val = IIS2ICLX_SLV_0;
      break;

    case IIS2ICLX_SLV_0_1:
      *val = IIS2ICLX_SLV_0_1;
      break;

    case IIS2ICLX_SLV_0_1_2:
      *val = IIS2ICLX_SLV_0_1_2;
      break;

    case IIS2ICLX_SLV_0_1_2_3:
      *val = IIS2ICLX_SLV_0_1_2_3;
      break;

    default:
      *val = IIS2ICLX_SLV_0;
      break;
  }

  return ret;
}

/**
  * @brief  Sensor hub I2C master enable.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of master_on in reg MASTER_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_master_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    master_config.master_on = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                             (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Sensor hub I2C master enable.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of master_on in reg MASTER_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_master_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    *val = master_config.master_on;
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Master I2C pull-up enable.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of shub_pu_en in reg MASTER_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_pin_mode_set(stmdev_ctx_t *ctx,
                                 iis2iclx_shub_pu_en_t val)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    master_config.shub_pu_en = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                             (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Master I2C pull-up enable.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of shub_pu_en in reg MASTER_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_pin_mode_get(stmdev_ctx_t *ctx,
                                 iis2iclx_shub_pu_en_t *val)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  switch (master_config.shub_pu_en) {
    case IIS2ICLX_EXT_PULL_UP:
      *val = IIS2ICLX_EXT_PULL_UP;
      break;

    case IIS2ICLX_INTERNAL_PULL_UP:
      *val = IIS2ICLX_INTERNAL_PULL_UP;
      break;

    default:
      *val = IIS2ICLX_EXT_PULL_UP;
      break;
  }

  return ret;
}

/**
  * @brief  I2C interface pass-through.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of pass_through_mode in reg MASTER_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_pass_through_set(stmdev_ctx_t *ctx, uint8_t val)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    master_config.pass_through_mode = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                             (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  I2C interface pass-through.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of pass_through_mode in reg MASTER_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_pass_through_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    *val = master_config.pass_through_mode;
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Sensor hub trigger signal selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of start_config in reg MASTER_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_syncro_mode_set(stmdev_ctx_t *ctx,
                                    iis2iclx_start_config_t val)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    master_config.start_config = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                             (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Sensor hub trigger signal selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of start_config in reg MASTER_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_syncro_mode_get(stmdev_ctx_t *ctx,
                                    iis2iclx_start_config_t *val)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  switch (master_config.start_config) {
    case IIS2ICLX_EXT_ON_INT2_PIN:
      *val = IIS2ICLX_EXT_ON_INT2_PIN;
      break;

    case IIS2ICLX_XL_GY_DRDY:
      *val = IIS2ICLX_XL_GY_DRDY;
      break;

    default:
      *val = IIS2ICLX_EXT_ON_INT2_PIN;
      break;
  }

  return ret;
}

/**
  * @brief  Slave 0 write operation is performed only at the first sensor
  *         hub cycle.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of write_once in reg MASTER_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_write_mode_set(stmdev_ctx_t *ctx,
                                   iis2iclx_write_once_t val)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    master_config.write_once = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                             (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Slave 0 write operation is performed only at the first sensor
  *         hub cycle.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of write_once in reg MASTER_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_write_mode_get(stmdev_ctx_t *ctx,
                                   iis2iclx_write_once_t *val)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  switch (master_config.write_once) {
    case IIS2ICLX_EACH_SH_CYCLE:
      *val = IIS2ICLX_EACH_SH_CYCLE;
      break;

    case IIS2ICLX_ONLY_FIRST_CYCLE:
      *val = IIS2ICLX_ONLY_FIRST_CYCLE;
      break;

    default:
      *val = IIS2ICLX_EACH_SH_CYCLE;
      break;
  }

  return ret;
}

/**
  * @brief  Reset Master logic and output registers.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_reset_set(stmdev_ctx_t *ctx)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    master_config.rst_master_regs = PROPERTY_ENABLE;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                             (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    master_config.rst_master_regs = PROPERTY_DISABLE;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                             (uint8_t *)&master_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Reset Master logic and output registers.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of rst_master_regs in reg MASTER_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  iis2iclx_master_config_t master_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_MASTER_CONFIG,
                            (uint8_t *)&master_config, 1);
    *val = master_config.rst_master_regs;
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Rate at which the master communicates.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of shub_odr in reg SLV0_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_data_rate_set(stmdev_ctx_t *ctx,
                                  iis2iclx_shub_odr_t val)
{
  iis2iclx_slv0_config_t slv0_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV0_CONFIG,
                            (uint8_t *)&slv0_config, 1);
  }

  if (ret == 0) {
    slv0_config.shub_odr = (uint8_t)val;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV0_CONFIG,
                             (uint8_t *)&slv0_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Rate at which the master communicates.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of shub_odr in reg slv1_CONFIG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_data_rate_get(stmdev_ctx_t *ctx,
                                  iis2iclx_shub_odr_t *val)
{
  iis2iclx_slv0_config_t slv0_config;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV0_CONFIG,
                            (uint8_t *)&slv0_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  switch (slv0_config.shub_odr) {
    case IIS2ICLX_SH_ODR_104Hz:
      *val = IIS2ICLX_SH_ODR_104Hz;
      break;

    case IIS2ICLX_SH_ODR_52Hz:
      *val = IIS2ICLX_SH_ODR_52Hz;
      break;

    case IIS2ICLX_SH_ODR_26Hz:
      *val = IIS2ICLX_SH_ODR_26Hz;
      break;

    case IIS2ICLX_SH_ODR_13Hz:
      *val = IIS2ICLX_SH_ODR_13Hz;
      break;

    default:
      *val = IIS2ICLX_SH_ODR_104Hz;
      break;
  }

  return ret;
}

/**
  * @brief  Configure slave 0 for perform a write.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure that contain
  *                - uint8_t slv0_add;    8 bit i2c device address
  *                - uint8_t slv0_subadd; 8 bit register device address
  *                - uint8_t slv0_data;   8 bit data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_cfg_write(stmdev_ctx_t *ctx,
                              iis2iclx_sh_cfg_write_t *val)
{
  iis2iclx_slv0_add_t slv0_add;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    slv0_add.slave0 = (uint8_t) (val->slv0_add >> 1);
    slv0_add.rw_0 = 0;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV0_ADD,
                             (uint8_t *) & (slv0_add), 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV0_SUBADD,
                             (uint8_t *) & (val->slv0_subadd), 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_DATAWRITE_SLV0,
                             (uint8_t *) & (val->slv0_data), 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief Configure slave 0 for perform a write/read.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure that contain
  *                - uint8_t slv_add;    8 bit i2c device address
  *                - uint8_t slv_subadd; 8 bit register device address
  *                - uint8_t slv_len;    num of bit to read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_slv0_cfg_read(stmdev_ctx_t *ctx,
                                  iis2iclx_sh_cfg_read_t *val)
{
  iis2iclx_slv0_config_t slv0_config;
  iis2iclx_slv0_add_t slv0_add;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    slv0_add.slave0 = (uint8_t) val->slv_add >> 1;
    slv0_add.rw_0 = 1;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV0_ADD,
                             (uint8_t *) & (slv0_add), 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV0_SUBADD,
                             &(val->slv_subadd), 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV0_CONFIG,
                            (uint8_t *)&slv0_config, 1);
  }

  if (ret == 0) {
    slv0_config.slave0_numop = val->slv_len;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV0_CONFIG,
                             (uint8_t *)&slv0_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Configure slave 0 for perform a write/read.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure that contain
  *                - uint8_t slv_add;    8 bit i2c device address
  *                - uint8_t slv_subadd; 8 bit register device address
  *                - uint8_t slv_len;    num of bit to read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_slv1_cfg_read(stmdev_ctx_t *ctx,
                                  iis2iclx_sh_cfg_read_t *val)
{
  iis2iclx_slv1_config_t slv1_config;
  iis2iclx_slv1_add_t slv1_add;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    slv1_add.slave1_add = (uint8_t) (val->slv_add >> 1);
    slv1_add.r_1 = 1;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV1_ADD, (uint8_t *)&slv1_add,
                             1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV1_SUBADD,
                             &(val->slv_subadd), 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV1_CONFIG,
                            (uint8_t *)&slv1_config, 1);
  }

  if (ret == 0) {
    slv1_config.slave1_numop = val->slv_len;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV1_CONFIG,
                             (uint8_t *)&slv1_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Configure slave 2 for perform a write/read.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure that contain
  *                - uint8_t slv_add;    8 bit i2c device address
  *                - uint8_t slv_subadd; 8 bit register device address
  *                - uint8_t slv_len;    num of bit to read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_slv2_cfg_read(stmdev_ctx_t *ctx,
                                  iis2iclx_sh_cfg_read_t *val)
{
  iis2iclx_slv2_config_t slv2_config;
  iis2iclx_slv2_add_t slv2_add;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    slv2_add.slave2_add = (uint8_t) (val->slv_add >> 1);
    slv2_add.r_2 = 1;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV2_ADD,
                             (uint8_t *)&slv2_add, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV2_SUBADD,
                             (uint8_t *) & (val->slv_subadd), 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV2_CONFIG,
                            (uint8_t *)&slv2_config, 1);
  }

  if (ret == 0) {
    slv2_config.slave2_numop = val->slv_len;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV2_CONFIG,
                             (uint8_t *)&slv2_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Configure slave 3 for perform a write/read.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Structure that contain
  *                - uint8_t slv_add;    8 bit i2c device address
  *                - uint8_t slv_subadd; 8 bit register device address
  *                - uint8_t slv_len;    num of bit to read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_slv3_cfg_read(stmdev_ctx_t *ctx,
                                  iis2iclx_sh_cfg_read_t *val)
{
  iis2iclx_slv3_config_t slv3_config;
  iis2iclx_slv3_add_t slv3_add;
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    slv3_add.slave3_add = (uint8_t) (val->slv_add >> 1);
    slv3_add.r_3 = 1;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV3_ADD,
                             (uint8_t *)&slv3_add, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV3_SUBADD,
                             &(val->slv_subadd), 1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_SLV3_CONFIG,
                            (uint8_t *)&slv3_config, 1);
  }

  if (ret == 0) {
    slv3_config.slave3_numop = val->slv_len;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_SLV3_CONFIG,
                             (uint8_t *)&slv3_config, 1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Sensor hub source register.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Registers from STATUS_MASTER
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t iis2iclx_sh_status_get(stmdev_ctx_t *ctx,
                               iis2iclx_status_master_t *val)
{
  int32_t ret;
  ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_STATUS_MASTER, (uint8_t *)val,
                            1);
  }

  if (ret == 0) {
    ret = iis2iclx_mem_bank_set(ctx, IIS2ICLX_USER_BANK);
  }

  return ret;
}

int32_t iis2iclx_bus_mode_set(stmdev_ctx_t *ctx,
                              iis2iclx_bus_mode_t val)
{
  iis2iclx_ctrl9_xl_t ctrl9_xl;
  iis2iclx_ctrl3_c_t  ctrl3_c;
  iis2iclx_ctrl4_c_t  ctrl4_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                          1);

  if (ret == 0) {
    ctrl9_xl.device_conf = PROPERTY_ENABLE;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL9_XL, (uint8_t *)&ctrl9_xl,
                             1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                            1);
  }

  if (ret == 0) {
    ctrl3_c.sim = ((uint8_t)val & 0x02U) >> 1;
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                             1);
  }

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL4_C, (uint8_t *)&ctrl4_c,
                            1);
  }

  if (ret == 0) {
    ctrl4_c.i2c_disable = ((uint8_t)val & 0x01U);
    ret = iis2iclx_write_reg(ctx, IIS2ICLX_CTRL4_C, (uint8_t *)&ctrl4_c,
                             1);
  }

  return ret;
}

int32_t iis2iclx_bus_mode_get(stmdev_ctx_t *ctx,
                              iis2iclx_bus_mode_t *val)
{
  iis2iclx_ctrl3_c_t  ctrl3_c;
  iis2iclx_ctrl4_c_t  ctrl4_c;
  int32_t ret;
  ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL3_C, (uint8_t *)&ctrl3_c,
                          1);

  if (ret == 0) {
    ret = iis2iclx_read_reg(ctx, IIS2ICLX_CTRL4_C, (uint8_t *)&ctrl4_c,
                            1);
  }

  switch ( ( ctrl3_c.sim << 1 ) | ctrl4_c.i2c_disable ) {
    case IIS2ICLX_SEL_BY_HW:
      *val = IIS2ICLX_SEL_BY_HW;
      break;

    case IIS2ICLX_SPI_4W:
      *val = IIS2ICLX_SPI_4W;
      break;

    case IIS2ICLX_SPI_3W:
      *val = IIS2ICLX_SPI_3W;
      break;

    default:
      *val = IIS2ICLX_SEL_BY_HW;
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
