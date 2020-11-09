/*
 ******************************************************************************
 * @file    lsm6dsox_reg.c
 * @author  Sensors Software Solution Team
 * @brief   LSM6DSOX driver file
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

#include "lsm6dsox_reg.h"

/**
  * @defgroup  LSM6DSOX
  * @brief     This file provides a set of functions needed to drive the
  *            lsm6dsox enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup  LSM6DSOX_Interfaces_Functions
  * @brief     This section provide a set of functions used to read and
  *            write a generic register of the device.
  *            MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  reg   first register address to read.
  * @param  data  buffer for data read.(ptr)
  * @param  len   number of consecutive register to read.
  * @retval       interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm6dsox_read_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
                          uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);
  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  reg   first register address to write.
  * @param  data  the buffer contains data to be written.(ptr)
  * @param  len   number of consecutive register to write.
  * @retval       interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm6dsox_write_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
  * @defgroup  LSM6DSOX_Private_functions
  * @brief     Section collect all the utility functions needed by APIs.
  * @{
  *
  */

static void bytecpy(uint8_t *target, uint8_t *source)
{
  if ( (target != NULL) && (source != NULL) ) {
    *target = *source;
  }
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_Sensitivity
  * @brief     These functions convert raw-data into engineering units.
  * @{
  *
  */
float_t lsm6dsox_from_fs2_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.061f;
}

float_t lsm6dsox_from_fs4_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.122f;
}

float_t lsm6dsox_from_fs8_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.244f;
}

float_t lsm6dsox_from_fs16_to_mg(int16_t lsb)
{
  return ((float_t)lsb) *0.488f;
}

float_t lsm6dsox_from_fs125_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) *4.375f;
}

float_t lsm6dsox_from_fs500_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) *17.50f;
}

float_t lsm6dsox_from_fs250_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) *8.750f;
}

float_t lsm6dsox_from_fs1000_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) *35.0f;
}

float_t lsm6dsox_from_fs2000_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) *70.0f;
}

float_t lsm6dsox_from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb / 256.0f) + 25.0f);
}

float_t lsm6dsox_from_lsb_to_nsec(int16_t lsb)
{
  return ((float_t)lsb * 25000.0f);
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_Data_Generation
  * @brief     This section groups all the functions concerning
  *            data generation.
  *
  */

/**
  * @brief  Accelerometer full-scale selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fs_xl in reg CTRL1_XL
  *
  */
int32_t lsm6dsox_xl_full_scale_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_fs_xl_t val)
{
  lsm6dsox_ctrl1_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL1_XL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.fs_xl = (uint8_t) val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL1_XL, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Accelerometer full-scale selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of fs_xl in reg CTRL1_XL
  *
  */
int32_t lsm6dsox_xl_full_scale_get(stmdev_ctx_t *ctx, lsm6dsox_fs_xl_t *val)
{
  lsm6dsox_ctrl1_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL1_XL, (uint8_t*)&reg, 1);
  switch (reg.fs_xl) {
    case LSM6DSOX_2g:
      *val = LSM6DSOX_2g;
      break;
    case LSM6DSOX_16g:
      *val = LSM6DSOX_16g;
      break;
    case LSM6DSOX_4g:
      *val = LSM6DSOX_4g;
      break;
    case LSM6DSOX_8g:
      *val = LSM6DSOX_8g;
      break;
    default:
      *val = LSM6DSOX_2g;
      break;
  }

  return ret;
}

/**
  * @brief  Accelerometer UI data rate selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of odr_xl in reg CTRL1_XL
  *
  */
int32_t lsm6dsox_xl_data_rate_set(stmdev_ctx_t *ctx, lsm6dsox_odr_xl_t val)
{
  lsm6dsox_odr_xl_t odr_xl =  val;
  lsm6dsox_emb_fsm_enable_t fsm_enable;
  lsm6dsox_fsm_odr_t fsm_odr;
  lsm6dsox_emb_sens_t emb_sens;
  lsm6dsox_mlc_odr_t mlc_odr;
  lsm6dsox_ctrl1_xl_t reg;
  int32_t ret;

  /* Check the Finite State Machine data rate constraints */
  ret =  lsm6dsox_fsm_enable_get(ctx, &fsm_enable);
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
          fsm_enable.fsm_enable_b.fsm16_en ) == PROPERTY_ENABLE ){

      ret =  lsm6dsox_fsm_data_rate_get(ctx, &fsm_odr);
      if (ret == 0) {
        switch (fsm_odr) {
          case LSM6DSOX_ODR_FSM_12Hz5:

            if (val == LSM6DSOX_XL_ODR_OFF){
              odr_xl = LSM6DSOX_XL_ODR_12Hz5;

            } else {
              odr_xl = val;
            }
            break;
          case LSM6DSOX_ODR_FSM_26Hz:

            if (val == LSM6DSOX_XL_ODR_OFF){
              odr_xl = LSM6DSOX_XL_ODR_26Hz;

            } else if (val == LSM6DSOX_XL_ODR_12Hz5){
              odr_xl = LSM6DSOX_XL_ODR_26Hz;

            } else {
              odr_xl = val;
            }
            break;
          case LSM6DSOX_ODR_FSM_52Hz:

            if (val == LSM6DSOX_XL_ODR_OFF){
              odr_xl = LSM6DSOX_XL_ODR_52Hz;

            } else if (val == LSM6DSOX_XL_ODR_12Hz5){
              odr_xl = LSM6DSOX_XL_ODR_52Hz;

            } else if (val == LSM6DSOX_XL_ODR_26Hz){
              odr_xl = LSM6DSOX_XL_ODR_52Hz;

            } else {
              odr_xl = val;
            }
            break;
          case LSM6DSOX_ODR_FSM_104Hz:

            if (val == LSM6DSOX_XL_ODR_OFF){
              odr_xl = LSM6DSOX_XL_ODR_104Hz;

            } else if (val == LSM6DSOX_XL_ODR_12Hz5){
              odr_xl = LSM6DSOX_XL_ODR_104Hz;

            } else if (val == LSM6DSOX_XL_ODR_26Hz){
              odr_xl = LSM6DSOX_XL_ODR_104Hz;

            } else if (val == LSM6DSOX_XL_ODR_52Hz){
              odr_xl = LSM6DSOX_XL_ODR_104Hz;

            } else {
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
  emb_sens.mlc = PROPERTY_DISABLE;
  if (ret == 0) {
    lsm6dsox_embedded_sens_get(ctx, &emb_sens);
    if ( emb_sens.mlc == PROPERTY_ENABLE ){

      ret =  lsm6dsox_mlc_data_rate_get(ctx, &mlc_odr);
      if (ret == 0) {
        switch (mlc_odr) {
          case LSM6DSOX_ODR_PRGS_12Hz5:

            if (val == LSM6DSOX_XL_ODR_OFF){
              odr_xl = LSM6DSOX_XL_ODR_12Hz5;

            } else {
              odr_xl = val;
            }
            break;
          case LSM6DSOX_ODR_PRGS_26Hz:
            if (val == LSM6DSOX_XL_ODR_OFF){
              odr_xl = LSM6DSOX_XL_ODR_26Hz;

            } else if (val == LSM6DSOX_XL_ODR_12Hz5){
              odr_xl = LSM6DSOX_XL_ODR_26Hz;

            } else {
              odr_xl = val;
            }
            break;
          case LSM6DSOX_ODR_PRGS_52Hz:

            if (val == LSM6DSOX_XL_ODR_OFF){
              odr_xl = LSM6DSOX_XL_ODR_52Hz;

            } else if (val == LSM6DSOX_XL_ODR_12Hz5){
              odr_xl = LSM6DSOX_XL_ODR_52Hz;

            } else if (val == LSM6DSOX_XL_ODR_26Hz){
              odr_xl = LSM6DSOX_XL_ODR_52Hz;

            } else {
              odr_xl = val;
            }
            break;
          case LSM6DSOX_ODR_PRGS_104Hz:
            if (val == LSM6DSOX_XL_ODR_OFF){
              odr_xl = LSM6DSOX_XL_ODR_104Hz;

            } else if (val == LSM6DSOX_XL_ODR_12Hz5){
              odr_xl = LSM6DSOX_XL_ODR_104Hz;

            } else if (val == LSM6DSOX_XL_ODR_26Hz){
              odr_xl = LSM6DSOX_XL_ODR_104Hz;

            } else if (val == LSM6DSOX_XL_ODR_52Hz){
              odr_xl = LSM6DSOX_XL_ODR_104Hz;

            } else {
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
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL1_XL, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.odr_xl = (uint8_t) odr_xl;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL1_XL, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Accelerometer UI data rate selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of odr_xl in reg CTRL1_XL
  *
  */
int32_t lsm6dsox_xl_data_rate_get(stmdev_ctx_t *ctx, lsm6dsox_odr_xl_t *val)
{
  lsm6dsox_ctrl1_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL1_XL, (uint8_t*)&reg, 1);

  switch (reg.odr_xl) {
    case LSM6DSOX_XL_ODR_OFF:
      *val = LSM6DSOX_XL_ODR_OFF;
      break;
    case LSM6DSOX_XL_ODR_12Hz5:
      *val = LSM6DSOX_XL_ODR_12Hz5;
      break;
    case LSM6DSOX_XL_ODR_26Hz:
      *val = LSM6DSOX_XL_ODR_26Hz;
      break;
    case LSM6DSOX_XL_ODR_52Hz:
      *val = LSM6DSOX_XL_ODR_52Hz;
      break;
    case LSM6DSOX_XL_ODR_104Hz:
      *val = LSM6DSOX_XL_ODR_104Hz;
      break;
    case LSM6DSOX_XL_ODR_208Hz:
      *val = LSM6DSOX_XL_ODR_208Hz;
      break;
    case LSM6DSOX_XL_ODR_417Hz:
      *val = LSM6DSOX_XL_ODR_417Hz;
      break;
    case LSM6DSOX_XL_ODR_833Hz:
      *val = LSM6DSOX_XL_ODR_833Hz;
      break;
    case LSM6DSOX_XL_ODR_1667Hz:
      *val = LSM6DSOX_XL_ODR_1667Hz;
      break;
    case LSM6DSOX_XL_ODR_3333Hz:
      *val = LSM6DSOX_XL_ODR_3333Hz;
      break;
    case LSM6DSOX_XL_ODR_6667Hz:
      *val = LSM6DSOX_XL_ODR_6667Hz;
      break;
    case LSM6DSOX_XL_ODR_1Hz6:
      *val = LSM6DSOX_XL_ODR_1Hz6;
      break;
    default:
      *val = LSM6DSOX_XL_ODR_OFF;
      break;
  }
  return ret;
}

/**
  * @brief  Gyroscope UI chain full-scale selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fs_g in reg CTRL2_G
  *
  */
int32_t lsm6dsox_gy_full_scale_set(stmdev_ctx_t *ctx, lsm6dsox_fs_g_t val)
{
  lsm6dsox_ctrl2_g_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL2_G, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.fs_g = (uint8_t) val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL2_G, (uint8_t*)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Gyroscope UI chain full-scale selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of fs_g in reg CTRL2_G
  *
  */
int32_t lsm6dsox_gy_full_scale_get(stmdev_ctx_t *ctx, lsm6dsox_fs_g_t *val)
{
  lsm6dsox_ctrl2_g_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL2_G, (uint8_t*)&reg, 1);
  switch (reg.fs_g) {
    case LSM6DSOX_250dps:
      *val = LSM6DSOX_250dps;
      break;
    case LSM6DSOX_125dps:
      *val = LSM6DSOX_125dps;
      break;
    case LSM6DSOX_500dps:
      *val = LSM6DSOX_500dps;
      break;
    case LSM6DSOX_1000dps:
      *val = LSM6DSOX_1000dps;
      break;
    case LSM6DSOX_2000dps:
      *val = LSM6DSOX_2000dps;
      break;
    default:
      *val = LSM6DSOX_250dps;
      break;
  }

  return ret;
}

/**
  * @brief  Gyroscope UI data rate selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of odr_g in reg CTRL2_G
  *
  */
int32_t lsm6dsox_gy_data_rate_set(stmdev_ctx_t *ctx, lsm6dsox_odr_g_t val)
{
  lsm6dsox_odr_g_t odr_gy =  val;
  lsm6dsox_emb_fsm_enable_t fsm_enable;
  lsm6dsox_fsm_odr_t fsm_odr;
  lsm6dsox_emb_sens_t emb_sens;
  lsm6dsox_mlc_odr_t mlc_odr;
  lsm6dsox_ctrl2_g_t reg;
  int32_t ret;

  /* Check the Finite State Machine data rate constraints */
  ret =  lsm6dsox_fsm_enable_get(ctx, &fsm_enable);
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
          fsm_enable.fsm_enable_b.fsm16_en ) == PROPERTY_ENABLE ){

      ret =  lsm6dsox_fsm_data_rate_get(ctx, &fsm_odr);
      if (ret == 0) {
        switch (fsm_odr) {
          case LSM6DSOX_ODR_FSM_12Hz5:

            if (val == LSM6DSOX_GY_ODR_OFF){
              odr_gy = LSM6DSOX_GY_ODR_12Hz5;

            } else {
              odr_gy = val;
            }
            break;
          case LSM6DSOX_ODR_FSM_26Hz:

            if (val == LSM6DSOX_GY_ODR_OFF){
              odr_gy = LSM6DSOX_GY_ODR_26Hz;

            } else if (val == LSM6DSOX_GY_ODR_12Hz5){
              odr_gy = LSM6DSOX_GY_ODR_26Hz;

            } else {
              odr_gy = val;
            }
            break;
          case LSM6DSOX_ODR_FSM_52Hz:

            if (val == LSM6DSOX_GY_ODR_OFF){
              odr_gy = LSM6DSOX_GY_ODR_52Hz;

            } else if (val == LSM6DSOX_GY_ODR_12Hz5){
              odr_gy = LSM6DSOX_GY_ODR_52Hz;

            } else if (val == LSM6DSOX_GY_ODR_26Hz){
              odr_gy = LSM6DSOX_GY_ODR_52Hz;

            } else {
              odr_gy = val;
            }
            break;
          case LSM6DSOX_ODR_FSM_104Hz:

            if (val == LSM6DSOX_GY_ODR_OFF){
              odr_gy = LSM6DSOX_GY_ODR_104Hz;

            } else if (val == LSM6DSOX_GY_ODR_12Hz5){
              odr_gy = LSM6DSOX_GY_ODR_104Hz;

            } else if (val == LSM6DSOX_GY_ODR_26Hz){
              odr_gy = LSM6DSOX_GY_ODR_104Hz;

            } else if (val == LSM6DSOX_GY_ODR_52Hz){
              odr_gy = LSM6DSOX_GY_ODR_104Hz;

            } else {
              odr_gy = val;
            }
            break;
          default:
            odr_gy = val;
            break;
        }
      }
    }
  }

  /* Check the Machine Learning Core data rate constraints */
  emb_sens.mlc = PROPERTY_DISABLE;
  if (ret == 0) {
    ret =  lsm6dsox_embedded_sens_get(ctx, &emb_sens);
    if ( emb_sens.mlc == PROPERTY_ENABLE ){

      ret =  lsm6dsox_mlc_data_rate_get(ctx, &mlc_odr);
      if (ret == 0) {
        switch (mlc_odr) {
          case LSM6DSOX_ODR_PRGS_12Hz5:

            if (val == LSM6DSOX_GY_ODR_OFF){
              odr_gy = LSM6DSOX_GY_ODR_12Hz5;

            } else {
              odr_gy = val;
            }
            break;
          case LSM6DSOX_ODR_PRGS_26Hz:

            if (val == LSM6DSOX_GY_ODR_OFF){
              odr_gy = LSM6DSOX_GY_ODR_26Hz;

            } else if (val == LSM6DSOX_GY_ODR_12Hz5){
              odr_gy = LSM6DSOX_GY_ODR_26Hz;

            } else {
              odr_gy = val;
            }
            break;
          case LSM6DSOX_ODR_PRGS_52Hz:

            if (val == LSM6DSOX_GY_ODR_OFF){
              odr_gy = LSM6DSOX_GY_ODR_52Hz;

            } else if (val == LSM6DSOX_GY_ODR_12Hz5){
              odr_gy = LSM6DSOX_GY_ODR_52Hz;

            } else if (val == LSM6DSOX_GY_ODR_26Hz){
              odr_gy = LSM6DSOX_GY_ODR_52Hz;

            } else {
              odr_gy = val;
            }
            break;
          case LSM6DSOX_ODR_PRGS_104Hz:

            if (val == LSM6DSOX_GY_ODR_OFF){
              odr_gy = LSM6DSOX_GY_ODR_104Hz;

            } else if (val == LSM6DSOX_GY_ODR_12Hz5){
              odr_gy = LSM6DSOX_GY_ODR_104Hz;

            } else if (val == LSM6DSOX_GY_ODR_26Hz){
              odr_gy = LSM6DSOX_GY_ODR_104Hz;

            } else if (val == LSM6DSOX_GY_ODR_52Hz){
              odr_gy = LSM6DSOX_GY_ODR_104Hz;

            } else {
              odr_gy = val;
            }
            break;
          default:
            odr_gy = val;
            break;
        }
      }
    }
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL2_G, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.odr_g = (uint8_t) odr_gy;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL2_G, (uint8_t*)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Gyroscope UI data rate selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of odr_g in reg CTRL2_G
  *
  */
int32_t lsm6dsox_gy_data_rate_get(stmdev_ctx_t *ctx, lsm6dsox_odr_g_t *val)
{
  lsm6dsox_ctrl2_g_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL2_G, (uint8_t*)&reg, 1);
  switch (reg.odr_g) {
    case LSM6DSOX_GY_ODR_OFF:
      *val = LSM6DSOX_GY_ODR_OFF;
      break;
    case LSM6DSOX_GY_ODR_12Hz5:
      *val = LSM6DSOX_GY_ODR_12Hz5;
      break;
    case LSM6DSOX_GY_ODR_26Hz:
      *val = LSM6DSOX_GY_ODR_26Hz;
      break;
    case LSM6DSOX_GY_ODR_52Hz:
      *val = LSM6DSOX_GY_ODR_52Hz;
      break;
    case LSM6DSOX_GY_ODR_104Hz:
      *val = LSM6DSOX_GY_ODR_104Hz;
      break;
    case LSM6DSOX_GY_ODR_208Hz:
      *val = LSM6DSOX_GY_ODR_208Hz;
      break;
    case LSM6DSOX_GY_ODR_417Hz:
      *val = LSM6DSOX_GY_ODR_417Hz;
      break;
    case LSM6DSOX_GY_ODR_833Hz:
      *val = LSM6DSOX_GY_ODR_833Hz;
      break;
    case LSM6DSOX_GY_ODR_1667Hz:
      *val = LSM6DSOX_GY_ODR_1667Hz;
      break;
    case LSM6DSOX_GY_ODR_3333Hz:
      *val = LSM6DSOX_GY_ODR_3333Hz;
      break;
    case LSM6DSOX_GY_ODR_6667Hz:
      *val = LSM6DSOX_GY_ODR_6667Hz;
      break;
    default:
      *val = LSM6DSOX_GY_ODR_OFF;
      break;
  }
  return ret;
}

/**
  * @brief  Block data update.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of bdu in reg CTRL3_C
  *
  */
int32_t lsm6dsox_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl3_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.bdu = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Block data update.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of bdu in reg CTRL3_C
  *
  */
int32_t lsm6dsox_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl3_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  *val = reg.bdu;

  return ret;
}

/**
  * @brief  Weight of XL user offset bits of registers X_OFS_USR (73h),
  *         Y_OFS_USR (74h), Z_OFS_USR (75h).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of usr_off_w in reg CTRL6_C
  *
  */
int32_t lsm6dsox_xl_offset_weight_set(stmdev_ctx_t *ctx,
                                     lsm6dsox_usr_off_w_t val)
{
  lsm6dsox_ctrl6_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL6_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.usr_off_w = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL6_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief    Weight of XL user offset bits of registers X_OFS_USR (73h),
  *           Y_OFS_USR (74h), Z_OFS_USR (75h).[get]
  *
  * @param    ctx      read / write interface definitions
  * @param    val      Get the values of usr_off_w in reg CTRL6_C
  *
  */
int32_t lsm6dsox_xl_offset_weight_get(stmdev_ctx_t *ctx,
                                     lsm6dsox_usr_off_w_t *val)
{
  lsm6dsox_ctrl6_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL6_C, (uint8_t*)&reg, 1);

  switch (reg.usr_off_w) {
    case LSM6DSOX_LSb_1mg:
      *val = LSM6DSOX_LSb_1mg;
      break;
    case LSM6DSOX_LSb_16mg:
      *val = LSM6DSOX_LSb_16mg;
      break;
    default:
      *val = LSM6DSOX_LSb_1mg;
      break;
  }
  return ret;
}

/**
  * @brief  Accelerometer power mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of xl_hm_mode in
  *                               reg CTRL6_C
  *
  */
int32_t lsm6dsox_xl_power_mode_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_xl_hm_mode_t val)
{
  lsm6dsox_ctrl5_c_t ctrl5_c;
  lsm6dsox_ctrl6_c_t ctrl6_c;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*) &ctrl5_c, 1);
  if (ret == 0) {
    ctrl5_c.xl_ulp_en = ((uint8_t)val & 0x02U) >> 1;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*) &ctrl5_c, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL6_C, (uint8_t*) &ctrl6_c, 1);
  }
  if (ret == 0) {
    ctrl6_c.xl_hm_mode = (uint8_t)val & 0x01U;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL6_C, (uint8_t*) &ctrl6_c, 1);
  }
  return ret;
}

/**
  * @brief  Accelerometer power mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of xl_hm_mode in reg CTRL6_C
  *
  */
int32_t lsm6dsox_xl_power_mode_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_xl_hm_mode_t *val)
{
  lsm6dsox_ctrl5_c_t ctrl5_c;
  lsm6dsox_ctrl6_c_t ctrl6_c;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*) &ctrl5_c, 1);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL6_C, (uint8_t*) &ctrl6_c, 1);
    switch ( (ctrl5_c.xl_ulp_en << 1) | ctrl6_c.xl_hm_mode) {
      case LSM6DSOX_HIGH_PERFORMANCE_MD:
        *val = LSM6DSOX_HIGH_PERFORMANCE_MD;
        break;
      case LSM6DSOX_LOW_NORMAL_POWER_MD:
        *val = LSM6DSOX_LOW_NORMAL_POWER_MD;
        break;
      case LSM6DSOX_ULTRA_LOW_POWER_MD:
        *val = LSM6DSOX_ULTRA_LOW_POWER_MD;
        break;
      default:
        *val = LSM6DSOX_HIGH_PERFORMANCE_MD;
        break;
    }
  }
  return ret;
}

/**
  * @brief  Operating mode for gyroscope.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of g_hm_mode in reg CTRL7_G
  *
  */
int32_t lsm6dsox_gy_power_mode_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_g_hm_mode_t val)
{
  lsm6dsox_ctrl7_g_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL7_G, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.g_hm_mode = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL7_G, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Operating mode for gyroscope.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of g_hm_mode in reg CTRL7_G
  *
  */
int32_t lsm6dsox_gy_power_mode_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_g_hm_mode_t *val)
{
  lsm6dsox_ctrl7_g_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL7_G, (uint8_t*)&reg, 1);
  switch (reg.g_hm_mode) {
    case LSM6DSOX_GY_HIGH_PERFORMANCE:
      *val = LSM6DSOX_GY_HIGH_PERFORMANCE;
      break;
    case LSM6DSOX_GY_NORMAL:
      *val = LSM6DSOX_GY_NORMAL;
      break;
    default:
      *val = LSM6DSOX_GY_HIGH_PERFORMANCE;
      break;
  }
  return ret;
}

/**
  * @brief  The STATUS_REG register is read by the primary interface.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register STATUS_REG
  *
  */
int32_t lsm6dsox_status_reg_get(stmdev_ctx_t *ctx, lsm6dsox_status_reg_t *val)
{
  int32_t ret;
  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_STATUS_REG, (uint8_t*) val, 1);
  return ret;
}

/**
  * @brief  Accelerometer new data available.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of xlda in reg STATUS_REG
  *
  */
int32_t lsm6dsox_xl_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_status_reg_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_STATUS_REG, (uint8_t*)&reg, 1);
  *val = reg.xlda;

  return ret;
}

/**
  * @brief  Gyroscope new data available.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of gda in reg STATUS_REG
  *
  */
int32_t lsm6dsox_gy_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_status_reg_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_STATUS_REG, (uint8_t*)&reg, 1);
  *val = reg.gda;

  return ret;
}

/**
  * @brief  Temperature new data available.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tda in reg STATUS_REG
  *
  */
int32_t lsm6dsox_temp_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_status_reg_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_STATUS_REG, (uint8_t*)&reg, 1);
  *val = reg.tda;

  return ret;
}

/**
  * @brief  Accelerometer X-axis user offset correction expressed in
  *         two’s complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  *
  */
int32_t lsm6dsox_xl_usr_offset_x_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_write_reg(ctx, LSM6DSOX_X_OFS_USR, buff, 1);
  return ret;
}

/**
  * @brief  Accelerometer X-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_xl_usr_offset_x_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_X_OFS_USR, buff, 1);
  return ret;
}

/**
  * @brief  Accelerometer Y-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  *
  */
int32_t lsm6dsox_xl_usr_offset_y_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_write_reg(ctx, LSM6DSOX_Y_OFS_USR, buff, 1);
  return ret;
}

/**
  * @brief  Accelerometer Y-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_xl_usr_offset_y_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_Y_OFS_USR, buff, 1);
  return ret;
}

/**
  * @brief  Accelerometer Z-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  *
  */
int32_t lsm6dsox_xl_usr_offset_z_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_write_reg(ctx, LSM6DSOX_Z_OFS_USR, buff, 1);
  return ret;
}

/**
  * @brief  Accelerometer Z-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_xl_usr_offset_z_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_Z_OFS_USR, buff, 1);
  return ret;
}

/**
  * @brief  Enables user offset on out.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of usr_off_on_out in reg CTRL7_G
  *
  */
int32_t lsm6dsox_xl_usr_offset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl7_g_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL7_G, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.usr_off_on_out = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL7_G, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  User offset on out flag.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      values of usr_off_on_out in reg CTRL7_G
  *
  */
int32_t lsm6dsox_xl_usr_offset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl7_g_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL7_G, (uint8_t*)&reg, 1);
  *val = reg.usr_off_on_out;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_Timestamp
  * @brief     This section groups all the functions that manage the
  *            timestamp generation.
  * @{
  *
  */

/**
  * @brief  Reset timestamp counter.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lsm6dsox_timestamp_rst(stmdev_ctx_t *ctx)
{
  uint8_t rst_val = 0xAA;

  return lsm6dsox_write_reg(ctx, LSM6DSOX_TIMESTAMP2, &rst_val, 1);
}

/**
  * @brief  Enables timestamp counter.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of timestamp_en in reg CTRL10_C
  *
  */
int32_t lsm6dsox_timestamp_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl10_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL10_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.timestamp_en = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL10_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enables timestamp counter.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of timestamp_en in reg CTRL10_C
  *
  */
int32_t lsm6dsox_timestamp_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl10_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL10_C, (uint8_t*)&reg, 1);
  *val = reg.timestamp_en;

  return ret;
}

/**
  * @brief  Timestamp first data output register (r).
  *         The value is expressed as a 32-bit word and the bit
  *         resolution is 25 μs.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_timestamp_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TIMESTAMP0, buff, 4);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_Data output
  * @brief     This section groups all the data output functions.
  * @{
  *
  */

/**
  * @brief  Circular burst-mode (rounding) read of the output
  *         registers.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of rounding in reg CTRL5_C
  *
  */
int32_t lsm6dsox_rounding_mode_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_rounding_t val)
{
  lsm6dsox_ctrl5_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.rounding = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Gyroscope UI chain full-scale selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of rounding in reg CTRL5_C
  *
  */
int32_t lsm6dsox_rounding_mode_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_rounding_t *val)
{
  lsm6dsox_ctrl5_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&reg, 1);
  switch (reg.rounding) {
    case LSM6DSOX_NO_ROUND:
      *val = LSM6DSOX_NO_ROUND;
      break;
    case LSM6DSOX_ROUND_XL:
      *val = LSM6DSOX_ROUND_XL;
      break;
    case LSM6DSOX_ROUND_GY:
      *val = LSM6DSOX_ROUND_GY;
      break;
    case LSM6DSOX_ROUND_GY_XL:
      *val = LSM6DSOX_ROUND_GY_XL;
      break;
    default:
      *val = LSM6DSOX_NO_ROUND;
      break;
  }
  return ret;
}

/**
  * @brief  rounding_on_status: [set] Source register rounding function in
  *                                   ALL_INT_SRC (1Ah), WAKE_UP_SRC(1Bh),
  *                                   TAP_SRC (1Ch), D6D_SRC (1Dh),
  *                                   STATUS_REG (1Eh) and
  *                                   EMB_FUNC_STATUS_MAINPAGE(35h),
  *                                   FSM_STATUS_A_MAINPAGE (36h),
  *                                   FSM_STATUS_B_MAINPAGE (37h),
  *                                   MLC_STATUS_MAINPAGE (38h),
  *                                   STATUS_MASTER_MAINPAGE (39h),
  *                                   FIFO_STATUS1 (3Ah), FIFO_STATUS2(3Bh).
  *
  * @param  ctx      read / write interface definitions
  * @param  lsm6dsox_rounding_status_t: change the values of rounding_status
  *                                    in reg CTRL7_G
  *
  */
int32_t lsm6dsox_rounding_on_status_set(stmdev_ctx_t *ctx,
                                       lsm6dsox_rounding_status_t val)
{
  lsm6dsox_ctrl5_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.rounding_status = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&reg, 1);
  }

  return ret;
}

/**
  * @brief  rounding_on_status: [get] Source register rounding function in
  *                                   ALL_INT_SRC (1Ah), WAKE_UP_SRC(1Bh),
  *                                   TAP_SRC (1Ch), D6D_SRC (1Dh),
  *                                   STATUS_REG (1Eh) and
  *                                   EMB_FUNC_STATUS_MAINPAGE(35h),
  *                                   FSM_STATUS_A_MAINPAGE (36h),
  *                                   FSM_STATUS_B_MAINPAGE (37h),
  *                                   MLC_STATUS_MAINPAGE (38h),
  *                                   STATUS_MASTER_MAINPAGE (39h),
  *                                   FIFO_STATUS1 (3Ah), FIFO_STATUS2(3Bh).
  *
  * @param  ctx      read / write interface definitions
  * @param  lsm6dsox_rounding_status_t: Get the values of rounding_status
  *                                    in reg CTRL7_G
  *
  */
int32_t lsm6dsox_rounding_on_status_get(stmdev_ctx_t *ctx,
                                       lsm6dsox_rounding_status_t *val)
{
  lsm6dsox_ctrl5_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&reg, 1);
  switch (reg.rounding_status) {
    case LSM6DSOX_STAT_RND_DISABLE:
      *val = LSM6DSOX_STAT_RND_DISABLE;
      break;
    case LSM6DSOX_STAT_RND_ENABLE:
      *val = LSM6DSOX_STAT_RND_ENABLE;
      break;
    default:
      *val = LSM6DSOX_STAT_RND_DISABLE;
      break;
  }
  return ret;
}

/**
  * @brief  Temperature data output register (r).
  *         L and H registers together express a 16-bit word in two’s
  *         complement.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_OUT_TEMP_L, buff, 2);
  return ret;
}

/**
  * @brief  Angular rate sensor. The value is expressed as a 16-bit
  *         word in two’s complement.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_angular_rate_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_OUTX_L_G, buff, 6);
  return ret;
}

/**
  * @brief  Linear acceleration output register.
  *         The value is expressed as a 16-bit word in two’s complement.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_acceleration_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_OUTX_L_A, buff, 6);
  return ret;
}

/**
  * @brief  FIFO data output [get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_fifo_out_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_DATA_OUT_X_L, buff, 6);
  return ret;
}

/**
  * @brief  ois_angular_rate_raw: [get]  OIS angular rate sensor.
  *                                      The value is expressed as a
  *                                      16-bit word in two’s complement.
  *
  * @param  ctx      read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm6dsox_ois_angular_rate_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  return lsm6dsox_read_reg(ctx, LSM6DSOX_UI_OUTX_L_G_OIS, buff, 6);
}

/**
  * @brief  ois_acceleration_raw: [get] OIS Linear acceleration output register.
  *                                     The value is expressed as a
  *                                     16-bit word in two’s complement.
  *
  * @param  ctx      read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm6dsox_ois_acceleration_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  return lsm6dsox_read_reg(ctx, LSM6DSOX_UI_OUTX_L_A_OIS, buff, 6);
}

/**
  * @brief  aux_temperature_raw: [get]  Temperature from auxiliary
  *                                     interface.
  *                                     The value is expressed as a
  *                                     16-bit word in two’s complement.
  *
  * @param  ctx      read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm6dsox_aux_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  return lsm6dsox_read_reg(ctx, LSM6DSOX_SPI2_OUT_TEMP_L, buff, 2);
}

/**
  * @brief  aux_ois_angular_rate_raw: [get] OIS angular rate sensor from
  *                                         auxiliary interface.
  *                                         The value is expressed as a
  *                                         16-bit word in two’s complement.
  *
  * @param  ctx      read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm6dsox_aux_ois_angular_rate_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  return lsm6dsox_read_reg(ctx, LSM6DSOX_SPI2_OUTX_L_G_OIS, buff, 6);
}

/**
  * @brief  aux_ois_acceleration_raw: [get] OIS linear acceleration output
  *                                         register from auxiliary interface.
  *                                         The value is expressed as a
  *                                         16-bit word in two’s complement.
  *
  * @param  ctx      read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t lsm6dsox_aux_ois_acceleration_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  return lsm6dsox_read_reg(ctx, LSM6DSOX_SPI2_OUTX_L_A_OIS, buff, 6);
}

/**
  * @brief  Step counter output register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_number_of_steps_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_STEP_COUNTER_L, buff, 2);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Reset step counter register.[get]
  *
  * @param  ctx      read / write interface definitions
  *
  */
int32_t lsm6dsox_steps_reset(stmdev_ctx_t *ctx)
{
  lsm6dsox_emb_func_src_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_SRC, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.pedo_rst_step = PROPERTY_ENABLE;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_SRC, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
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
int32_t lsm6dsox_mlc_out_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MLC0_SRC, buff, 8);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_common
  * @brief   This section groups common usefull functions.
  * @{
  *
  */

/**
  * @brief  Difference in percentage of the effective ODR(and timestamp rate)
  *         with respect to the typical.
  *         Step:  0.15%. 8-bit format, 2's complement.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of freq_fine in reg
  *                      INTERNAL_FREQ_FINE
  *
  */
int32_t lsm6dsox_odr_cal_reg_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_internal_freq_fine_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_INTERNAL_FREQ_FINE, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.freq_fine = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_INTERNAL_FREQ_FINE,
                            (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Difference in percentage of the effective ODR(and timestamp rate)
  *         with respect to the typical.
  *         Step:  0.15%. 8-bit format, 2's complement.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of freq_fine in reg INTERNAL_FREQ_FINE
  *
  */
int32_t lsm6dsox_odr_cal_reg_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_internal_freq_fine_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_INTERNAL_FREQ_FINE, (uint8_t*)&reg, 1);
  *val = reg.freq_fine;

  return ret;
}


/**
  * @brief  Enable access to the embedded functions/sensor
  *         hub configuration registers.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of reg_access in
  *                               reg FUNC_CFG_ACCESS
  *
  */
int32_t lsm6dsox_mem_bank_set(stmdev_ctx_t *ctx, lsm6dsox_reg_access_t val)
{
  lsm6dsox_func_cfg_access_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FUNC_CFG_ACCESS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.reg_access = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FUNC_CFG_ACCESS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enable access to the embedded functions/sensor
  *         hub configuration registers.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of reg_access in
  *                               reg FUNC_CFG_ACCESS
  *
  */
int32_t lsm6dsox_mem_bank_get(stmdev_ctx_t *ctx, lsm6dsox_reg_access_t *val)
{
  lsm6dsox_func_cfg_access_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FUNC_CFG_ACCESS, (uint8_t*)&reg, 1);
  switch (reg.reg_access) {
    case LSM6DSOX_USER_BANK:
      *val = LSM6DSOX_USER_BANK;
      break;
    case LSM6DSOX_SENSOR_HUB_BANK:
      *val = LSM6DSOX_SENSOR_HUB_BANK;
      break;
    case LSM6DSOX_EMBEDDED_FUNC_BANK:
      *val = LSM6DSOX_EMBEDDED_FUNC_BANK;
      break;
    default:
      *val = LSM6DSOX_USER_BANK;
      break;
  }
  return ret;
}

/**
  * @brief  Write a line(byte) in a page.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  uint8_t address: page line address
  * @param  val      value to write
  *
  */
int32_t lsm6dsox_ln_pg_write_byte(stmdev_ctx_t *ctx, uint16_t address,
                                 uint8_t *val)
{
  lsm6dsox_page_rw_t page_rw;
  lsm6dsox_page_sel_t page_sel;
  lsm6dsox_page_address_t page_address;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);

  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    page_rw.page_rw = 0x02; /* page_write enable */
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_SEL, (uint8_t*) &page_sel, 1);
  }

  if (ret == 0) {
    page_sel.page_sel = ((uint8_t)(address >> 8) & 0x0FU);
    page_sel.not_used_01 = 1;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_SEL, (uint8_t*) &page_sel, 1);
  }
  if (ret == 0) {
    page_address.page_addr = (uint8_t)address & 0xFFU;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_ADDRESS,
                            (uint8_t*)&page_address, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_VALUE, val, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    page_rw.page_rw = 0x00; /* page_write disable */
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {

    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Write buffer in a page.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  uint8_t address: page line address
  * @param  uint8_t *buf: buffer to write
  * @param  uint8_t len: buffer len
  *
  */
int32_t lsm6dsox_ln_pg_write(stmdev_ctx_t *ctx, uint16_t address,
                             uint8_t *buf, uint8_t len)
{
  lsm6dsox_page_rw_t page_rw;
  lsm6dsox_page_sel_t page_sel;
  lsm6dsox_page_address_t  page_address;
  int32_t ret;
  uint8_t msb, lsb;
  uint8_t i ;

  msb = ((uint8_t)(address >> 8) & 0x0FU);
  lsb = (uint8_t)address & 0xFFU;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {

    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    page_rw.page_rw = 0x02; /* page_write enable*/
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_SEL, (uint8_t*) &page_sel, 1);
  }
  if (ret == 0) {
    page_sel.page_sel = msb;
    page_sel.not_used_01 = 1;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_SEL, (uint8_t*) &page_sel, 1);
  }
  if (ret == 0) {
    page_address.page_addr = lsb;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_ADDRESS,
                            (uint8_t*)&page_address, 1);
  }

  if (ret == 0) {
    for (i = 0; ( (i < len) && (ret == 0) ); i++)
    {
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_VALUE, &buf[i], 1);
      lsb++;
      /* Check if page wrap */
      if ( (lsb == 0x00U) && (ret == 0) ) {
        msb++;
        ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_SEL, (uint8_t*)&page_sel, 1);
        if (ret == 0) {
          page_sel.page_sel = msb;
          page_sel.not_used_01 = 1;
          ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_SEL,
                                  (uint8_t*)&page_sel, 1);
        }
      }
    }
  }
  page_sel.page_sel = 0;
  page_sel.not_used_01 = 1;
  ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_SEL, (uint8_t*) &page_sel, 1);

  if (ret == 0) {

    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    page_rw.page_rw = 0x00; /* page_write disable */
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }

  if (ret == 0) {

    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Read a line(byte) in a page.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  uint8_t address: page line address
  * @param  val      read value
  *
  */
int32_t lsm6dsox_ln_pg_read_byte(stmdev_ctx_t *ctx, uint16_t address,
                                uint8_t *val)
{
  lsm6dsox_page_rw_t page_rw;
  lsm6dsox_page_sel_t page_sel;
  lsm6dsox_page_address_t  page_address;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {

    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    page_rw.page_rw = 0x01; /* page_read enable*/
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {

    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_SEL, (uint8_t*) &page_sel, 1);
  }
  if (ret == 0) {
    page_sel.page_sel = ((uint8_t)(address >> 8) & 0x0FU);
    page_sel.not_used_01 = 1;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_SEL, (uint8_t*) &page_sel, 1);
  }
  if (ret == 0) {
    page_address.page_addr = (uint8_t)address & 0x00FFU;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_ADDRESS,
                            (uint8_t*)&page_address, 1);
  }
  if (ret == 0) {

    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_VALUE, val, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    page_rw.page_rw = 0x00; /* page_read disable */
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Data-ready pulsed / letched mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of
  *                                     dataready_pulsed in
  *                                     reg COUNTER_BDR_REG1
  *
  */
int32_t lsm6dsox_data_ready_mode_set(stmdev_ctx_t *ctx,
                                    lsm6dsox_dataready_pulsed_t val)
{
  lsm6dsox_counter_bdr_reg1_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_COUNTER_BDR_REG1, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.dataready_pulsed = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_COUNTER_BDR_REG1, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Data-ready pulsed / letched mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of
  *                                     dataready_pulsed in
  *                                     reg COUNTER_BDR_REG1
  *
  */
int32_t lsm6dsox_data_ready_mode_get(stmdev_ctx_t *ctx,
                                    lsm6dsox_dataready_pulsed_t *val)
{
  lsm6dsox_counter_bdr_reg1_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_COUNTER_BDR_REG1, (uint8_t*)&reg, 1);
  switch (reg.dataready_pulsed) {
    case LSM6DSOX_DRDY_LATCHED:
      *val = LSM6DSOX_DRDY_LATCHED;
      break;
    case LSM6DSOX_DRDY_PULSED:
      *val = LSM6DSOX_DRDY_PULSED;
      break;
    default:
      *val = LSM6DSOX_DRDY_LATCHED;
      break;
  }
  return ret;
}

/**
  * @brief  Device "Who am I".[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WHO_AM_I, buff, 1);
  return ret;
}

/**
  * @brief  Software reset. Restore the default values
  *         in user registers[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sw_reset in reg CTRL3_C
  *
  */
int32_t lsm6dsox_reset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl3_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.sw_reset = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sw_reset in reg CTRL3_C
  *
  */
int32_t lsm6dsox_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl3_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  *val = reg.sw_reset;

  return ret;
}

/**
  * @brief  Register address automatically incremented during a multiple byte
  *         access with a serial interface.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of if_inc in reg CTRL3_C
  *
  */
int32_t lsm6dsox_auto_increment_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl3_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.if_inc = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Register address automatically incremented during a multiple byte
  *         access with a serial interface.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of if_inc in reg CTRL3_C
  *
  */
int32_t lsm6dsox_auto_increment_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl3_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  *val = reg.if_inc;

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of boot in reg CTRL3_C
  *
  */
int32_t lsm6dsox_boot_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl3_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.boot = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of boot in reg CTRL3_C
  *
  */
int32_t lsm6dsox_boot_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl3_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  *val = reg.boot;

  return ret;
}

/**
  * @brief  Linear acceleration sensor self-test enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of st_xl in reg CTRL5_C
  *
  */
int32_t lsm6dsox_xl_self_test_set(stmdev_ctx_t *ctx, lsm6dsox_st_xl_t val)
{
  lsm6dsox_ctrl5_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.st_xl = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Linear acceleration sensor self-test enable.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of st_xl in reg CTRL5_C
  *
  */
int32_t lsm6dsox_xl_self_test_get(stmdev_ctx_t *ctx, lsm6dsox_st_xl_t *val)
{
  lsm6dsox_ctrl5_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&reg, 1);
  switch (reg.st_xl) {
    case LSM6DSOX_XL_ST_DISABLE:
      *val = LSM6DSOX_XL_ST_DISABLE;
      break;
    case LSM6DSOX_XL_ST_POSITIVE:
      *val = LSM6DSOX_XL_ST_POSITIVE;
      break;
    case LSM6DSOX_XL_ST_NEGATIVE:
      *val = LSM6DSOX_XL_ST_NEGATIVE;
      break;
    default:
      *val = LSM6DSOX_XL_ST_DISABLE;
      break;
  }
  return ret;
}

/**
  * @brief  Angular rate sensor self-test enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of st_g in reg CTRL5_C
  *
  */
int32_t lsm6dsox_gy_self_test_set(stmdev_ctx_t *ctx, lsm6dsox_st_g_t val)
{
  lsm6dsox_ctrl5_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.st_g = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Angular rate sensor self-test enable.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of st_g in reg CTRL5_C
  *
  */
int32_t lsm6dsox_gy_self_test_get(stmdev_ctx_t *ctx, lsm6dsox_st_g_t *val)
{
  lsm6dsox_ctrl5_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&reg, 1);
  switch (reg.st_g) {
    case LSM6DSOX_GY_ST_DISABLE:
      *val = LSM6DSOX_GY_ST_DISABLE;
      break;
    case LSM6DSOX_GY_ST_POSITIVE:
      *val = LSM6DSOX_GY_ST_POSITIVE;
      break;
    case LSM6DSOX_GY_ST_NEGATIVE:
      *val = LSM6DSOX_GY_ST_NEGATIVE;
      break;
    default:
      *val = LSM6DSOX_GY_ST_DISABLE;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_filters
  * @brief     This section group all the functions concerning the
  *            filters configuration
  * @{
  *
  */

/**
  * @brief  Accelerometer output from LPF2 filtering stage selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lpf2_xl_en in reg CTRL1_XL
  *
  */
int32_t lsm6dsox_xl_filter_lp2_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl1_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL1_XL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.lpf2_xl_en = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL1_XL, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Accelerometer output from LPF2 filtering stage selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lpf2_xl_en in reg CTRL1_XL
  *
  */
int32_t lsm6dsox_xl_filter_lp2_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl1_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL1_XL, (uint8_t*)&reg, 1);
  *val = reg.lpf2_xl_en;

  return ret;
}

/**
  * @brief  Enables gyroscope digital LPF1 if auxiliary SPI is disabled;
  *         the bandwidth can be selected through FTYPE [2:0]
  *         in CTRL6_C (15h).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lpf1_sel_g in reg CTRL4_C
  *
  */
int32_t lsm6dsox_gy_filter_lp1_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl4_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.lpf1_sel_g = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enables gyroscope digital LPF1 if auxiliary SPI is disabled;
  *         the bandwidth can be selected through FTYPE [2:0]
  *         in CTRL6_C (15h).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lpf1_sel_g in reg CTRL4_C
  *
  */
int32_t lsm6dsox_gy_filter_lp1_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl4_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  *val = reg.lpf1_sel_g;

  return ret;
}

/**
  * @brief  Mask DRDY on pin (both XL & Gyro) until filter settling ends
  *         (XL and Gyro independently masked).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of drdy_mask in reg CTRL4_C
  *
  */
int32_t lsm6dsox_filter_settling_mask_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl4_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.drdy_mask = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Mask DRDY on pin (both XL & Gyro) until filter settling ends
  *         (XL and Gyro independently masked).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of drdy_mask in reg CTRL4_C
  *
  */
int32_t lsm6dsox_filter_settling_mask_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl4_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  *val = reg.drdy_mask;

  return ret;
}

/**
  * @brief  Gyroscope lp1 bandwidth.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ftype in reg CTRL6_C
  *
  */
int32_t lsm6dsox_gy_lp1_bandwidth_set(stmdev_ctx_t *ctx, lsm6dsox_ftype_t val)
{
  lsm6dsox_ctrl6_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL6_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.ftype = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL6_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Gyroscope lp1 bandwidth.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val       Get the values of ftype in reg CTRL6_C
  *
  */
int32_t lsm6dsox_gy_lp1_bandwidth_get(stmdev_ctx_t *ctx, lsm6dsox_ftype_t *val)
{
  lsm6dsox_ctrl6_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL6_C, (uint8_t*)&reg, 1);
  switch (reg.ftype) {
    case LSM6DSOX_ULTRA_LIGHT:
      *val = LSM6DSOX_ULTRA_LIGHT;
      break;
    case LSM6DSOX_VERY_LIGHT:
      *val = LSM6DSOX_VERY_LIGHT;
      break;
    case LSM6DSOX_LIGHT:
      *val = LSM6DSOX_LIGHT;
      break;
    case LSM6DSOX_MEDIUM:
      *val = LSM6DSOX_MEDIUM;
      break;
    case LSM6DSOX_STRONG:
      *val = LSM6DSOX_STRONG;
      break;
    case LSM6DSOX_VERY_STRONG:
      *val = LSM6DSOX_VERY_STRONG;
      break;
    case LSM6DSOX_AGGRESSIVE:
      *val = LSM6DSOX_AGGRESSIVE;
      break;
    case LSM6DSOX_XTREME:
      *val = LSM6DSOX_XTREME;
      break;
    default:
      *val = LSM6DSOX_ULTRA_LIGHT;
      break;
  }
  return ret;
}

/**
  * @brief  Low pass filter 2 on 6D function selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of low_pass_on_6d in reg CTRL8_XL
  *
  */
int32_t lsm6dsox_xl_lp2_on_6d_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl8_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL8_XL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.low_pass_on_6d = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL8_XL, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Low pass filter 2 on 6D function selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of low_pass_on_6d in reg CTRL8_XL
  *
  */
int32_t lsm6dsox_xl_lp2_on_6d_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl8_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL8_XL, (uint8_t*)&reg, 1);
  *val = reg.low_pass_on_6d;

  return ret;
}

/**
  * @brief  Accelerometer slope filter / high-pass filter selection
  *         on output.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of hp_slope_xl_en
  *                                   in reg CTRL8_XL
  *
  */
int32_t lsm6dsox_xl_hp_path_on_out_set(stmdev_ctx_t *ctx,
                                      lsm6dsox_hp_slope_xl_en_t val)
{
  lsm6dsox_ctrl8_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL8_XL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.hp_slope_xl_en = ((uint8_t)val & 0x10U) >> 4;
    reg.hp_ref_mode_xl = ((uint8_t)val & 0x20U) >> 5;
    reg.hpcf_xl = (uint8_t)val & 0x07U;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL8_XL, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Accelerometer slope filter / high-pass filter selection
  *         on output.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of hp_slope_xl_en
  *                                   in reg CTRL8_XL
  *
  */
int32_t lsm6dsox_xl_hp_path_on_out_get(stmdev_ctx_t *ctx,
                                      lsm6dsox_hp_slope_xl_en_t *val)
{
  lsm6dsox_ctrl8_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL8_XL, (uint8_t*)&reg, 1);
  switch ((reg.hp_ref_mode_xl << 5) | (reg.hp_slope_xl_en << 4) |
          reg.hpcf_xl) {
    case LSM6DSOX_HP_PATH_DISABLE_ON_OUT:
      *val = LSM6DSOX_HP_PATH_DISABLE_ON_OUT;
      break;
    case LSM6DSOX_SLOPE_ODR_DIV_4:
      *val = LSM6DSOX_SLOPE_ODR_DIV_4;
      break;
    case LSM6DSOX_HP_ODR_DIV_10:
      *val = LSM6DSOX_HP_ODR_DIV_10;
      break;
    case LSM6DSOX_HP_ODR_DIV_20:
      *val = LSM6DSOX_HP_ODR_DIV_20;
      break;
    case LSM6DSOX_HP_ODR_DIV_45:
      *val = LSM6DSOX_HP_ODR_DIV_45;
      break;
    case LSM6DSOX_HP_ODR_DIV_100:
      *val = LSM6DSOX_HP_ODR_DIV_100;
      break;
    case LSM6DSOX_HP_ODR_DIV_200:
      *val = LSM6DSOX_HP_ODR_DIV_200;
      break;
    case LSM6DSOX_HP_ODR_DIV_400:
      *val = LSM6DSOX_HP_ODR_DIV_400;
      break;
    case LSM6DSOX_HP_ODR_DIV_800:
      *val = LSM6DSOX_HP_ODR_DIV_800;
      break;
    case LSM6DSOX_HP_REF_MD_ODR_DIV_10:
      *val = LSM6DSOX_HP_REF_MD_ODR_DIV_10;
      break;
    case LSM6DSOX_HP_REF_MD_ODR_DIV_20:
      *val = LSM6DSOX_HP_REF_MD_ODR_DIV_20;
      break;
    case LSM6DSOX_HP_REF_MD_ODR_DIV_45:
      *val = LSM6DSOX_HP_REF_MD_ODR_DIV_45;
      break;
    case LSM6DSOX_HP_REF_MD_ODR_DIV_100:
      *val = LSM6DSOX_HP_REF_MD_ODR_DIV_100;
      break;
    case LSM6DSOX_HP_REF_MD_ODR_DIV_200:
      *val = LSM6DSOX_HP_REF_MD_ODR_DIV_200;
      break;
    case LSM6DSOX_HP_REF_MD_ODR_DIV_400:
      *val = LSM6DSOX_HP_REF_MD_ODR_DIV_400;
      break;
    case LSM6DSOX_HP_REF_MD_ODR_DIV_800:
      *val = LSM6DSOX_HP_REF_MD_ODR_DIV_800;
      break;
    case LSM6DSOX_LP_ODR_DIV_10:
      *val = LSM6DSOX_LP_ODR_DIV_10;
      break;
    case LSM6DSOX_LP_ODR_DIV_20:
      *val = LSM6DSOX_LP_ODR_DIV_20;
      break;
    case LSM6DSOX_LP_ODR_DIV_45:
      *val = LSM6DSOX_LP_ODR_DIV_45;
      break;
    case LSM6DSOX_LP_ODR_DIV_100:
      *val = LSM6DSOX_LP_ODR_DIV_100;
      break;
    case LSM6DSOX_LP_ODR_DIV_200:
      *val = LSM6DSOX_LP_ODR_DIV_200;
      break;
    case LSM6DSOX_LP_ODR_DIV_400:
      *val = LSM6DSOX_LP_ODR_DIV_400;
      break;
    case LSM6DSOX_LP_ODR_DIV_800:
      *val = LSM6DSOX_LP_ODR_DIV_800;
      break;
    default:
      *val = LSM6DSOX_HP_PATH_DISABLE_ON_OUT;
      break;
  }

  return ret;
}

/**
  * @brief  Enables accelerometer LPF2 and HPF fast-settling mode.
  *         The filter sets the second samples after writing this bit.
  *         Active only during device exit from power-down mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fastsettl_mode_xl in
  *                  reg CTRL8_XL
  *
  */
int32_t lsm6dsox_xl_fast_settling_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl8_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL8_XL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.fastsettl_mode_xl = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL8_XL, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enables accelerometer LPF2 and HPF fast-settling mode.
  *         The filter sets the second samples after writing this bit.
  *         Active only during device exit from power-down mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fastsettl_mode_xl in reg CTRL8_XL
  *
  */
int32_t lsm6dsox_xl_fast_settling_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl8_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL8_XL, (uint8_t*)&reg, 1);
  *val = reg.fastsettl_mode_xl;

  return ret;
}

/**
  * @brief  HPF or SLOPE filter selection on wake-up and Activity/Inactivity
  *         functions.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of slope_fds in reg TAP_CFG0
  *
  */
int32_t lsm6dsox_xl_hp_path_internal_set(stmdev_ctx_t *ctx,
                                        lsm6dsox_slope_fds_t val)
{
  lsm6dsox_tap_cfg0_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.slope_fds = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  HPF or SLOPE filter selection on wake-up and Activity/Inactivity
  *         functions.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Change the values of slope_fds in reg TAP_CFG0
  *
  */
int32_t lsm6dsox_xl_hp_path_internal_get(stmdev_ctx_t *ctx,
                                        lsm6dsox_slope_fds_t *val)
{
  lsm6dsox_tap_cfg0_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  switch (reg.slope_fds) {
    case LSM6DSOX_USE_SLOPE:
      *val = LSM6DSOX_USE_SLOPE;
      break;
    case LSM6DSOX_USE_HPF:
      *val = LSM6DSOX_USE_HPF;
      break;
    default:
      *val = LSM6DSOX_USE_SLOPE;
      break;
  }
  return ret;
}

/**
  * @brief  Enables gyroscope digital high-pass filter. The filter is
  *         enabled only if the gyro is in HP mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of hp_en_g and hp_en_g
  *                            in reg CTRL7_G
  *
  */
int32_t lsm6dsox_gy_hp_path_internal_set(stmdev_ctx_t *ctx,
                                        lsm6dsox_hpm_g_t val)
{
  lsm6dsox_ctrl7_g_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL7_G, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.hp_en_g = ((uint8_t)val & 0x80U) >> 7;
    reg.hpm_g = (uint8_t)val & 0x03U;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL7_G, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enables gyroscope digital high-pass filter. The filter is
  *         enabled only if the gyro is in HP mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of hp_en_g and hp_en_g
  *                            in reg CTRL7_G
  *
  */
int32_t lsm6dsox_gy_hp_path_internal_get(stmdev_ctx_t *ctx,
                                        lsm6dsox_hpm_g_t *val)
{
  lsm6dsox_ctrl7_g_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL7_G, (uint8_t*)&reg, 1);
  switch ((reg.hp_en_g << 7) + reg.hpm_g) {
    case LSM6DSOX_HP_FILTER_NONE:
      *val = LSM6DSOX_HP_FILTER_NONE;
      break;
    case LSM6DSOX_HP_FILTER_16mHz:
      *val = LSM6DSOX_HP_FILTER_16mHz;
      break;
    case LSM6DSOX_HP_FILTER_65mHz:
      *val = LSM6DSOX_HP_FILTER_65mHz;
      break;
    case LSM6DSOX_HP_FILTER_260mHz:
      *val = LSM6DSOX_HP_FILTER_260mHz;
      break;
    case LSM6DSOX_HP_FILTER_1Hz04:
      *val = LSM6DSOX_HP_FILTER_1Hz04;
      break;
    default:
      *val = LSM6DSOX_HP_FILTER_NONE;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_ Auxiliary_interface
  * @brief     This section groups all the functions concerning
  *            auxiliary interface.
  * @{
  *
  */

/**
  * @brief   OIS data reading from Auxiliary / Main SPI.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of spi2_read_en in reg UI_INT_OIS
  *
  */
int32_t lsm6dsox_ois_mode_set(stmdev_ctx_t *ctx, lsm6dsox_spi2_read_en_t val)
{
  lsm6dsox_func_cfg_access_t func_cfg_access;
  lsm6dsox_ui_int_ois_t ui_int_ois;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_INT_OIS, (uint8_t*)&ui_int_ois, 1);
  if (ret == 0) {
    ui_int_ois.spi2_read_en = ((uint8_t)val & 0x01U);
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_INT_OIS,
                             (uint8_t*)&ui_int_ois, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FUNC_CFG_ACCESS,
                            (uint8_t*)&func_cfg_access, 1);
  }
  if (ret == 0) {
    func_cfg_access.ois_ctrl_from_ui = ( ((uint8_t)val & 0x02U) >> 1 );
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FUNC_CFG_ACCESS,
                             (uint8_t*)&func_cfg_access, 1);
  }
  return ret;
}

/**
  * @brief  aux_ois_data: [get] OIS data reading from Auxiliary / Main SPI
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of spi2_read_en
  *                                 in reg UI_INT_OIS
  *
  */
int32_t lsm6dsox_ois_mode_get(stmdev_ctx_t *ctx,
                                 lsm6dsox_spi2_read_en_t *val)
{
  lsm6dsox_func_cfg_access_t func_cfg_access;
  lsm6dsox_ui_int_ois_t ui_int_ois;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_INT_OIS, (uint8_t*)&ui_int_ois, 1);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FUNC_CFG_ACCESS,
                            (uint8_t*)&func_cfg_access, 1);
  }
  switch ((func_cfg_access.ois_ctrl_from_ui << 1) + ui_int_ois.spi2_read_en) {
    case LSM6DSOX_OIS_CTRL_AUX_DATA_UI:
      *val = LSM6DSOX_OIS_CTRL_AUX_DATA_UI;
      break;
    case LSM6DSOX_OIS_CTRL_AUX_DATA_UI_AUX:
      *val = LSM6DSOX_OIS_CTRL_AUX_DATA_UI_AUX;
      break;
    case LSM6DSOX_OIS_CTRL_UI_AUX_DATA_UI:
      *val = LSM6DSOX_OIS_CTRL_UI_AUX_DATA_UI;
      break;
    case LSM6DSOX_OIS_CTRL_UI_AUX_DATA_UI_AUX:
      *val = LSM6DSOX_OIS_CTRL_UI_AUX_DATA_UI_AUX;
      break;
    default:
      *val = LSM6DSOX_OIS_CTRL_AUX_DATA_UI;
      break;
  }
  return ret;
}

/**
  * @brief  aOn auxiliary interface connect/disconnect SDO and OCS
  *         internal pull-up.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ois_pu_dis in
  *                               reg PIN_CTRL
  *
  */
int32_t lsm6dsox_aux_sdo_ocs_mode_set(stmdev_ctx_t *ctx,
                                     lsm6dsox_ois_pu_dis_t val)
{
  lsm6dsox_pin_ctrl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PIN_CTRL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.ois_pu_dis = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PIN_CTRL, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  On auxiliary interface connect/disconnect SDO and OCS
  *         internal pull-up.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of ois_pu_dis in reg PIN_CTRL
  *
  */
int32_t lsm6dsox_aux_sdo_ocs_mode_get(stmdev_ctx_t *ctx,
                                     lsm6dsox_ois_pu_dis_t *val)
{
  lsm6dsox_pin_ctrl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PIN_CTRL, (uint8_t*)&reg, 1);
  switch (reg.ois_pu_dis) {
    case LSM6DSOX_AUX_PULL_UP_DISC:
      *val = LSM6DSOX_AUX_PULL_UP_DISC;
      break;
    case LSM6DSOX_AUX_PULL_UP_CONNECT:
      *val = LSM6DSOX_AUX_PULL_UP_CONNECT;
      break;
    default:
      *val = LSM6DSOX_AUX_PULL_UP_DISC;
      break;
  }
  return ret;
}

/**
  * @brief  OIS chain on aux interface power on mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ois_on in reg CTRL7_G
  *
  */
int32_t lsm6dsox_aux_pw_on_ctrl_set(stmdev_ctx_t *ctx, lsm6dsox_ois_on_t val)
{
  lsm6dsox_ctrl7_g_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL7_G, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.ois_on_en = (uint8_t)val & 0x01U;
    reg.ois_on = (uint8_t)val & 0x01U;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL7_G, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  aux_pw_on_ctrl: [get]  OIS chain on aux interface power on mode
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of ois_on in reg CTRL7_G
  *
  */
int32_t lsm6dsox_aux_pw_on_ctrl_get(stmdev_ctx_t *ctx, lsm6dsox_ois_on_t *val)
{
  lsm6dsox_ctrl7_g_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL7_G, (uint8_t*)&reg, 1);
  switch (reg.ois_on) {
    case LSM6DSOX_AUX_ON:
      *val = LSM6DSOX_AUX_ON;
      break;
    case LSM6DSOX_AUX_ON_BY_AUX_INTERFACE:
      *val = LSM6DSOX_AUX_ON_BY_AUX_INTERFACE;
      break;
    default:
      *val = LSM6DSOX_AUX_ON;
      break;
  }

  return ret;
}

/**
  * @brief  Accelerometer full-scale management between UI chain and
  *         OIS chain. When XL UI is on, the full scale is the same
  *         between UI/OIS and is chosen by the UI CTRL registers;
  *         when XL UI is in PD, the OIS can choose the FS.
  *         Full scales are independent between the UI/OIS chain
  *         but both bound to 8 g.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of xl_fs_mode in
  *                               reg CTRL8_XL
  *
  */
int32_t lsm6dsox_aux_xl_fs_mode_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_xl_fs_mode_t val)
{
  lsm6dsox_ctrl8_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL8_XL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.xl_fs_mode = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL8_XL, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Accelerometer full-scale management between UI chain and
  *         OIS chain. When XL UI is on, the full scale is the same
  *         between UI/OIS and is chosen by the UI CTRL registers;
  *         when XL UI is in PD, the OIS can choose the FS.
  *         Full scales are independent between the UI/OIS chain
  *         but both bound to 8 g.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of xl_fs_mode in reg CTRL8_XL
  *
  */
int32_t lsm6dsox_aux_xl_fs_mode_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_xl_fs_mode_t *val)
{
  lsm6dsox_ctrl8_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL8_XL, (uint8_t*)&reg, 1);
  switch (reg.xl_fs_mode) {
    case LSM6DSOX_USE_SAME_XL_FS:
      *val = LSM6DSOX_USE_SAME_XL_FS;
      break;
    case LSM6DSOX_USE_DIFFERENT_XL_FS:
      *val = LSM6DSOX_USE_DIFFERENT_XL_FS;
      break;
    default:
      *val = LSM6DSOX_USE_SAME_XL_FS;
      break;
  }

  return ret;
}

/**
  * @brief  The STATUS_SPIAux register is read by the auxiliary SPI.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get registers STATUS_SPIAUX
  *
  */
int32_t lsm6dsox_aux_status_reg_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_spi2_status_reg_ois_t *val)
{
  int32_t ret;
  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SPI2_STATUS_REG_OIS, (uint8_t*) val, 1);
  return ret;
}

/**
  * @brief  aux_xl_flag_data_ready: [get]  AUX accelerometer data available
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of xlda in reg STATUS_SPIAUX
  *
  */
int32_t lsm6dsox_aux_xl_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_spi2_status_reg_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SPI2_STATUS_REG_OIS, (uint8_t*)&reg, 1);
  *val = reg.xlda;

  return ret;
}

/**
  * @brief  aux_gy_flag_data_ready: [get]  AUX gyroscope data available.
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of gda in reg STATUS_SPIAUX
  *
  */
int32_t lsm6dsox_aux_gy_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_spi2_status_reg_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SPI2_STATUS_REG_OIS, (uint8_t*)&reg, 1);
  *val = reg.gda;

  return ret;
}

/**
  * @brief  High when the gyroscope output is in the settling phase.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of gyro_settling in reg STATUS_SPIAUX
  *
  */
int32_t lsm6dsox_aux_gy_flag_settling_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_spi2_status_reg_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SPI2_STATUS_REG_OIS, (uint8_t*)&reg, 1);
  *val = reg.gyro_settling;

  return ret;
}

/**
  * @brief  Indicates polarity of DEN signal on OIS chain.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of den_lh_ois in
  *                  reg INT_OIS
  *
  */
int32_t lsm6dsox_aux_den_polarity_set(stmdev_ctx_t *ctx,
                                     lsm6dsox_den_lh_ois_t val)
{
  lsm6dsox_ui_int_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_INT_OIS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.den_lh_ois = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_INT_OIS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Indicates polarity of DEN signal on OIS chain.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of den_lh_ois in reg INT_OIS
  *
  */
int32_t lsm6dsox_aux_den_polarity_get(stmdev_ctx_t *ctx,
                                     lsm6dsox_den_lh_ois_t *val)
{
  lsm6dsox_ui_int_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_INT_OIS, (uint8_t*)&reg, 1);
  switch (reg.den_lh_ois) {
    case LSM6DSOX_AUX_DEN_ACTIVE_LOW:
      *val = LSM6DSOX_AUX_DEN_ACTIVE_LOW;
      break;
    case LSM6DSOX_AUX_DEN_ACTIVE_HIGH:
      *val = LSM6DSOX_AUX_DEN_ACTIVE_HIGH;
      break;
    default:
      *val = LSM6DSOX_AUX_DEN_ACTIVE_LOW;
      break;
  }
  return ret;
}

/**
  * @brief  Configure DEN mode on the OIS chain.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lvl2_ois in reg INT_OIS
  *
  */
int32_t lsm6dsox_aux_den_mode_set(stmdev_ctx_t *ctx, lsm6dsox_lvl2_ois_t val)
{
  lsm6dsox_ui_ctrl1_ois_t ctrl1_ois;
  lsm6dsox_ui_int_ois_t int_ois;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_INT_OIS, (uint8_t*) &int_ois, 1);
  if (ret == 0) {
    int_ois.lvl2_ois = (uint8_t)val & 0x01U;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_INT_OIS, (uint8_t*) &int_ois, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, (uint8_t*) &ctrl1_ois, 1);
  }
  if (ret == 0) {
    ctrl1_ois.lvl1_ois = ((uint8_t)val & 0x02U) >> 1;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, (uint8_t*) &ctrl1_ois, 1);
  }
  return ret;
}

/**
  * @brief  Configure DEN mode on the OIS chain.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of lvl2_ois in reg INT_OIS
  *
  */
int32_t lsm6dsox_aux_den_mode_get(stmdev_ctx_t *ctx, lsm6dsox_lvl2_ois_t *val)
{
  lsm6dsox_ui_ctrl1_ois_t ctrl1_ois;
  lsm6dsox_ui_int_ois_t int_ois;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_INT_OIS, (uint8_t*) &int_ois, 1);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, (uint8_t*) &ctrl1_ois, 1);
    switch ((ctrl1_ois.lvl1_ois << 1) + int_ois.lvl2_ois) {
      case LSM6DSOX_AUX_DEN_DISABLE:
        *val = LSM6DSOX_AUX_DEN_DISABLE;
        break;
      case LSM6DSOX_AUX_DEN_LEVEL_LATCH:
        *val = LSM6DSOX_AUX_DEN_LEVEL_LATCH;
        break;
      case LSM6DSOX_AUX_DEN_LEVEL_TRIG:
        *val = LSM6DSOX_AUX_DEN_LEVEL_TRIG;
        break;
      default:
        *val = LSM6DSOX_AUX_DEN_DISABLE;
        break;
    }
  }
  return ret;
}

/**
  * @brief  Enables/Disable OIS chain DRDY on INT2 pin.
  *         This setting has priority over all other INT2 settings.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of int2_drdy_ois in reg INT_OIS
  *
  */
int32_t lsm6dsox_aux_drdy_on_int2_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ui_int_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_INT_OIS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.int2_drdy_ois = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_INT_OIS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enables/Disable OIS chain DRDY on INT2 pin.
  *         This setting has priority over all other INT2 settings.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of int2_drdy_ois in reg INT_OIS
  *
  */
int32_t lsm6dsox_aux_drdy_on_int2_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ui_int_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_INT_OIS, (uint8_t*)&reg, 1);
  *val = reg.int2_drdy_ois;

  return ret;
}

/**
  * @brief  Enables OIS chain data processing for gyro in Mode 3 and Mode 4
  *         (mode4_en = 1) and accelerometer data in and Mode 4 (mode4_en = 1).
  *         When the OIS chain is enabled, the OIS outputs are available
  *         through the SPI2 in registers OUTX_L_G (22h) through
  *         OUTZ_H_G (27h) and STATUS_REG (1Eh) / STATUS_SPIAux, and
  *         LPF1 is dedicated to this chain.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ois_en_spi2 in
  *                                reg CTRL1_OIS
  *
  */
int32_t lsm6dsox_aux_mode_set(stmdev_ctx_t *ctx, lsm6dsox_ois_en_spi2_t val)
{
  lsm6dsox_ui_ctrl1_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.ois_en_spi2 = (uint8_t)val & 0x01U;
    reg.mode4_en = ((uint8_t)val & 0x02U) >> 1;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enables OIS chain data processing for gyro in Mode 3 and Mode 4
  *         (mode4_en = 1) and accelerometer data in and Mode 4 (mode4_en = 1).
  *         When the OIS chain is enabled, the OIS outputs are available
  *         through the SPI2 in registers OUTX_L_G (22h) through
  *         OUTZ_H_G (27h) and STATUS_REG (1Eh) / STATUS_SPIAux, and
  *         LPF1 is dedicated to this chain.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of ois_en_spi2 in
  *                                reg CTRL1_OIS
  *
  */
int32_t lsm6dsox_aux_mode_get(stmdev_ctx_t *ctx, lsm6dsox_ois_en_spi2_t *val)
{
  lsm6dsox_ui_ctrl1_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, (uint8_t*)&reg, 1);
  switch ((reg.mode4_en << 1) | reg.ois_en_spi2) {
    case LSM6DSOX_AUX_DISABLE:
      *val = LSM6DSOX_AUX_DISABLE;
      break;
    case LSM6DSOX_MODE_3_GY:
      *val = LSM6DSOX_MODE_3_GY;
      break;
    case LSM6DSOX_MODE_4_GY_XL:
      *val = LSM6DSOX_MODE_4_GY_XL;
      break;
    default:
      *val = LSM6DSOX_AUX_DISABLE;
      break;
  }
  return ret;
}

/**
  * @brief  Selects gyroscope OIS chain full-scale.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fs_g_ois in reg CTRL1_OIS
  *
  */
int32_t lsm6dsox_aux_gy_full_scale_set(stmdev_ctx_t *ctx,
                                      lsm6dsox_fs_g_ois_t val)
{
  lsm6dsox_ui_ctrl1_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.fs_g_ois = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Selects gyroscope OIS chain full-scale.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of fs_g_ois in reg CTRL1_OIS
  *
  */
int32_t lsm6dsox_aux_gy_full_scale_get(stmdev_ctx_t *ctx,
                                      lsm6dsox_fs_g_ois_t *val)
{
  lsm6dsox_ui_ctrl1_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, (uint8_t*)&reg, 1);
  switch (reg.fs_g_ois) {
    case LSM6DSOX_250dps_AUX:
      *val = LSM6DSOX_250dps_AUX;
      break;
    case LSM6DSOX_125dps_AUX:
      *val = LSM6DSOX_125dps_AUX;
      break;
    case LSM6DSOX_500dps_AUX:
      *val = LSM6DSOX_500dps_AUX;
      break;
    case LSM6DSOX_1000dps_AUX:
      *val = LSM6DSOX_1000dps_AUX;
      break;
    case LSM6DSOX_2000dps_AUX:
      *val = LSM6DSOX_2000dps_AUX;
      break;
    default:
      *val = LSM6DSOX_250dps_AUX;
      break;
  }
  return ret;
}

/**
  * @brief  SPI2 3- or 4-wire interface.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sim_ois in reg CTRL1_OIS
  *
  */
int32_t lsm6dsox_aux_spi_mode_set(stmdev_ctx_t *ctx, lsm6dsox_sim_ois_t val)
{
  lsm6dsox_ui_ctrl1_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.sim_ois = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  SPI2 3- or 4-wire interface.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of sim_ois in reg CTRL1_OIS
  *
  */
int32_t lsm6dsox_aux_spi_mode_get(stmdev_ctx_t *ctx, lsm6dsox_sim_ois_t *val)
{
  lsm6dsox_ui_ctrl1_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, (uint8_t*)&reg, 1);
  switch (reg.sim_ois) {
    case LSM6DSOX_AUX_SPI_4_WIRE:
      *val = LSM6DSOX_AUX_SPI_4_WIRE;
      break;
    case LSM6DSOX_AUX_SPI_3_WIRE:
      *val = LSM6DSOX_AUX_SPI_3_WIRE;
      break;
    default:
      *val = LSM6DSOX_AUX_SPI_4_WIRE;
      break;
  }
  return ret;
}

/**
  * @brief  Selects gyroscope digital LPF1 filter bandwidth.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ftype_ois in
  *                              reg CTRL2_OIS
  *
  */
int32_t lsm6dsox_aux_gy_lp1_bandwidth_set(stmdev_ctx_t *ctx,
                                         lsm6dsox_ftype_ois_t val)
{
  lsm6dsox_ui_ctrl2_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL2_OIS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.ftype_ois = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_CTRL2_OIS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Selects gyroscope digital LPF1 filter bandwidth.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of ftype_ois in reg CTRL2_OIS
  *
  */
int32_t lsm6dsox_aux_gy_lp1_bandwidth_get(stmdev_ctx_t *ctx,
                                         lsm6dsox_ftype_ois_t *val)
{
  lsm6dsox_ui_ctrl2_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL2_OIS, (uint8_t*)&reg, 1);
  switch (reg.ftype_ois) {
    case LSM6DSOX_351Hz39:
      *val = LSM6DSOX_351Hz39;
      break;
    case LSM6DSOX_236Hz63:
      *val = LSM6DSOX_236Hz63;
      break;
    case LSM6DSOX_172Hz70:
      *val = LSM6DSOX_172Hz70;
      break;
    case LSM6DSOX_937Hz91:
      *val = LSM6DSOX_937Hz91;
      break;
    default:
      *val = LSM6DSOX_351Hz39;
      break;
  }
  return ret;
}

/**
  * @brief  Selects gyroscope OIS chain digital high-pass filter cutoff.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of hpm_ois in reg CTRL2_OIS
  *
  */
int32_t lsm6dsox_aux_gy_hp_bandwidth_set(stmdev_ctx_t *ctx,
                                        lsm6dsox_hpm_ois_t val)
{
  lsm6dsox_ui_ctrl2_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL2_OIS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.hpm_ois = (uint8_t)val & 0x03U;
    reg.hp_en_ois = ((uint8_t)val & 0x10U) >> 4;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_CTRL2_OIS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Selects gyroscope OIS chain digital high-pass filter cutoff.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of hpm_ois in reg CTRL2_OIS
  *
  */
int32_t lsm6dsox_aux_gy_hp_bandwidth_get(stmdev_ctx_t *ctx,
                                        lsm6dsox_hpm_ois_t *val)
{
  lsm6dsox_ui_ctrl2_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL2_OIS, (uint8_t*)&reg, 1);
  switch ((reg.hp_en_ois << 4) | reg.hpm_ois) {
    case LSM6DSOX_AUX_HP_DISABLE:
      *val = LSM6DSOX_AUX_HP_DISABLE;
      break;
    case LSM6DSOX_AUX_HP_Hz016:
      *val = LSM6DSOX_AUX_HP_Hz016;
      break;
    case LSM6DSOX_AUX_HP_Hz065:
      *val = LSM6DSOX_AUX_HP_Hz065;
      break;
    case LSM6DSOX_AUX_HP_Hz260:
      *val = LSM6DSOX_AUX_HP_Hz260;
      break;
    case LSM6DSOX_AUX_HP_1Hz040:
      *val = LSM6DSOX_AUX_HP_1Hz040;
      break;
    default:
      *val = LSM6DSOX_AUX_HP_DISABLE;
      break;
  }
  return ret;
}

/**
  * @brief  Enable / Disables OIS chain clamp.
  *         Enable: All OIS chain outputs = 8000h
  *         during self-test; Disable: OIS chain self-test
  *         outputs dependent from the aux gyro full
  *         scale selected.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of st_ois_clampdis in
  *                                    reg CTRL3_OIS
  *
  */
int32_t lsm6dsox_aux_gy_clamp_set(stmdev_ctx_t *ctx,
                                 lsm6dsox_st_ois_clampdis_t val)
{
  lsm6dsox_ui_ctrl3_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL3_OIS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.st_ois_clampdis = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_CTRL3_OIS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enable / Disables OIS chain clamp.
  *         Enable: All OIS chain outputs = 8000h
  *         during self-test; Disable: OIS chain self-test
  *         outputs dependent from the aux gyro full
  *         scale selected.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of st_ois_clampdis in
  *                                    reg CTRL3_OIS
  *
  */
int32_t lsm6dsox_aux_gy_clamp_get(stmdev_ctx_t *ctx,
                                 lsm6dsox_st_ois_clampdis_t *val)
{
  lsm6dsox_ui_ctrl3_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL3_OIS, (uint8_t*)&reg, 1);
  switch (reg.st_ois_clampdis) {
    case LSM6DSOX_ENABLE_CLAMP:
      *val = LSM6DSOX_ENABLE_CLAMP;
      break;
    case LSM6DSOX_DISABLE_CLAMP:
      *val = LSM6DSOX_DISABLE_CLAMP;
      break;
    default:
      *val = LSM6DSOX_ENABLE_CLAMP;
      break;
  }
  return ret;
}

/**
  * @brief  Selects accelerometer OIS channel bandwidth.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of
  *                                       filter_xl_conf_ois in reg CTRL3_OIS
  *
  */
int32_t lsm6dsox_aux_xl_bandwidth_set(stmdev_ctx_t *ctx,
                                     lsm6dsox_filter_xl_conf_ois_t val)
{
  lsm6dsox_ui_ctrl3_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL3_OIS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.filter_xl_conf_ois = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_CTRL3_OIS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Selects accelerometer OIS channel bandwidth.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of
  *                                       filter_xl_conf_ois in reg CTRL3_OIS
  *
  */
int32_t lsm6dsox_aux_xl_bandwidth_get(stmdev_ctx_t *ctx,
                                     lsm6dsox_filter_xl_conf_ois_t *val)
{
  lsm6dsox_ui_ctrl3_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL3_OIS, (uint8_t*)&reg, 1);

  switch (reg.filter_xl_conf_ois) {
    case LSM6DSOX_289Hz:
      *val = LSM6DSOX_289Hz;
      break;
    case LSM6DSOX_258Hz:
      *val = LSM6DSOX_258Hz;
      break;
    case LSM6DSOX_120Hz:
      *val = LSM6DSOX_120Hz;
      break;
    case LSM6DSOX_65Hz2:
      *val = LSM6DSOX_65Hz2;
      break;
    case LSM6DSOX_33Hz2:
      *val = LSM6DSOX_33Hz2;
      break;
    case LSM6DSOX_16Hz6:
      *val = LSM6DSOX_16Hz6;
      break;
    case LSM6DSOX_8Hz30:
      *val = LSM6DSOX_8Hz30;
      break;
    case LSM6DSOX_4Hz15:
      *val = LSM6DSOX_4Hz15;
      break;
    default:
      *val = LSM6DSOX_289Hz;
      break;
  }
  return ret;
}

/**
  * @brief  Selects accelerometer OIS channel full-scale.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fs_xl_ois in
  *                              reg CTRL3_OIS
  *
  */
int32_t lsm6dsox_aux_xl_full_scale_set(stmdev_ctx_t *ctx,
                                      lsm6dsox_fs_xl_ois_t val)
{
  lsm6dsox_ui_ctrl3_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL3_OIS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.fs_xl_ois = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_CTRL3_OIS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Selects accelerometer OIS channel full-scale.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of fs_xl_ois in reg CTRL3_OIS
  *
  */
int32_t lsm6dsox_aux_xl_full_scale_get(stmdev_ctx_t *ctx,
                                      lsm6dsox_fs_xl_ois_t *val)
{
  lsm6dsox_ui_ctrl3_ois_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL3_OIS, (uint8_t*)&reg, 1);
  switch (reg.fs_xl_ois) {
    case LSM6DSOX_AUX_2g:
      *val = LSM6DSOX_AUX_2g;
      break;
    case LSM6DSOX_AUX_16g:
      *val = LSM6DSOX_AUX_16g;
      break;
    case LSM6DSOX_AUX_4g:
      *val = LSM6DSOX_AUX_4g;
      break;
    case LSM6DSOX_AUX_8g:
      *val = LSM6DSOX_AUX_8g;
      break;
    default:
      *val = LSM6DSOX_AUX_2g;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_ main_serial_interface
  * @brief     This section groups all the functions concerning main
  *            serial interface management (not auxiliary)
  * @{
  *
  */

/**
  * @brief  Connect/Disconnect SDO/SA0 internal pull-up.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sdo_pu_en in
  *                              reg PIN_CTRL
  *
  */
int32_t lsm6dsox_sdo_sa0_mode_set(stmdev_ctx_t *ctx, lsm6dsox_sdo_pu_en_t val)
{
  lsm6dsox_pin_ctrl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PIN_CTRL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.sdo_pu_en = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PIN_CTRL, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Connect/Disconnect SDO/SA0 internal pull-up.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of sdo_pu_en in reg PIN_CTRL
  *
  */
int32_t lsm6dsox_sdo_sa0_mode_get(stmdev_ctx_t *ctx, lsm6dsox_sdo_pu_en_t *val)
{
  lsm6dsox_pin_ctrl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PIN_CTRL, (uint8_t*)&reg, 1);
  switch (reg.sdo_pu_en) {
    case LSM6DSOX_PULL_UP_DISC:
      *val = LSM6DSOX_PULL_UP_DISC;
      break;
    case LSM6DSOX_PULL_UP_CONNECT:
      *val = LSM6DSOX_PULL_UP_CONNECT;
      break;
    default:
      *val = LSM6DSOX_PULL_UP_DISC;
      break;
  }
  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sim in reg CTRL3_C
  *
  */
int32_t lsm6dsox_spi_mode_set(stmdev_ctx_t *ctx, lsm6dsox_sim_t val)
{
  lsm6dsox_ctrl3_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.sim = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of sim in reg CTRL3_C
  *
  */
int32_t lsm6dsox_spi_mode_get(stmdev_ctx_t *ctx, lsm6dsox_sim_t *val)
{
  lsm6dsox_ctrl3_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  switch (reg.sim) {
    case LSM6DSOX_SPI_4_WIRE:
      *val = LSM6DSOX_SPI_4_WIRE;
      break;
    case LSM6DSOX_SPI_3_WIRE:
      *val = LSM6DSOX_SPI_3_WIRE;
      break;
    default:
      *val = LSM6DSOX_SPI_4_WIRE;
      break;
  }
  return ret;
}

/**
  * @brief  Disable / Enable I2C interface.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of i2c_disable in
  *                                reg CTRL4_C
  *
  */
int32_t lsm6dsox_i2c_interface_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_i2c_disable_t val)
{
  lsm6dsox_ctrl4_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.i2c_disable = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Disable / Enable I2C interface.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of i2c_disable in
  *                                reg CTRL4_C
  *
  */
int32_t lsm6dsox_i2c_interface_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_i2c_disable_t *val)
{
  lsm6dsox_ctrl4_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  switch (reg.i2c_disable) {
    case LSM6DSOX_I2C_ENABLE:
      *val = LSM6DSOX_I2C_ENABLE;
      break;
    case LSM6DSOX_I2C_DISABLE:
      *val = LSM6DSOX_I2C_DISABLE;
      break;
    default:
      *val = LSM6DSOX_I2C_ENABLE;
      break;
  }
  return ret;
}

/**
  * @brief  I3C Enable/Disable communication protocol[.set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of i3c_disable
  *                                    in reg CTRL9_XL
  *
  */
int32_t lsm6dsox_i3c_disable_set(stmdev_ctx_t *ctx, lsm6dsox_i3c_disable_t val)
{
  lsm6dsox_i3c_bus_avb_t i3c_bus_avb;
  lsm6dsox_ctrl9_xl_t ctrl9_xl;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&ctrl9_xl, 1);
  if (ret == 0) {
    ctrl9_xl.i3c_disable = ((uint8_t)val & 0x80U) >> 7;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&ctrl9_xl, 1);
  }
  if (ret == 0) {

    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_I3C_BUS_AVB,
                           (uint8_t*)&i3c_bus_avb, 1);
  }
  if (ret == 0) {
    i3c_bus_avb.i3c_bus_avb_sel = (uint8_t)val & 0x03U;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_I3C_BUS_AVB,
                            (uint8_t*)&i3c_bus_avb, 1);
  }

  return ret;
}

/**
  * @brief  I3C Enable/Disable communication protocol.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of i3c_disable in
  *                                reg CTRL9_XL
  *
  */
int32_t lsm6dsox_i3c_disable_get(stmdev_ctx_t *ctx, lsm6dsox_i3c_disable_t *val)
{
  lsm6dsox_ctrl9_xl_t ctrl9_xl;
  lsm6dsox_i3c_bus_avb_t i3c_bus_avb;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&ctrl9_xl, 1);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_I3C_BUS_AVB,
                           (uint8_t*)&i3c_bus_avb, 1);

    switch ((ctrl9_xl.i3c_disable << 7) | i3c_bus_avb.i3c_bus_avb_sel) {
      case LSM6DSOX_I3C_DISABLE:
        *val = LSM6DSOX_I3C_DISABLE;
        break;
      case LSM6DSOX_I3C_ENABLE_T_50us:
        *val = LSM6DSOX_I3C_ENABLE_T_50us;
        break;
      case LSM6DSOX_I3C_ENABLE_T_2us:
        *val = LSM6DSOX_I3C_ENABLE_T_2us;
        break;
      case LSM6DSOX_I3C_ENABLE_T_1ms:
        *val = LSM6DSOX_I3C_ENABLE_T_1ms;
        break;
      case LSM6DSOX_I3C_ENABLE_T_25ms:
        *val = LSM6DSOX_I3C_ENABLE_T_25ms;
        break;
      default:
        *val = LSM6DSOX_I3C_DISABLE;
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
  * @defgroup  LSM6DSOX_interrupt_pins
  * @brief     This section groups all the functions that manage interrup pins
  * @{
  *
  */

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of pp_od in reg CTRL3_C
  *
  */
int32_t lsm6dsox_pin_mode_set(stmdev_ctx_t *ctx, lsm6dsox_pp_od_t val)
{
  lsm6dsox_i3c_bus_avb_t i3c_bus_avb;
  lsm6dsox_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  if (ret == 0) {
    ctrl3_c.pp_od = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_I3C_BUS_AVB,
                            (uint8_t*)&i3c_bus_avb, 1);
  }
  if (ret == 0) {
    i3c_bus_avb.pd_dis_int1 = ( (uint8_t) val & 0x02U ) >> 1;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_I3C_BUS_AVB,
                             (uint8_t*)&i3c_bus_avb, 1);
  }
  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of pp_od in reg CTRL3_C
  *
  */
int32_t lsm6dsox_pin_mode_get(stmdev_ctx_t *ctx, lsm6dsox_pp_od_t *val)
{
  lsm6dsox_i3c_bus_avb_t i3c_bus_avb;
  lsm6dsox_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_I3C_BUS_AVB,
                            (uint8_t*)&i3c_bus_avb, 1);
  }

  switch ( (i3c_bus_avb.pd_dis_int1 << 1) + ctrl3_c.pp_od) {
    case LSM6DSOX_PUSH_PULL:
      *val = LSM6DSOX_PUSH_PULL;
      break;
    case LSM6DSOX_OPEN_DRAIN:
      *val = LSM6DSOX_OPEN_DRAIN;
      break;
    case LSM6DSOX_INT1_NOPULL_DOWN_INT2_PUSH_PULL:
      *val = LSM6DSOX_INT1_NOPULL_DOWN_INT2_PUSH_PULL;
      break;
    case LSM6DSOX_INT1_NOPULL_DOWN_INT2_OPEN_DRAIN:
      *val = LSM6DSOX_INT1_NOPULL_DOWN_INT2_OPEN_DRAIN;
      break;
    default:
      *val = LSM6DSOX_PUSH_PULL;
      break;
  }
  return ret;
}

/**
  * @brief  Interrupt active-high/low.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of h_lactive in reg CTRL3_C
  *
  */
int32_t lsm6dsox_pin_polarity_set(stmdev_ctx_t *ctx, lsm6dsox_h_lactive_t val)
{
  lsm6dsox_ctrl3_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.h_lactive = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);
  }

  return ret;
}

/**
  * @brief  Interrupt active-high/low.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of h_lactive in reg CTRL3_C
  *
  */
int32_t lsm6dsox_pin_polarity_get(stmdev_ctx_t *ctx, lsm6dsox_h_lactive_t *val)
{
  lsm6dsox_ctrl3_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&reg, 1);

  switch (reg.h_lactive) {
    case LSM6DSOX_ACTIVE_HIGH:
      *val = LSM6DSOX_ACTIVE_HIGH;
      break;
    case LSM6DSOX_ACTIVE_LOW:
      *val = LSM6DSOX_ACTIVE_LOW;
      break;
    default:
      *val = LSM6DSOX_ACTIVE_HIGH;
      break;
  }
  return ret;
}

/**
  * @brief  All interrupt signals become available on INT1 pin.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of int2_on_int1 in reg CTRL4_C
  *
  */
int32_t lsm6dsox_all_on_int1_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl4_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.int2_on_int1 = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  }

  return ret;
}

/**
  * @brief  All interrupt signals become available on INT1 pin.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of int2_on_int1 in reg CTRL4_C
  *
  */
int32_t lsm6dsox_all_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl4_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  *val = reg.int2_on_int1;

  return ret;
}

/**
  * @brief  Interrupt notification mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lir in reg TAP_CFG0
  *
  */
int32_t lsm6dsox_int_notification_set(stmdev_ctx_t *ctx, lsm6dsox_lir_t val)
{
  lsm6dsox_tap_cfg0_t tap_cfg0;
  lsm6dsox_page_rw_t page_rw;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*) &tap_cfg0, 1);
  if (ret == 0) {
    tap_cfg0.lir = (uint8_t)val & 0x01U;
    tap_cfg0.int_clr_on_read = (uint8_t)val & 0x01U;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*) &tap_cfg0, 1);
  }
  if (ret == 0) {

    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    page_rw.emb_func_lir = ((uint8_t)val & 0x02U) >> 1;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Interrupt notification mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of lir in reg TAP_CFG0
  *
  */
int32_t lsm6dsox_int_notification_get(stmdev_ctx_t *ctx, lsm6dsox_lir_t *val)
{
  lsm6dsox_tap_cfg0_t tap_cfg0;
  lsm6dsox_page_rw_t page_rw;
  int32_t ret;


  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*) &tap_cfg0, 1);
  if (ret == 0) {

      ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  if (ret == 0) {
    switch ((page_rw.emb_func_lir << 1) | tap_cfg0.lir) {
      case LSM6DSOX_ALL_INT_PULSED:
        *val = LSM6DSOX_ALL_INT_PULSED;
        break;
      case LSM6DSOX_BASE_LATCHED_EMB_PULSED:
        *val = LSM6DSOX_BASE_LATCHED_EMB_PULSED;
        break;
      case LSM6DSOX_BASE_PULSED_EMB_LATCHED:
        *val = LSM6DSOX_BASE_PULSED_EMB_LATCHED;
        break;
      case LSM6DSOX_ALL_INT_LATCHED:
        *val = LSM6DSOX_ALL_INT_LATCHED;
        break;
      default:
        *val = LSM6DSOX_ALL_INT_PULSED;
        break;
    }
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_Wake_Up_event
  * @brief     This section groups all the functions that manage the Wake Up
  *            event generation.
  * @{
  *
  */

/**
  * @brief  Weight of 1 LSB of wakeup threshold.[set]
  *         0: 1 LSB =FS_XL  /  64
  *         1: 1 LSB = FS_XL / 256
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wake_ths_w in
  *                                 reg WAKE_UP_DUR
  *
  */
int32_t lsm6dsox_wkup_ths_weight_set(stmdev_ctx_t *ctx,
                                    lsm6dsox_wake_ths_w_t val)
{
  lsm6dsox_wake_up_dur_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_DUR, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.wake_ths_w = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_WAKE_UP_DUR, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Weight of 1 LSB of wakeup threshold.[get]
  *         0: 1 LSB =FS_XL  /  64
  *         1: 1 LSB = FS_XL / 256
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of wake_ths_w in
  *                                 reg WAKE_UP_DUR
  *
  */
int32_t lsm6dsox_wkup_ths_weight_get(stmdev_ctx_t *ctx,
                                    lsm6dsox_wake_ths_w_t *val)
{
  lsm6dsox_wake_up_dur_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_DUR, (uint8_t*)&reg, 1);

  switch (reg.wake_ths_w) {
    case LSM6DSOX_LSb_FS_DIV_64:
      *val = LSM6DSOX_LSb_FS_DIV_64;
      break;
    case LSM6DSOX_LSb_FS_DIV_256:
      *val = LSM6DSOX_LSb_FS_DIV_256;
      break;
    default:
      *val = LSM6DSOX_LSb_FS_DIV_64;
      break;
  }
  return ret;
}

/**
  * @brief  Threshold for wakeup: 1 LSB weight depends on WAKE_THS_W in
  *         WAKE_UP_DUR.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wk_ths in reg WAKE_UP_THS
  *
  */
int32_t lsm6dsox_wkup_threshold_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_wake_up_ths_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_THS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.wk_ths = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_WAKE_UP_THS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Threshold for wakeup: 1 LSB weight depends on WAKE_THS_W in
  *         WAKE_UP_DUR.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wk_ths in reg WAKE_UP_THS
  *
  */
int32_t lsm6dsox_wkup_threshold_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_wake_up_ths_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_THS, (uint8_t*)&reg, 1);
  *val = reg.wk_ths;

  return ret;
}

/**
  * @brief  Wake up duration event.[set]
  *         1LSb = 1 / ODR
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of usr_off_on_wu in reg WAKE_UP_THS
  *
  */
int32_t lsm6dsox_xl_usr_offset_on_wkup_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_wake_up_ths_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_THS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.usr_off_on_wu = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_WAKE_UP_THS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Wake up duration event.[get]
  *         1LSb = 1 / ODR
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of usr_off_on_wu in reg WAKE_UP_THS
  *
  */
int32_t lsm6dsox_xl_usr_offset_on_wkup_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_wake_up_ths_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_THS, (uint8_t*)&reg, 1);
  *val = reg.usr_off_on_wu;

  return ret;
}

/**
  * @brief  Wake up duration event.[set]
  *         1LSb = 1 / ODR
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wake_dur in reg WAKE_UP_DUR
  *
  */
int32_t lsm6dsox_wkup_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_wake_up_dur_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_DUR, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.wake_dur = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_WAKE_UP_DUR, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Wake up duration event.[get]
  *         1LSb = 1 / ODR
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wake_dur in reg WAKE_UP_DUR
  *
  */
int32_t lsm6dsox_wkup_dur_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_wake_up_dur_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_DUR, (uint8_t*)&reg, 1);
  *val = reg.wake_dur;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_ Activity/Inactivity_detection
  * @brief     This section groups all the functions concerning
  *            activity/inactivity detection.
  * @{
  *
  */

/**
  * @brief  Enables gyroscope Sleep mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sleep_g in reg CTRL4_C
  *
  */
int32_t lsm6dsox_gy_sleep_mode_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl4_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.sleep_g = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enables gyroscope Sleep mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sleep_g in reg CTRL4_C
  *
  */
int32_t lsm6dsox_gy_sleep_mode_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl4_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&reg, 1);
  *val = reg.sleep_g;

  return ret;
}

/**
  * @brief  Drives the sleep status instead of
  *         sleep change on INT pins
  *         (only if INT1_SLEEP_CHANGE or
  *         INT2_SLEEP_CHANGE bits are enabled).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sleep_status_on_int in reg TAP_CFG0
  *
  */
int32_t lsm6dsox_act_pin_notification_set(stmdev_ctx_t *ctx,
                                         lsm6dsox_sleep_status_on_int_t val)
{
  lsm6dsox_tap_cfg0_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.sleep_status_on_int = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Drives the sleep status instead of
  *         sleep change on INT pins (only if
  *         INT1_SLEEP_CHANGE or
  *         INT2_SLEEP_CHANGE bits are enabled).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of sleep_status_on_int in reg TAP_CFG0
  *
  */
int32_t lsm6dsox_act_pin_notification_get(stmdev_ctx_t *ctx,
                                         lsm6dsox_sleep_status_on_int_t *val)
{
  lsm6dsox_tap_cfg0_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  switch (reg.sleep_status_on_int) {
    case LSM6DSOX_DRIVE_SLEEP_CHG_EVENT:
      *val = LSM6DSOX_DRIVE_SLEEP_CHG_EVENT;
      break;
    case LSM6DSOX_DRIVE_SLEEP_STATUS:
      *val = LSM6DSOX_DRIVE_SLEEP_STATUS;
      break;
    default:
      *val = LSM6DSOX_DRIVE_SLEEP_CHG_EVENT;
      break;
  }
  return ret;
}

/**
  * @brief  Enable inactivity function.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of inact_en in reg TAP_CFG2
  *
  */
int32_t lsm6dsox_act_mode_set(stmdev_ctx_t *ctx, lsm6dsox_inact_en_t val)
{
  lsm6dsox_tap_cfg2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG2, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.inact_en = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_CFG2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enable inactivity function.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of inact_en in reg TAP_CFG2
  *
  */
int32_t lsm6dsox_act_mode_get(stmdev_ctx_t *ctx, lsm6dsox_inact_en_t *val)
{
  lsm6dsox_tap_cfg2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG2, (uint8_t*)&reg, 1);
  switch (reg.inact_en) {
    case LSM6DSOX_XL_AND_GY_NOT_AFFECTED:
      *val = LSM6DSOX_XL_AND_GY_NOT_AFFECTED;
      break;
    case LSM6DSOX_XL_12Hz5_GY_NOT_AFFECTED:
      *val = LSM6DSOX_XL_12Hz5_GY_NOT_AFFECTED;
      break;
    case LSM6DSOX_XL_12Hz5_GY_SLEEP:
      *val = LSM6DSOX_XL_12Hz5_GY_SLEEP;
      break;
    case LSM6DSOX_XL_12Hz5_GY_PD:
      *val = LSM6DSOX_XL_12Hz5_GY_PD;
      break;
    default:
      *val = LSM6DSOX_XL_AND_GY_NOT_AFFECTED;
      break;
  }
  return ret;
}

/**
  * @brief  Duration to go in sleep mode.[set]
  *         1 LSb = 512 / ODR
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sleep_dur in reg WAKE_UP_DUR
  *
  */
int32_t lsm6dsox_act_sleep_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_wake_up_dur_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_DUR, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.sleep_dur = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_WAKE_UP_DUR, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Duration to go in sleep mode.[get]
  *         1 LSb = 512 / ODR
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sleep_dur in reg WAKE_UP_DUR
  *
  */
int32_t lsm6dsox_act_sleep_dur_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_wake_up_dur_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_DUR, (uint8_t*)&reg, 1);
  *val = reg.sleep_dur;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_tap_generator
  * @brief     This section groups all the functions that manage the
  *            tap and double tap event generation.
  * @{
  *
  */

/**
  * @brief  Enable Z direction in tap recognition.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_z_en in reg TAP_CFG0
  *
  */
int32_t lsm6dsox_tap_detection_on_z_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_tap_cfg0_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.tap_z_en = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enable Z direction in tap recognition.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_z_en in reg TAP_CFG0
  *
  */
int32_t lsm6dsox_tap_detection_on_z_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_tap_cfg0_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  *val = reg.tap_z_en;

  return ret;
}

/**
  * @brief  Enable Y direction in tap recognition.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_y_en in reg TAP_CFG0
  *
  */
int32_t lsm6dsox_tap_detection_on_y_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_tap_cfg0_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.tap_y_en = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enable Y direction in tap recognition.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_y_en in reg TAP_CFG0
  *
  */
int32_t lsm6dsox_tap_detection_on_y_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_tap_cfg0_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  *val = reg.tap_y_en;

  return ret;
}

/**
  * @brief  Enable X direction in tap recognition.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_x_en in reg TAP_CFG0
  *
  */
int32_t lsm6dsox_tap_detection_on_x_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_tap_cfg0_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.tap_x_en = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enable X direction in tap recognition.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_x_en in reg TAP_CFG0
  *
  */
int32_t lsm6dsox_tap_detection_on_x_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_tap_cfg0_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*)&reg, 1);
  *val = reg.tap_x_en;

  return ret;
}

/**
  * @brief  X-axis tap recognition threshold.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_ths_x in reg TAP_CFG1
  *
  */
int32_t lsm6dsox_tap_threshold_x_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_tap_cfg1_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG1, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.tap_ths_x = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_CFG1, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  X-axis tap recognition threshold.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_ths_x in reg TAP_CFG1
  *
  */
int32_t lsm6dsox_tap_threshold_x_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_tap_cfg1_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG1, (uint8_t*)&reg, 1);
  *val = reg.tap_ths_x;

  return ret;
}

/**
  * @brief  Selection of axis priority for TAP detection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_priority in
  *                                 reg TAP_CFG1
  *
  */
int32_t lsm6dsox_tap_axis_priority_set(stmdev_ctx_t *ctx,
                                      lsm6dsox_tap_priority_t val)
{
  lsm6dsox_tap_cfg1_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG1, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.tap_priority = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_CFG1, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Selection of axis priority for TAP detection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of tap_priority in
  *                                 reg TAP_CFG1
  *
  */
int32_t lsm6dsox_tap_axis_priority_get(stmdev_ctx_t *ctx,
                                      lsm6dsox_tap_priority_t *val)
{
  lsm6dsox_tap_cfg1_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG1, (uint8_t*)&reg, 1);
  switch (reg.tap_priority) {
    case LSM6DSOX_XYZ:
      *val = LSM6DSOX_XYZ;
      break;
    case LSM6DSOX_YXZ:
      *val = LSM6DSOX_YXZ;
      break;
    case LSM6DSOX_XZY:
      *val = LSM6DSOX_XZY;
      break;
    case LSM6DSOX_ZYX:
      *val = LSM6DSOX_ZYX;
      break;
    case LSM6DSOX_YZX:
      *val = LSM6DSOX_YZX;
      break;
    case LSM6DSOX_ZXY:
      *val = LSM6DSOX_ZXY;
      break;
    default:
      *val = LSM6DSOX_XYZ;
      break;
  }
  return ret;
}

/**
  * @brief  Y-axis tap recognition threshold.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_ths_y in reg TAP_CFG2
  *
  */
int32_t lsm6dsox_tap_threshold_y_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_tap_cfg2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG2, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.tap_ths_y = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_CFG2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Y-axis tap recognition threshold.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_ths_y in reg TAP_CFG2
  *
  */
int32_t lsm6dsox_tap_threshold_y_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_tap_cfg2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG2, (uint8_t*)&reg, 1);
  *val = reg.tap_ths_y;

  return ret;
}

/**
  * @brief  Z-axis recognition threshold.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_ths_z in reg TAP_THS_6D
  *
  */
int32_t lsm6dsox_tap_threshold_z_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_tap_ths_6d_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_THS_6D, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.tap_ths_z = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_THS_6D, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Z-axis recognition threshold.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tap_ths_z in reg TAP_THS_6D
  *
  */
int32_t lsm6dsox_tap_threshold_z_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_tap_ths_6d_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_THS_6D, (uint8_t*)&reg, 1);
  *val = reg.tap_ths_z;

  return ret;
}

/**
  * @brief  Maximum duration is the maximum time of an
  *         over threshold signal detection to be recognized
  *         as a tap event. The default value of these bits
  *         is 00b which corresponds to 4*ODR_XL time.
  *         If the SHOCK[1:0] bits are set to a different
  *         value, 1LSB corresponds to 8*ODR_XL time.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of shock in reg INT_DUR2
  *
  */
int32_t lsm6dsox_tap_shock_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_int_dur2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_INT_DUR2, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.shock = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_INT_DUR2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Maximum duration is the maximum time of an
  *         over threshold signal detection to be recognized
  *         as a tap event. The default value of these bits
  *         is 00b which corresponds to 4*ODR_XL time.
  *         If the SHOCK[1:0] bits are set to a different
  *         value, 1LSB corresponds to 8*ODR_XL time.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of shock in reg INT_DUR2
  *
  */
int32_t lsm6dsox_tap_shock_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_int_dur2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_INT_DUR2, (uint8_t*)&reg, 1);
  *val = reg.shock;

  return ret;
}

/**
  * @brief   Quiet time is the time after the first detected
  *          tap in which there must not be any over threshold
  *          event.
  *          The default value of these bits is 00b which
  *          corresponds to 2*ODR_XL time. If the QUIET[1:0]
  *          bits are set to a different value,
  *          1LSB corresponds to 4*ODR_XL time.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of quiet in reg INT_DUR2
  *
  */
int32_t lsm6dsox_tap_quiet_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_int_dur2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_INT_DUR2, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.quiet = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_INT_DUR2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Quiet time is the time after the first detected
  *         tap in which there must not be any over threshold
  *         event.
  *         The default value of these bits is 00b which
  *         corresponds to 2*ODR_XL time.
  *         If the QUIET[1:0] bits are set to a different
  *         value, 1LSB corresponds to 4*ODR_XL time.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of quiet in reg INT_DUR2
  *
  */
int32_t lsm6dsox_tap_quiet_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_int_dur2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_INT_DUR2, (uint8_t*)&reg, 1);
  *val = reg.quiet;

  return ret;
}

/**
  * @brief  When double tap recognition is enabled,
  *         this register expresses the maximum time
  *         between two consecutive detected taps to
  *         determine a double tap event.
  *         The default value of these bits is 0000b which
  *         corresponds to 16*ODR_XL time.
  *         If the DUR[3:0] bits are set to a different value,
  *         1LSB corresponds to 32*ODR_XL time.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of dur in reg INT_DUR2
  *
  */
int32_t lsm6dsox_tap_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_int_dur2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_INT_DUR2, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.dur = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_INT_DUR2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  When double tap recognition is enabled,
  *         this register expresses the maximum time
  *         between two consecutive detected taps to
  *         determine a double tap event.
  *         The default value of these bits is 0000b which
  *         corresponds to 16*ODR_XL time. If the DUR[3:0]
  *         bits are set to a different value,
  *         1LSB corresponds to 32*ODR_XL time.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of dur in reg INT_DUR2
  *
  */
int32_t lsm6dsox_tap_dur_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_int_dur2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_INT_DUR2, (uint8_t*)&reg, 1);
  *val = reg.dur;

  return ret;
}

/**
  * @brief  Single/double-tap event enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of single_double_tap in reg WAKE_UP_THS
  *
  */
int32_t lsm6dsox_tap_mode_set(stmdev_ctx_t *ctx,
                             lsm6dsox_single_double_tap_t val)
{
  lsm6dsox_wake_up_ths_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_THS, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.single_double_tap = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_WAKE_UP_THS, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Single/double-tap event enable.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of single_double_tap in reg WAKE_UP_THS
  *
  */
int32_t lsm6dsox_tap_mode_get(stmdev_ctx_t *ctx,
                             lsm6dsox_single_double_tap_t *val)
{
  lsm6dsox_wake_up_ths_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_THS, (uint8_t*)&reg, 1);

  switch (reg.single_double_tap) {
    case LSM6DSOX_ONLY_SINGLE:
      *val = LSM6DSOX_ONLY_SINGLE;
      break;
    case LSM6DSOX_BOTH_SINGLE_DOUBLE:
      *val = LSM6DSOX_BOTH_SINGLE_DOUBLE;
      break;
    default:
      *val = LSM6DSOX_ONLY_SINGLE;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_ Six_position_detection(6D/4D)
  * @brief   This section groups all the functions concerning six position
  *          detection (6D).
  * @{
  *
  */

/**
  * @brief  Threshold for 4D/6D function.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sixd_ths in reg TAP_THS_6D
  *
  */
int32_t lsm6dsox_6d_threshold_set(stmdev_ctx_t *ctx, lsm6dsox_sixd_ths_t val)
{
  lsm6dsox_tap_ths_6d_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_THS_6D, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.sixd_ths = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_THS_6D, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Threshold for 4D/6D function.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of sixd_ths in reg TAP_THS_6D
  *
  */
int32_t lsm6dsox_6d_threshold_get(stmdev_ctx_t *ctx, lsm6dsox_sixd_ths_t *val)
{
  lsm6dsox_tap_ths_6d_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_THS_6D, (uint8_t*)&reg, 1);
  switch (reg.sixd_ths) {
    case LSM6DSOX_DEG_80:
      *val = LSM6DSOX_DEG_80;
      break;
    case LSM6DSOX_DEG_70:
      *val = LSM6DSOX_DEG_70;
      break;
    case LSM6DSOX_DEG_60:
      *val = LSM6DSOX_DEG_60;
      break;
    case LSM6DSOX_DEG_50:
      *val = LSM6DSOX_DEG_50;
      break;
    default:
      *val = LSM6DSOX_DEG_80;
      break;
  }
  return ret;
}

/**
  * @brief  4D orientation detection enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d4d_en in reg TAP_THS_6D
  *
  */
int32_t lsm6dsox_4d_mode_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_tap_ths_6d_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_THS_6D, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.d4d_en = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_THS_6D, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  4D orientation detection enable.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d4d_en in reg TAP_THS_6D
  *
  */
int32_t lsm6dsox_4d_mode_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_tap_ths_6d_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_THS_6D, (uint8_t*)&reg, 1);
  *val = reg.d4d_en;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_free_fall
  * @brief   This section group all the functions concerning the free
  *          fall detection.
  * @{
  *
*/
/**
  * @brief  Free fall threshold setting.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ff_ths in reg FREE_FALL
  *
  */
int32_t lsm6dsox_ff_threshold_set(stmdev_ctx_t *ctx, lsm6dsox_ff_ths_t val)
{
  lsm6dsox_free_fall_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FREE_FALL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.ff_ths = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FREE_FALL, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Free fall threshold setting.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of ff_ths in reg FREE_FALL
  *
  */
int32_t lsm6dsox_ff_threshold_get(stmdev_ctx_t *ctx, lsm6dsox_ff_ths_t *val)
{
  lsm6dsox_free_fall_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FREE_FALL, (uint8_t*)&reg, 1);
  switch (reg.ff_ths) {
    case LSM6DSOX_FF_TSH_156mg:
      *val = LSM6DSOX_FF_TSH_156mg;
      break;
    case LSM6DSOX_FF_TSH_219mg:
      *val = LSM6DSOX_FF_TSH_219mg;
      break;
    case LSM6DSOX_FF_TSH_250mg:
      *val = LSM6DSOX_FF_TSH_250mg;
      break;
    case LSM6DSOX_FF_TSH_312mg:
      *val = LSM6DSOX_FF_TSH_312mg;
      break;
    case LSM6DSOX_FF_TSH_344mg:
      *val = LSM6DSOX_FF_TSH_344mg;
      break;
    case LSM6DSOX_FF_TSH_406mg:
      *val = LSM6DSOX_FF_TSH_406mg;
      break;
    case LSM6DSOX_FF_TSH_469mg:
      *val = LSM6DSOX_FF_TSH_469mg;
      break;
    case LSM6DSOX_FF_TSH_500mg:
      *val = LSM6DSOX_FF_TSH_500mg;
      break;
    default:
      *val = LSM6DSOX_FF_TSH_156mg;
      break;
  }
  return ret;
}

/**
  * @brief  Free-fall duration event.[set]
  *         1LSb = 1 / ODR
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ff_dur in reg FREE_FALL
  *
  */
int32_t lsm6dsox_ff_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_wake_up_dur_t wake_up_dur;
  lsm6dsox_free_fall_t free_fall;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_DUR, (uint8_t*)&wake_up_dur, 1);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FREE_FALL, (uint8_t*)&free_fall, 1);
  }
  if (ret == 0) {
    wake_up_dur.ff_dur = ((uint8_t)val & 0x20U) >> 5;
    free_fall.ff_dur = (uint8_t)val & 0x1FU;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_WAKE_UP_DUR,
                            (uint8_t*)&wake_up_dur, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FREE_FALL, (uint8_t*)&free_fall, 1);
  }
  return ret;
}

/**
  * @brief  Free-fall duration event.[get]
  *         1LSb = 1 / ODR
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ff_dur in reg FREE_FALL
  *
  */
int32_t lsm6dsox_ff_dur_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_wake_up_dur_t wake_up_dur;
  lsm6dsox_free_fall_t free_fall;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WAKE_UP_DUR, (uint8_t*)&wake_up_dur, 1);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FREE_FALL, (uint8_t*)&free_fall, 1);
    *val = (wake_up_dur.ff_dur << 5) + free_fall.ff_dur;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_fifo
  * @brief   This section group all the functions concerning the fifo usage
  * @{
  *
  */

/**
  * @brief  FIFO watermark level selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wtm in reg FIFO_CTRL1
  *
  */
int32_t lsm6dsox_fifo_watermark_set(stmdev_ctx_t *ctx, uint16_t val)
{
  lsm6dsox_fifo_ctrl1_t fifo_ctrl1;
  lsm6dsox_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL2, (uint8_t*)&fifo_ctrl2, 1);
  if (ret == 0) {
    fifo_ctrl1.wtm = 0x00FFU & (uint8_t)val;
    fifo_ctrl2.wtm = (uint8_t)(( 0x0100U & val ) >> 8);
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FIFO_CTRL1, (uint8_t*)&fifo_ctrl1, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FIFO_CTRL2, (uint8_t*)&fifo_ctrl2, 1);
  }
  return ret;
}

/**
  * @brief  FIFO watermark level selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wtm in reg FIFO_CTRL1
  *
  */
int32_t lsm6dsox_fifo_watermark_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  lsm6dsox_fifo_ctrl1_t fifo_ctrl1;
  lsm6dsox_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL1, (uint8_t*)&fifo_ctrl1, 1);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL2, (uint8_t*)&fifo_ctrl2, 1);
    *val = ((uint16_t)fifo_ctrl2.wtm << 8) + (uint16_t)fifo_ctrl1.wtm;
  }
  return ret;
}

/**
  * @brief  FIFO compression feature initialization request [set].
  *
  * @param  ctx       read / write interface definitions
  * @param  val       change the values of FIFO_COMPR_INIT in
  *                   reg EMB_FUNC_INIT_B
  *
  */
int32_t lsm6dsox_compression_algo_init_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_emb_func_init_b_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_INIT_B, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.fifo_compr_init = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_INIT_B, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  FIFO compression feature initialization request [get].
  *
  * @param  ctx    read / write interface definitions
  * @param  val    change the values of FIFO_COMPR_INIT in
  *                reg EMB_FUNC_INIT_B
  *
  */
int32_t lsm6dsox_compression_algo_init_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_emb_func_init_b_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_INIT_B, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    *val = reg.fifo_compr_init;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Enable and configure compression algo.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of uncoptr_rate in
  *                  reg FIFO_CTRL2
  *
  */
int32_t lsm6dsox_compression_algo_set(stmdev_ctx_t *ctx,
                                     lsm6dsox_uncoptr_rate_t val)
{
  lsm6dsox_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL2,
                          (uint8_t*)&fifo_ctrl2, 1);
  if (ret == 0) {
    fifo_ctrl2.fifo_compr_rt_en = ((uint8_t)val & 0x04U) >> 2;
    fifo_ctrl2.uncoptr_rate = (uint8_t)val & 0x03U;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FIFO_CTRL2,
                            (uint8_t*)&fifo_ctrl2, 1);
  }
  return ret;
}

/**
  * @brief  Enable and configure compression algo.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of uncoptr_rate in
  *                  reg FIFO_CTRL2
  *
  */
int32_t lsm6dsox_compression_algo_get(stmdev_ctx_t *ctx,
                                     lsm6dsox_uncoptr_rate_t *val)
{
  lsm6dsox_fifo_ctrl2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL2, (uint8_t*)&reg, 1);

  switch ((reg.fifo_compr_rt_en<<2) | reg.uncoptr_rate) {
    case LSM6DSOX_CMP_DISABLE:
      *val = LSM6DSOX_CMP_DISABLE;
      break;
    case LSM6DSOX_CMP_ALWAYS:
      *val = LSM6DSOX_CMP_ALWAYS;
      break;
    case LSM6DSOX_CMP_8_TO_1:
      *val = LSM6DSOX_CMP_8_TO_1;
      break;
    case LSM6DSOX_CMP_16_TO_1:
      *val = LSM6DSOX_CMP_16_TO_1;
      break;
    case LSM6DSOX_CMP_32_TO_1:
      *val = LSM6DSOX_CMP_32_TO_1;
      break;
    default:
      *val = LSM6DSOX_CMP_DISABLE;
      break;
  }
  return ret;
}

/**
  * @brief  Enables ODR CHANGE virtual sensor to be batched in FIFO.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of odrchg_en in reg FIFO_CTRL2
  *
  */
int32_t lsm6dsox_fifo_virtual_sens_odr_chg_set(stmdev_ctx_t *ctx,
                                              uint8_t val)
{
  lsm6dsox_fifo_ctrl2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL2, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.odrchg_en = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FIFO_CTRL2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Enables ODR CHANGE virtual sensor to be batched in FIFO.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of odrchg_en in reg FIFO_CTRL2
  *
  */
int32_t lsm6dsox_fifo_virtual_sens_odr_chg_get(stmdev_ctx_t *ctx,
                                              uint8_t *val)
{
  lsm6dsox_fifo_ctrl2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL2, (uint8_t*)&reg, 1);
  *val = reg.odrchg_en;

  return ret;
}

/**
  * @brief  Enables/Disables compression algorithm runtime.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fifo_compr_rt_en in
  *                  reg FIFO_CTRL2
  *
  */
int32_t lsm6dsox_compression_algo_real_time_set(stmdev_ctx_t *ctx,
                                               uint8_t val)
{
  lsm6dsox_fifo_ctrl2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL2, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.fifo_compr_rt_en = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FIFO_CTRL2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief   Enables/Disables compression algorithm runtime. [get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fifo_compr_rt_en in reg FIFO_CTRL2
  *
  */
int32_t lsm6dsox_compression_algo_real_time_get(stmdev_ctx_t *ctx,
                                               uint8_t *val)
{
  lsm6dsox_fifo_ctrl2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL2, (uint8_t*)&reg, 1);
  *val = reg.fifo_compr_rt_en;

  return ret;
}

/**
  * @brief  Sensing chain FIFO stop values memorization at
  *         threshold level.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of stop_on_wtm in reg FIFO_CTRL2
  *
  */
int32_t lsm6dsox_fifo_stop_on_wtm_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_fifo_ctrl2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL2, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.stop_on_wtm = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FIFO_CTRL2, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Sensing chain FIFO stop values memorization at
  *         threshold level.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of stop_on_wtm in reg FIFO_CTRL2
  *
  */
int32_t lsm6dsox_fifo_stop_on_wtm_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_fifo_ctrl2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL2, (uint8_t*)&reg, 1);
  *val = reg.stop_on_wtm;

  return ret;
}

/**
  * @brief  Selects Batching Data Rate (writing frequency in FIFO)
  *         for accelerometer data.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of bdr_xl in reg FIFO_CTRL3
  *
  */
int32_t lsm6dsox_fifo_xl_batch_set(stmdev_ctx_t *ctx, lsm6dsox_bdr_xl_t val)
{
  lsm6dsox_fifo_ctrl3_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL3, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.bdr_xl = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FIFO_CTRL3, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Selects Batching Data Rate (writing frequency in FIFO)
  *         for accelerometer data.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of bdr_xl in reg FIFO_CTRL3
  *
  */
int32_t lsm6dsox_fifo_xl_batch_get(stmdev_ctx_t *ctx, lsm6dsox_bdr_xl_t *val)
{
  lsm6dsox_fifo_ctrl3_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL3, (uint8_t*)&reg, 1);
  switch (reg.bdr_xl) {
    case LSM6DSOX_XL_NOT_BATCHED:
      *val = LSM6DSOX_XL_NOT_BATCHED;
      break;
    case LSM6DSOX_XL_BATCHED_AT_12Hz5:
      *val = LSM6DSOX_XL_BATCHED_AT_12Hz5;
      break;
    case LSM6DSOX_XL_BATCHED_AT_26Hz:
      *val = LSM6DSOX_XL_BATCHED_AT_26Hz;
      break;
    case LSM6DSOX_XL_BATCHED_AT_52Hz:
      *val = LSM6DSOX_XL_BATCHED_AT_52Hz;
      break;
    case LSM6DSOX_XL_BATCHED_AT_104Hz:
      *val = LSM6DSOX_XL_BATCHED_AT_104Hz;
      break;
    case LSM6DSOX_XL_BATCHED_AT_208Hz:
      *val = LSM6DSOX_XL_BATCHED_AT_208Hz;
      break;
    case LSM6DSOX_XL_BATCHED_AT_417Hz:
      *val = LSM6DSOX_XL_BATCHED_AT_417Hz;
      break;
    case LSM6DSOX_XL_BATCHED_AT_833Hz:
      *val = LSM6DSOX_XL_BATCHED_AT_833Hz;
      break;
    case LSM6DSOX_XL_BATCHED_AT_1667Hz:
      *val = LSM6DSOX_XL_BATCHED_AT_1667Hz;
      break;
    case LSM6DSOX_XL_BATCHED_AT_3333Hz:
      *val = LSM6DSOX_XL_BATCHED_AT_3333Hz;
      break;
    case LSM6DSOX_XL_BATCHED_AT_6667Hz:
      *val = LSM6DSOX_XL_BATCHED_AT_6667Hz;
      break;
    case LSM6DSOX_XL_BATCHED_AT_6Hz5:
      *val = LSM6DSOX_XL_BATCHED_AT_6Hz5;
      break;
    default:
      *val = LSM6DSOX_XL_NOT_BATCHED;
      break;
  }

  return ret;
}

/**
  * @brief  Selects Batching Data Rate (writing frequency in FIFO)
  *         for gyroscope data.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of bdr_gy in reg FIFO_CTRL3
  *
  */
int32_t lsm6dsox_fifo_gy_batch_set(stmdev_ctx_t *ctx, lsm6dsox_bdr_gy_t val)
{
  lsm6dsox_fifo_ctrl3_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL3, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.bdr_gy = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FIFO_CTRL3, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Selects Batching Data Rate (writing frequency in FIFO)
  *         for gyroscope data.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of bdr_gy in reg FIFO_CTRL3
  *
  */
int32_t lsm6dsox_fifo_gy_batch_get(stmdev_ctx_t *ctx, lsm6dsox_bdr_gy_t *val)
{
  lsm6dsox_fifo_ctrl3_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL3, (uint8_t*)&reg, 1);
  switch (reg.bdr_gy) {
    case LSM6DSOX_GY_NOT_BATCHED:
      *val = LSM6DSOX_GY_NOT_BATCHED;
      break;
    case LSM6DSOX_GY_BATCHED_AT_12Hz5:
      *val = LSM6DSOX_GY_BATCHED_AT_12Hz5;
      break;
    case LSM6DSOX_GY_BATCHED_AT_26Hz:
      *val = LSM6DSOX_GY_BATCHED_AT_26Hz;
      break;
    case LSM6DSOX_GY_BATCHED_AT_52Hz:
      *val = LSM6DSOX_GY_BATCHED_AT_52Hz;
      break;
    case LSM6DSOX_GY_BATCHED_AT_104Hz:
      *val = LSM6DSOX_GY_BATCHED_AT_104Hz;
      break;
    case LSM6DSOX_GY_BATCHED_AT_208Hz:
      *val = LSM6DSOX_GY_BATCHED_AT_208Hz;
      break;
    case LSM6DSOX_GY_BATCHED_AT_417Hz:
      *val = LSM6DSOX_GY_BATCHED_AT_417Hz;
      break;
    case LSM6DSOX_GY_BATCHED_AT_833Hz:
      *val = LSM6DSOX_GY_BATCHED_AT_833Hz;
      break;
    case LSM6DSOX_GY_BATCHED_AT_1667Hz:
      *val = LSM6DSOX_GY_BATCHED_AT_1667Hz;
      break;
    case LSM6DSOX_GY_BATCHED_AT_3333Hz:
      *val = LSM6DSOX_GY_BATCHED_AT_3333Hz;
      break;
    case LSM6DSOX_GY_BATCHED_AT_6667Hz:
      *val = LSM6DSOX_GY_BATCHED_AT_6667Hz;
      break;
    case LSM6DSOX_GY_BATCHED_AT_6Hz5:
      *val = LSM6DSOX_GY_BATCHED_AT_6Hz5;
      break;
    default:
      *val = LSM6DSOX_GY_NOT_BATCHED;
      break;
  }
  return ret;
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fifo_mode in reg FIFO_CTRL4
  *
  */
int32_t lsm6dsox_fifo_mode_set(stmdev_ctx_t *ctx, lsm6dsox_fifo_mode_t val)
{
  lsm6dsox_fifo_ctrl4_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL4, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.fifo_mode = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FIFO_CTRL4, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of fifo_mode in reg FIFO_CTRL4
  *
  */
int32_t lsm6dsox_fifo_mode_get(stmdev_ctx_t *ctx, lsm6dsox_fifo_mode_t *val)
{
  lsm6dsox_fifo_ctrl4_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL4, (uint8_t*)&reg, 1);

  switch (reg.fifo_mode) {
    case LSM6DSOX_BYPASS_MODE:
      *val = LSM6DSOX_BYPASS_MODE;
      break;
    case LSM6DSOX_FIFO_MODE:
      *val = LSM6DSOX_FIFO_MODE;
      break;
    case LSM6DSOX_STREAM_TO_FIFO_MODE:
      *val = LSM6DSOX_STREAM_TO_FIFO_MODE;
      break;
    case LSM6DSOX_BYPASS_TO_STREAM_MODE:
      *val = LSM6DSOX_BYPASS_TO_STREAM_MODE;
      break;
    case LSM6DSOX_STREAM_MODE:
      *val = LSM6DSOX_STREAM_MODE;
      break;
    case LSM6DSOX_BYPASS_TO_FIFO_MODE:
      *val = LSM6DSOX_BYPASS_TO_FIFO_MODE;
      break;
    default:
      *val = LSM6DSOX_BYPASS_MODE;
      break;
  }
  return ret;
}

/**
  * @brief  Selects Batching Data Rate (writing frequency in FIFO)
  *         for temperature data.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of odr_t_batch in reg FIFO_CTRL4
  *
  */
int32_t lsm6dsox_fifo_temp_batch_set(stmdev_ctx_t *ctx,
                                    lsm6dsox_odr_t_batch_t val)
{
  lsm6dsox_fifo_ctrl4_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL4, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.odr_t_batch = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FIFO_CTRL4, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Selects Batching Data Rate (writing frequency in FIFO)
  *         for temperature data.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of odr_t_batch in reg FIFO_CTRL4
  *
  */
int32_t lsm6dsox_fifo_temp_batch_get(stmdev_ctx_t *ctx,
                                    lsm6dsox_odr_t_batch_t *val)
{
  lsm6dsox_fifo_ctrl4_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL4, (uint8_t*)&reg, 1);

  switch (reg.odr_t_batch) {
    case LSM6DSOX_TEMP_NOT_BATCHED:
      *val = LSM6DSOX_TEMP_NOT_BATCHED;
      break;
    case LSM6DSOX_TEMP_BATCHED_AT_1Hz6:
      *val = LSM6DSOX_TEMP_BATCHED_AT_1Hz6;
      break;
    case LSM6DSOX_TEMP_BATCHED_AT_12Hz5:
      *val = LSM6DSOX_TEMP_BATCHED_AT_12Hz5;
      break;
    case LSM6DSOX_TEMP_BATCHED_AT_52Hz:
      *val = LSM6DSOX_TEMP_BATCHED_AT_52Hz;
      break;
    default:
      *val = LSM6DSOX_TEMP_NOT_BATCHED;
      break;
  }
  return ret;
}

/**
  * @brief  Selects decimation for timestamp batching in FIFO.
  *         Writing rate will be the maximum rate between XL and
  *         GYRO BDR divided by decimation decoder.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of odr_ts_batch in reg FIFO_CTRL4
  *
  */
int32_t lsm6dsox_fifo_timestamp_decimation_set(stmdev_ctx_t *ctx,
                                              lsm6dsox_odr_ts_batch_t val)
{
  lsm6dsox_fifo_ctrl4_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL4, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.odr_ts_batch = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FIFO_CTRL4, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief   Selects decimation for timestamp batching in FIFO.
  *          Writing rate will be the maximum rate between XL and
  *          GYRO BDR divided by decimation decoder.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of odr_ts_batch in reg FIFO_CTRL4
  *
  */
int32_t lsm6dsox_fifo_timestamp_decimation_get(stmdev_ctx_t *ctx,
                                              lsm6dsox_odr_ts_batch_t *val)
{
  lsm6dsox_fifo_ctrl4_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_CTRL4, (uint8_t*)&reg, 1);
  switch (reg.odr_ts_batch) {
    case LSM6DSOX_NO_DECIMATION:
      *val = LSM6DSOX_NO_DECIMATION;
      break;
    case LSM6DSOX_DEC_1:
      *val = LSM6DSOX_DEC_1;
      break;
    case LSM6DSOX_DEC_8:
      *val = LSM6DSOX_DEC_8;
      break;
    case LSM6DSOX_DEC_32:
      *val = LSM6DSOX_DEC_32;
      break;
    default:
      *val = LSM6DSOX_NO_DECIMATION;
      break;
  }
  return ret;
}

/**
  * @brief  Selects the trigger for the internal counter of batching events
  *         between XL and gyro.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of trig_counter_bdr
  *                  in reg COUNTER_BDR_REG1
  *
  */
int32_t lsm6dsox_fifo_cnt_event_batch_set(stmdev_ctx_t *ctx,
                                         lsm6dsox_trig_counter_bdr_t val)
{
  lsm6dsox_counter_bdr_reg1_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_COUNTER_BDR_REG1, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.trig_counter_bdr = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_COUNTER_BDR_REG1, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Selects the trigger for the internal counter of batching events
  *         between XL and gyro.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of trig_counter_bdr
  *                                     in reg COUNTER_BDR_REG1
  *
  */
int32_t lsm6dsox_fifo_cnt_event_batch_get(stmdev_ctx_t *ctx,
                                         lsm6dsox_trig_counter_bdr_t *val)
{
  lsm6dsox_counter_bdr_reg1_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_COUNTER_BDR_REG1, (uint8_t*)&reg, 1);
  switch (reg.trig_counter_bdr) {
    case LSM6DSOX_XL_BATCH_EVENT:
      *val = LSM6DSOX_XL_BATCH_EVENT;
      break;
    case LSM6DSOX_GYRO_BATCH_EVENT:
      *val = LSM6DSOX_GYRO_BATCH_EVENT;
      break;
    default:
      *val = LSM6DSOX_XL_BATCH_EVENT;
      break;
  }
  return ret;
}

/**
  * @brief  Resets the internal counter of batching vents for a single sensor.
  *         This bit is automatically reset to zero if it was set to ‘1’.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of rst_counter_bdr in
  *                      reg COUNTER_BDR_REG1
  *
  */
int32_t lsm6dsox_rst_batch_counter_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_counter_bdr_reg1_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_COUNTER_BDR_REG1, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.rst_counter_bdr = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_COUNTER_BDR_REG1, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  Resets the internal counter of batching events for a single sensor.
  *         This bit is automatically reset to zero if it was set to ‘1’.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of rst_counter_bdr in
  *                  reg COUNTER_BDR_REG1
  *
  */
int32_t lsm6dsox_rst_batch_counter_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_counter_bdr_reg1_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_COUNTER_BDR_REG1, (uint8_t*)&reg, 1);
  *val = reg.rst_counter_bdr;

  return ret;
}

/**
  * @brief  Batch data rate counter.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of cnt_bdr_th in
  *                  reg COUNTER_BDR_REG2 and COUNTER_BDR_REG1.
  *
  */
int32_t lsm6dsox_batch_counter_threshold_set(stmdev_ctx_t *ctx, uint16_t val)
{
  lsm6dsox_counter_bdr_reg1_t counter_bdr_reg1;
  lsm6dsox_counter_bdr_reg2_t counter_bdr_reg2;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_COUNTER_BDR_REG1,
                         (uint8_t*)&counter_bdr_reg1, 1);
  if (ret == 0) {
    counter_bdr_reg2.cnt_bdr_th =  0x00FFU & (uint8_t)val;
    counter_bdr_reg1.cnt_bdr_th = (uint8_t)(0x0700U & val) >> 8;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_COUNTER_BDR_REG1,
                            (uint8_t*)&counter_bdr_reg1, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_COUNTER_BDR_REG2,
                            (uint8_t*)&counter_bdr_reg2, 1);
  }
  return ret;
}

/**
  * @brief  Batch data rate counter.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of cnt_bdr_th in
  *                  reg COUNTER_BDR_REG2 and COUNTER_BDR_REG1.
  *
  */
int32_t lsm6dsox_batch_counter_threshold_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  lsm6dsox_counter_bdr_reg1_t counter_bdr_reg1;
  lsm6dsox_counter_bdr_reg2_t counter_bdr_reg2;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_COUNTER_BDR_REG1,
                         (uint8_t*)&counter_bdr_reg1, 1);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_COUNTER_BDR_REG2,
                           (uint8_t*)&counter_bdr_reg2, 1);

    *val = ((uint16_t)counter_bdr_reg1.cnt_bdr_th << 8)
    + (uint16_t)counter_bdr_reg2.cnt_bdr_th;
  }

  return ret;
}

/**
  * @brief  Number of unread sensor data(TAG + 6 bytes) stored in FIFO.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of diff_fifo in reg FIFO_STATUS1
  *
  */
int32_t lsm6dsox_fifo_data_level_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  lsm6dsox_fifo_status1_t fifo_status1;
  lsm6dsox_fifo_status2_t fifo_status2;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_STATUS1,
                         (uint8_t*)&fifo_status1, 1);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_STATUS2,
                           (uint8_t*)&fifo_status2, 1);
    *val = ((uint16_t)fifo_status2.diff_fifo << 8) +
            (uint16_t)fifo_status1.diff_fifo;
  }
  return ret;
}

/**
  * @brief  FIFO status.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers FIFO_STATUS2
  *
  */
int32_t lsm6dsox_fifo_status_get(stmdev_ctx_t *ctx,
                                lsm6dsox_fifo_status2_t *val)
{
  int32_t ret;
  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_STATUS2, (uint8_t*) val, 1);
  return ret;
}

/**
  * @brief  Smart FIFO full status.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fifo_full_ia in reg FIFO_STATUS2
  *
  */
int32_t lsm6dsox_fifo_full_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_fifo_status2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_STATUS2, (uint8_t*)&reg, 1);
  *val = reg.fifo_full_ia;

  return ret;
}

/**
  * @brief  FIFO overrun status.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of  fifo_over_run_latched in
  *                  reg FIFO_STATUS2
  *
  */
int32_t lsm6dsox_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_fifo_status2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_STATUS2, (uint8_t*)&reg, 1);
  *val = reg.fifo_ovr_ia;

  return ret;
}

/**
  * @brief  FIFO watermark status.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fifo_wtm_ia in reg FIFO_STATUS2
  *
  */
int32_t lsm6dsox_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_fifo_status2_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_STATUS2, (uint8_t*)&reg, 1);
  *val = reg.fifo_wtm_ia;

  return ret;
}

/**
  * @brief  Identifies the sensor in FIFO_DATA_OUT.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tag_sensor in reg FIFO_DATA_OUT_TAG
  *
  */
int32_t lsm6dsox_fifo_sensor_tag_get(stmdev_ctx_t *ctx,
                                    lsm6dsox_fifo_tag_t *val)
{
  lsm6dsox_fifo_data_out_tag_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FIFO_DATA_OUT_TAG, (uint8_t*)&reg, 1);
  switch (reg.tag_sensor) {
    case LSM6DSOX_GYRO_NC_TAG:
      *val = LSM6DSOX_GYRO_NC_TAG;
      break;
    case LSM6DSOX_XL_NC_TAG:
      *val = LSM6DSOX_XL_NC_TAG;
      break;
    case LSM6DSOX_TEMPERATURE_TAG:
      *val = LSM6DSOX_TEMPERATURE_TAG;
      break;
    case LSM6DSOX_TIMESTAMP_TAG:
      *val = LSM6DSOX_TIMESTAMP_TAG;
      break;
    case LSM6DSOX_CFG_CHANGE_TAG:
      *val = LSM6DSOX_CFG_CHANGE_TAG;
      break;
    case LSM6DSOX_XL_NC_T_2_TAG:
      *val = LSM6DSOX_XL_NC_T_2_TAG;
      break;
    case LSM6DSOX_XL_NC_T_1_TAG:
      *val = LSM6DSOX_XL_NC_T_1_TAG;
      break;
    case LSM6DSOX_XL_2XC_TAG:
      *val = LSM6DSOX_XL_2XC_TAG;
      break;
    case LSM6DSOX_XL_3XC_TAG:
      *val = LSM6DSOX_XL_3XC_TAG;
      break;
    case LSM6DSOX_GYRO_NC_T_2_TAG:
      *val = LSM6DSOX_GYRO_NC_T_2_TAG;
      break;
    case LSM6DSOX_GYRO_NC_T_1_TAG:
      *val = LSM6DSOX_GYRO_NC_T_1_TAG;
      break;
    case LSM6DSOX_GYRO_2XC_TAG:
      *val = LSM6DSOX_GYRO_2XC_TAG;
      break;
    case LSM6DSOX_GYRO_3XC_TAG:
      *val = LSM6DSOX_GYRO_3XC_TAG;
      break;
    case LSM6DSOX_SENSORHUB_SLAVE0_TAG:
      *val = LSM6DSOX_SENSORHUB_SLAVE0_TAG;
      break;
    case LSM6DSOX_SENSORHUB_SLAVE1_TAG:
      *val = LSM6DSOX_SENSORHUB_SLAVE1_TAG;
      break;
    case LSM6DSOX_SENSORHUB_SLAVE2_TAG:
      *val = LSM6DSOX_SENSORHUB_SLAVE2_TAG;
      break;
    case LSM6DSOX_SENSORHUB_SLAVE3_TAG:
      *val = LSM6DSOX_SENSORHUB_SLAVE3_TAG;
      break;
    case LSM6DSOX_STEP_CPUNTER_TAG:
      *val = LSM6DSOX_STEP_CPUNTER_TAG;
      break;
    case LSM6DSOX_GAME_ROTATION_TAG:
      *val = LSM6DSOX_GAME_ROTATION_TAG;
      break;
    case LSM6DSOX_GEOMAG_ROTATION_TAG:
      *val = LSM6DSOX_GEOMAG_ROTATION_TAG;
      break;
    case LSM6DSOX_ROTATION_TAG:
      *val = LSM6DSOX_ROTATION_TAG;
      break;
    case LSM6DSOX_SENSORHUB_NACK_TAG:
      *val = LSM6DSOX_SENSORHUB_NACK_TAG;
      break;
    default:
      *val = LSM6DSOX_GYRO_NC_TAG;
      break;
  }
  return ret;
}

/**
  * @brief  :  Enable FIFO batching of pedometer embedded
  *            function values.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of gbias_fifo_en in
  *                  reg LSM6DSOX_EMB_FUNC_FIFO_CFG
  *
  */
int32_t lsm6dsox_fifo_pedo_batch_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_emb_func_fifo_cfg_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_FIFO_CFG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.pedo_fifo_en = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_FIFO_CFG,
                            (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Enable FIFO batching of pedometer embedded function values.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of pedo_fifo_en in
  *                  reg LSM6DSOX_EMB_FUNC_FIFO_CFG
  *
  */
int32_t lsm6dsox_fifo_pedo_batch_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_emb_func_fifo_cfg_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_FIFO_CFG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    *val = reg.pedo_fifo_en;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief   Enable FIFO batching data of first slave.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of  batch_ext_sens_0_en in
  *                  reg SLV0_CONFIG
  *
  */
int32_t lsm6dsox_sh_batch_slave_0_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_slv0_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV0_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.batch_ext_sens_0_en = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV0_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Enable FIFO batching data of first slave.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of  batch_ext_sens_0_en in
  *                  reg SLV0_CONFIG
  *
  */
int32_t lsm6dsox_sh_batch_slave_0_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_slv0_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV0_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    *val = reg.batch_ext_sens_0_en;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Enable FIFO batching data of second slave.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of  batch_ext_sens_1_en in
  *                  reg SLV1_CONFIG
  *
  */
int32_t lsm6dsox_sh_batch_slave_1_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_slv1_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV1_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.batch_ext_sens_1_en = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV1_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief   Enable FIFO batching data of second slave.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of  batch_ext_sens_1_en in
  *                  reg SLV1_CONFIG
  *
  */
int32_t lsm6dsox_sh_batch_slave_1_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_slv1_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV1_CONFIG, (uint8_t*)&reg, 1);
    *val = reg.batch_ext_sens_1_en;
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Enable FIFO batching data of third slave.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of  batch_ext_sens_2_en in
  *                  reg SLV2_CONFIG
  *
  */
int32_t lsm6dsox_sh_batch_slave_2_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_slv2_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);

  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV2_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.batch_ext_sens_2_en = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV2_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Enable FIFO batching data of third slave.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of  batch_ext_sens_2_en in
  *                  reg SLV2_CONFIG
  *
  */
int32_t lsm6dsox_sh_batch_slave_2_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_slv2_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV2_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    *val = reg.batch_ext_sens_2_en;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief   Enable FIFO batching data of fourth slave.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of  batch_ext_sens_3_en
  *                  in reg SLV3_CONFIG
  *
  */
int32_t lsm6dsox_sh_batch_slave_3_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_slv3_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV3_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.batch_ext_sens_3_en = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV3_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Enable FIFO batching data of fourth slave.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of  batch_ext_sens_3_en in
  *                  reg SLV3_CONFIG
  *
  */
int32_t lsm6dsox_sh_batch_slave_3_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_slv3_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV3_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    *val = reg.batch_ext_sens_3_en;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_DEN_functionality
  * @brief     This section groups all the functions concerning
  *            DEN functionality.
  * @{
  *
*/

/**
  * @brief  DEN functionality marking mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of den_mode in reg CTRL6_C
  *
  */
int32_t lsm6dsox_den_mode_set(stmdev_ctx_t *ctx, lsm6dsox_den_mode_t val)
{
  lsm6dsox_ctrl6_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL6_C, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.den_mode = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL6_C, (uint8_t*)&reg, 1);
  }

  return ret;
}

/**
  * @brief  DEN functionality marking mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of den_mode in reg CTRL6_C
  *
  */
int32_t lsm6dsox_den_mode_get(stmdev_ctx_t *ctx, lsm6dsox_den_mode_t *val)
{
  lsm6dsox_ctrl6_c_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL6_C, (uint8_t*)&reg, 1);

  switch (reg.den_mode) {
    case LSM6DSOX_DEN_DISABLE:
      *val = LSM6DSOX_DEN_DISABLE;
      break;
    case LSM6DSOX_LEVEL_FIFO:
      *val = LSM6DSOX_LEVEL_FIFO;
      break;
    case LSM6DSOX_LEVEL_LETCHED:
      *val = LSM6DSOX_LEVEL_LETCHED;
      break;
    case LSM6DSOX_LEVEL_TRIGGER:
      *val = LSM6DSOX_LEVEL_TRIGGER;
      break;
    case LSM6DSOX_EDGE_TRIGGER:
      *val = LSM6DSOX_EDGE_TRIGGER;
      break;
    default:
      *val = LSM6DSOX_DEN_DISABLE;
      break;
  }
  return ret;
}

/**
  * @brief  DEN active level configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of den_lh in reg CTRL9_XL
  *
  */
int32_t lsm6dsox_den_polarity_set(stmdev_ctx_t *ctx, lsm6dsox_den_lh_t val)
{
  lsm6dsox_ctrl9_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.den_lh = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);
  }

  return ret;
}

/**
  * @brief  DEN active level configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of den_lh in reg CTRL9_XL
  *
  */
int32_t lsm6dsox_den_polarity_get(stmdev_ctx_t *ctx, lsm6dsox_den_lh_t *val)
{
  lsm6dsox_ctrl9_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);

  switch (reg.den_lh) {
    case LSM6DSOX_DEN_ACT_LOW:
      *val = LSM6DSOX_DEN_ACT_LOW;
      break;
    case LSM6DSOX_DEN_ACT_HIGH:
      *val = LSM6DSOX_DEN_ACT_HIGH;
      break;
    default:
      *val = LSM6DSOX_DEN_ACT_LOW;
      break;
  }
  return ret;
}

/**
  * @brief  DEN enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of den_xl_g in reg CTRL9_XL
  *
  */
int32_t lsm6dsox_den_enable_set(stmdev_ctx_t *ctx, lsm6dsox_den_xl_g_t val)
{
  lsm6dsox_ctrl9_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.den_xl_g = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);
  }

  return ret;
}

/**
  * @brief  DEN enable.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of den_xl_g in reg CTRL9_XL
  *
  */
int32_t lsm6dsox_den_enable_get(stmdev_ctx_t *ctx, lsm6dsox_den_xl_g_t *val)
{
  lsm6dsox_ctrl9_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);

  switch (reg.den_xl_g) {
    case LSM6DSOX_STAMP_IN_GY_DATA:
      *val = LSM6DSOX_STAMP_IN_GY_DATA;
      break;
    case LSM6DSOX_STAMP_IN_XL_DATA:
      *val = LSM6DSOX_STAMP_IN_XL_DATA;
      break;
    case LSM6DSOX_STAMP_IN_GY_XL_DATA:
      *val = LSM6DSOX_STAMP_IN_GY_XL_DATA;
      break;
    default:
      *val = LSM6DSOX_STAMP_IN_GY_DATA;
      break;
  }
  return ret;
}

/**
  * @brief  DEN value stored in LSB of X-axis.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of den_z in reg CTRL9_XL
  *
  */
int32_t lsm6dsox_den_mark_axis_x_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl9_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.den_z = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);
  }

  return ret;
}

/**
  * @brief  DEN value stored in LSB of X-axis.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of den_z in reg CTRL9_XL
  *
  */
int32_t lsm6dsox_den_mark_axis_x_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl9_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);
  *val = reg.den_z;

  return ret;
}

/**
  * @brief  DEN value stored in LSB of Y-axis.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of den_y in reg CTRL9_XL
  *
  */
int32_t lsm6dsox_den_mark_axis_y_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl9_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.den_y = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);
  }

  return ret;
}

/**
  * @brief  DEN value stored in LSB of Y-axis.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of den_y in reg CTRL9_XL
  *
  */
int32_t lsm6dsox_den_mark_axis_y_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl9_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);
  *val = reg.den_y;

  return ret;
}

/**
  * @brief  DEN value stored in LSB of Z-axis.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of den_x in reg CTRL9_XL
  *
  */
int32_t lsm6dsox_den_mark_axis_z_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_ctrl9_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.den_x = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);
  }

  return ret;
}

/**
  * @brief  DEN value stored in LSB of Z-axis.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of den_x in reg CTRL9_XL
  *
  */
int32_t lsm6dsox_den_mark_axis_z_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_ctrl9_xl_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL, (uint8_t*)&reg, 1);
  *val = reg.den_x;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_Pedometer
  * @brief     This section groups all the functions that manage pedometer.
  * @{
  *
*/

/**
  * @brief  Enable pedometer algorithm.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      turn on and configure pedometer
  *
  */
int32_t lsm6dsox_pedo_sens_set(stmdev_ctx_t *ctx, lsm6dsox_pedo_md_t val)
{
  lsm6dsox_pedo_cmd_reg_t pedo_cmd_reg;
  int32_t ret;

  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_PEDO_CMD_REG,
                                (uint8_t*)&pedo_cmd_reg);

  if (ret == 0) {
    pedo_cmd_reg.fp_rejection_en = ((uint8_t)val & 0x10U)>>4;
    pedo_cmd_reg.ad_det_en = ((uint8_t)val & 0x20U)>>5;

    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_PEDO_CMD_REG,
                                   (uint8_t*)&pedo_cmd_reg);
  }
  return ret;
}

/**
  * @brief  Enable pedometer algorithm.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      turn on and configure pedometer
  *
  */
int32_t lsm6dsox_pedo_sens_get(stmdev_ctx_t *ctx, lsm6dsox_pedo_md_t *val)
{
  lsm6dsox_pedo_cmd_reg_t pedo_cmd_reg;
  int32_t ret;

  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_PEDO_CMD_REG,
                                 (uint8_t*)&pedo_cmd_reg);
  switch ( (pedo_cmd_reg.ad_det_en <<5) | (pedo_cmd_reg.fp_rejection_en << 4) ){
    case LSM6DSOX_PEDO_BASE_MODE:
      *val = LSM6DSOX_PEDO_BASE_MODE;
      break;
    case LSM6DSOX_FALSE_STEP_REJ:
      *val = LSM6DSOX_FALSE_STEP_REJ;
      break;
    case LSM6DSOX_FALSE_STEP_REJ_ADV_MODE:
      *val = LSM6DSOX_FALSE_STEP_REJ_ADV_MODE;
      break;
    default:
      *val = LSM6DSOX_PEDO_BASE_MODE;
      break;
  }
  return ret;
}

/**
  * @brief  Interrupt status bit for step detection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of is_step_det in reg EMB_FUNC_STATUS
  *
  */
int32_t lsm6dsox_pedo_step_detect_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_emb_func_status_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_STATUS, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    *val = reg.is_step_det;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Pedometer debounce configuration register (r/w).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  *
  */
int32_t lsm6dsox_pedo_debounce_steps_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_PEDO_DEB_STEPS_CONF, buff);
  return ret;
}

/**
  * @brief  Pedometer debounce configuration register (r/w).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_pedo_debounce_steps_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_PEDO_DEB_STEPS_CONF, buff);
  return ret;
}

/**
  * @brief  Time period register for step detection on delta time (r/w).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  *
  */
int32_t lsm6dsox_pedo_steps_period_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  uint8_t index;

  index = 0x00U;
  ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_PEDO_SC_DELTAT_L, &buff[index]);
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_PEDO_SC_DELTAT_H,
                                   &buff[index]);
  }
  return ret;
}

/**
  * @brief   Time period register for step detection on delta time (r/w).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_pedo_steps_period_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  uint8_t index;

  index = 0x00U;
  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_PEDO_SC_DELTAT_L, &buff[index]);
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_PEDO_SC_DELTAT_H,
                                  &buff[index]);
  }
  return ret;
}

/**
  * @brief  Set when user wants to generate interrupt on count overflow
  *         event/every step.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of carry_count_en in reg PEDO_CMD_REG
  *
  */
int32_t lsm6dsox_pedo_int_mode_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_carry_count_en_t val)
{
  lsm6dsox_pedo_cmd_reg_t reg;
  int32_t ret;

  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_PEDO_CMD_REG, (uint8_t*)&reg);
  if (ret == 0) {
    reg.carry_count_en = (uint8_t)val;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_PEDO_CMD_REG,
                                   (uint8_t*)&reg);
  }
  return ret;
}

/**
  * @brief  Set when user wants to generate interrupt on count overflow
  *         event/every step.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of carry_count_en in reg PEDO_CMD_REG
  *
  */
int32_t lsm6dsox_pedo_int_mode_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_carry_count_en_t *val)
{
  lsm6dsox_pedo_cmd_reg_t reg;
  int32_t ret;

  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_PEDO_CMD_REG, (uint8_t*)&reg);
  switch (reg.carry_count_en) {
    case LSM6DSOX_EVERY_STEP:
      *val = LSM6DSOX_EVERY_STEP;
      break;
    case LSM6DSOX_COUNT_OVERFLOW:
      *val = LSM6DSOX_COUNT_OVERFLOW;
      break;
    default:
      *val = LSM6DSOX_EVERY_STEP;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_significant_motion
  * @brief   This section groups all the functions that manage the
  *          significant motion detection.
  * @{
  *
  */

/**
  * @brief   Interrupt status bit for significant motion detection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of is_sigmot in reg EMB_FUNC_STATUS
  *
  */
int32_t lsm6dsox_motion_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_emb_func_status_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_STATUS, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    *val = reg.is_sigmot;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_tilt_detection
  * @brief     This section groups all the functions that manage the tilt
  *            event detection.
  * @{
  *
  */

/**
  * @brief  Interrupt status bit for tilt detection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of is_tilt in reg EMB_FUNC_STATUS
  *
  */
int32_t lsm6dsox_tilt_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_emb_func_status_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_STATUS, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    *val = reg.is_tilt;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_ magnetometer_sensor
  * @brief     This section groups all the functions that manage additional
  *            magnetometer sensor.
  * @{
  *
  */

/**
  * @brief  External magnetometer sensitivity value register for
  *         Sensor hub.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  *
  */
int32_t lsm6dsox_sh_mag_sensitivity_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  uint8_t index;

  index = 0x00U;
  ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SENSITIVITY_L,
                                 &buff[index]);
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SENSITIVITY_H,
                                   &buff[index]);
  }

  return ret;
}

/**
  * @brief  External magnetometer sensitivity value register for
  *         Sensor hub.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_sh_mag_sensitivity_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  uint8_t index;

  index = 0x00U;
  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SENSITIVITY_L,
                                &buff[index]);
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SENSITIVITY_H,
                                  &buff[index]);
  }

  return ret;
}

/**
  * @brief  External magnetometer sensitivity value register for
  *         Machine Learning Core.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  *
  */
int32_t lsm6dsox_mlc_mag_sensitivity_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  uint8_t index;

  index = 0x00U;
  ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MLC_MAG_SENSITIVITY_L,
                                  &buff[index]);
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MLC_MAG_SENSITIVITY_H,
                                    &buff[index]);
  }
  return ret;
}

/**
  * @brief  External magnetometer sensitivity value register for
  *         Machine Learning Core.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_mlc_mag_sensitivity_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  uint8_t index;

  index = 0x00U;
  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MLC_MAG_SENSITIVITY_L,
                                 &buff[index]);
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MLC_MAG_SENSITIVITY_H,
                                   &buff[index]);
  }
  return ret;
}


/**
  * @brief  Offset for hard-iron compensation register (r/w).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  *
  */
int32_t lsm6dsox_mag_offset_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  uint8_t index;

  index = 0x00U;
  ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_OFFX_L, &buff[index]);
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_OFFX_H, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_OFFY_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_OFFY_H, &buff[index]);
  }
  if (ret == 0) {
    index++;

    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_OFFZ_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_OFFZ_H, &buff[index]);
  }

  return ret;
}

/**
  * @brief  Offset for hard-iron compensation register (r/w).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_mag_offset_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  uint8_t index;

  index = 0x00U;
  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_OFFX_L, &buff[index]);
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_OFFX_H, &buff[index]);
  }
  if (ret == 0) {
    index++;

    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_OFFY_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_OFFY_H, &buff[index]);
  }
  if (ret == 0) {
    index++;

    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_OFFZ_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_OFFZ_H, &buff[index]);
  }
  return ret;
}

/**
  * @brief  Soft-iron (3x3 symmetric) matrix correction
  *         register (r/w). The value is expressed as
  *         half-precision floating-point format:
  *         SEEEEEFFFFFFFFFF
  *         S: 1 sign bit;
  *         E: 5 exponent bits;
  *         F: 10 fraction bits).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  *
  */
int32_t lsm6dsox_mag_soft_iron_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  uint8_t index;

  index = 0x00U;
  ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SI_XX_L, &buff[index]);
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SI_XX_H, &buff[index]);
  }
  if (ret == 0) {
    index++;

    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SI_XY_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SI_XY_H, &buff[index]);
  }
  if (ret == 0) {
    index++;

    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SI_XZ_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SI_XZ_H, &buff[index]);
  }
  if (ret == 0) {
    index++;

    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SI_YY_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SI_YY_H, &buff[index]);
  }
  if (ret == 0) {
    index++;

    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SI_YZ_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SI_YZ_H, &buff[index]);
  }
  if (ret == 0) {
    index++;

    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SI_ZZ_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_SI_ZZ_H, &buff[index]);
  }

  return ret;
}

/**
  * @brief  Soft-iron (3x3 symmetric) matrix
  *         correction register (r/w).
  *         The value is expressed as half-precision
  *         floating-point format:
  *         SEEEEEFFFFFFFFFF
  *         S: 1 sign bit;
  *         E: 5 exponent bits;
  *         F: 10 fraction bits.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_mag_soft_iron_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  uint8_t index;

  index = 0x00U;
  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SI_XX_L, &buff[index]);
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SI_XX_H, &buff[index]);
  }
  if (ret == 0) {
    index++;

    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SI_XY_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SI_XY_H, &buff[index]);
  }
  if (ret == 0) {
    index++;

    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SI_XZ_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SI_XZ_H, &buff[index]);
  }
  if (ret == 0) {
    index++;

    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SI_YY_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SI_YY_H, &buff[index]);
  }
  if (ret == 0) {
    index++;

    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SI_YZ_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SI_YZ_H, &buff[index]);
  }
  if (ret == 0) {
    index++;

    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SI_ZZ_L, &buff[index]);
  }
  if (ret == 0) {
    index++;
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_SI_ZZ_H, &buff[index]);
  }

  return ret;
}

/**
  * @brief  Magnetometer Z-axis coordinates
  *         rotation (to be aligned to
  *         accelerometer/gyroscope axes
  *         orientation).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of mag_z_axis in reg MAG_CFG_A
  *
  */
int32_t lsm6dsox_mag_z_orient_set(stmdev_ctx_t *ctx, lsm6dsox_mag_z_axis_t val)
{
  lsm6dsox_mag_cfg_a_t reg;
  int32_t ret;

  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_CFG_A, (uint8_t*)&reg);
  if (ret == 0) {
    reg.mag_z_axis = (uint8_t) val;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_CFG_A, (uint8_t*)&reg);
  }

  return ret;
}

/**
  * @brief  Magnetometer Z-axis coordinates
  *         rotation (to be aligned to
  *         accelerometer/gyroscope axes
  *         orientation).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of mag_z_axis in reg MAG_CFG_A
  *
  */
int32_t lsm6dsox_mag_z_orient_get(stmdev_ctx_t *ctx,
                                 lsm6dsox_mag_z_axis_t *val)
{
  lsm6dsox_mag_cfg_a_t reg;
  int32_t ret;
  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_CFG_A, (uint8_t*)&reg);
  switch (reg.mag_z_axis) {
    case LSM6DSOX_Z_EQ_Y:
      *val = LSM6DSOX_Z_EQ_Y;
      break;
    case LSM6DSOX_Z_EQ_MIN_Y:
      *val = LSM6DSOX_Z_EQ_MIN_Y;
      break;
    case LSM6DSOX_Z_EQ_X:
      *val = LSM6DSOX_Z_EQ_X;
      break;
    case LSM6DSOX_Z_EQ_MIN_X:
      *val = LSM6DSOX_Z_EQ_MIN_X;
      break;
    case LSM6DSOX_Z_EQ_MIN_Z:
      *val = LSM6DSOX_Z_EQ_MIN_Z;
      break;
    case LSM6DSOX_Z_EQ_Z:
      *val = LSM6DSOX_Z_EQ_Z;
      break;
    default:
      *val = LSM6DSOX_Z_EQ_Y;
      break;
  }
  return ret;
}

/**
  * @brief   Magnetometer Y-axis coordinates
  *          rotation (to be aligned to
  *          accelerometer/gyroscope axes
  *          orientation).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of mag_y_axis in reg MAG_CFG_A
  *
  */
int32_t lsm6dsox_mag_y_orient_set(stmdev_ctx_t *ctx,
                                 lsm6dsox_mag_y_axis_t val)
{
  lsm6dsox_mag_cfg_a_t reg;
  int32_t ret;

  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_CFG_A, (uint8_t*)&reg);
  if (ret == 0) {
    reg.mag_y_axis = (uint8_t)val;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_CFG_A,(uint8_t*) &reg);
  }
  return ret;
}

/**
  * @brief  Magnetometer Y-axis coordinates
  *         rotation (to be aligned to
  *         accelerometer/gyroscope axes
  *         orientation).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of mag_y_axis in reg MAG_CFG_A
  *
  */
int32_t lsm6dsox_mag_y_orient_get(stmdev_ctx_t *ctx,
                                 lsm6dsox_mag_y_axis_t *val)
{
  lsm6dsox_mag_cfg_a_t reg;
  int32_t ret;

  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_CFG_A, (uint8_t*)&reg);
  switch (reg.mag_y_axis) {
    case LSM6DSOX_Y_EQ_Y:
      *val = LSM6DSOX_Y_EQ_Y;
      break;
    case LSM6DSOX_Y_EQ_MIN_Y:
      *val = LSM6DSOX_Y_EQ_MIN_Y;
      break;
    case LSM6DSOX_Y_EQ_X:
      *val = LSM6DSOX_Y_EQ_X;
      break;
    case LSM6DSOX_Y_EQ_MIN_X:
      *val = LSM6DSOX_Y_EQ_MIN_X;
      break;
    case LSM6DSOX_Y_EQ_MIN_Z:
      *val = LSM6DSOX_Y_EQ_MIN_Z;
      break;
    case LSM6DSOX_Y_EQ_Z:
      *val = LSM6DSOX_Y_EQ_Z;
      break;
    default:
      *val = LSM6DSOX_Y_EQ_Y;
      break;
  }
  return ret;
}

/**
  * @brief  Magnetometer X-axis coordinates
  *         rotation (to be aligned to
  *         accelerometer/gyroscope axes
  *         orientation).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of mag_x_axis in reg MAG_CFG_B
  *
  */
int32_t lsm6dsox_mag_x_orient_set(stmdev_ctx_t *ctx,
                                 lsm6dsox_mag_x_axis_t val)
{
  lsm6dsox_mag_cfg_b_t reg;
  int32_t ret;

  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_CFG_B, (uint8_t*)&reg);
  if (ret == 0) {
    reg.mag_x_axis = (uint8_t)val;
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_MAG_CFG_B, (uint8_t*)&reg);
  }
  return ret;
}

/**
  * @brief   Magnetometer X-axis coordinates
  *          rotation (to be aligned to
  *          accelerometer/gyroscope axes
  *          orientation).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of mag_x_axis in reg MAG_CFG_B
  *
  */
int32_t lsm6dsox_mag_x_orient_get(stmdev_ctx_t *ctx,
                                 lsm6dsox_mag_x_axis_t *val)
{
  lsm6dsox_mag_cfg_b_t reg;
  int32_t ret;

  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_MAG_CFG_B, (uint8_t*)&reg);
  switch (reg.mag_x_axis) {
    case LSM6DSOX_X_EQ_Y:
      *val = LSM6DSOX_X_EQ_Y;
      break;
    case LSM6DSOX_X_EQ_MIN_Y:
      *val = LSM6DSOX_X_EQ_MIN_Y;
      break;
    case LSM6DSOX_X_EQ_X:
      *val = LSM6DSOX_X_EQ_X;
      break;
    case LSM6DSOX_X_EQ_MIN_X:
      *val = LSM6DSOX_X_EQ_MIN_X;
      break;
    case LSM6DSOX_X_EQ_MIN_Z:
      *val = LSM6DSOX_X_EQ_MIN_Z;
      break;
    case LSM6DSOX_X_EQ_Z:
      *val = LSM6DSOX_X_EQ_Z;
      break;
    default:
      *val = LSM6DSOX_X_EQ_Y;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_finite_state_machine
  * @brief     This section groups all the functions that manage the
  *            state_machine.
  * @{
  *
  */

/**
  * @brief   Interrupt status bit for FSM long counter
  *          timeout interrupt event.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of is_fsm_lc in reg EMB_FUNC_STATUS
  *
  */
int32_t lsm6dsox_long_cnt_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_emb_func_status_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_STATUS, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    *val = reg.is_fsm_lc;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Finite State Machine enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      union of registers from FSM_ENABLE_A to FSM_ENABLE_B
  *
  */
int32_t lsm6dsox_fsm_enable_set(stmdev_ctx_t *ctx,
                                lsm6dsox_emb_fsm_enable_t *val)
{
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FSM_ENABLE_A,
                            (uint8_t*)&val->fsm_enable_a, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FSM_ENABLE_B,
                            (uint8_t*)&val->fsm_enable_b, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Finite State Machine enable.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      union of registers from FSM_ENABLE_A to FSM_ENABLE_B
  *
  */
int32_t lsm6dsox_fsm_enable_get(stmdev_ctx_t *ctx,
                               lsm6dsox_emb_fsm_enable_t *val)
{
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FSM_ENABLE_A, (uint8_t*) val, 2);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  FSM long counter status register. Long counter value is an
  *         unsigned integer value (16-bit format).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  *
  */
int32_t lsm6dsox_long_cnt_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FSM_LONG_COUNTER_L, buff, 2);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  FSM long counter status register. Long counter value is an
  *         unsigned integer value (16-bit format).[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
int32_t lsm6dsox_long_cnt_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FSM_LONG_COUNTER_L, buff, 2);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Clear FSM long counter value.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fsm_lc_clr in
  *                  reg FSM_LONG_COUNTER_CLEAR
  *
  */
int32_t lsm6dsox_long_clr_set(stmdev_ctx_t *ctx, lsm6dsox_fsm_lc_clr_t val)
{
  lsm6dsox_fsm_long_counter_clear_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FSM_LONG_COUNTER_CLEAR,
    (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg. fsm_lc_clr = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FSM_LONG_COUNTER_CLEAR,
    (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Clear FSM long counter value.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of fsm_lc_clr in
  *                  reg FSM_LONG_COUNTER_CLEAR
  *
  */
int32_t lsm6dsox_long_clr_get(stmdev_ctx_t *ctx, lsm6dsox_fsm_lc_clr_t *val)
{
  lsm6dsox_fsm_long_counter_clear_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FSM_LONG_COUNTER_CLEAR,
    (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    switch (reg.fsm_lc_clr) {
      case LSM6DSOX_LC_NORMAL:
        *val = LSM6DSOX_LC_NORMAL;
        break;
      case LSM6DSOX_LC_CLEAR:
        *val = LSM6DSOX_LC_CLEAR;
        break;
      case LSM6DSOX_LC_CLEAR_DONE:
        *val = LSM6DSOX_LC_CLEAR_DONE;
        break;
      default:
        *val = LSM6DSOX_LC_NORMAL;
        break;
    }
  }

  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  FSM output registers[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      struct of registers from FSM_OUTS1 to FSM_OUTS16
  *
  */
int32_t lsm6dsox_fsm_out_get(stmdev_ctx_t *ctx, lsm6dsox_fsm_out_t *val)
{
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FSM_OUTS1, (uint8_t*)val, 16);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Finite State Machine ODR configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fsm_odr in reg EMB_FUNC_ODR_CFG_B
  *
  */
int32_t lsm6dsox_fsm_data_rate_set(stmdev_ctx_t *ctx, lsm6dsox_fsm_odr_t val)
{
  lsm6dsox_emb_func_odr_cfg_b_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_ODR_CFG_B,
                           (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.not_used_01 = 3; /* set default values */
    reg.not_used_02 = 2; /* set default values */
    reg.fsm_odr = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_ODR_CFG_B,
                            (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Finite State Machine ODR configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of fsm_odr in reg EMB_FUNC_ODR_CFG_B
  *
  */
int32_t lsm6dsox_fsm_data_rate_get(stmdev_ctx_t *ctx, lsm6dsox_fsm_odr_t *val)
{
  lsm6dsox_emb_func_odr_cfg_b_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_ODR_CFG_B,
                           (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    switch (reg.fsm_odr) {
      case LSM6DSOX_ODR_FSM_12Hz5:
        *val = LSM6DSOX_ODR_FSM_12Hz5;
        break;
      case LSM6DSOX_ODR_FSM_26Hz:
        *val = LSM6DSOX_ODR_FSM_26Hz;
        break;
      case LSM6DSOX_ODR_FSM_52Hz:
        *val = LSM6DSOX_ODR_FSM_52Hz;
        break;
      case LSM6DSOX_ODR_FSM_104Hz:
        *val = LSM6DSOX_ODR_FSM_104Hz;
        break;
      default:
        *val = LSM6DSOX_ODR_FSM_12Hz5;
        break;
    }
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  FSM initialization request.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fsm_init in reg FSM_INIT
  *
  */
int32_t lsm6dsox_fsm_init_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_emb_func_init_b_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_INIT_B, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.fsm_init = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_INIT_B, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  FSM initialization request.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fsm_init in reg FSM_INIT
  *
  */
int32_t lsm6dsox_fsm_init_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_emb_func_init_b_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_INIT_B, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    *val = reg.fsm_init;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  FSM long counter timeout register (r/w). The long counter
  *         timeout value is an unsigned integer value (16-bit format).
  *         When the long counter value reached this value,
  *         the FSM generates an interrupt.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      the value of long counter
  *
  */
int32_t lsm6dsox_long_cnt_int_value_set(stmdev_ctx_t *ctx, uint16_t val)
{
  int32_t ret;
  uint8_t add_l;
  uint8_t add_h;

  add_h = (uint8_t)( ( val & 0xFF00U ) >> 8 );
  add_l = (uint8_t)( val & 0x00FFU );

  ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_FSM_LC_TIMEOUT_L, &add_l);
  if (ret == 0) {
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_FSM_LC_TIMEOUT_H, &add_h);
  }

  return ret;
}

/**
  * @brief  FSM long counter timeout register (r/w). The long counter
  *         timeout value is an unsigned integer value (16-bit format).
  *         When the long counter value reached this value,
  *         the FSM generates an interrupt.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     buffer that stores the value of long counter
  *
  */
int32_t lsm6dsox_long_cnt_int_value_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  int32_t ret;
  uint8_t add_l;
  uint8_t add_h;

  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_FSM_LC_TIMEOUT_L, &add_l);
  if (ret == 0) {
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_FSM_LC_TIMEOUT_H, &add_h);
    *val = add_h;
    *val = *val << 8;
    *val += add_l;
  }

  return ret;
}

/**
  * @brief  FSM number of programs register.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      value to write
  *
  */
int32_t lsm6dsox_fsm_number_of_programs_set(stmdev_ctx_t *ctx, uint8_t val)
{
  int32_t ret;

  ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_FSM_PROGRAMS, &val);

  return ret;
}

/**
  * @brief  FSM number of programs register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      buffer that stores data read.
  *
  */
int32_t lsm6dsox_fsm_number_of_programs_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  int32_t ret;

  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_FSM_PROGRAMS, val);

  return ret;
}

/**
  * @brief  FSM start address register (r/w).
  *         First available address is 0x033C.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      the value of start address
  *
  */
int32_t lsm6dsox_fsm_start_address_set(stmdev_ctx_t *ctx, uint16_t val)
{
  int32_t ret;
  uint8_t add_l;
  uint8_t add_h;

  add_h = (uint8_t)( ( val & 0xFF00U ) >> 8 );
  add_l = (uint8_t)( val & 0x00FFU );

  ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_FSM_START_ADD_L, &add_l);
  if (ret == 0) {
    ret = lsm6dsox_ln_pg_write_byte(ctx, LSM6DSOX_FSM_START_ADD_H, &add_h);
  }
  return ret;
}

/**
  * @brief  FSM start address register (r/w).
  *         First available address is 0x033C.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      buffer the value of start address.
  *
  */
int32_t lsm6dsox_fsm_start_address_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  int32_t ret;
  uint8_t add_l;
  uint8_t add_h;

  ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_FSM_START_ADD_L, &add_l);
  if (ret == 0) {
    ret = lsm6dsox_ln_pg_read_byte(ctx, LSM6DSOX_FSM_START_ADD_H, &add_h);
    *val = add_h;
    *val = *val << 8;
    *val += add_l;
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
  * @brief  Machine Learning Core status register[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register MLC_STATUS_MAINPAGE
  *
  */
int32_t lsm6dsox_mlc_status_get(stmdev_ctx_t *ctx,
                                lsm6dsox_mlc_status_mainpage_t *val)
{
  return lsm6dsox_read_reg(ctx, LSM6DSOX_MLC_STATUS_MAINPAGE,
                           (uint8_t*) val, 1);
}

/**
  * @brief  Machine Learning Core data rate selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of mlc_odr in
  *                  reg EMB_FUNC_ODR_CFG_C
  *
  */
int32_t lsm6dsox_mlc_data_rate_set(stmdev_ctx_t *ctx,
                                       lsm6dsox_mlc_odr_t val)
{
  lsm6dsox_emb_func_odr_cfg_c_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_ODR_CFG_C,
                            (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.mlc_odr = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_ODR_CFG_C, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
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
int32_t lsm6dsox_mlc_data_rate_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_mlc_odr_t *val)
{
  lsm6dsox_emb_func_odr_cfg_c_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_ODR_CFG_C,
                            (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    switch (reg.mlc_odr) {
      case LSM6DSOX_ODR_PRGS_12Hz5:
        *val = LSM6DSOX_ODR_PRGS_12Hz5;
        break;
      case LSM6DSOX_ODR_PRGS_26Hz:
        *val = LSM6DSOX_ODR_PRGS_26Hz;
        break;
      case LSM6DSOX_ODR_PRGS_52Hz:
        *val = LSM6DSOX_ODR_PRGS_52Hz;
        break;
      case LSM6DSOX_ODR_PRGS_104Hz:
        *val = LSM6DSOX_ODR_PRGS_104Hz;
        break;
      default:
        *val = LSM6DSOX_ODR_PRGS_12Hz5;
        break;
    }
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  LSM6DSOX_Sensor_hub
  * @brief     This section groups all the functions that manage the
  *            sensor hub.
  * @{
  *
  */

/**
* @brief  Sensor hub output registers.[get]
*
* @param  ctx      read / write interface definitions
* @param  val      union of registers from SENSOR_HUB_1 to SENSOR_HUB_18
*
  */
int32_t lsm6dsox_sh_read_data_raw_get(stmdev_ctx_t *ctx,
                                     lsm6dsox_emb_sh_read_t *val,
                                     uint8_t len)
{
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SENSOR_HUB_1, (uint8_t*) val, len);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Number of external sensors to be read by the sensor hub.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of aux_sens_on in reg MASTER_CONFIG
  *
  */
int32_t lsm6dsox_sh_slave_connected_set(stmdev_ctx_t *ctx,
                                       lsm6dsox_aux_sens_on_t val)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.aux_sens_on = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Number of external sensors to be read by the sensor hub.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of aux_sens_on in reg MASTER_CONFIG
  *
  */
int32_t lsm6dsox_sh_slave_connected_get(stmdev_ctx_t *ctx,
                                       lsm6dsox_aux_sens_on_t *val)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    switch (reg.aux_sens_on) {
      case LSM6DSOX_SLV_0:
        *val = LSM6DSOX_SLV_0;
        break;
      case LSM6DSOX_SLV_0_1:
        *val = LSM6DSOX_SLV_0_1;
        break;
      case LSM6DSOX_SLV_0_1_2:
        *val = LSM6DSOX_SLV_0_1_2;
        break;
      case LSM6DSOX_SLV_0_1_2_3:
        *val = LSM6DSOX_SLV_0_1_2_3;
        break;
      default:
        *val = LSM6DSOX_SLV_0;
        break;
    }
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Sensor hub I2C master enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of master_on in reg MASTER_CONFIG
  *
  */
int32_t lsm6dsox_sh_master_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.master_on = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Sensor hub I2C master enable.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of master_on in reg MASTER_CONFIG
  *
  */
int32_t lsm6dsox_sh_master_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    *val = reg.master_on;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Master I2C pull-up enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of shub_pu_en in reg MASTER_CONFIG
  *
  */
int32_t lsm6dsox_sh_pin_mode_set(stmdev_ctx_t *ctx, lsm6dsox_shub_pu_en_t val)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.shub_pu_en = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Master I2C pull-up enable.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of shub_pu_en in reg MASTER_CONFIG
  *
  */
int32_t lsm6dsox_sh_pin_mode_get(stmdev_ctx_t *ctx,
                                lsm6dsox_shub_pu_en_t *val)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    switch (reg.shub_pu_en) {
      case LSM6DSOX_EXT_PULL_UP:
        *val = LSM6DSOX_EXT_PULL_UP;
        break;
      case LSM6DSOX_INTERNAL_PULL_UP:
        *val = LSM6DSOX_INTERNAL_PULL_UP;
        break;
      default:
        *val = LSM6DSOX_EXT_PULL_UP;
        break;
    }
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  I2C interface pass-through.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of pass_through_mode in
  *                  reg MASTER_CONFIG
  *
  */
int32_t lsm6dsox_sh_pass_through_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.pass_through_mode = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  I2C interface pass-through.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of pass_through_mode in
  *                  reg MASTER_CONFIG
  *
  */
int32_t lsm6dsox_sh_pass_through_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    *val = reg.pass_through_mode;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Sensor hub trigger signal selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of start_config in reg MASTER_CONFIG
  *
  */
int32_t lsm6dsox_sh_syncro_mode_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_start_config_t val)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.start_config = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Sensor hub trigger signal selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of start_config in reg MASTER_CONFIG
  *
  */
int32_t lsm6dsox_sh_syncro_mode_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_start_config_t *val)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    switch (reg.start_config) {
      case LSM6DSOX_EXT_ON_INT2_PIN:
        *val = LSM6DSOX_EXT_ON_INT2_PIN;
        break;
      case LSM6DSOX_XL_GY_DRDY:
        *val = LSM6DSOX_XL_GY_DRDY;
        break;
      default:
        *val = LSM6DSOX_EXT_ON_INT2_PIN;
        break;
    }
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Slave 0 write operation is performed only at the first
  *         sensor hub cycle.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of write_once in reg MASTER_CONFIG
  *
  */
int32_t lsm6dsox_sh_write_mode_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_write_once_t val)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.write_once = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Slave 0 write operation is performed only at the first sensor
  *         hub cycle.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of write_once in reg MASTER_CONFIG
  *
  */
int32_t lsm6dsox_sh_write_mode_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_write_once_t *val)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    switch (reg.write_once) {
      case LSM6DSOX_EACH_SH_CYCLE:
        *val = LSM6DSOX_EACH_SH_CYCLE;
        break;
      case LSM6DSOX_ONLY_FIRST_CYCLE:
        *val = LSM6DSOX_ONLY_FIRST_CYCLE;
        break;
      default:
        *val = LSM6DSOX_EACH_SH_CYCLE;
        break;
    }
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Reset Master logic and output registers.[set]
  *
  * @param  ctx      read / write interface definitions
  *
  */
int32_t lsm6dsox_sh_reset_set(stmdev_ctx_t *ctx)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.rst_master_regs = PROPERTY_ENABLE;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.rst_master_regs = PROPERTY_DISABLE;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Reset Master logic and output registers.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of rst_master_regs in reg MASTER_CONFIG
  *
  */
int32_t lsm6dsox_sh_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_master_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MASTER_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    *val = reg.rst_master_regs;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Rate at which the master communicates.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of shub_odr in reg slv1_CONFIG
  *
  */
int32_t lsm6dsox_sh_data_rate_set(stmdev_ctx_t *ctx, lsm6dsox_shub_odr_t val)
{
  lsm6dsox_slv0_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV0_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    reg.shub_odr = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV0_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Rate at which the master communicates.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of shub_odr in reg slv1_CONFIG
  *
  */
int32_t lsm6dsox_sh_data_rate_get(stmdev_ctx_t *ctx,
                                 lsm6dsox_shub_odr_t *val)
{
  lsm6dsox_slv0_config_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV0_CONFIG, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    switch (reg.shub_odr) {
      case LSM6DSOX_SH_ODR_104Hz:
        *val = LSM6DSOX_SH_ODR_104Hz;
        break;
      case LSM6DSOX_SH_ODR_52Hz:
        *val = LSM6DSOX_SH_ODR_52Hz;
        break;
      case LSM6DSOX_SH_ODR_26Hz:
        *val = LSM6DSOX_SH_ODR_26Hz;
        break;
      case LSM6DSOX_SH_ODR_13Hz:
        *val = LSM6DSOX_SH_ODR_13Hz;
        break;
      default:
        *val = LSM6DSOX_SH_ODR_104Hz;
        break;
    }
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Configure slave 0 for perform a write.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      a structure that contain
  *                      - uint8_t slv1_add;    8 bit i2c device address
  *                      - uint8_t slv1_subadd; 8 bit register device address
  *                      - uint8_t slv1_data;   8 bit data to write
  *
  */
int32_t lsm6dsox_sh_cfg_write(stmdev_ctx_t *ctx, lsm6dsox_sh_cfg_write_t *val)
{
  lsm6dsox_slv0_add_t reg;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    reg.slave0 = val->slv0_add;
    reg.rw_0 = 0;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV0_ADD, (uint8_t*)&reg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV0_SUBADD,
    &(val->slv0_subadd), 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_DATAWRITE_SLV0,
    &(val->slv0_data), 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Configure slave 0 for perform a read.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Structure that contain
  *                      - uint8_t slv1_add;    8 bit i2c device address
  *                      - uint8_t slv1_subadd; 8 bit register device address
  *                      - uint8_t slv1_len;    num of bit to read
  *
  */
int32_t lsm6dsox_sh_slv0_cfg_read(stmdev_ctx_t *ctx,
                                 lsm6dsox_sh_cfg_read_t *val)
{
  lsm6dsox_slv0_add_t slv0_add;
  lsm6dsox_slv0_config_t slv0_config;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    slv0_add.slave0 = val->slv_add;
    slv0_add.rw_0 = 1;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV0_ADD, (uint8_t*)&slv0_add, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV0_SUBADD,
    &(val->slv_subadd), 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV0_CONFIG,
                           (uint8_t*)&slv0_config, 1);
  }
  if (ret == 0) {
    slv0_config.slave0_numop = val->slv_len;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV0_CONFIG,
                            (uint8_t*)&slv0_config, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Configure slave 0 for perform a write/read.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Structure that contain
  *                      - uint8_t slv1_add;    8 bit i2c device address
  *                      - uint8_t slv1_subadd; 8 bit register device address
  *                      - uint8_t slv1_len;    num of bit to read
  *
  */
int32_t lsm6dsox_sh_slv1_cfg_read(stmdev_ctx_t *ctx,
                                 lsm6dsox_sh_cfg_read_t *val)
{
  lsm6dsox_slv1_add_t slv1_add;
  lsm6dsox_slv1_config_t slv1_config;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    slv1_add.slave1_add = val->slv_add;
    slv1_add.r_1 = 1;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV1_ADD, (uint8_t*)&slv1_add, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV1_SUBADD,
    &(val->slv_subadd), 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV1_CONFIG,
                           (uint8_t*)&slv1_config, 1);
  }
  if (ret == 0) {
    slv1_config.slave1_numop = val->slv_len;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV1_CONFIG,
                            (uint8_t*)&slv1_config, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Configure slave 0 for perform a write/read.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Structure that contain
  *                      - uint8_t slv2_add;    8 bit i2c device address
  *                      - uint8_t slv2_subadd; 8 bit register device address
  *                      - uint8_t slv2_len;    num of bit to read
  *
  */
int32_t lsm6dsox_sh_slv2_cfg_read(stmdev_ctx_t *ctx,
                                 lsm6dsox_sh_cfg_read_t *val)
{
  lsm6dsox_slv2_add_t slv2_add;
  lsm6dsox_slv2_config_t slv2_config;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    slv2_add.slave2_add = val->slv_add;
    slv2_add.r_2 = 1;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV2_ADD, (uint8_t*)&slv2_add, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV2_SUBADD,
    &(val->slv_subadd), 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV2_CONFIG,
                           (uint8_t*)&slv2_config, 1);
  }
  if (ret == 0) {
    slv2_config.slave2_numop = val->slv_len;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV2_CONFIG,
                            (uint8_t*)&slv2_config, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief Configure slave 0 for perform a write/read.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Structure that contain
  *                      - uint8_t slv3_add;    8 bit i2c device address
  *                      - uint8_t slv3_subadd; 8 bit register device address
  *                      - uint8_t slv3_len;    num of bit to read
  *
  */
int32_t lsm6dsox_sh_slv3_cfg_read(stmdev_ctx_t *ctx,
                                 lsm6dsox_sh_cfg_read_t *val)
{
  lsm6dsox_slv3_add_t slv3_add;
  lsm6dsox_slv3_config_t slv3_config;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    slv3_add.slave3_add = val->slv_add;
    slv3_add.r_3 = 1;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV3_ADD, (uint8_t*)&slv3_add, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV3_SUBADD,
    &(val->slv_subadd), 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_SLV3_CONFIG,
                           (uint8_t*)&slv3_config, 1);
  }
  if (ret == 0) {
    slv3_config.slave3_numop = val->slv_len;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_SLV3_CONFIG,
                            (uint8_t*)&slv3_config, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Sensor hub source register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      union of registers from STATUS_MASTER to
  *
  */
int32_t lsm6dsox_sh_status_get(stmdev_ctx_t *ctx,
                              lsm6dsox_status_master_t *val)
{
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_SENSOR_HUB_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_STATUS_MASTER, (uint8_t*) val, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @}
  *
  */

  /**
  * @addtogroup  Sensors for Smart Mobile Devices
  * @brief   This section groups all the functions that manage the
  *          Sensors for Smart Mobile Devices.
  * @{
  *
  */

/**
  * @brief  s4s_tph_res: [set] Sensor synchronization time frame resolution
  *
  * @param  *ctx   read / write interface definitions
  * @param  val    change the values of tph_h_sel in LSM6DSOX_S4S_TPH_L
  *
  */
int32_t lsm6dsox_s4s_tph_res_set(stmdev_ctx_t *ctx,
                                lsm6dsox_s4s_tph_res_t val)
{
  lsm6dsox_s4s_tph_l_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_S4S_TPH_L, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.tph_h_sel = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_S4S_TPH_L, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  s4s_tph_res: [get] Sensor synchronization time frame resolution
  *
  * @param  *ctx   read / write interface definitions
  * @param  val    get the values of tph_h_sel in LSM6DSOX_S4S_TPH_L
  *
  */
int32_t lsm6dsox_s4s_tph_res_get(stmdev_ctx_t *ctx,
                                lsm6dsox_s4s_tph_res_t *val)
{
  lsm6dsox_s4s_tph_l_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_S4S_TPH_L, (uint8_t*)&reg, 1);
  switch (reg.tph_h_sel) {
      case LSM6DSOX_S4S_TPH_7bit:
        *val = LSM6DSOX_S4S_TPH_7bit;
        break;
      case LSM6DSOX_S4S_TPH_15bit:
        *val = LSM6DSOX_S4S_TPH_15bit;
        break;
      default:
        *val = LSM6DSOX_S4S_TPH_7bit;
        break;
    }

  return ret;
}

/**
  * @brief  s4s_tph_val: [set] Sensor synchronization time frame
  *
  * @param  *ctx   read / write interface definitions
  * @param  val    change the values of tph_l in S4S_TPH_L and
  *                tph_h in S4S_TPH_H
  *
  */
int32_t lsm6dsox_s4s_tph_val_set(stmdev_ctx_t *ctx, uint16_t val)
{
  lsm6dsox_s4s_tph_l_t s4s_tph_l;
  lsm6dsox_s4s_tph_h_t s4s_tph_h;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_S4S_TPH_L, (uint8_t*)&s4s_tph_l, 1);
  if (ret == 0) {
    s4s_tph_l.tph_l = (uint8_t)(val & 0x007FU);
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_S4S_TPH_L, (uint8_t*)&s4s_tph_l, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_S4S_TPH_H, (uint8_t*)&s4s_tph_h, 1);
    s4s_tph_h.tph_h = (uint8_t)(val & 0x7F80U) >> 7;
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_S4S_TPH_H, (uint8_t*)&s4s_tph_h, 1);
  }
  return ret;
}

/**
  * @brief  s4s_tph_val: [get] Sensor synchronization time frame.
  *
  * @param  *ctx   read / write interface definitions
  * @param  val    get the values of tph_l in S4S_TPH_L and
  *                tph_h in S4S_TPH_H
  *
  */
int32_t lsm6dsox_s4s_tph_val_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  lsm6dsox_s4s_tph_l_t s4s_tph_l;
  lsm6dsox_s4s_tph_h_t s4s_tph_h;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_S4S_TPH_L, (uint8_t*)&s4s_tph_l, 1);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_S4S_TPH_H, (uint8_t*)&s4s_tph_h, 1);
    *val =  s4s_tph_h.tph_h;
    *val =  *val << 7;
    *val += s4s_tph_l.tph_l;
  }
  return ret;
}

/**
  * @brief  s4s_res_ratio: [set]Sensor synchronization resolution
  *                        ratio register.
  *
  * @param  *ctx   read / write interface definitions.
  * @param  val    change the values of rr in S4S_RR.
  *
  */
int32_t lsm6dsox_s4s_res_ratio_set(stmdev_ctx_t *ctx,
                                  lsm6dsox_s4s_res_ratio_t val)
{
  lsm6dsox_s4s_rr_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_S4S_RR, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.rr = (uint8_t)val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_S4S_RR, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  s4s_res_ratio: [get]Sensor synchronization resolution
  *                        ratio register.
  *
  * @param  *ctx   read / write interface definitions
  * @param  val    get the values of rr in S4S_RR
  *
  */
int32_t lsm6dsox_s4s_res_ratio_get(stmdev_ctx_t *ctx,
                                  lsm6dsox_s4s_res_ratio_t *val)
{
  lsm6dsox_s4s_rr_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_S4S_RR, (uint8_t*)&reg, 1);
  switch (reg.rr) {
    case LSM6DSOX_S4S_DT_RES_11:
      *val = LSM6DSOX_S4S_DT_RES_11;
      break;
    case LSM6DSOX_S4S_DT_RES_12:
      *val = LSM6DSOX_S4S_DT_RES_12;
      break;
    case LSM6DSOX_S4S_DT_RES_13:
      *val = LSM6DSOX_S4S_DT_RES_13;
      break;
    case LSM6DSOX_S4S_DT_RES_14:
      *val = LSM6DSOX_S4S_DT_RES_14;
      break;
    default:
      *val = LSM6DSOX_S4S_DT_RES_11;
      break;
  }
  return ret;
}

/**
  * @brief  s4s_command: [set] s4s master command.
  *
  * @param  *ctx   read / write interface definitions.
  * @param  val    change the values of S4S_ST_CMD_CODE.
  *
  */
int32_t lsm6dsox_s4s_command_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_s4s_st_cmd_code_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_S4S_ST_CMD_CODE, (uint8_t*)&reg, 1);

  if (ret == 0) {
    reg.s4s_st_cmd_code = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_S4S_ST_CMD_CODE, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  s4s_command: [get] s4s master command.
  *
  * @param  *ctx   read / write interface definitions.
  * @param  val    get the values of S4S_ST_CMD_CODE.
  *
  */
int32_t lsm6dsox_s4s_command_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_s4s_st_cmd_code_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_S4S_ST_CMD_CODE, (uint8_t*)&reg, 1);
  *val = reg.s4s_st_cmd_code;

  return ret;
}

/**
  * @brief  s4s_dt: [set] S4S DT register.
  *
  * @param  *ctx   read / write interface definitions.
  * @param  val    change the values of S4S_DT_REG.
  *
  */
int32_t lsm6dsox_s4s_dt_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm6dsox_s4s_dt_reg_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_S4S_DT_REG, (uint8_t*)&reg, 1);
  if (ret == 0) {
    reg.dt = val;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_S4S_DT_REG, (uint8_t*)&reg, 1);
  }
  return ret;
}

/**
  * @brief  s4s_dt: [get] S4S DT register.
  *
  * @param  *ctx   read / write interface definitions.
  * @param  val    get the values of S4S_DT_REG.
  *
  */
int32_t lsm6dsox_s4s_dt_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lsm6dsox_s4s_dt_reg_t reg;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_S4S_DT_REG, (uint8_t*)&reg, 1);
  *val = reg.dt;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Basic configuration
  * @brief     This section groups all the functions concerning
  *            device basic configuration.
  * @{
  *
  */

/**
  * @brief  Device "Who am I".[get]
  *
  * @param  ctx          communication interface handler. Use NULL to ingnore
  *                      this interface.(ptr)
  * @param  aux_ctx      auxiliary communication interface handler. Use NULL
  *                      to ingnore this interface.(ptr)
  * @param  val          ID values read from the two interfaces. ID values
  *                      will be the same.(ptr)
  *
  */
int32_t lsm6dsox_id_get(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                        lsm6dsox_id_t *val)
{
  int32_t ret = 0;

  if (ctx != NULL){
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_WHO_AM_I,
                              (uint8_t*)&(val->ui), 1);
  }
  if (aux_ctx != NULL){
    if (ret == 0) {
      ret = lsm6dsox_read_reg(aux_ctx, LSM6DSOX_SPI2_WHO_AM_I,
                              (uint8_t*)&(val->aux), 1);
    }
  }
  return ret;
}

/**
  * @brief  Re-initialize the device.[set]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          re-initialization mode. Refer to datasheet
  *                      and application note for more information
  *                      about differencies beetween boot and sw_reset
  *                      procedure.
  *
  */
int32_t lsm6dsox_init_set(stmdev_ctx_t *ctx, lsm6dsox_init_t val)
{
  lsm6dsox_emb_func_init_a_t emb_func_init_a;
  lsm6dsox_emb_func_init_b_t emb_func_init_b;
  lsm6dsox_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_INIT_B,
                            (uint8_t*)&emb_func_init_b, 1);
  }
  if (ret == 0) {
    emb_func_init_b.fifo_compr_init = (uint8_t)val
                                      & ( (uint8_t)LSM6DSOX_FIFO_COMP >> 2 );
    emb_func_init_b.fsm_init = (uint8_t)val
                               & ( (uint8_t)LSM6DSOX_FSM >> 3 );
    emb_func_init_b.mlc_init = (uint8_t)val
                               & ( (uint8_t)LSM6DSOX_MLC >> 4 );
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_INIT_B,
                             (uint8_t*)&emb_func_init_b, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_INIT_A,
                            (uint8_t*)&emb_func_init_a, 1);
  }
  if (ret == 0) {
    emb_func_init_a.step_det_init = ( (uint8_t)val
                                       & (uint8_t)LSM6DSOX_PEDO ) >> 5;
    emb_func_init_a.tilt_init = ( (uint8_t)val
                                  & (uint8_t)LSM6DSOX_TILT ) >> 6;
    emb_func_init_a.sig_mot_init = ( (uint8_t)val
                                     & (uint8_t)LSM6DSOX_SMOTION ) >> 7;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_INIT_A,
                             (uint8_t*)&emb_func_init_a, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  if ( ( (val == LSM6DSOX_BOOT) || (val == LSM6DSOX_RESET) ) && (ret == 0) ) {
    ctrl3_c.boot = (uint8_t)val & (uint8_t)LSM6DSOX_BOOT;
    ctrl3_c.sw_reset = ( (uint8_t)val & (uint8_t)LSM6DSOX_RESET) >> 1;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  if ( ( val == LSM6DSOX_DRV_RDY )
       && ( (ctrl3_c.bdu == PROPERTY_DISABLE)
            || (ctrl3_c.if_inc == PROPERTY_DISABLE) ) && (ret == 0) ) {
    ctrl3_c.bdu = PROPERTY_ENABLE;
    ctrl3_c.if_inc = PROPERTY_ENABLE;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }

  return ret;
}

/**
  * @brief  Configures the bus operating mode.[set]
  *
  * @param  ctx          communication interface handler. Use NULL to ingnore
  *                      this interface.(ptr)
  * @param  aux_ctx      auxiliary communication interface handler. Use NULL
  *                      to ingnore this interface.(ptr)
  * @param  val          configures the bus operating mode for both the
  *                      main and the auxiliary interface.
  *
  */
int32_t lsm6dsox_bus_mode_set(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                              lsm6dsox_bus_mode_t val)
{
  lsm6dsox_spi2_ctrl1_ois_t spi2_ctrl1_ois;
  lsm6dsox_i3c_bus_avb_t i3c_bus_avb;
  lsm6dsox_ctrl9_xl_t ctrl9_xl;
  lsm6dsox_ctrl3_c_t ctrl3_c;
  lsm6dsox_ctrl4_c_t ctrl4_c;
  uint8_t bit_val;
  int32_t ret;

  ret = 0;

  if (aux_ctx != NULL) {
    ret = lsm6dsox_read_reg(aux_ctx, LSM6DSOX_SPI2_CTRL1_OIS,
                            (uint8_t*)&spi2_ctrl1_ois, 1);

    bit_val = ( (uint8_t)val.aux_bus_md & 0x04U ) >> 2;
    if ( ( ret == 0 ) && ( spi2_ctrl1_ois.sim_ois != bit_val ) ) {
      spi2_ctrl1_ois.sim_ois = bit_val;
      ret = lsm6dsox_write_reg(aux_ctx, LSM6DSOX_SPI2_CTRL1_OIS,
                               (uint8_t*)&spi2_ctrl1_ois, 1);
    }
  }

  if (ctx != NULL) {
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL,
                              (uint8_t*)&ctrl9_xl, 1);
    }

    bit_val = ((uint8_t)val.ui_bus_md & 0x04U) >> 2;
    if ( ( ret == 0 ) && ( ctrl9_xl.i3c_disable != bit_val ) ) {
      ctrl9_xl.i3c_disable = bit_val;
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL9_XL,
                               (uint8_t*)&ctrl9_xl, 1);
    }

    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_I3C_BUS_AVB,
                              (uint8_t*)&i3c_bus_avb, 1);
    }

    bit_val = ((uint8_t)val.ui_bus_md & 0x30U) >> 4;
    if ( ( ret == 0 ) && ( i3c_bus_avb.i3c_bus_avb_sel != bit_val ) ) {
      i3c_bus_avb.i3c_bus_avb_sel = bit_val;
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_I3C_BUS_AVB,
                               (uint8_t*)&i3c_bus_avb, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C,
                              (uint8_t*)&ctrl4_c, 1);
    }
    bit_val = ( (uint8_t)val.ui_bus_md & 0x02U ) >> 1;
    if ( ( ret == 0 ) && ( ctrl4_c.i2c_disable != bit_val ) ) {
      ctrl4_c.i2c_disable = bit_val;
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL4_C,
                               (uint8_t*)&ctrl4_c, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C,
                              (uint8_t*)&ctrl3_c, 1);
    }
    bit_val = (uint8_t)val.ui_bus_md & 0x01U;
    if ( ( ret == 0 ) && ( ctrl3_c.sim != bit_val ) ) {
      ctrl3_c.sim = bit_val;
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL3_C,
                               (uint8_t*)&ctrl3_c, 1);
    }
  }

  return ret;

}

/**
  * @brief  Get the bus operating mode.[get]
  *
  * @param  ctx          communication interface handler. Use NULL to ingnore
  *                      this interface.(ptr)
  * @param  aux_ctx      auxiliary communication interface handler. Use NULL
  *                      to ingnore this interface.(ptr)
  * @param  val          retrieves the bus operating mode for both the main
  *                      and the auxiliary interface.(ptr)
  *
  */
int32_t lsm6dsox_bus_mode_get(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                              lsm6dsox_bus_mode_t *val)
{
  lsm6dsox_spi2_ctrl1_ois_t spi2_ctrl1_ois;
  lsm6dsox_i3c_bus_avb_t i3c_bus_avb;
  lsm6dsox_ctrl9_xl_t ctrl9_xl;
  lsm6dsox_ctrl3_c_t ctrl3_c;
  lsm6dsox_ctrl4_c_t ctrl4_c;

  int32_t ret = 0;

  if (aux_ctx != NULL) {
    ret = lsm6dsox_read_reg(aux_ctx, LSM6DSOX_SPI2_CTRL1_OIS,
                            (uint8_t*)&spi2_ctrl1_ois, 1);
    switch ( spi2_ctrl1_ois.sim_ois ) {
      case LSM6DSOX_SPI_4W_AUX:
        val->aux_bus_md = LSM6DSOX_SPI_4W_AUX;
        break;
      case LSM6DSOX_SPI_3W_AUX:
        val->aux_bus_md = LSM6DSOX_SPI_3W_AUX;
        break;
      default:
        val->aux_bus_md = LSM6DSOX_SPI_4W_AUX;
        break;
    }
  }

  if (ctx != NULL) {
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL9_XL,
                              (uint8_t*)&ctrl9_xl, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_I3C_BUS_AVB,
                              (uint8_t*)&i3c_bus_avb, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C,
                              (uint8_t*)&ctrl4_c, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C,
                              (uint8_t*)&ctrl3_c, 1);

      switch ( ( i3c_bus_avb.i3c_bus_avb_sel << 4 ) &
               ( ctrl9_xl.i3c_disable << 2 ) &
               ( ctrl4_c.i2c_disable << 1) & ctrl3_c.sim ) {
        case LSM6DSOX_SEL_BY_HW:
          val->ui_bus_md = LSM6DSOX_SEL_BY_HW;
          break;
        case LSM6DSOX_SPI_4W:
          val->ui_bus_md = LSM6DSOX_SPI_4W;
          break;
        case LSM6DSOX_SPI_3W:
          val->ui_bus_md = LSM6DSOX_SPI_3W;
          break;
        case LSM6DSOX_I2C:
          val->ui_bus_md = LSM6DSOX_I2C;
          break;
        case LSM6DSOX_I3C_T_50us:
          val->ui_bus_md = LSM6DSOX_I3C_T_50us;
          break;
        case LSM6DSOX_I3C_T_2us:
          val->ui_bus_md = LSM6DSOX_I3C_T_2us;
          break;
        case LSM6DSOX_I3C_T_1ms:
          val->ui_bus_md = LSM6DSOX_I3C_T_1ms;
          break;
        case LSM6DSOX_I3C_T_25ms:
          val->ui_bus_md = LSM6DSOX_I3C_T_25ms;
          break;
        default:
          val->ui_bus_md = LSM6DSOX_SEL_BY_HW;
          break;
      }
    }
  }
  return ret;
}

/**
  * @brief  Get the status of the device.[get]
  *
  * @param  ctx          communication interface handler. Use NULL to ingnore
  *                      this interface.(ptr)
  * @param  aux_ctx      auxiliary communication interface handler. Use NULL
  *                      to ingnore this interface.(ptr)
  * @param  val          the status of the device.(ptr)
  *
  */
int32_t lsm6dsox_status_get(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                            lsm6dsox_status_t *val)
{
  lsm6dsox_spi2_status_reg_ois_t spi2_status_reg_ois;
  lsm6dsox_ui_status_reg_ois_t   ui_status_reg_ois;
  lsm6dsox_status_reg_t          status_reg;
  lsm6dsox_ctrl3_c_t             ctrl3_c;
  int32_t                        ret;

  ret = 0;

  if (aux_ctx != NULL){
    ret = lsm6dsox_read_reg(aux_ctx, LSM6DSOX_SPI2_STATUS_REG_OIS,
                            (uint8_t*)&spi2_status_reg_ois, 1);
    val->ois_drdy_xl        = spi2_status_reg_ois.xlda;
    val->ois_drdy_g         = spi2_status_reg_ois.gda;
    val->ois_gyro_settling  = spi2_status_reg_ois.gyro_settling;
  }

  if (ctx != NULL){
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
    val->sw_reset = ctrl3_c.sw_reset;
    val->boot = ctrl3_c.boot;

    if ( (ret == 0) && ( ctrl3_c.sw_reset == PROPERTY_DISABLE ) &&
         ( ctrl3_c.boot == PROPERTY_DISABLE ) ) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_STATUS_REG,
                              (uint8_t*)&status_reg, 1);
      val->drdy_xl   = status_reg.xlda;
      val->drdy_g    = status_reg.gda;
      val->drdy_temp = status_reg.tda;
    }
    if (aux_ctx == NULL){
      if (ret == 0) {
        ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_STATUS_REG_OIS,
                                (uint8_t*)&ui_status_reg_ois, 1);
        val->ois_drdy_xl       = ui_status_reg_ois.xlda;
        val->ois_drdy_g        = ui_status_reg_ois.gda;
        val->ois_gyro_settling = ui_status_reg_ois.gyro_settling;
      }
    }
  }
  return ret;
}

/**
  * @brief  Electrical pin configuration.[set]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the electrical settings for the configurable
  *                      pins.
  *
  */
int32_t lsm6dsox_pin_conf_set(stmdev_ctx_t *ctx, lsm6dsox_pin_conf_t val)
{
  lsm6dsox_i3c_bus_avb_t i3c_bus_avb;
  lsm6dsox_pin_ctrl_t pin_ctrl;
  lsm6dsox_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PIN_CTRL, (uint8_t*)&pin_ctrl, 1);
  if (ret == 0) {
    pin_ctrl.ois_pu_dis = ~val.aux_sdo_ocs_pull_up;
    pin_ctrl.sdo_pu_en  = val.sdo_sa0_pull_up;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PIN_CTRL, (uint8_t*)&pin_ctrl, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  if (ret == 0) {
    ctrl3_c.pp_od = ~val.int1_int2_push_pull;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_I3C_BUS_AVB,
                            (uint8_t*)&i3c_bus_avb, 1);
  }
  if (ret == 0) {
    i3c_bus_avb.pd_dis_int1 = ~val.int1_pull_down;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_I3C_BUS_AVB,
                             (uint8_t*)&i3c_bus_avb, 1);
  }
  return ret;
}

/**
  * @brief  Electrical pin configuration.[get]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the electrical settings for the configurable
  *                      pins.(ptr)
  *
  */
int32_t lsm6dsox_pin_conf_get(stmdev_ctx_t *ctx, lsm6dsox_pin_conf_t *val)
{
  lsm6dsox_i3c_bus_avb_t i3c_bus_avb;
  lsm6dsox_pin_ctrl_t pin_ctrl;
  lsm6dsox_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PIN_CTRL, (uint8_t*)&pin_ctrl, 1);
  if (ret == 0) {
    val->aux_sdo_ocs_pull_up = ~pin_ctrl.ois_pu_dis;
    val->aux_sdo_ocs_pull_up =  pin_ctrl.sdo_pu_en;
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  if (ret == 0) {
    val->int1_int2_push_pull = ~ctrl3_c.pp_od;
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_I3C_BUS_AVB,
                            (uint8_t*)&i3c_bus_avb, 1);
  }
  if (ret == 0) {
    val->int1_pull_down = ~i3c_bus_avb.pd_dis_int1;
  }
  return ret;
}

/**
  * @brief  Interrupt pins hardware signal configuration.[set]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the pins hardware signal settings.
  *
  */
int32_t lsm6dsox_interrupt_mode_set(stmdev_ctx_t *ctx,
                                    lsm6dsox_int_mode_t val)
{
  lsm6dsox_tap_cfg0_t tap_cfg0;
  lsm6dsox_page_rw_t page_rw;
  lsm6dsox_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  if (ret == 0) {
    ctrl3_c.h_lactive = val.active_low;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  }
  if (ret == 0) {
  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*) &tap_cfg0, 1);
  }
  if (ret == 0) {
    tap_cfg0.lir = val.base_latched;
    tap_cfg0.int_clr_on_read = val.base_latched | val.emb_latched;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*) &tap_cfg0, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    page_rw.emb_func_lir = val.emb_latched;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Interrupt pins hardware signal configuration.[get]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the pins hardware signal settings.(ptr)
  *
  */
int32_t lsm6dsox_interrupt_mode_get(stmdev_ctx_t *ctx,
                                    lsm6dsox_int_mode_t *val)
{
  lsm6dsox_tap_cfg0_t tap_cfg0;
  lsm6dsox_page_rw_t page_rw;
  lsm6dsox_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL3_C, (uint8_t*)&ctrl3_c, 1);
  if (ret == 0) {
    ctrl3_c.h_lactive = val->active_low;
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG0, (uint8_t*) &tap_cfg0, 1);
  }
  if (ret == 0) {
    tap_cfg0.lir = val->base_latched;
    tap_cfg0.int_clr_on_read = val->base_latched | val->emb_latched;
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    page_rw.emb_func_lir = val->emb_latched;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_PAGE_RW, (uint8_t*) &page_rw, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  return ret;
}

/**
  * @brief  Route interrupt signals on int1 pin.[set]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the signals to route on int1 pin.
  *
  */
int32_t lsm6dsox_pin_int1_route_set(stmdev_ctx_t *ctx,
                                    lsm6dsox_pin_int1_route_t val)
{
  lsm6dsox_pin_int2_route_t  pin_int2_route;
  lsm6dsox_emb_func_int1_t   emb_func_int1;
  lsm6dsox_fsm_int1_a_t      fsm_int1_a;
  lsm6dsox_fsm_int1_b_t      fsm_int1_b;
  lsm6dsox_int1_ctrl_t       int1_ctrl;
  lsm6dsox_int2_ctrl_t       int2_ctrl;
  lsm6dsox_mlc_int1_t        mlc_int1;
  lsm6dsox_tap_cfg2_t        tap_cfg2;
  lsm6dsox_md2_cfg_t         md2_cfg;
  lsm6dsox_md1_cfg_t         md1_cfg;
  lsm6dsox_ctrl4_c_t         ctrl4_c;
  int32_t                    ret;

  int1_ctrl.int1_drdy_xl   = val.drdy_xl;
  int1_ctrl.int1_drdy_g    = val.drdy_g;
  int1_ctrl.int1_boot      = val.boot;
  int1_ctrl.int1_fifo_th   = val.fifo_th;
  int1_ctrl.int1_fifo_ovr  = val.fifo_ovr;
  int1_ctrl.int1_fifo_full = val.fifo_full;
  int1_ctrl.int1_cnt_bdr   = val.fifo_bdr;
  int1_ctrl.den_drdy_flag  = val.den_flag;

  md1_cfg.int1_shub         = val.sh_endop;
  md1_cfg.int1_6d           = val.six_d;
  md1_cfg.int1_double_tap   = val.double_tap;
  md1_cfg.int1_ff           = val.free_fall;
  md1_cfg.int1_wu           = val.wake_up;
  md1_cfg.int1_single_tap   = val.single_tap;
  md1_cfg.int1_sleep_change = val.sleep_change;

  emb_func_int1.int1_step_detector = val.step_detector;
  emb_func_int1.int1_tilt          = val.tilt;
  emb_func_int1.int1_sig_mot       = val.sig_mot;
  emb_func_int1.int1_fsm_lc        = val.fsm_lc;

  fsm_int1_a.int1_fsm1 = val.fsm1;
  fsm_int1_a.int1_fsm2 = val.fsm2;
  fsm_int1_a.int1_fsm3 = val.fsm3;
  fsm_int1_a.int1_fsm4 = val.fsm4;
  fsm_int1_a.int1_fsm5 = val.fsm5;
  fsm_int1_a.int1_fsm6 = val.fsm6;
  fsm_int1_a.int1_fsm7 = val.fsm7;
  fsm_int1_a.int1_fsm8 = val.fsm8;

  fsm_int1_b.int1_fsm9  = val.fsm9 ;
  fsm_int1_b.int1_fsm10 = val.fsm10;
  fsm_int1_b.int1_fsm11 = val.fsm11;
  fsm_int1_b.int1_fsm12 = val.fsm12;
  fsm_int1_b.int1_fsm13 = val.fsm13;
  fsm_int1_b.int1_fsm14 = val.fsm14;
  fsm_int1_b.int1_fsm15 = val.fsm15;
  fsm_int1_b.int1_fsm16 = val.fsm16;

  mlc_int1.int1_mlc1 = val.mlc1;
  mlc_int1.int1_mlc2 = val.mlc2;
  mlc_int1.int1_mlc3 = val.mlc3;
  mlc_int1.int1_mlc4 = val.mlc4;
  mlc_int1.int1_mlc5 = val.mlc5;
  mlc_int1.int1_mlc6 = val.mlc6;
  mlc_int1.int1_mlc7 = val.mlc7;
  mlc_int1.int1_mlc8 = val.mlc8;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  if (ret == 0) {
    if( ( val.drdy_temp | val.timestamp ) != PROPERTY_DISABLE) {
      ctrl4_c.int2_on_int1 = PROPERTY_ENABLE;
    }
    else{
      ctrl4_c.int2_on_int1 = PROPERTY_DISABLE;
    }
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  }

  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_MLC_INT1,
                            (uint8_t*)&mlc_int1, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_INT1,
                            (uint8_t*)&emb_func_int1, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FSM_INT1_A,
                            (uint8_t*)&fsm_int1_a, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FSM_INT1_B,
                            (uint8_t*)&fsm_int1_b, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  if (ret == 0) {
    if ( ( emb_func_int1.int1_fsm_lc
         | emb_func_int1.int1_sig_mot
         | emb_func_int1.int1_step_detector
         | emb_func_int1.int1_tilt
         | fsm_int1_a.int1_fsm1
         | fsm_int1_a.int1_fsm2
         | fsm_int1_a.int1_fsm3
         | fsm_int1_a.int1_fsm4
         | fsm_int1_a.int1_fsm5
         | fsm_int1_a.int1_fsm6
         | fsm_int1_a.int1_fsm7
         | fsm_int1_a.int1_fsm8
         | fsm_int1_b.int1_fsm9
         | fsm_int1_b.int1_fsm10
         | fsm_int1_b.int1_fsm11
         | fsm_int1_b.int1_fsm12
         | fsm_int1_b.int1_fsm13
         | fsm_int1_b.int1_fsm14
         | fsm_int1_b.int1_fsm15
         | fsm_int1_b.int1_fsm16
         | mlc_int1.int1_mlc1
         | mlc_int1.int1_mlc2
         | mlc_int1.int1_mlc3
         | mlc_int1.int1_mlc4
         | mlc_int1.int1_mlc5
         | mlc_int1.int1_mlc6
         | mlc_int1.int1_mlc7
         | mlc_int1.int1_mlc8) != PROPERTY_DISABLE){
      md1_cfg.int1_emb_func = PROPERTY_ENABLE;
    }
    else{
      md1_cfg.int1_emb_func = PROPERTY_DISABLE;
    }
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_INT1_CTRL,
                            (uint8_t*)&int1_ctrl, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_MD1_CFG, (uint8_t*)&md1_cfg, 1);
  }

  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_INT2_CTRL, (uint8_t*)&int2_ctrl, 1);
  }
  if (ret == 0) {
    int2_ctrl.int2_drdy_temp = val.drdy_temp;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_INT2_CTRL, (uint8_t*)&int2_ctrl, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MD2_CFG, (uint8_t*)&md2_cfg, 1);
  }
  if (ret == 0) {
    md2_cfg.int2_timestamp = val.timestamp;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_MD2_CFG, (uint8_t*)&md2_cfg, 1);
  }

  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG2, (uint8_t*) &tap_cfg2, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_pin_int2_route_get(ctx, NULL, &pin_int2_route);
  }
  if (ret == 0) {
    if ( ( pin_int2_route.fifo_bdr
         | pin_int2_route.drdy_g
         | pin_int2_route.drdy_temp
         | pin_int2_route.drdy_xl
         | pin_int2_route.fifo_full
         | pin_int2_route.fifo_ovr
         | pin_int2_route.fifo_th
         | pin_int2_route.six_d
         | pin_int2_route.double_tap
         | pin_int2_route.free_fall
         | pin_int2_route.wake_up
         | pin_int2_route.single_tap
         | pin_int2_route.sleep_change
         | int1_ctrl.den_drdy_flag
         | int1_ctrl.int1_boot
         | int1_ctrl.int1_cnt_bdr
         | int1_ctrl.int1_drdy_g
         | int1_ctrl.int1_drdy_xl
         | int1_ctrl.int1_fifo_full
         | int1_ctrl.int1_fifo_ovr
         | int1_ctrl.int1_fifo_th
         | md1_cfg.int1_shub
         | md1_cfg.int1_6d
         | md1_cfg.int1_double_tap
         | md1_cfg.int1_ff
         | md1_cfg.int1_wu
         | md1_cfg.int1_single_tap
         | md1_cfg.int1_sleep_change) != PROPERTY_DISABLE) {
      tap_cfg2.interrupts_enable = PROPERTY_ENABLE;
    }
    else{
      tap_cfg2.interrupts_enable = PROPERTY_DISABLE;
    }
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_CFG2, (uint8_t*) &tap_cfg2, 1);
  }
  return ret;
}

/**
  * @brief  Route interrupt signals on int1 pin.[get]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the signals that are routed on int1 pin.(ptr)
  *
  */
int32_t lsm6dsox_pin_int1_route_get(stmdev_ctx_t *ctx,
                                    lsm6dsox_pin_int1_route_t *val)
{
  lsm6dsox_emb_func_int1_t   emb_func_int1;
  lsm6dsox_fsm_int1_a_t      fsm_int1_a;
  lsm6dsox_fsm_int1_b_t      fsm_int1_b;
  lsm6dsox_int1_ctrl_t       int1_ctrl;
  lsm6dsox_int2_ctrl_t       int2_ctrl;
  lsm6dsox_mlc_int1_t        mlc_int1;
  lsm6dsox_md2_cfg_t         md2_cfg;
  lsm6dsox_md1_cfg_t         md1_cfg;
  lsm6dsox_ctrl4_c_t         ctrl4_c;
  int32_t                    ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MLC_INT1,
                            (uint8_t*)&mlc_int1, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_INT1,
                           (uint8_t*)&emb_func_int1, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FSM_INT1_A,
                           (uint8_t*)&fsm_int1_a, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FSM_INT1_B,
                           (uint8_t*)&fsm_int1_b, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_INT1_CTRL,
                           (uint8_t*)&int1_ctrl, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MD1_CFG, (uint8_t*)&md1_cfg, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
  }
  if (ctrl4_c.int2_on_int1 == PROPERTY_ENABLE){
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_INT2_CTRL, (uint8_t*)&int2_ctrl, 1);
      val->drdy_temp = int2_ctrl.int2_drdy_temp;
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MD2_CFG, (uint8_t*)&md2_cfg, 1);
      val->timestamp = md2_cfg.int2_timestamp;
    }
  }
  else {
    val->drdy_temp = PROPERTY_DISABLE;
    val->timestamp = PROPERTY_DISABLE;
  }

  val->drdy_xl   = int1_ctrl.int1_drdy_xl;
  val->drdy_g    = int1_ctrl.int1_drdy_g;
  val->boot      = int1_ctrl.int1_boot;
  val->fifo_th   = int1_ctrl.int1_fifo_th;
  val->fifo_ovr  = int1_ctrl.int1_fifo_ovr;
  val->fifo_full = int1_ctrl.int1_fifo_full;
  val->fifo_bdr  = int1_ctrl.int1_cnt_bdr;
  val->den_flag  = int1_ctrl.den_drdy_flag;

  val->sh_endop     = md1_cfg.int1_shub;
  val->six_d        = md1_cfg.int1_6d;
  val->double_tap   = md1_cfg.int1_double_tap;
  val->free_fall    = md1_cfg.int1_ff;
  val->wake_up      = md1_cfg.int1_wu;
  val->single_tap   = md1_cfg.int1_single_tap;
  val->sleep_change = md1_cfg.int1_sleep_change;

  val->step_detector = emb_func_int1.int1_step_detector;
  val->tilt          = emb_func_int1.int1_tilt;
  val->sig_mot       = emb_func_int1.int1_sig_mot;
  val->fsm_lc        = emb_func_int1.int1_fsm_lc;

  val->fsm1 = fsm_int1_a.int1_fsm1;
  val->fsm2 = fsm_int1_a.int1_fsm2;
  val->fsm3 = fsm_int1_a.int1_fsm3;
  val->fsm4 = fsm_int1_a.int1_fsm4;
  val->fsm5 = fsm_int1_a.int1_fsm5;
  val->fsm6 = fsm_int1_a.int1_fsm6;
  val->fsm7 = fsm_int1_a.int1_fsm7;
  val->fsm8 = fsm_int1_a.int1_fsm8;

  val->fsm9  = fsm_int1_b.int1_fsm9;
  val->fsm10 = fsm_int1_b.int1_fsm10;
  val->fsm11 = fsm_int1_b.int1_fsm11;
  val->fsm12 = fsm_int1_b.int1_fsm12;
  val->fsm13 = fsm_int1_b.int1_fsm13;
  val->fsm14 = fsm_int1_b.int1_fsm14;
  val->fsm15 = fsm_int1_b.int1_fsm15;
  val->fsm16 = fsm_int1_b.int1_fsm16;

  val->mlc1 = mlc_int1.int1_mlc1;
  val->mlc2 = mlc_int1.int1_mlc2;
  val->mlc3 = mlc_int1.int1_mlc3;
  val->mlc4 = mlc_int1.int1_mlc4;
  val->mlc5 = mlc_int1.int1_mlc5;
  val->mlc6 = mlc_int1.int1_mlc6;
  val->mlc7 = mlc_int1.int1_mlc7;
  val->mlc8 = mlc_int1.int1_mlc8;

  return ret;
}

/**
  * @brief  Route interrupt signals on int2 pin.[set]
  *
  * @param  ctx          communication interface handler. Use NULL to ingnore
  *                      this interface.(ptr)
  * @param  aux_ctx      auxiliary communication interface handler. Use NULL
  *                      to ingnore this interface.(ptr)
  * @param  val          the signals to route on int2 pin.
  *
  */
int32_t lsm6dsox_pin_int2_route_set(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                                    lsm6dsox_pin_int2_route_t val)
{
  lsm6dsox_pin_int1_route_t pin_int1_route;
  lsm6dsox_emb_func_int2_t  emb_func_int2;
  lsm6dsox_spi2_int_ois_t   spi2_int_ois;
  lsm6dsox_fsm_int2_a_t     fsm_int2_a;
  lsm6dsox_fsm_int2_b_t     fsm_int2_b;
  lsm6dsox_int2_ctrl_t      int2_ctrl;
  lsm6dsox_mlc_int2_t       mlc_int2;
  lsm6dsox_tap_cfg2_t       tap_cfg2;
  lsm6dsox_md2_cfg_t        md2_cfg;
  lsm6dsox_ctrl4_c_t        ctrl4_c;
  int32_t                   ret;

  ret = 0;

  if( aux_ctx != NULL ) {
    ret = lsm6dsox_read_reg(aux_ctx, LSM6DSOX_SPI2_INT_OIS,
                            (uint8_t*)&spi2_int_ois, 1);
    if (ret == 0) {
      spi2_int_ois.int2_drdy_ois = val.drdy_ois;
      ret = lsm6dsox_write_reg(aux_ctx, LSM6DSOX_SPI2_INT_OIS,
                               (uint8_t*)&spi2_int_ois, 1);
    }
  }

  if( ctx != NULL ) {
    int2_ctrl.int2_drdy_xl   = val.drdy_xl;
    int2_ctrl.int2_drdy_g    = val.drdy_g;
    int2_ctrl.int2_drdy_temp = val.drdy_temp;
    int2_ctrl.int2_fifo_th   = val.fifo_th;
    int2_ctrl.int2_fifo_ovr  = val.fifo_ovr;
    int2_ctrl.int2_fifo_full = val.fifo_full;
    int2_ctrl.int2_cnt_bdr   = val.fifo_bdr;

    md2_cfg.int2_timestamp    = val.timestamp;
    md2_cfg.int2_6d           = val.six_d;
    md2_cfg.int2_double_tap   = val.double_tap;
    md2_cfg.int2_ff           = val.free_fall;
    md2_cfg.int2_wu           = val.wake_up;
    md2_cfg.int2_single_tap   = val.single_tap;
    md2_cfg.int2_sleep_change = val.sleep_change;

    emb_func_int2. int2_step_detector = val.step_detector;
    emb_func_int2.int2_tilt           = val.tilt;
    emb_func_int2.int2_fsm_lc         = val.fsm_lc;

    fsm_int2_a.int2_fsm1 = val.fsm1;
    fsm_int2_a.int2_fsm2 = val.fsm2;
    fsm_int2_a.int2_fsm3 = val.fsm3;
    fsm_int2_a.int2_fsm4 = val.fsm4;
    fsm_int2_a.int2_fsm5 = val.fsm5;
    fsm_int2_a.int2_fsm6 = val.fsm6;
    fsm_int2_a.int2_fsm7 = val.fsm7;
    fsm_int2_a.int2_fsm8 = val.fsm8;

    fsm_int2_b.int2_fsm9  = val.fsm9 ;
    fsm_int2_b.int2_fsm10 = val.fsm10;
    fsm_int2_b.int2_fsm11 = val.fsm11;
    fsm_int2_b.int2_fsm12 = val.fsm12;
    fsm_int2_b.int2_fsm13 = val.fsm13;
    fsm_int2_b.int2_fsm14 = val.fsm14;
    fsm_int2_b.int2_fsm15 = val.fsm15;
    fsm_int2_b.int2_fsm16 = val.fsm16;

    mlc_int2.int2_mlc1 = val.mlc1;
    mlc_int2.int2_mlc2 = val.mlc2;
    mlc_int2.int2_mlc3 = val.mlc3;
    mlc_int2.int2_mlc4 = val.mlc4;
    mlc_int2.int2_mlc5 = val.mlc5;
    mlc_int2.int2_mlc6 = val.mlc6;
    mlc_int2.int2_mlc7 = val.mlc7;
    mlc_int2.int2_mlc8 = val.mlc8;

    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
      if (ret == 0) {
        if ( ( val.drdy_temp | val.timestamp ) != PROPERTY_DISABLE ) {
          ctrl4_c.int2_on_int1 = PROPERTY_DISABLE;
        }
        else{
          ctrl4_c.int2_on_int1 = PROPERTY_ENABLE;
        }
        ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
      }
    }

    if (ret == 0) {
      ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
    }
    if (ret == 0) {
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_MLC_INT2,
                              (uint8_t*)&mlc_int2, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_INT2,
                              (uint8_t*)&emb_func_int2, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FSM_INT2_A,
                              (uint8_t*)&fsm_int2_a, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FSM_INT2_B,
                              (uint8_t*)&fsm_int2_b, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
    }

    if (ret == 0) {
      if (( emb_func_int2.int2_fsm_lc
          | emb_func_int2.int2_sig_mot
          | emb_func_int2.int2_step_detector
          | emb_func_int2.int2_tilt
          | fsm_int2_a.int2_fsm1
          | fsm_int2_a.int2_fsm2
          | fsm_int2_a.int2_fsm3
          | fsm_int2_a.int2_fsm4
          | fsm_int2_a.int2_fsm5
          | fsm_int2_a.int2_fsm6
          | fsm_int2_a.int2_fsm7
          | fsm_int2_a.int2_fsm8
          | fsm_int2_b.int2_fsm9
          | fsm_int2_b.int2_fsm10
          | fsm_int2_b.int2_fsm11
          | fsm_int2_b.int2_fsm12
          | fsm_int2_b.int2_fsm13
          | fsm_int2_b.int2_fsm14
          | fsm_int2_b.int2_fsm15
          | fsm_int2_b.int2_fsm16
          | mlc_int2.int2_mlc1
          | mlc_int2.int2_mlc2
          | mlc_int2.int2_mlc3
          | mlc_int2.int2_mlc4
          | mlc_int2.int2_mlc5
          | mlc_int2.int2_mlc6
          | mlc_int2.int2_mlc7
          | mlc_int2.int2_mlc8)!= PROPERTY_DISABLE ){
        md2_cfg.int2_emb_func = PROPERTY_ENABLE;
      }
      else{
        md2_cfg.int2_emb_func = PROPERTY_DISABLE;
      }
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_INT2_CTRL,
                              (uint8_t*)&int2_ctrl, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_MD2_CFG, (uint8_t*)&md2_cfg, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_TAP_CFG2, (uint8_t*) &tap_cfg2, 1);
    }

    if (ret == 0) {
      ret = lsm6dsox_pin_int1_route_get(ctx, &pin_int1_route);
    }

    if (ret == 0) {
      if ( ( val.fifo_bdr
           | val.drdy_g
           | val.drdy_temp
           | val.drdy_xl
           | val.fifo_full
           | val.fifo_ovr
           | val.fifo_th
           | val.six_d
           | val.double_tap
           | val.free_fall
           | val.wake_up
           | val.single_tap
           | val.sleep_change
           | pin_int1_route.den_flag
           | pin_int1_route.boot
           | pin_int1_route.fifo_bdr
           | pin_int1_route.drdy_g
           | pin_int1_route.drdy_xl
           | pin_int1_route.fifo_full
           | pin_int1_route.fifo_ovr
           | pin_int1_route.fifo_th
           | pin_int1_route.six_d
           | pin_int1_route.double_tap
           | pin_int1_route.free_fall
           | pin_int1_route.wake_up
           | pin_int1_route.single_tap
           | pin_int1_route.sleep_change ) != PROPERTY_DISABLE) {
        tap_cfg2.interrupts_enable = PROPERTY_ENABLE;
      }
      else{
        tap_cfg2.interrupts_enable = PROPERTY_DISABLE;
      }
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_TAP_CFG2, (uint8_t*) &tap_cfg2, 1);
    }
  }
  return ret;
}

/**
  * @brief  Route interrupt signals on int2 pin.[get]
  *
  * @param  ctx          communication interface handler. Use NULL to ingnore
  *                      this interface.(ptr)
  * @param  aux_ctx      auxiliary communication interface handler. Use NULL
  *                      to ingnore this interface.(ptr)
  * @param  val          the signals that are routed on int2 pin.(ptr)
  *
  */
int32_t lsm6dsox_pin_int2_route_get(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                                    lsm6dsox_pin_int2_route_t *val)
{
  lsm6dsox_emb_func_int2_t  emb_func_int2;
  lsm6dsox_spi2_int_ois_t   spi2_int_ois;
  lsm6dsox_fsm_int2_a_t     fsm_int2_a;
  lsm6dsox_fsm_int2_b_t     fsm_int2_b;
  lsm6dsox_int2_ctrl_t      int2_ctrl;
  lsm6dsox_mlc_int2_t       mlc_int2;
  lsm6dsox_md2_cfg_t        md2_cfg;
  lsm6dsox_ctrl4_c_t        ctrl4_c;
  int32_t                   ret;

  ret = 0;

  if( aux_ctx != NULL ) {
    ret = lsm6dsox_read_reg(aux_ctx, LSM6DSOX_SPI2_INT_OIS,
                            (uint8_t*)&spi2_int_ois, 1);
    val->drdy_ois = spi2_int_ois.int2_drdy_ois;
  }

  if( ctx != NULL ) {
    if (ret == 0) {
     ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
    }
    if (ret == 0) {
        ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MLC_INT2,
                              (uint8_t*)&mlc_int2, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_INT2,
                             (uint8_t*)&emb_func_int2, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FSM_INT2_A,
                             (uint8_t*)&fsm_int2_a, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FSM_INT2_B,
                             (uint8_t*)&fsm_int2_b, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
    }
    if (ret == 0) {

      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_INT2_CTRL,
                             (uint8_t*)&int2_ctrl, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MD2_CFG,
                              (uint8_t*)&md2_cfg, 1);
    }

    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
    }
    if (ctrl4_c.int2_on_int1 == PROPERTY_DISABLE){
      if (ret == 0) {
        ret = lsm6dsox_read_reg(ctx, LSM6DSOX_INT2_CTRL,
                                (uint8_t*)&int2_ctrl, 1);
        val->drdy_temp = int2_ctrl.int2_drdy_temp;
      }
      if (ret == 0) {
        ret = lsm6dsox_read_reg(ctx, LSM6DSOX_MD2_CFG, (uint8_t*)&md2_cfg, 1);
        val->timestamp = md2_cfg.int2_timestamp;
      }
    }
    else {
      val->drdy_temp = PROPERTY_DISABLE;
      val->timestamp = PROPERTY_DISABLE;
    }

    val->drdy_xl   = int2_ctrl.int2_drdy_xl;
    val->drdy_g    = int2_ctrl.int2_drdy_g;
    val->drdy_temp = int2_ctrl.int2_drdy_temp;
    val->fifo_th   = int2_ctrl.int2_fifo_th;
    val->fifo_ovr  = int2_ctrl.int2_fifo_ovr;
    val->fifo_full = int2_ctrl.int2_fifo_full;
    val->fifo_bdr   = int2_ctrl.int2_cnt_bdr;

    val->timestamp    = md2_cfg.int2_timestamp;
    val->six_d        = md2_cfg.int2_6d;
    val->double_tap   = md2_cfg.int2_double_tap;
    val->free_fall    = md2_cfg.int2_ff;
    val->wake_up      = md2_cfg.int2_wu;
    val->single_tap   = md2_cfg.int2_single_tap;
    val->sleep_change = md2_cfg.int2_sleep_change;

    val->step_detector = emb_func_int2. int2_step_detector;
    val->tilt          = emb_func_int2.int2_tilt;
    val->fsm_lc        = emb_func_int2.int2_fsm_lc;

    val->fsm1 = fsm_int2_a.int2_fsm1;
    val->fsm2 = fsm_int2_a.int2_fsm2;
    val->fsm3 = fsm_int2_a.int2_fsm3;
    val->fsm4 = fsm_int2_a.int2_fsm4;
    val->fsm5 = fsm_int2_a.int2_fsm5;
    val->fsm6 = fsm_int2_a.int2_fsm6;
    val->fsm7 = fsm_int2_a.int2_fsm7;
    val->fsm8 = fsm_int2_a.int2_fsm8;

    val->fsm9  = fsm_int2_b.int2_fsm9;
    val->fsm10 = fsm_int2_b.int2_fsm10;
    val->fsm11 = fsm_int2_b.int2_fsm11;
    val->fsm12 = fsm_int2_b.int2_fsm12;
    val->fsm13 = fsm_int2_b.int2_fsm13;
    val->fsm14 = fsm_int2_b.int2_fsm14;
    val->fsm15 = fsm_int2_b.int2_fsm15;
    val->fsm16 = fsm_int2_b.int2_fsm16;

    val->mlc1 = mlc_int2.int2_mlc1;
    val->mlc2 = mlc_int2.int2_mlc2;
    val->mlc3 = mlc_int2.int2_mlc3;
    val->mlc4 = mlc_int2.int2_mlc4;
    val->mlc5 = mlc_int2.int2_mlc5;
    val->mlc6 = mlc_int2.int2_mlc6;
    val->mlc7 = mlc_int2.int2_mlc7;
    val->mlc8 = mlc_int2.int2_mlc8;
  }

  return ret;
}

/**
  * @brief  Get the status of all the interrupt sources.[get]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the status of all the interrupt sources.(ptr)
  *
  */
int32_t lsm6dsox_all_sources_get(stmdev_ctx_t *ctx,
                                 lsm6dsox_all_sources_t *val)
{
  lsm6dsox_emb_func_status_mainpage_t emb_func_status_mainpage;
  lsm6dsox_status_master_mainpage_t   status_master_mainpage;
  lsm6dsox_fsm_status_a_mainpage_t    fsm_status_a_mainpage;
  lsm6dsox_fsm_status_b_mainpage_t    fsm_status_b_mainpage;
  lsm6dsox_mlc_status_mainpage_t      mlc_status_mainpage;
  lsm6dsox_fifo_status1_t             fifo_status1;
  lsm6dsox_fifo_status2_t             fifo_status2;
  lsm6dsox_all_int_src_t              all_int_src;
  lsm6dsox_wake_up_src_t              wake_up_src;
  lsm6dsox_status_reg_t               status_reg;
  lsm6dsox_tap_src_t                  tap_src;
  lsm6dsox_d6d_src_t                  d6d_src;
  lsm6dsox_ctrl5_c_t                  ctrl5_c;
  uint8_t                             reg[12];
  int32_t                             ret;

  ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&ctrl5_c, 1);
  if (ret == 0) {
    ctrl5_c.rounding_status = PROPERTY_ENABLE;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&ctrl5_c, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_ALL_INT_SRC, reg, 12);
  }

  if (ret == 0) {
    bytecpy(( uint8_t*)&all_int_src, &reg[0]);
    bytecpy(( uint8_t*)&wake_up_src, &reg[1]);
    bytecpy(( uint8_t*)&tap_src, &reg[2]);
    bytecpy(( uint8_t*)&d6d_src, &reg[3]);
    bytecpy(( uint8_t*)&status_reg, &reg[4]);
    bytecpy(( uint8_t*)&emb_func_status_mainpage, &reg[5]);
    bytecpy(( uint8_t*)&fsm_status_a_mainpage, &reg[6]);
    bytecpy(( uint8_t*)&fsm_status_b_mainpage, &reg[7]);
    bytecpy(( uint8_t*)&mlc_status_mainpage, &reg[8]);
    bytecpy(( uint8_t*)&status_master_mainpage, &reg[9]);
    bytecpy(( uint8_t*)&fifo_status1, &reg[10]);
    bytecpy(( uint8_t*)&fifo_status2, &reg[11]);

    val->timestamp = all_int_src.timestamp_endcount;

    val->wake_up_z    = wake_up_src.z_wu;
    val->wake_up_y    = wake_up_src.y_wu;
    val->wake_up_x    = wake_up_src.x_wu;
    val->wake_up      = wake_up_src.wu_ia;
    val->sleep_state  = wake_up_src.sleep_state;
    val->free_fall    = wake_up_src.ff_ia;
    val->sleep_change = wake_up_src.sleep_change_ia;

    val->tap_x      = tap_src.x_tap;
    val->tap_y      = tap_src.y_tap;
    val->tap_z      = tap_src.z_tap;
    val->tap_sign   = tap_src.tap_sign;
    val->double_tap = tap_src.double_tap;
    val->single_tap = tap_src.single_tap;

    val->six_d_xl = d6d_src.xl;
    val->six_d_xh = d6d_src.xh;
    val->six_d_yl = d6d_src.yl;
    val->six_d_yh = d6d_src.yh;
    val->six_d_zl = d6d_src.zl;
    val->six_d_zh = d6d_src.zh;
    val->six_d    = d6d_src.d6d_ia;
    val->den_flag = d6d_src.den_drdy;

    val->drdy_xl   = status_reg.xlda;
    val->drdy_g    = status_reg.gda;
    val->drdy_temp = status_reg.tda;

    val->step_detector = emb_func_status_mainpage.is_step_det;
    val->tilt          = emb_func_status_mainpage.is_tilt;
    val->sig_mot       = emb_func_status_mainpage.is_sigmot;
    val->fsm_lc        = emb_func_status_mainpage.is_fsm_lc;

    val->fsm1 = fsm_status_a_mainpage.is_fsm1;
    val->fsm2 = fsm_status_a_mainpage.is_fsm2;
    val->fsm3 = fsm_status_a_mainpage.is_fsm3;
    val->fsm4 = fsm_status_a_mainpage.is_fsm4;
    val->fsm5 = fsm_status_a_mainpage.is_fsm5;
    val->fsm6 = fsm_status_a_mainpage.is_fsm6;
    val->fsm7 = fsm_status_a_mainpage.is_fsm7;
    val->fsm8 = fsm_status_a_mainpage.is_fsm8;

    val->fsm9  = fsm_status_b_mainpage.is_fsm9;
    val->fsm10 = fsm_status_b_mainpage.is_fsm10;
    val->fsm11 = fsm_status_b_mainpage.is_fsm11;
    val->fsm12 = fsm_status_b_mainpage.is_fsm12;
    val->fsm13 = fsm_status_b_mainpage.is_fsm13;
    val->fsm14 = fsm_status_b_mainpage.is_fsm14;
    val->fsm15 = fsm_status_b_mainpage.is_fsm15;
    val->fsm16 = fsm_status_b_mainpage.is_fsm16;

    val->mlc1 = mlc_status_mainpage.is_mlc1;
    val->mlc2 = mlc_status_mainpage.is_mlc2;
    val->mlc3 = mlc_status_mainpage.is_mlc3;
    val->mlc4 = mlc_status_mainpage.is_mlc4;
    val->mlc5 = mlc_status_mainpage.is_mlc5;
    val->mlc6 = mlc_status_mainpage.is_mlc6;
    val->mlc7 = mlc_status_mainpage.is_mlc7;
    val->mlc8 = mlc_status_mainpage.is_mlc8;

    val->sh_endop       = status_master_mainpage.sens_hub_endop;
    val->sh_slave0_nack = status_master_mainpage.slave0_nack;
    val->sh_slave1_nack = status_master_mainpage.slave1_nack;
    val->sh_slave2_nack = status_master_mainpage.slave2_nack;
    val->sh_slave3_nack = status_master_mainpage.slave3_nack;
    val->sh_wr_once     = status_master_mainpage.wr_once_done;

    val->fifo_diff = (256U * fifo_status2.diff_fifo) + fifo_status1.diff_fifo;

    val->fifo_ovr_latched = fifo_status2.over_run_latched;
    val->fifo_bdr         = fifo_status2.counter_bdr_ia;
    val->fifo_full        = fifo_status2.fifo_full_ia;
    val->fifo_ovr         = fifo_status2.fifo_ovr_ia;
    val->fifo_th          = fifo_status2.fifo_wtm_ia;

    ctrl5_c.rounding_status = PROPERTY_DISABLE;
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL5_C, (uint8_t*)&ctrl5_c, 1);

  }

  return ret;
}

/**
  * @brief  Sensor conversion parameters selection.[set]
  *
  * @param  ctx          communication interface handler. Use NULL to ingnore
  *                      this interface.(ptr)
  * @param  aux_ctx      auxiliary communication interface handler. Use NULL
  *                      to ingnore this interface.(ptr)
  * @param  val          set the sensor conversion parameters by checking
  *                      the constraints of the device.(ptr)
  *
  */
int32_t lsm6dsox_mode_set(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                          lsm6dsox_md_t *val)
{
  lsm6dsox_func_cfg_access_t func_cfg_access;
  lsm6dsox_spi2_ctrl1_ois_t spi2_ctrl1_ois;
  lsm6dsox_spi2_ctrl2_ois_t spi2_ctrl2_ois;
  lsm6dsox_spi2_ctrl3_ois_t spi2_ctrl3_ois;
  lsm6dsox_ui_ctrl1_ois_t ui_ctrl1_ois;
  lsm6dsox_ui_ctrl2_ois_t ui_ctrl2_ois;
  lsm6dsox_ui_ctrl3_ois_t ui_ctrl3_ois;
  lsm6dsox_ctrl1_xl_t ctrl1_xl;
  lsm6dsox_ctrl8_xl_t ctrl8_xl;
  lsm6dsox_ctrl2_g_t ctrl2_g;
  lsm6dsox_ctrl3_c_t ctrl3_c;
  lsm6dsox_ctrl4_c_t ctrl4_c;
  lsm6dsox_ctrl5_c_t ctrl5_c;
  lsm6dsox_ctrl6_c_t ctrl6_c;
  lsm6dsox_ctrl7_g_t ctrl7_g;
  uint8_t xl_hm_mode;
  uint8_t g_hm_mode;
  uint8_t xl_ulp_en;
  uint8_t odr_gy;
  uint8_t odr_xl;
  uint8_t reg[8];
  int32_t ret;

  ret = 0;
  /* FIXME: Remove warnings with STM32CubeIDE */
  ctrl3_c.not_used_01 = 0;
  ctrl4_c.not_used_01 = 0;

  /* reading input configuration */
  xl_hm_mode = ( (uint8_t)val->ui.xl.odr & 0x10U ) >> 4;
  xl_ulp_en = ( (uint8_t)val->ui.xl.odr & 0x20U ) >> 5;
  odr_xl = (uint8_t)val->ui.xl.odr & 0x0FU;

  /* if enable xl ultra low power mode disable gy and OIS chain */
  if (xl_ulp_en == PROPERTY_ENABLE) {
    val->ois.xl.odr = LSM6DSOX_XL_OIS_OFF;
    val->ois.gy.odr = LSM6DSOX_GY_OIS_OFF;
    val->ui.gy.odr  = LSM6DSOX_GY_UI_OFF;
  }
  /* if OIS xl is enabled also gyro OIS is enabled */
  if (val->ois.xl.odr == LSM6DSOX_XL_OIS_6667Hz_HP){
    val->ois.gy.odr = LSM6DSOX_GY_OIS_6667Hz_HP;
  }
  g_hm_mode = ( (uint8_t)val->ui.gy.odr & 0x10U ) >> 4;
  odr_gy = (uint8_t)val->ui.gy.odr & 0x0FU;

  /* reading registers to be configured */
  if( ctx != NULL ) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL1_XL, reg, 8);
    bytecpy(( uint8_t*)&ctrl1_xl, &reg[0]);
    bytecpy(( uint8_t*)&ctrl2_g,  &reg[1]);
    bytecpy(( uint8_t*)&ctrl3_c,  &reg[2]);
    bytecpy(( uint8_t*)&ctrl4_c,  &reg[3]);
    bytecpy(( uint8_t*)&ctrl5_c,  &reg[4]);
    bytecpy(( uint8_t*)&ctrl6_c,  &reg[5]);
    bytecpy(( uint8_t*)&ctrl7_g,  &reg[6]);
    bytecpy(( uint8_t*)&ctrl8_xl, &reg[7]);
    if ( ret == 0 ) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FUNC_CFG_ACCESS,
                              (uint8_t*)&func_cfg_access, 1);
    }
    /* if toggle xl ultra low power mode, turn off xl before reconfigure */
    if (ctrl5_c.xl_ulp_en != xl_ulp_en) {
        ctrl1_xl.odr_xl = (uint8_t) 0x00U;
        ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL1_XL,
                                 (uint8_t*)&ctrl1_xl, 1);
    }
  }

  /* reading OIS registers to be configured */
  if( aux_ctx != NULL ) {
    if (ret == 0) {
      ret = lsm6dsox_read_reg(aux_ctx, LSM6DSOX_SPI2_CTRL1_OIS, reg, 3);
    }
    bytecpy(( uint8_t*)&spi2_ctrl1_ois, &reg[0]);
    bytecpy(( uint8_t*)&spi2_ctrl2_ois, &reg[1]);
    bytecpy(( uint8_t*)&spi2_ctrl3_ois, &reg[2]);
  }
  else {
    if( ctx != NULL ) {
      if (ret == 0) {
        ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, reg, 3);
      }
      bytecpy(( uint8_t*)&ui_ctrl1_ois, &reg[0]);
      bytecpy(( uint8_t*)&ui_ctrl2_ois, &reg[1]);
      bytecpy(( uint8_t*)&ui_ctrl3_ois, &reg[2]);
    }
  }

  /* Check the Finite State Machine data rate constraints */
  if (val->fsm.sens != LSM6DSOX_FSM_DISABLE) {
    switch (val->fsm.odr) {
      case LSM6DSOX_FSM_12Hz5:
        if ( (val->fsm.sens != LSM6DSOX_FSM_GY) && (odr_xl == 0x00U) ) {
          odr_xl = 0x01U;
        }
        if ( (val->fsm.sens != LSM6DSOX_FSM_XL) && (odr_gy == 0x00U) ) {
          xl_ulp_en = PROPERTY_DISABLE;
          odr_gy = 0x01U;
        }
        break;
      case LSM6DSOX_FSM_26Hz:
        if ( (val->fsm.sens != LSM6DSOX_FSM_GY) && (odr_xl < 0x02U) ) {
          odr_xl = 0x02U;
        }
        if ( (val->fsm.sens != LSM6DSOX_FSM_XL) && (odr_gy < 0x02U) ) {
          xl_ulp_en = PROPERTY_DISABLE;
          odr_gy = 0x02U;
        }
        break;
      case LSM6DSOX_FSM_52Hz:
        if ( (val->fsm.sens != LSM6DSOX_FSM_GY) && (odr_xl < 0x03U) ) {
          odr_xl = 0x03U;
        }
        if ( (val->fsm.sens != LSM6DSOX_FSM_XL) && (odr_gy < 0x03U) ) {
          xl_ulp_en = PROPERTY_DISABLE;
          odr_gy = 0x03U;
        }
        break;
      case LSM6DSOX_FSM_104Hz:
        if ( (val->fsm.sens != LSM6DSOX_FSM_GY) && (odr_xl < 0x04U) ) {
          odr_xl = 0x04U;
        }
        if ( (val->fsm.sens != LSM6DSOX_FSM_XL) && (odr_gy < 0x04U) ) {
          xl_ulp_en = PROPERTY_DISABLE;
          odr_gy = 0x04U;
        }
        break;
      default:
        odr_xl = 0x00U;
        odr_gy = 0x00U;
        break;
    }
  }

  /* Check the Machine Learning Core data rate constraints */
  if (val->mlc.sens != LSM6DSOX_MLC_DISABLE) {
    switch (val->mlc.odr) {
      case LSM6DSOX_MLC_12Hz5:
        if (odr_xl == 0x00U) {
          odr_xl = 0x01U;
        }
        if ( (val->mlc.sens != LSM6DSOX_MLC_XL) && (odr_gy == 0x00U) ) {
          xl_ulp_en = PROPERTY_DISABLE;
          odr_gy = 0x01U;
        }
        break;
      case LSM6DSOX_MLC_26Hz:
        if (odr_xl < 0x02U) {
          odr_xl = 0x02U;
        }
        if ( (val->mlc.sens != LSM6DSOX_MLC_XL) && (odr_gy < 0x02U) ) {
          xl_ulp_en = PROPERTY_DISABLE;
          odr_gy = 0x02U;
        }
        break;
      case LSM6DSOX_MLC_52Hz:
        if (odr_xl < 0x03U) {
          odr_xl = 0x03U;
        }
        if ( (val->mlc.sens != LSM6DSOX_MLC_XL) && (odr_gy < 0x03U) ) {
          xl_ulp_en = PROPERTY_DISABLE;
          odr_gy = 0x03U;
        }
        break;
      case LSM6DSOX_MLC_104Hz:
        if (odr_xl < 0x04U) {
          odr_xl = 0x04U;
        }
        if ( (val->mlc.sens != LSM6DSOX_MLC_XL) && (odr_gy < 0x04U) ) {
          xl_ulp_en = PROPERTY_DISABLE;
          odr_gy = 0x04U;
        }
        break;
      default:
        odr_xl = 0x00U;
        odr_gy = 0x00U;
        break;
    }
  }

  /* Updating the accelerometer data rate configuration */
  switch ( ( ctrl5_c.xl_ulp_en << 5 ) | ( ctrl6_c.xl_hm_mode << 4 ) |
           ctrl1_xl.odr_xl ) {
    case LSM6DSOX_XL_UI_OFF:
      val->ui.xl.odr = LSM6DSOX_XL_UI_OFF;
      break;
    case LSM6DSOX_XL_UI_12Hz5_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_12Hz5_HP;
      break;
    case LSM6DSOX_XL_UI_26Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_26Hz_HP;
      break;
    case LSM6DSOX_XL_UI_52Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_52Hz_HP;
      break;
    case LSM6DSOX_XL_UI_104Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_104Hz_HP;
      break;
    case LSM6DSOX_XL_UI_208Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_208Hz_HP;
      break;
    case LSM6DSOX_XL_UI_416Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_416Hz_HP;
      break;
    case LSM6DSOX_XL_UI_833Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_833Hz_HP;
      break;
    case LSM6DSOX_XL_UI_1667Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_1667Hz_HP;
      break;
    case LSM6DSOX_XL_UI_3333Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_3333Hz_HP;
      break;
    case LSM6DSOX_XL_UI_6667Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_6667Hz_HP;
      break;
    case LSM6DSOX_XL_UI_1Hz6_LP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_1Hz6_LP;
      break;
    case LSM6DSOX_XL_UI_12Hz5_LP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_12Hz5_LP;
      break;
    case LSM6DSOX_XL_UI_26Hz_LP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_26Hz_LP;
      break;
    case LSM6DSOX_XL_UI_52Hz_LP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_52Hz_LP;
      break;
    case LSM6DSOX_XL_UI_104Hz_NM:
      val->ui.xl.odr = LSM6DSOX_XL_UI_104Hz_NM;
      break;
    case LSM6DSOX_XL_UI_208Hz_NM:
      val->ui.xl.odr = LSM6DSOX_XL_UI_208Hz_NM;
      break;
    case LSM6DSOX_XL_UI_1Hz6_ULP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_1Hz6_ULP;
      break;
    case LSM6DSOX_XL_UI_12Hz5_ULP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_12Hz5_ULP;
      break;
    case LSM6DSOX_XL_UI_26Hz_ULP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_26Hz_ULP;
      break;
    case LSM6DSOX_XL_UI_52Hz_ULP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_52Hz_ULP;
      break;
    case LSM6DSOX_XL_UI_104Hz_ULP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_104Hz_ULP;
      break;
    case LSM6DSOX_XL_UI_208Hz_ULP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_208Hz_ULP;
      break;
    default:
      val->ui.xl.odr = LSM6DSOX_XL_UI_OFF;
      break;
  }

  /* Updating the accelerometer data rate configuration */
  switch ( (ctrl7_g.g_hm_mode << 4) | ctrl2_g.odr_g) {
    case LSM6DSOX_GY_UI_OFF:
      val->ui.gy.odr = LSM6DSOX_GY_UI_OFF;
      break;
    case LSM6DSOX_GY_UI_12Hz5_LP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_12Hz5_LP;
      break;
    case LSM6DSOX_GY_UI_12Hz5_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_12Hz5_HP;
      break;
    case LSM6DSOX_GY_UI_26Hz_LP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_26Hz_LP;
      break;
    case LSM6DSOX_GY_UI_26Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_26Hz_HP;
      break;
    case LSM6DSOX_GY_UI_52Hz_LP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_52Hz_LP;
      break;
    case LSM6DSOX_GY_UI_52Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_52Hz_HP;
      break;
    case LSM6DSOX_GY_UI_104Hz_NM:
      val->ui.gy.odr = LSM6DSOX_GY_UI_104Hz_NM;
      break;
    case LSM6DSOX_GY_UI_104Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_104Hz_HP;
      break;
    case LSM6DSOX_GY_UI_208Hz_NM:
      val->ui.gy.odr = LSM6DSOX_GY_UI_208Hz_NM;
      break;
    case LSM6DSOX_GY_UI_208Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_208Hz_HP;
      break;
    case LSM6DSOX_GY_UI_416Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_416Hz_HP;
      break;
    case LSM6DSOX_GY_UI_833Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_833Hz_HP;
      break;
    case LSM6DSOX_GY_UI_1667Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_1667Hz_HP;
      break;
    case LSM6DSOX_GY_UI_3333Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_3333Hz_HP;
      break;
    case LSM6DSOX_GY_UI_6667Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_6667Hz_HP;
      break;
    default:
      val->ui.gy.odr = LSM6DSOX_GY_UI_OFF;
      break;
  }

  /* Check accelerometer full scale constraints */
  /* Full scale of 16g must be the same for UI and OIS */
  if ( (val->ui.xl.fs == LSM6DSOX_XL_UI_16g) ||
       (val->ois.xl.fs == LSM6DSOX_XL_OIS_16g) ){
    val->ui.xl.fs = LSM6DSOX_XL_UI_16g;
    val->ois.xl.fs = LSM6DSOX_XL_OIS_16g;
  }

  /* prapare new configuration */

  /* Full scale of 16g must be the same for UI and OIS */
  if (val->ui.xl.fs == LSM6DSOX_XL_UI_16g) {
    ctrl8_xl.xl_fs_mode = PROPERTY_DISABLE;
  }
  else {
    ctrl8_xl.xl_fs_mode = PROPERTY_ENABLE;
  }

  /* OIS new configuration */
  ctrl7_g.ois_on_en = val->ois.ctrl_md & 0x01U;
  func_cfg_access.ois_ctrl_from_ui = (val->ois.ctrl_md & 0x02U) >> 1;

  switch (val->ois.ctrl_md) {
    case LSM6DSOX_OIS_ONLY_AUX:
      spi2_ctrl1_ois.fs_g_ois = (uint8_t)val->ois.gy.fs;
      spi2_ctrl1_ois.ois_en_spi2 = (uint8_t)val->ois.gy.odr | (uint8_t)val->ois.xl.odr;
      spi2_ctrl1_ois.mode4_en = (uint8_t) val->ois.xl.odr;
      spi2_ctrl3_ois.fs_xl_ois = (uint8_t)val->ois.xl.fs;
      break;
    case LSM6DSOX_OIS_ONLY_UI:
      ui_ctrl1_ois.fs_g_ois = (uint8_t)val->ois.gy.fs;
      ui_ctrl1_ois.ois_en_spi2 = (uint8_t)val->ois.gy.odr | (uint8_t)val->ois.xl.odr;
      ui_ctrl1_ois.mode4_en = (uint8_t)val->ois.xl.odr;
      ui_ctrl3_ois.fs_xl_ois = (uint8_t)val->ois.xl.fs;
      break;
    case LSM6DSOX_OIS_MIXED:
      spi2_ctrl1_ois.fs_g_ois = (uint8_t)val->ois.gy.fs;
      ctrl7_g.ois_on = (uint8_t)val->ois.gy.odr | (uint8_t)val->ois.xl.odr;
      spi2_ctrl1_ois.mode4_en = (uint8_t) val->ois.xl.odr;
      spi2_ctrl3_ois.fs_xl_ois = (uint8_t)val->ois.xl.fs;
      break;
    default:
      spi2_ctrl1_ois.fs_g_ois = (uint8_t)val->ois.gy.fs;
      spi2_ctrl1_ois.ois_en_spi2 = (uint8_t)val->ois.gy.odr | (uint8_t)val->ois.xl.odr;
      spi2_ctrl1_ois.mode4_en = (uint8_t) val->ois.xl.odr;
      spi2_ctrl3_ois.fs_xl_ois = (uint8_t)val->ois.xl.fs;
      break;
  }

  /* UI new configuration */
  ctrl1_xl.odr_xl = odr_xl;
  ctrl1_xl.fs_xl = (uint8_t)val->ui.xl.fs;
  ctrl5_c.xl_ulp_en = xl_ulp_en;
  ctrl6_c.xl_hm_mode = xl_hm_mode;
  ctrl7_g.g_hm_mode = g_hm_mode;
  ctrl2_g.odr_g = odr_gy;
  ctrl2_g.fs_g = (uint8_t) val->ui.gy.fs;

  /* writing checked configuration */
  if( ctx != NULL ) {
    bytecpy(&reg[0], ( uint8_t*)&ctrl1_xl);
    bytecpy(&reg[1], ( uint8_t*)&ctrl2_g);
    bytecpy(&reg[2], ( uint8_t*)&ctrl3_c);
    bytecpy(&reg[3], ( uint8_t*)&ctrl4_c);
    bytecpy(&reg[4], ( uint8_t*)&ctrl5_c);
    bytecpy(&reg[5], ( uint8_t*)&ctrl6_c);
    bytecpy(&reg[6], ( uint8_t*)&ctrl7_g);
    bytecpy(&reg[7], ( uint8_t*)&ctrl8_xl);
    if ( ret == 0 ) {
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_CTRL1_XL, (uint8_t*)&reg, 8);
    }
    if ( ret == 0 ) {
      ret = lsm6dsox_write_reg(ctx, LSM6DSOX_FUNC_CFG_ACCESS,
                               (uint8_t*)&func_cfg_access, 1);
    }
  }

  /* writing OIS checked configuration */
  if( aux_ctx != NULL ) {
    bytecpy(&reg[0], ( uint8_t*)&spi2_ctrl1_ois);
    bytecpy(&reg[1], ( uint8_t*)&spi2_ctrl2_ois);
    bytecpy(&reg[2], ( uint8_t*)&spi2_ctrl3_ois);
    if (ret == 0) {
      ret = lsm6dsox_write_reg(aux_ctx, LSM6DSOX_SPI2_CTRL1_OIS, reg, 3);
    }
  }
  else {
    if( ctx != NULL ) {
      bytecpy(&reg[0], ( uint8_t*)&ui_ctrl1_ois);
      bytecpy(&reg[1], ( uint8_t*)&ui_ctrl2_ois);
      bytecpy(&reg[2], ( uint8_t*)&ui_ctrl3_ois);
      if (ret == 0) {
        ret = lsm6dsox_write_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, reg, 3);
      }
    }
  }

  return ret;
}

/**
  * @brief  Sensor conversion parameters selection.[get]
  *
  * @param  ctx          communication interface handler. Use NULL to ingnore
  *                      this interface.(ptr)
  * @param  aux_ctx      auxiliary communication interface handler. Use NULL
  *                      to ingnore this interface.(ptr)
  * @param  val          get the sensor conversion parameters.(ptr)
  *
  */
int32_t lsm6dsox_mode_get(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                          lsm6dsox_md_t *val)
{

  lsm6dsox_emb_func_odr_cfg_b_t emb_func_odr_cfg_b;
  lsm6dsox_emb_func_odr_cfg_c_t emb_func_odr_cfg_c;
  lsm6dsox_func_cfg_access_t func_cfg_access;
  lsm6dsox_spi2_ctrl1_ois_t spi2_ctrl1_ois;
  lsm6dsox_spi2_ctrl2_ois_t spi2_ctrl2_ois;
  lsm6dsox_spi2_ctrl3_ois_t spi2_ctrl3_ois;
  lsm6dsox_emb_func_en_b_t emb_func_en_b;
  lsm6dsox_ui_ctrl1_ois_t ui_ctrl1_ois;
  lsm6dsox_ui_ctrl2_ois_t ui_ctrl2_ois;
  lsm6dsox_ui_ctrl3_ois_t ui_ctrl3_ois;
  lsm6dsox_fsm_enable_a_t fsm_enable_a;
  lsm6dsox_fsm_enable_b_t fsm_enable_b;
  lsm6dsox_ctrl1_xl_t ctrl1_xl;
  lsm6dsox_ctrl2_g_t ctrl2_g;
  lsm6dsox_ctrl3_c_t ctrl3_c;
  lsm6dsox_ctrl4_c_t ctrl4_c;
  lsm6dsox_ctrl5_c_t ctrl5_c;
  lsm6dsox_ctrl6_c_t ctrl6_c;
  lsm6dsox_ctrl7_g_t ctrl7_g;

  uint8_t reg[8];
  int32_t ret;

  ret = 0;

  /* reading the registers of the device */
  if( ctx != NULL ) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_CTRL1_XL, reg, 7);
    bytecpy(( uint8_t*)&ctrl1_xl, &reg[0]);
    bytecpy(( uint8_t*)&ctrl2_g,  &reg[1]);
    bytecpy(( uint8_t*)&ctrl3_c,  &reg[2]);
    bytecpy(( uint8_t*)&ctrl4_c,  &reg[3]);
    bytecpy(( uint8_t*)&ctrl5_c,  &reg[4]);
    bytecpy(( uint8_t*)&ctrl6_c,  &reg[5]);
    bytecpy(( uint8_t*)&ctrl7_g,  &reg[6]);
    if ( ret == 0 ) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FUNC_CFG_ACCESS,
                              (uint8_t*)&func_cfg_access, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_ODR_CFG_B, reg, 2);
      bytecpy(( uint8_t*)&emb_func_odr_cfg_b, &reg[0]);
      bytecpy(( uint8_t*)&emb_func_odr_cfg_c, &reg[1]);
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_EN_B,
                              (uint8_t*)&emb_func_en_b, 1);
    }
    if (ret == 0) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_FSM_ENABLE_A, reg, 2);
      bytecpy(( uint8_t*)&fsm_enable_a, &reg[0]);
      bytecpy(( uint8_t*)&fsm_enable_b, &reg[1]);
    }
    if (ret == 0) {
      ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
    }
  }

  if( aux_ctx != NULL ) {
    if (ret == 0) {
      ret = lsm6dsox_read_reg(aux_ctx, LSM6DSOX_SPI2_CTRL1_OIS, reg, 3);
    }
    bytecpy(( uint8_t*)&spi2_ctrl1_ois, &reg[0]);
    bytecpy(( uint8_t*)&spi2_ctrl2_ois, &reg[1]);
    bytecpy(( uint8_t*)&spi2_ctrl3_ois, &reg[2]);
  }
  else {
    if( ctx != NULL ) {
      if (ret == 0) {
        ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_CTRL1_OIS, reg, 3);
      }
      bytecpy(( uint8_t*)&ui_ctrl1_ois, &reg[0]);
      bytecpy(( uint8_t*)&ui_ctrl2_ois, &reg[1]);
      bytecpy(( uint8_t*)&ui_ctrl3_ois, &reg[2]);
    }
  }

  /* fill the input structure */

  /* get accelerometer configuration */
  switch ( (ctrl5_c.xl_ulp_en << 5) | (ctrl6_c.xl_hm_mode << 4) |
           ctrl1_xl.odr_xl ) {
    case LSM6DSOX_XL_UI_OFF:
      val->ui.xl.odr = LSM6DSOX_XL_UI_OFF;
      break;
    case LSM6DSOX_XL_UI_12Hz5_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_12Hz5_HP;
      break;
    case LSM6DSOX_XL_UI_26Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_26Hz_HP;
      break;
    case LSM6DSOX_XL_UI_52Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_52Hz_HP;
      break;
    case LSM6DSOX_XL_UI_104Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_104Hz_HP;
      break;
    case LSM6DSOX_XL_UI_208Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_208Hz_HP;
      break;
    case LSM6DSOX_XL_UI_416Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_416Hz_HP;
      break;
    case LSM6DSOX_XL_UI_833Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_833Hz_HP;
      break;
    case LSM6DSOX_XL_UI_1667Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_1667Hz_HP;
      break;
    case LSM6DSOX_XL_UI_3333Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_3333Hz_HP;
      break;
    case LSM6DSOX_XL_UI_6667Hz_HP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_6667Hz_HP;
      break;
    case LSM6DSOX_XL_UI_1Hz6_LP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_1Hz6_LP;
      break;
    case LSM6DSOX_XL_UI_12Hz5_LP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_12Hz5_LP;
      break;
    case LSM6DSOX_XL_UI_26Hz_LP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_26Hz_LP;
      break;
    case LSM6DSOX_XL_UI_52Hz_LP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_52Hz_LP;
      break;
    case LSM6DSOX_XL_UI_104Hz_NM:
      val->ui.xl.odr = LSM6DSOX_XL_UI_104Hz_NM;
      break;
    case LSM6DSOX_XL_UI_208Hz_NM:
      val->ui.xl.odr = LSM6DSOX_XL_UI_208Hz_NM;
      break;
    case LSM6DSOX_XL_UI_1Hz6_ULP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_1Hz6_ULP;
      break;
    case LSM6DSOX_XL_UI_12Hz5_ULP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_12Hz5_ULP;
      break;
    case LSM6DSOX_XL_UI_26Hz_ULP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_26Hz_ULP;
      break;
    case LSM6DSOX_XL_UI_52Hz_ULP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_52Hz_ULP;
      break;
    case LSM6DSOX_XL_UI_104Hz_ULP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_104Hz_ULP;
      break;
    case LSM6DSOX_XL_UI_208Hz_ULP:
      val->ui.xl.odr = LSM6DSOX_XL_UI_208Hz_ULP;
      break;
    default:
      val->ui.xl.odr = LSM6DSOX_XL_UI_OFF;
      break;
  }

  switch ( ctrl1_xl.fs_xl ) {
    case LSM6DSOX_XL_UI_2g:
      val->ui.xl.fs = LSM6DSOX_XL_UI_2g;
      break;
    case LSM6DSOX_XL_UI_4g:
      val->ui.xl.fs = LSM6DSOX_XL_UI_4g;
      break;
    case LSM6DSOX_XL_UI_8g:
      val->ui.xl.fs = LSM6DSOX_XL_UI_8g;
      break;
    case LSM6DSOX_XL_UI_16g:
      val->ui.xl.fs = LSM6DSOX_XL_UI_16g;
      break;
    default:
      val->ui.xl.fs = LSM6DSOX_XL_UI_2g;
      break;
  }

  /* get gyroscope configuration */
  switch ( (ctrl7_g.g_hm_mode << 4) | ctrl2_g.odr_g) {
    case LSM6DSOX_GY_UI_OFF:
      val->ui.gy.odr = LSM6DSOX_GY_UI_OFF;
      break;
    case LSM6DSOX_GY_UI_12Hz5_LP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_12Hz5_LP;
      break;
    case LSM6DSOX_GY_UI_12Hz5_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_12Hz5_HP;
      break;
    case LSM6DSOX_GY_UI_26Hz_LP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_26Hz_LP;
      break;
    case LSM6DSOX_GY_UI_26Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_26Hz_HP;
      break;
    case LSM6DSOX_GY_UI_52Hz_LP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_52Hz_LP;
      break;
    case LSM6DSOX_GY_UI_52Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_52Hz_HP;
      break;
    case LSM6DSOX_GY_UI_104Hz_NM:
      val->ui.gy.odr = LSM6DSOX_GY_UI_104Hz_NM;
      break;
    case LSM6DSOX_GY_UI_104Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_104Hz_HP;
      break;
    case LSM6DSOX_GY_UI_208Hz_NM:
      val->ui.gy.odr = LSM6DSOX_GY_UI_208Hz_NM;
      break;
    case LSM6DSOX_GY_UI_208Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_208Hz_HP;
      break;
    case LSM6DSOX_GY_UI_416Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_416Hz_HP;
      break;
    case LSM6DSOX_GY_UI_833Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_833Hz_HP;
      break;
    case LSM6DSOX_GY_UI_1667Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_1667Hz_HP;
      break;
    case LSM6DSOX_GY_UI_3333Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_3333Hz_HP;
      break;
    case LSM6DSOX_GY_UI_6667Hz_HP:
      val->ui.gy.odr = LSM6DSOX_GY_UI_6667Hz_HP;
      break;
    default:
      val->ui.gy.odr = LSM6DSOX_GY_UI_OFF;
      break;
  }

  switch (ctrl2_g.fs_g) {
    case LSM6DSOX_GY_UI_125dps:
      val->ui.gy.fs = LSM6DSOX_GY_UI_125dps;
      break;
    case LSM6DSOX_GY_UI_250dps:
      val->ui.gy.fs = LSM6DSOX_GY_UI_250dps;
      break;
    case LSM6DSOX_GY_UI_500dps:
      val->ui.gy.fs = LSM6DSOX_GY_UI_500dps;
      break;
    case LSM6DSOX_GY_UI_1000dps:
      val->ui.gy.fs = LSM6DSOX_GY_UI_1000dps;
      break;
    case LSM6DSOX_GY_UI_2000dps:
      val->ui.gy.fs = LSM6DSOX_GY_UI_2000dps;
      break;
    default:
      val->ui.gy.fs = LSM6DSOX_GY_UI_125dps;
      break;
  }

  /* get finite state machine configuration */
  if ( (fsm_enable_a.fsm1_en | fsm_enable_a.fsm2_en | fsm_enable_a.fsm3_en |
        fsm_enable_a.fsm4_en | fsm_enable_a.fsm5_en | fsm_enable_a.fsm6_en |
        fsm_enable_a.fsm7_en | fsm_enable_a.fsm8_en | fsm_enable_b.fsm9_en |
        fsm_enable_b.fsm10_en | fsm_enable_b.fsm11_en |
        fsm_enable_b.fsm12_en | fsm_enable_b.fsm13_en |
        fsm_enable_b.fsm14_en | fsm_enable_b.fsm15_en |
        fsm_enable_b.fsm16_en) == PROPERTY_ENABLE ){
    switch (emb_func_odr_cfg_b.fsm_odr) {
      case LSM6DSOX_FSM_12Hz5:
        val->fsm.odr = LSM6DSOX_FSM_12Hz5;
        break;
      case LSM6DSOX_FSM_26Hz:
        val->fsm.odr = LSM6DSOX_FSM_26Hz;
        break;
      case LSM6DSOX_FSM_52Hz:
        val->fsm.odr = LSM6DSOX_FSM_52Hz;
        break;
      case LSM6DSOX_FSM_104Hz:
        val->fsm.odr = LSM6DSOX_FSM_104Hz;
        break;
      default:
        val->fsm.odr = LSM6DSOX_FSM_12Hz5;
        break;
    }

    val->fsm.sens = LSM6DSOX_FSM_XL_GY;
    if (val->ui.gy.odr == LSM6DSOX_GY_UI_OFF) {
      val->fsm.sens = LSM6DSOX_FSM_XL;
    }
    if (val->ui.xl.odr == LSM6DSOX_XL_UI_OFF) {
      val->fsm.sens = LSM6DSOX_FSM_GY;
    }
  }
  else {
    val->fsm.sens = LSM6DSOX_FSM_DISABLE;
  }

  /* get machine learning core configuration */
  if (emb_func_en_b.mlc_en == PROPERTY_ENABLE) {
    switch (emb_func_odr_cfg_c.mlc_odr) {
      case LSM6DSOX_MLC_12Hz5:
        val->mlc.odr = LSM6DSOX_MLC_12Hz5;
        break;
      case LSM6DSOX_MLC_26Hz:
        val->mlc.odr = LSM6DSOX_MLC_26Hz;
        break;
      case LSM6DSOX_MLC_52Hz:
        val->mlc.odr = LSM6DSOX_MLC_52Hz;
        break;
      case LSM6DSOX_MLC_104Hz:
        val->mlc.odr = LSM6DSOX_MLC_104Hz;
        break;
      default:
        val->mlc.odr = LSM6DSOX_MLC_12Hz5;
        break;
    }

    val->mlc.sens = LSM6DSOX_MLC_XL_GY;
    if (val->ui.gy.odr == LSM6DSOX_GY_UI_OFF) {
      val->mlc.sens = LSM6DSOX_MLC_XL;
    }
    if (val->ui.xl.odr == LSM6DSOX_XL_UI_OFF) {
      val->mlc.sens = LSM6DSOX_MLC_DISABLE;
    }
  }
  else {
    val->mlc.sens = LSM6DSOX_MLC_DISABLE;
  }

  /* get ois configuration */

  /* OIS configuration mode */
  switch ( (func_cfg_access.ois_ctrl_from_ui << 1) + ctrl7_g.ois_on_en ) {
    case LSM6DSOX_OIS_ONLY_AUX:
      switch ( spi2_ctrl3_ois.fs_xl_ois ) {
        case LSM6DSOX_XL_OIS_2g:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_2g;
          break;
        case LSM6DSOX_XL_OIS_4g:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_4g;
          break;
        case LSM6DSOX_XL_OIS_8g:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_8g;
          break;
        case LSM6DSOX_XL_OIS_16g:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_16g;
          break;
        default:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_2g;
          break;
      }
      switch ( spi2_ctrl1_ois.mode4_en ) {
        case LSM6DSOX_XL_OIS_OFF:
          val->ois.xl.odr = LSM6DSOX_XL_OIS_OFF;
          break;
        case LSM6DSOX_XL_OIS_6667Hz_HP:
          val->ois.xl.odr = LSM6DSOX_XL_OIS_6667Hz_HP;
          break;
        default:
          val->ois.xl.odr = LSM6DSOX_XL_OIS_OFF;
          break;
      }
      switch ( spi2_ctrl1_ois.fs_g_ois ) {
        case LSM6DSOX_GY_OIS_250dps:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_250dps;
          break;
        case LSM6DSOX_GY_OIS_500dps:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_500dps;
          break;
        case LSM6DSOX_GY_OIS_1000dps:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_1000dps;
          break;
        case LSM6DSOX_GY_OIS_2000dps:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_2000dps;
          break;
        default:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_250dps;
          break;
      }
      switch ( spi2_ctrl1_ois.ois_en_spi2 ) {
        case LSM6DSOX_GY_OIS_OFF:
          val->ois.gy.odr = LSM6DSOX_GY_OIS_OFF;
          break;
        case LSM6DSOX_GY_OIS_6667Hz_HP:
          val->ois.gy.odr = LSM6DSOX_GY_OIS_6667Hz_HP;
          break;
        default:
          val->ois.gy.odr = LSM6DSOX_GY_OIS_OFF;
          break;
      }
      val->ois.ctrl_md = LSM6DSOX_OIS_ONLY_AUX;
      break;
    case LSM6DSOX_OIS_ONLY_UI:
      switch ( ui_ctrl3_ois.fs_xl_ois ) {
        case LSM6DSOX_XL_OIS_2g:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_2g;
          break;
        case LSM6DSOX_XL_OIS_4g:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_4g;
          break;
        case LSM6DSOX_XL_OIS_8g:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_8g;
          break;
        case LSM6DSOX_XL_OIS_16g:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_16g;
          break;
        default:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_2g;
          break;
      }
      switch ( ui_ctrl1_ois.ois_en_spi2 ) {
        case LSM6DSOX_GY_OIS_OFF:
          val->ois.gy.odr = LSM6DSOX_GY_OIS_OFF;
          break;
        case LSM6DSOX_GY_OIS_6667Hz_HP:
          val->ois.gy.odr = LSM6DSOX_GY_OIS_6667Hz_HP;
          break;
        default:
          val->ois.gy.odr = LSM6DSOX_GY_OIS_OFF;
          break;
      }
      switch ( ui_ctrl1_ois.fs_g_ois ) {
        case LSM6DSOX_GY_OIS_250dps:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_250dps;
          break;
        case LSM6DSOX_GY_OIS_125dps:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_125dps;
          break;
        case LSM6DSOX_GY_OIS_500dps:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_500dps;
          break;
        case LSM6DSOX_GY_OIS_1000dps:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_1000dps;
          break;
        case LSM6DSOX_GY_OIS_2000dps:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_2000dps;
          break;
        default:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_250dps;
          break;
      }
      switch ( ui_ctrl1_ois.mode4_en ) {
        case LSM6DSOX_XL_OIS_OFF:
          val->ois.xl.odr = LSM6DSOX_XL_OIS_OFF;
          break;
        case LSM6DSOX_XL_OIS_6667Hz_HP:
          val->ois.xl.odr = LSM6DSOX_XL_OIS_6667Hz_HP;
          break;
        default:
          val->ois.xl.odr = LSM6DSOX_XL_OIS_OFF;
          break;
      }
      val->ois.ctrl_md = LSM6DSOX_OIS_ONLY_UI;
      break;
    case LSM6DSOX_OIS_MIXED:
      switch ( spi2_ctrl3_ois.fs_xl_ois ) {
        case LSM6DSOX_XL_OIS_2g:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_2g;
          break;
        case LSM6DSOX_XL_OIS_4g:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_4g;
          break;
        case LSM6DSOX_XL_OIS_8g:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_8g;
          break;
        case LSM6DSOX_XL_OIS_16g:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_16g;
          break;
        default:
          val->ois.xl.fs = LSM6DSOX_XL_OIS_2g;
          break;
      }
      switch ( spi2_ctrl1_ois.mode4_en ) {
        case LSM6DSOX_XL_OIS_OFF:
          val->ois.xl.odr = LSM6DSOX_XL_OIS_OFF;
          break;
        case LSM6DSOX_XL_OIS_6667Hz_HP:
          val->ois.xl.odr = LSM6DSOX_XL_OIS_6667Hz_HP;
          break;
        default:
          val->ois.xl.odr = LSM6DSOX_XL_OIS_OFF;
          break;
      }
      switch ( spi2_ctrl1_ois.fs_g_ois ) {
        case LSM6DSOX_GY_OIS_250dps:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_250dps;
          break;
        case LSM6DSOX_GY_OIS_500dps:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_500dps;
          break;
        case LSM6DSOX_GY_OIS_1000dps:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_1000dps;
          break;
        case LSM6DSOX_GY_OIS_2000dps:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_2000dps;
          break;
        default:
          val->ois.gy.fs = LSM6DSOX_GY_OIS_250dps;
          break;
      }
      switch ( ui_ctrl1_ois.ois_en_spi2 ) {
        case LSM6DSOX_GY_OIS_OFF:
          val->ois.gy.odr = LSM6DSOX_GY_OIS_OFF;
          break;
        case LSM6DSOX_GY_OIS_6667Hz_HP:
          val->ois.gy.odr = LSM6DSOX_GY_OIS_6667Hz_HP;
          break;
        default:
          val->ois.gy.odr = LSM6DSOX_GY_OIS_OFF;
          break;
      }
      val->ois.ctrl_md = LSM6DSOX_OIS_MIXED;
      break;
    default:
      spi2_ctrl1_ois.fs_g_ois = (uint8_t)val->ois.gy.fs;
      spi2_ctrl1_ois.ois_en_spi2 = (uint8_t)val->ois.gy.odr | (uint8_t)val->ois.xl.odr;
      spi2_ctrl1_ois.mode4_en = (uint8_t) val->ois.xl.odr;
      spi2_ctrl3_ois.fs_xl_ois = (uint8_t)val->ois.xl.fs;
      val->ois.ctrl_md = LSM6DSOX_OIS_ONLY_AUX;
      break;
  }

  return ret;
}

/**
  * @brief  Read data in engineering unit.[get]
  *
  * @param  ctx     communication interface handler.(ptr)
  * @param  md      the sensor conversion parameters.(ptr)
  *
  */
int32_t lsm6dsox_data_get(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                          lsm6dsox_md_t *md, lsm6dsox_data_t *data)
{
  uint8_t buff[14];
  int32_t ret;
  uint8_t i;
  uint8_t j;

  ret = 0;

  /* read data */
  if( ctx != NULL ) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_OUT_TEMP_L, buff, 14);
  }
  j = 0;

  /* temperature conversion */
  data->ui.heat.raw = (int16_t)buff[j+1U];
  data->ui.heat.raw = ( ((int16_t)data->ui.heat.raw * (int16_t)256) +
                                                      (int16_t)buff[j] );
  j+=2U;
  data->ui.heat.deg_c = lsm6dsox_from_lsb_to_celsius((int16_t)data->ui.heat.raw);

  /* angular rate conversion */
  for (i = 0U; i < 3U; i++) {
    data->ui.gy.raw[i] = (int16_t)buff[j+1U];
    data->ui.gy.raw[i] = (data->ui.gy.raw[i] * 256) + (int16_t) buff[j];
    j+=2U;
    switch ( md->ui.gy.fs ) {
      case LSM6DSOX_GY_UI_250dps:
        data->ui.gy.mdps[i] = lsm6dsox_from_fs250_to_mdps(data->ui.gy.raw[i]);
        break;
      case LSM6DSOX_GY_UI_125dps:
        data->ui.gy.mdps[i] = lsm6dsox_from_fs125_to_mdps(data->ui.gy.raw[i]);
        break;
      case LSM6DSOX_GY_UI_500dps:
        data->ui.gy.mdps[i] = lsm6dsox_from_fs500_to_mdps(data->ui.gy.raw[i]);
        break;
      case LSM6DSOX_GY_UI_1000dps:
        data->ui.gy.mdps[i] = lsm6dsox_from_fs1000_to_mdps(data->ui.gy.raw[i]);
        break;
      case LSM6DSOX_GY_UI_2000dps:
        data->ui.gy.mdps[i] = lsm6dsox_from_fs2000_to_mdps(data->ui.gy.raw[i]);
        break;
      default:
        data->ui.gy.mdps[i] = 0.0f;
        break;
    }
  }

  /* acceleration conversion */
  for (i = 0U; i < 3U; i++) {
    data->ui.xl.raw[i] = (int16_t)buff[j+1U];
    data->ui.xl.raw[i] = (data->ui.xl.raw[i] * 256) + (int16_t) buff[j];
    j+=2U;
    switch ( md->ui.xl.fs ) {
      case LSM6DSOX_XL_UI_2g:
        data->ui.xl.mg[i] =lsm6dsox_from_fs2_to_mg(data->ui.xl.raw[i]);
        break;
      case LSM6DSOX_XL_UI_4g:
        data->ui.xl.mg[i] =lsm6dsox_from_fs4_to_mg(data->ui.xl.raw[i]);
        break;
      case LSM6DSOX_XL_UI_8g:
        data->ui.xl.mg[i] =lsm6dsox_from_fs8_to_mg(data->ui.xl.raw[i]);
        break;
      case LSM6DSOX_XL_UI_16g:
        data->ui.xl.mg[i] =lsm6dsox_from_fs16_to_mg(data->ui.xl.raw[i]);
        break;
      default:
        data->ui.xl.mg[i] = 0.0f;
        break;
    }

  }

  /* read data from ois chain */
  if (aux_ctx != NULL) {
    if (ret == 0) {
      ret = lsm6dsox_read_reg(aux_ctx, LSM6DSOX_SPI2_OUTX_L_G_OIS, buff, 12);
    }
  }
  else {
    if ((ctx != NULL) && (md->ois.ctrl_md == LSM6DSOX_OIS_ONLY_UI)) {
      ret = lsm6dsox_read_reg(ctx, LSM6DSOX_UI_OUTX_L_G_OIS, buff, 12);
    }
  }
  j = 0;

  /* ois angular rate conversion */
  for (i = 0U; i < 3U; i++) {
    data->ois.gy.raw[i] = (int16_t) buff[j+1U];
    data->ois.gy.raw[i] = (data->ois.gy.raw[i] * 256) + (int16_t) buff[j];
    j+=2U;
    switch ( md->ois.gy.fs ) {
      case LSM6DSOX_GY_UI_250dps:
        data->ois.gy.mdps[i] = lsm6dsox_from_fs250_to_mdps(data->ois.gy.raw[i]);
        break;
      case LSM6DSOX_GY_UI_125dps:
        data->ois.gy.mdps[i] = lsm6dsox_from_fs125_to_mdps(data->ois.gy.raw[i]);
        break;
      case LSM6DSOX_GY_UI_500dps:
        data->ois.gy.mdps[i] = lsm6dsox_from_fs500_to_mdps(data->ois.gy.raw[i]);
        break;
      case LSM6DSOX_GY_UI_1000dps:
        data->ois.gy.mdps[i] = lsm6dsox_from_fs1000_to_mdps(data->ois.gy.raw[i]);
        break;
      case LSM6DSOX_GY_UI_2000dps:
        data->ois.gy.mdps[i] = lsm6dsox_from_fs2000_to_mdps(data->ois.gy.raw[i]);
        break;
      default:
        data->ois.gy.mdps[i] = 0.0f;
        break;
    }
  }

  /* ois acceleration conversion */
  for (i = 0U; i < 3U; i++) {
    data->ois.xl.raw[i] = (int16_t) buff[j+1U];
    data->ois.xl.raw[i] = (data->ois.xl.raw[i] * 256) + (int16_t) buff[j];
    j+=2U;
    switch ( md->ois.xl.fs ) {
      case LSM6DSOX_XL_UI_2g:
        data->ois.xl.mg[i] =lsm6dsox_from_fs2_to_mg(data->ois.xl.raw[i]);
        break;
      case LSM6DSOX_XL_UI_4g:
        data->ois.xl.mg[i] =lsm6dsox_from_fs4_to_mg(data->ois.xl.raw[i]);
        break;
      case LSM6DSOX_XL_UI_8g:
        data->ois.xl.mg[i] =lsm6dsox_from_fs8_to_mg(data->ois.xl.raw[i]);
        break;
      case LSM6DSOX_XL_UI_16g:
        data->ois.xl.mg[i] =lsm6dsox_from_fs16_to_mg(data->ois.xl.raw[i]);
        break;
      default:
        data->ois.xl.mg[i] = 0.0f;
        break;
    }
  }

  return ret;
}

/**
  * @brief  Embedded functions.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of registers
  *                  EMB_FUNC_EN_A e EMB_FUNC_EN_B.
  *
  */
int32_t lsm6dsox_embedded_sens_set(stmdev_ctx_t *ctx,
                                   lsm6dsox_emb_sens_t *val)
{
  lsm6dsox_emb_func_en_a_t emb_func_en_a;
  lsm6dsox_emb_func_en_b_t emb_func_en_b;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_EN_A,
                            (uint8_t*)&emb_func_en_a, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_EN_B,
                           (uint8_t*)&emb_func_en_b, 1);

    emb_func_en_b.mlc_en = val->mlc;
    emb_func_en_b.fsm_en = val->fsm;
    emb_func_en_a.tilt_en = val->tilt;
    emb_func_en_a.pedo_en = val->step;
    emb_func_en_a.sign_motion_en = val->sig_mot;
    emb_func_en_b.fifo_compr_en = val->fifo_compr;

  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_EN_A,
                            (uint8_t*)&emb_func_en_a, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_EN_B,
                            (uint8_t*)&emb_func_en_b, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  Embedded functions.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of registers
  *                  EMB_FUNC_EN_A e EMB_FUNC_EN_B.
  *
  */
int32_t lsm6dsox_embedded_sens_get(stmdev_ctx_t *ctx,
                                   lsm6dsox_emb_sens_t *emb_sens)
{
  lsm6dsox_emb_func_en_a_t emb_func_en_a;
  lsm6dsox_emb_func_en_b_t emb_func_en_b;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_EN_A,
                           (uint8_t*)&emb_func_en_a, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_EN_B,
                           (uint8_t*)&emb_func_en_b, 1);

    emb_sens->mlc = emb_func_en_b.mlc_en;
    emb_sens->fsm = emb_func_en_b.fsm_en;
    emb_sens->tilt = emb_func_en_a.tilt_en;
    emb_sens->step = emb_func_en_a.pedo_en;
    emb_sens->sig_mot = emb_func_en_a.sign_motion_en;
    emb_sens->fifo_compr = emb_func_en_b.fifo_compr_en;

  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
  }

  return ret;
}

/**
  * @brief  turn off all embedded functions.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of registers
  *                  EMB_FUNC_EN_A e EMB_FUNC_EN_B.
  *
  */
int32_t lsm6dsox_embedded_sens_off(stmdev_ctx_t *ctx)
{
  lsm6dsox_emb_func_en_a_t emb_func_en_a;
  lsm6dsox_emb_func_en_b_t emb_func_en_b;
  int32_t ret;

  ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_EMBEDDED_FUNC_BANK);
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_EN_A,
                            (uint8_t*)&emb_func_en_a, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_read_reg(ctx, LSM6DSOX_EMB_FUNC_EN_B,
                           (uint8_t*)&emb_func_en_b, 1);

    emb_func_en_b.mlc_en = PROPERTY_DISABLE;
    emb_func_en_b.fsm_en = PROPERTY_DISABLE;
    emb_func_en_a.tilt_en = PROPERTY_DISABLE;
    emb_func_en_a.pedo_en = PROPERTY_DISABLE;
    emb_func_en_a.sign_motion_en = PROPERTY_DISABLE;
    emb_func_en_b.fifo_compr_en = PROPERTY_DISABLE;

  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_EN_A,
                            (uint8_t*)&emb_func_en_a, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_write_reg(ctx, LSM6DSOX_EMB_FUNC_EN_B,
                            (uint8_t*)&emb_func_en_b, 1);
  }
  if (ret == 0) {
    ret = lsm6dsox_mem_bank_set(ctx, LSM6DSOX_USER_BANK);
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
