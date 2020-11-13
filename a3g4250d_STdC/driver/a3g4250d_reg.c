/*
 ******************************************************************************
 * @file    a3g4250d_reg.c
 * @author  Sensors Software Solution Team
 * @brief   A3G4250D driver file
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

#include "a3g4250d_reg.h"

/**
  * @defgroup    A3G4250D
  * @brief       This file provides a set of functions needed to drive the
  *              a3g4250d enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup    A3G4250D_Interfaces_Functions
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
int32_t a3g4250d_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
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
int32_t a3g4250d_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
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
  * @defgroup    A3G4250D_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t a3g4250d_from_fs245dps_to_mdps(int16_t lsb)
{
  return ( (float_t)lsb * 8.75f );
}

float_t a3g4250d_from_lsb_to_celsius(int16_t lsb)
{
  return ( (float_t)lsb + 25.0f );
}

/**
  * @}
  *
  */

/**
  * @defgroup   A3G4250D_data_generation
  * @brief      This section groups all the functions concerning
  *             data generation
  * @{
  *
  */

/**
  * @brief  Accelerometer data rate selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of dr in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_data_rate_set(stmdev_ctx_t *ctx, a3g4250d_dr_t val)
{
  a3g4250d_ctrl_reg1_t ctrl_reg1;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG1,
                          (uint8_t *)&ctrl_reg1, 1);

  if (ret == 0) {
    ctrl_reg1.dr = ((uint8_t)val & 0x30U) >> 4;
    ctrl_reg1.pd = ((uint8_t)val & 0x0FU);
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG1,
                             (uint8_t *)&ctrl_reg1, 1);
  }

  return ret;
}

/**
  * @brief  Accelerometer data rate selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of dr in reg CTRL_REG1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_data_rate_get(stmdev_ctx_t *ctx, a3g4250d_dr_t *val)
{
  a3g4250d_ctrl_reg1_t ctrl_reg1;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG1,
                          (uint8_t *)&ctrl_reg1, 1);

  switch ( ( ctrl_reg1.dr  << 4 ) + ctrl_reg1.pd ) {
    case A3G4250D_ODR_OFF:
      *val = A3G4250D_ODR_OFF;
      break;

    case A3G4250D_ODR_SLEEP:
      *val = A3G4250D_ODR_SLEEP;
      break;

    case A3G4250D_ODR_100Hz:
      *val = A3G4250D_ODR_100Hz;
      break;

    case A3G4250D_ODR_200Hz:
      *val = A3G4250D_ODR_200Hz;
      break;

    case A3G4250D_ODR_400Hz:
      *val = A3G4250D_ODR_400Hz;
      break;

    case A3G4250D_ODR_800Hz:
      *val = A3G4250D_ODR_800Hz;
      break;

    default:
      *val = A3G4250D_ODR_OFF;
      break;
  }

  return ret;
}

/**
  * @brief  The STATUS_REG register is read by the primary interface.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    registers STATUS_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_status_reg_get(stmdev_ctx_t *ctx,
                                a3g4250d_status_reg_t *val)
{
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_STATUS_REG, (uint8_t *) val, 1);
  return ret;
}

/**
  * @brief  Accelerometer new data available.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "zyxda" in reg STATUS_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_status_reg_t status_reg;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_STATUS_REG,
                          (uint8_t *)&status_reg, 1);
  *val = status_reg.zyxda;
  return ret;
}
/**
  * @}
  *
  */

/**
  * @defgroup   A3G4250D_Dataoutput
  * @brief      This section groups all the data output functions.
  * @{
  *
  */

/**
  * @brief  Temperature data.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_OUT_TEMP, buff, 1);
  return ret;
}

/**
  * @brief  Angular rate sensor. The value is expressed as a 16-bit word in
  *         two’s complement.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_angular_rate_raw_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;
  ret =  a3g4250d_read_reg(ctx, A3G4250D_OUT_X_L, buff, 6);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   A3G4250D_common
  * @brief      This section groups common useful functions.
  * @{
  *
  */

/**
  * @brief  Device Who amI.[get]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  buff     Buffer that stores the data read.(ptr)
  * @retval          Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_WHO_AM_I, buff, 1);
  return ret;
}

/**
  * @brief  Angular rate sensor self-test enable. [set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    change the values of st in reg CTRL_REG4.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_self_test_set(stmdev_ctx_t *ctx, a3g4250d_st_t val)
{
  a3g4250d_ctrl_reg4_t ctrl_reg4;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG4,
                          (uint8_t *)&ctrl_reg4, 1);

  if (ret == 0) {
    ctrl_reg4.st = (uint8_t)val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG4,
                             (uint8_t *)&ctrl_reg4, 1);
  }

  return ret;
}

/**
  * @brief  Angular rate sensor self-test enable. [get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of st in reg CTRL_REG4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_self_test_get(stmdev_ctx_t *ctx, a3g4250d_st_t *val)
{
  a3g4250d_ctrl_reg4_t ctrl_reg4;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG4,
                          (uint8_t *)&ctrl_reg4, 1);

  switch (ctrl_reg4.st) {
    case A3G4250D_GY_ST_DISABLE:
      *val = A3G4250D_GY_ST_DISABLE;
      break;

    case A3G4250D_GY_ST_POSITIVE:
      *val = A3G4250D_GY_ST_POSITIVE;
      break;

    case A3G4250D_GY_ST_NEGATIVE:
      *val = A3G4250D_GY_ST_NEGATIVE;
      break;

    default:
      *val = A3G4250D_GY_ST_DISABLE;
      break;
  }

  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[set]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Change the values of "ble" in reg CTRL_REG4.
  * @retval         Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_data_format_set(stmdev_ctx_t *ctx,
                                 a3g4250d_ble_t val)
{
  a3g4250d_ctrl_reg4_t ctrl_reg4;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG4,
                          (uint8_t *)&ctrl_reg4, 1);

  if (ret == 0) {
    ctrl_reg4.ble = (uint8_t)val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG4,
                             (uint8_t *)&ctrl_reg4, 1);
  }

  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[get]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Get the values of "ble" in reg CTRL_REG4.(ptr)
  * @retval         Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_data_format_get(stmdev_ctx_t *ctx,
                                 a3g4250d_ble_t *val)
{
  a3g4250d_ctrl_reg4_t ctrl_reg4;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG4,
                          (uint8_t *)&ctrl_reg4, 1);

  switch (ctrl_reg4.ble) {
    case A3G4250D_AUX_LSB_AT_LOW_ADD:
      *val = A3G4250D_AUX_LSB_AT_LOW_ADD;
      break;

    case A3G4250D_AUX_MSB_AT_LOW_ADD:
      *val = A3G4250D_AUX_MSB_AT_LOW_ADD;
      break;

    default:
      *val = A3G4250D_AUX_LSB_AT_LOW_ADD;
      break;
  }

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Change the values of boot in reg CTRL_REG5.
  * @retval         Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_boot_set(stmdev_ctx_t *ctx, uint8_t val)
{
  a3g4250d_ctrl_reg5_t ctrl_reg5;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5,
                          (uint8_t *)&ctrl_reg5, 1);

  if (ret == 0) {
    ctrl_reg5.boot = val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG5,
                             (uint8_t *)&ctrl_reg5, 1);
  }

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Get the values of boot in reg CTRL_REG5.(ptr)
  * @retval         Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_boot_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_ctrl_reg5_t ctrl_reg5;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5,
                          (uint8_t *)&ctrl_reg5, 1);
  *val = ctrl_reg5.boot;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   A3G4250D_filters
  * @brief      This section group all the functions concerning the
  *             filters configuration.
  * @{
  *
  */

/**
  * @brief  Lowpass filter bandwidth selection.[set]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Change the values of "bw" in reg CTRL_REG1.
  * @retval         Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_lp_bandwidth_set(stmdev_ctx_t *ctx,
                                  a3g4250d_bw_t val)
{
  a3g4250d_ctrl_reg1_t ctrl_reg1;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG1,
                          (uint8_t *)&ctrl_reg1, 1);

  if (ret == 0) {
    ctrl_reg1.bw = (uint8_t)val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG1,
                             (uint8_t *)&ctrl_reg1, 1);
  }

  return ret;
}

/**
  * @brief  Lowpass filter bandwidth selection.[get]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Get the values of "bw" in reg CTRL_REG1.(ptr)
  * @retval         Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_lp_bandwidth_get(stmdev_ctx_t *ctx,
                                  a3g4250d_bw_t *val)
{
  a3g4250d_ctrl_reg1_t ctrl_reg1;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG1,
                          (uint8_t *)&ctrl_reg1, 1);

  switch (ctrl_reg1.bw) {
    case A3G4250D_CUT_OFF_LOW:
      *val = A3G4250D_CUT_OFF_LOW;
      break;

    case A3G4250D_CUT_OFF_MEDIUM:
      *val = A3G4250D_CUT_OFF_MEDIUM;
      break;

    case A3G4250D_CUT_OFF_HIGH:
      *val = A3G4250D_CUT_OFF_HIGH;
      break;

    case A3G4250D_CUT_OFF_VERY_HIGH:
      *val = A3G4250D_CUT_OFF_VERY_HIGH;
      break;

    default:
      *val = A3G4250D_CUT_OFF_LOW;
      break;
  }

  return ret;
}

/**
  * @brief  High-pass filter bandwidth selection.[set]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Change the values of "hpcf" in reg CTRL_REG2.
  * @retval         Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_hp_bandwidth_set(stmdev_ctx_t *ctx,
                                  a3g4250d_hpcf_t val)
{
  a3g4250d_ctrl_reg2_t ctrl_reg2;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG2,
                          (uint8_t *)&ctrl_reg2, 1);

  if (ret == 0) {
    ctrl_reg2.hpcf = (uint8_t)val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG2,
                             (uint8_t *)&ctrl_reg2, 1);
  }

  return ret;
}

/**
  * @brief  High-pass filter bandwidth selection.[get]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Get the values of hpcf in reg CTRL_REG2.(ptr)
  * @retval         Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_hp_bandwidth_get(stmdev_ctx_t *ctx,
                                  a3g4250d_hpcf_t *val)
{
  a3g4250d_ctrl_reg2_t ctrl_reg2;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG2,
                          (uint8_t *)&ctrl_reg2, 1);

  switch (ctrl_reg2.hpcf) {
    case A3G4250D_HP_LEVEL_0:
      *val = A3G4250D_HP_LEVEL_0;
      break;

    case A3G4250D_HP_LEVEL_1:
      *val = A3G4250D_HP_LEVEL_1;
      break;

    case A3G4250D_HP_LEVEL_2:
      *val = A3G4250D_HP_LEVEL_2;
      break;

    case A3G4250D_HP_LEVEL_3:
      *val = A3G4250D_HP_LEVEL_3;
      break;

    case A3G4250D_HP_LEVEL_4:
      *val = A3G4250D_HP_LEVEL_4;
      break;

    case A3G4250D_HP_LEVEL_5:
      *val = A3G4250D_HP_LEVEL_5;
      break;

    case A3G4250D_HP_LEVEL_6:
      *val = A3G4250D_HP_LEVEL_6;
      break;

    case A3G4250D_HP_LEVEL_7:
      *val = A3G4250D_HP_LEVEL_7;
      break;

    case A3G4250D_HP_LEVEL_8:
      *val = A3G4250D_HP_LEVEL_8;
      break;

    case A3G4250D_HP_LEVEL_9:
      *val = A3G4250D_HP_LEVEL_9;
      break;

    default:
      *val = A3G4250D_HP_LEVEL_0;
      break;
  }

  return ret;
}

/**
  * @brief  High-pass filter mode selection. [set]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Change the values of "hpm" in reg CTRL_REG2.
  * @retval         Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_hp_mode_set(stmdev_ctx_t *ctx, a3g4250d_hpm_t val)
{
  a3g4250d_ctrl_reg2_t ctrl_reg2;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG2,
                          (uint8_t *)&ctrl_reg2, 1);

  if (ret == 0) {
    ctrl_reg2.hpm = (uint8_t)val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG2,
                             (uint8_t *)&ctrl_reg2, 1);
  }

  return ret;
}

/**
  * @brief  High-pass filter mode selection. [get]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Get the values of hpm in reg CTRL_REG2.(ptr)
  * @retval         Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_hp_mode_get(stmdev_ctx_t *ctx, a3g4250d_hpm_t *val)
{
  a3g4250d_ctrl_reg2_t ctrl_reg2;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG2,
                          (uint8_t *)&ctrl_reg2, 1);

  switch (ctrl_reg2.hpm) {
    case A3G4250D_HP_NORMAL_MODE_WITH_RST:
      *val = A3G4250D_HP_NORMAL_MODE_WITH_RST;
      break;

    case A3G4250D_HP_REFERENCE_SIGNAL:
      *val = A3G4250D_HP_REFERENCE_SIGNAL;
      break;

    case A3G4250D_HP_NORMAL_MODE:
      *val = A3G4250D_HP_NORMAL_MODE;
      break;

    case A3G4250D_HP_AUTO_RESET_ON_INT:
      *val = A3G4250D_HP_AUTO_RESET_ON_INT;
      break;

    default:
      *val = A3G4250D_HP_NORMAL_MODE_WITH_RST;
      break;
  }

  return ret;
}

/**
  * @brief  Out/FIFO selection path. [set]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Change the values of "out_sel" in reg CTRL_REG5.
  * @retval         Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_filter_path_set(stmdev_ctx_t *ctx,
                                 a3g4250d_out_sel_t val)
{
  a3g4250d_ctrl_reg5_t ctrl_reg5;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5,
                          (uint8_t *)&ctrl_reg5, 1);

  if (ret == 0) {
    ctrl_reg5.out_sel = (uint8_t)val & 0x03U;
    ctrl_reg5.hpen = ( (uint8_t)val & 0x04U ) >> 2;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG5,
                             (uint8_t *)&ctrl_reg5, 1);
  }

  return ret;
}

/**
  * @brief  Out/FIFO selection path. [get]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Get the values of out_sel in reg CTRL_REG5.(ptr)
  * @retval         Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_filter_path_get(stmdev_ctx_t *ctx,
                                 a3g4250d_out_sel_t *val)
{
  a3g4250d_ctrl_reg5_t ctrl_reg5;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5,
                          (uint8_t *)&ctrl_reg5, 1);

  switch ( ( ctrl_reg5.hpen << 2 ) + ctrl_reg5.out_sel ) {
    case A3G4250D_ONLY_LPF1_ON_OUT:
      *val = A3G4250D_ONLY_LPF1_ON_OUT;
      break;

    case A3G4250D_LPF1_HP_ON_OUT:
      *val = A3G4250D_LPF1_HP_ON_OUT;
      break;

    case A3G4250D_LPF1_LPF2_ON_OUT:
      *val = A3G4250D_LPF1_LPF2_ON_OUT;
      break;

    case A3G4250D_LPF1_HP_LPF2_ON_OUT:
      *val = A3G4250D_LPF1_HP_LPF2_ON_OUT;
      break;

    default:
      *val = A3G4250D_ONLY_LPF1_ON_OUT;
      break;
  }

  return ret;
}

/**
  * @brief  Interrupt generator selection path.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of int1_sel in reg CTRL_REG5
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_filter_path_internal_set(stmdev_ctx_t *ctx,
                                          a3g4250d_int1_sel_t val)
{
  a3g4250d_ctrl_reg5_t ctrl_reg5;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5,
                          (uint8_t *)&ctrl_reg5, 1);

  if (ret == 0) {
    ctrl_reg5.int1_sel = (uint8_t)val & 0x03U;
    ctrl_reg5.hpen = ( (uint8_t)val & 0x04U ) >> 2;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG5,
                             (uint8_t *)&ctrl_reg5, 1);
  }

  return ret;
}

/**
  * @brief  Interrupt generator selection path.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of int1_sel in reg CTRL_REG5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_filter_path_internal_get(stmdev_ctx_t *ctx,
                                          a3g4250d_int1_sel_t *val)
{
  a3g4250d_ctrl_reg5_t ctrl_reg5;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5,
                          (uint8_t *)&ctrl_reg5, 1);

  switch ( ( ctrl_reg5.hpen << 2 ) + ctrl_reg5.int1_sel ) {
    case A3G4250D_ONLY_LPF1_ON_INT:
      *val = A3G4250D_ONLY_LPF1_ON_INT;
      break;

    case A3G4250D_LPF1_HP_ON_INT:
      *val = A3G4250D_LPF1_HP_ON_INT;
      break;

    case A3G4250D_LPF1_LPF2_ON_INT:
      *val = A3G4250D_LPF1_LPF2_ON_INT;
      break;

    case A3G4250D_LPF1_HP_LPF2_ON_INT:
      *val = A3G4250D_LPF1_HP_LPF2_ON_INT;
      break;

    default:
      *val = A3G4250D_ONLY_LPF1_ON_INT;
      break;
  }

  return ret;
}

/**
  * @brief  Reference value for high-pass filter.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of ref in reg REFERENCE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_hp_reference_value_set(stmdev_ctx_t *ctx,
                                        uint8_t val)
{
  a3g4250d_reference_t reference;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_REFERENCE,
                          (uint8_t *)&reference, 1);

  if (ret == 0) {
    reference.ref = val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_REFERENCE,
                             (uint8_t *)&reference, 1);
  }

  return ret;
}

/**
  * @brief  Reference value for high-pass filter.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of ref in reg REFERENCE.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_hp_reference_value_get(stmdev_ctx_t *ctx,
                                        uint8_t *val)
{
  a3g4250d_reference_t reference;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_REFERENCE,
                          (uint8_t *)&reference, 1);
  *val = reference.ref;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   A3G4250D_serial_interface
  * @brief   This section groups all the functions concerning main serial
  *          interface management.
  * @{
  *
  */

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of sim in reg CTRL_REG4
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_spi_mode_set(stmdev_ctx_t *ctx, a3g4250d_sim_t val)
{
  a3g4250d_ctrl_reg4_t ctrl_reg4;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG4,
                          (uint8_t *)&ctrl_reg4, 1);

  if (ret == 0) {
    ctrl_reg4.sim = (uint8_t)val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG4,
                             (uint8_t *)&ctrl_reg4, 1);
  }

  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of sim in reg CTRL_REG4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_spi_mode_get(stmdev_ctx_t *ctx, a3g4250d_sim_t *val)
{
  a3g4250d_ctrl_reg4_t ctrl_reg4;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG4,
                          (uint8_t *)&ctrl_reg4, 1);

  switch (ctrl_reg4.sim) {
    case A3G4250D_SPI_4_WIRE:
      *val = A3G4250D_SPI_4_WIRE;
      break;

    case A3G4250D_SPI_3_WIRE:
      *val = A3G4250D_SPI_3_WIRE;
      break;

    default:
      *val = A3G4250D_SPI_4_WIRE;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   A3G4250D_interrupt_pins
  * @brief      This section groups all the functions that manage interrupt pins
  * @{
  *
  */


/**
  * @brief  Select the signal that need to route on int1 pad.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Configure CTRL_REG3 int1 pad
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_pin_int1_route_set(stmdev_ctx_t *ctx,
                                    a3g4250d_int1_route_t val)
{
  a3g4250d_ctrl_reg3_t ctrl_reg3;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3,
                          (uint8_t *)&ctrl_reg3, 1);

  if (ret == 0) {
    ctrl_reg3.i1_int1         = val.i1_int1;
    ctrl_reg3.i1_boot         = val.i1_boot;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG3,
                             (uint8_t *)&ctrl_reg3, 1);
  }

  return ret;
}

/**
  * @brief  Select the signal that need to route on int1 pad.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Read CTRL_REG3 int1 pad.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */

int32_t a3g4250d_pin_int1_route_get(stmdev_ctx_t *ctx,
                                    a3g4250d_int1_route_t *val)
{
  a3g4250d_ctrl_reg3_t ctrl_reg3;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3,
                          (uint8_t *)&ctrl_reg3, 1);
  val->i1_int1       = ctrl_reg3.i1_int1;
  val->i1_boot       = ctrl_reg3.i1_boot;
  return ret;
}
/**
  * @brief  Select the signal that need to route on int2 pad.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Configure CTRL_REG3 int2 pad
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_pin_int2_route_set(stmdev_ctx_t *ctx,
                                    a3g4250d_int2_route_t val)
{
  a3g4250d_ctrl_reg3_t ctrl_reg3;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3,
                          (uint8_t *)&ctrl_reg3, 1);

  if (ret == 0) {
    ctrl_reg3.i2_empty         = val.i2_empty;
    ctrl_reg3.i2_orun          = val.i2_orun;
    ctrl_reg3.i2_wtm           = val.i2_wtm;
    ctrl_reg3.i2_drdy          = val.i2_drdy;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG3,
                             (uint8_t *)&ctrl_reg3, 1);
  }

  return ret;
}

/**
  * @brief  Select the signal that need to route on int2 pad.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Read CTRL_REG3 int2 pad.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_pin_int2_route_get(stmdev_ctx_t *ctx,
                                    a3g4250d_int2_route_t *val)
{
  a3g4250d_ctrl_reg3_t ctrl_reg3;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3,
                          (uint8_t *)&ctrl_reg3, 1);
  val->i2_empty       = ctrl_reg3.i2_empty;
  val->i2_orun        = ctrl_reg3.i2_orun;
  val->i2_wtm         = ctrl_reg3.i2_wtm;
  val->i2_drdy        = ctrl_reg3.i2_drdy;
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

int32_t a3g4250d_pin_mode_set(stmdev_ctx_t *ctx, a3g4250d_pp_od_t val)
{
  a3g4250d_ctrl_reg3_t ctrl_reg3;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3,
                          (uint8_t *)&ctrl_reg3, 1);

  if (ret == 0) {
    ctrl_reg3.pp_od = (uint8_t)val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG3,
                             (uint8_t *)&ctrl_reg3, 1);
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
int32_t a3g4250d_pin_mode_get(stmdev_ctx_t *ctx,
                              a3g4250d_pp_od_t *val)
{
  a3g4250d_ctrl_reg3_t ctrl_reg3;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3,
                          (uint8_t *)&ctrl_reg3, 1);

  switch (ctrl_reg3.pp_od ) {
    case A3G4250D_PUSH_PULL:
      *val = A3G4250D_PUSH_PULL;
      break;

    case A3G4250D_OPEN_DRAIN:
      *val = A3G4250D_OPEN_DRAIN;
      break;

    default:
      *val = A3G4250D_PUSH_PULL;
      break;
  }

  return ret;
}

/**
  * @brief  Pin active-high/low.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of h_lactive in reg CTRL_REG3.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_pin_polarity_set(stmdev_ctx_t *ctx,
                                  a3g4250d_h_lactive_t val)
{
  a3g4250d_ctrl_reg3_t ctrl_reg3;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3,
                          (uint8_t *)&ctrl_reg3, 1);

  if (ret == 0) {
    ctrl_reg3.h_lactive = (uint8_t)val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG3,
                             (uint8_t *)&ctrl_reg3, 1);
  }

  return ret;
}

/**
  * @brief  Pin active-high/low.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of h_lactive in reg CTRL_REG3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_pin_polarity_get(stmdev_ctx_t *ctx,
                                  a3g4250d_h_lactive_t *val)
{
  a3g4250d_ctrl_reg3_t ctrl_reg3;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG3,
                          (uint8_t *)&ctrl_reg3, 1);

  switch (ctrl_reg3.h_lactive) {
    case A3G4250D_ACTIVE_HIGH:
      *val = A3G4250D_ACTIVE_HIGH;
      break;

    case A3G4250D_ACTIVE_LOW:
      *val = A3G4250D_ACTIVE_LOW;
      break;

    default:
      *val = A3G4250D_ACTIVE_HIGH;
      break;
  }

  return ret;
}

/**
  * @brief  Latched/pulsed interrupt.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of lir in reg INT1_CFG.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_notification_set(stmdev_ctx_t *ctx,
                                      a3g4250d_lir_t val)
{
  a3g4250d_int1_cfg_t int1_cfg;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_CFG, (uint8_t *)&int1_cfg,
                          1);

  if (ret == 0) {
    int1_cfg.lir = (uint8_t)val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_INT1_CFG, (uint8_t *)&int1_cfg,
                             1);
  }

  return ret;
}

/**
  * @brief  Latched/pulsed interrupt.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of lir in reg INT1_CFG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_notification_get(stmdev_ctx_t *ctx,
                                      a3g4250d_lir_t *val)
{
  a3g4250d_int1_cfg_t int1_cfg;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_CFG, (uint8_t *)&int1_cfg,
                          1);

  switch (int1_cfg.lir) {
    case A3G4250D_INT_PULSED:
      *val = A3G4250D_INT_PULSED;
      break;

    case A3G4250D_INT_LATCHED:
      *val = A3G4250D_INT_LATCHED;
      break;

    default:
      *val = A3G4250D_INT_PULSED;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   A3G4250D_ interrupt_on_threshold
  * @brief      This section groups all the functions that manage the event
  *             generation on threshold.
  * @{
  *
  */

/**
  * @brief  Configure the interrupt threshold sign.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Struct of registers INT1_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_on_threshold_conf_set(stmdev_ctx_t *ctx,
                                           a3g4250d_int1_cfg_t *val)
{
  int32_t ret;
  ret = a3g4250d_write_reg(ctx, A3G4250D_INT1_CFG, (uint8_t *) val, 1);
  return ret;
}

/**
  * @brief  Configure the interrupt threshold sign.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Struct of registers from INT1_CFG to.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_on_threshold_conf_get(stmdev_ctx_t *ctx,
                                           a3g4250d_int1_cfg_t *val)
{
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_CFG, (uint8_t *) val, 1);
  return ret;
}
/**
  * @brief  AND/OR combination of interrupt events.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of and_or in reg INT1_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_on_threshold_mode_set(stmdev_ctx_t *ctx,
                                           a3g4250d_and_or_t val)
{
  a3g4250d_int1_cfg_t int1_cfg;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_CFG, (uint8_t *)&int1_cfg,
                          1);

  if (ret == 0) {
    int1_cfg.and_or = (uint8_t)val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_INT1_CFG, (uint8_t *)&int1_cfg,
                             1);
  }

  return ret;
}

/**
  * @brief  AND/OR combination of interrupt events.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of and_or in reg INT1_CFG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_on_threshold_mode_get(stmdev_ctx_t *ctx,
                                           a3g4250d_and_or_t *val)
{
  a3g4250d_int1_cfg_t int1_cfg;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_CFG, (uint8_t *)&int1_cfg,
                          1);

  switch (int1_cfg.and_or) {
    case A3G4250D_INT1_ON_TH_OR:
      *val = A3G4250D_INT1_ON_TH_OR;
      break;

    case A3G4250D_INT1_ON_TH_AND:
      *val = A3G4250D_INT1_ON_TH_AND;
      break;

    default:
      *val = A3G4250D_INT1_ON_TH_OR;
      break;
  }

  return ret;
}

/**
  * @brief   int_on_threshold_src: [get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Union of registers from INT1_SRC to.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_on_threshold_src_get(stmdev_ctx_t *ctx,
                                          a3g4250d_int1_src_t *val)
{
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_SRC, (uint8_t *) val, 1);
  return ret;
}

/**
  * @brief  Interrupt threshold on X.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of thsx in reg INT1_TSH_XH
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_x_treshold_set(stmdev_ctx_t *ctx, uint16_t val)
{
  a3g4250d_int1_tsh_xh_t int1_tsh_xh;
  a3g4250d_int1_tsh_xl_t int1_tsh_xl;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_XH,
                          (uint8_t *)&int1_tsh_xh, 1);

  if (ret == 0) {
    int1_tsh_xh.thsx = (uint8_t)(val / 256U) & 0x7FU;
    ret = a3g4250d_write_reg(ctx, A3G4250D_INT1_TSH_XH,
                             (uint8_t *)&int1_tsh_xh, 1);
  }

  if (ret == 0) {
    ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_XL,
                            (uint8_t *)&int1_tsh_xl, 1);
  }

  if (ret == 0) {
    int1_tsh_xl.thsx = (uint8_t)(val - (int1_tsh_xh.thsx * 256U));
    ret = a3g4250d_write_reg(ctx, A3G4250D_INT1_TSH_XL,
                             (uint8_t *)&int1_tsh_xl, 1);
  }

  return ret;
}

/**
  * @brief  Interrupt threshold on X.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of thsx in reg INT1_TSH_XH.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_x_treshold_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  a3g4250d_int1_tsh_xh_t int1_tsh_xh;
  a3g4250d_int1_tsh_xl_t int1_tsh_xl;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_XH,
                          (uint8_t *)&int1_tsh_xh, 1);

  if (ret == 0) {
    ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_XL,
                            (uint8_t *)&int1_tsh_xl, 1);
    *val = int1_tsh_xh.thsx;
    *val = *val * 256U;
    *val +=  int1_tsh_xl.thsx;
  }

  return ret;
}

/**
  * @brief  Interrupt threshold on Y.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of thsy in reg INT1_TSH_YH
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_y_treshold_set(stmdev_ctx_t *ctx, uint16_t val)
{
  a3g4250d_int1_tsh_yh_t int1_tsh_yh;
  a3g4250d_int1_tsh_yl_t int1_tsh_yl;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_YH,
                          (uint8_t *)&int1_tsh_yh, 1);
  int1_tsh_yh.thsy = (uint8_t)(val / 256U) & 0x7FU;

  if (ret == 0) {
    ret = a3g4250d_write_reg(ctx, A3G4250D_INT1_TSH_YH,
                             (uint8_t *)&int1_tsh_yh, 1);
  }

  if (ret == 0) {
    ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_YL,
                            (uint8_t *)&int1_tsh_yl, 1);
    int1_tsh_yl.thsy = (uint8_t)(val - (int1_tsh_yh.thsy * 256U));
  }

  if (ret == 0) {
    ret = a3g4250d_write_reg(ctx, A3G4250D_INT1_TSH_YL,
                             (uint8_t *)&int1_tsh_yl, 1);
  }

  return ret;
}

/**
  * @brief  Interrupt threshold on Y.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of thsy in reg INT1_TSH_YH.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_y_treshold_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  a3g4250d_int1_tsh_yh_t int1_tsh_yh;
  a3g4250d_int1_tsh_yl_t int1_tsh_yl;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_YH,
                          (uint8_t *)&int1_tsh_yh, 1);

  if (ret == 0) {
    ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_YL,
                            (uint8_t *)&int1_tsh_yl, 1);
    *val = int1_tsh_yh.thsy;
    *val = *val * 256U;
    *val += int1_tsh_yl.thsy;
  }

  return ret;
}

/**
  * @brief  Interrupt threshold on Z.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of thsz in reg INT1_TSH_ZH.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_z_treshold_set(stmdev_ctx_t *ctx, uint16_t val)
{
  a3g4250d_int1_tsh_zh_t int1_tsh_zh;
  a3g4250d_int1_tsh_zl_t int1_tsh_zl;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_ZH,
                          (uint8_t *)&int1_tsh_zh, 1);
  int1_tsh_zh.thsz = (uint8_t)(val / 256U) & 0x7FU;;

  if (ret == 0) {
    ret = a3g4250d_write_reg(ctx, A3G4250D_INT1_TSH_ZH,
                             (uint8_t *)&int1_tsh_zh, 1);
  }

  if (ret == 0) {
    ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_ZL,
                            (uint8_t *)&int1_tsh_zl, 1);
    int1_tsh_zl.thsz = (uint8_t)(val - (int1_tsh_zh.thsz * 256U));
  }

  if (ret == 0) {
    ret = a3g4250d_write_reg(ctx, A3G4250D_INT1_TSH_ZL,
                             (uint8_t *)&int1_tsh_zl, 1);
  }

  return ret;
}

/**
  * @brief  Interrupt threshold on Z.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of thsz in reg INT1_TSH_ZH.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_z_treshold_get(stmdev_ctx_t *ctx, uint16_t *val)
{
  a3g4250d_int1_tsh_zh_t int1_tsh_zh;
  a3g4250d_int1_tsh_zl_t int1_tsh_zl;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_ZH,
                          (uint8_t *)&int1_tsh_zh, 1);

  if (ret == 0) {
    ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_TSH_ZL,
                            (uint8_t *)&int1_tsh_zl, 1);
    *val = int1_tsh_zh.thsz;
    *val = *val * 256U;
    *val += int1_tsh_zl.thsz;
  }

  return ret;
}

/**
  * @brief  Durationvalue.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of d in reg INT1_DURATION
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_on_threshold_dur_set(stmdev_ctx_t *ctx,
                                          uint8_t val)
{
  a3g4250d_int1_duration_t int1_duration;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_DURATION,
                          (uint8_t *)&int1_duration, 1);

  if (ret == 0) {
    int1_duration.d = val;

    if (val != PROPERTY_DISABLE) {
      int1_duration.wait = PROPERTY_ENABLE;
    }

    else {
      int1_duration.wait = PROPERTY_DISABLE;
    }

    ret = a3g4250d_write_reg(ctx, A3G4250D_INT1_DURATION,
                             (uint8_t *)&int1_duration, 1);
  }

  return ret;
}

/**
  * @brief  Durationvalue.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of d in reg INT1_DURATION.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_int_on_threshold_dur_get(stmdev_ctx_t *ctx,
                                          uint8_t *val)
{
  a3g4250d_int1_duration_t int1_duration;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_INT1_DURATION,
                          (uint8_t *)&int1_duration, 1);
  *val = int1_duration.d;
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   A3G4250D_fifo
  * @brief   This section group all the functions concerning the fifo usage
  * @{
  *
  */

/**
  * @brief  FIFOenable.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fifo_en in reg CTRL_REG5
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_fifo_enable_set(stmdev_ctx_t *ctx, uint8_t val)
{
  a3g4250d_ctrl_reg5_t ctrl_reg5;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5,
                          (uint8_t *)&ctrl_reg5, 1);

  if (ret == 0) {
    ctrl_reg5.fifo_en = val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_CTRL_REG5,
                             (uint8_t *)&ctrl_reg5, 1);
  }

  return ret;
}

/**
  * @brief  FIFOenable.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fifo_en in reg CTRL_REG5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_fifo_enable_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_ctrl_reg5_t ctrl_reg5;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_CTRL_REG5,
                          (uint8_t *)&ctrl_reg5, 1);
  *val = ctrl_reg5.fifo_en;
  return ret;
}

/**
  * @brief  FIFO watermark level selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of wtm in reg FIFO_CTRL_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val)
{
  a3g4250d_fifo_ctrl_reg_t fifo_ctrl_reg;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_FIFO_CTRL_REG,
                          (uint8_t *)&fifo_ctrl_reg, 1);

  if (ret == 0) {
    fifo_ctrl_reg.wtm = val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_FIFO_CTRL_REG,
                             (uint8_t *)&fifo_ctrl_reg, 1);
  }

  return ret;
}

/**
  * @brief  FIFO watermark level selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of wtm in reg FIFO_CTRL_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_fifo_ctrl_reg_t fifo_ctrl_reg;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_FIFO_CTRL_REG,
                          (uint8_t *)&fifo_ctrl_reg, 1);
  *val = fifo_ctrl_reg.wtm;
  return ret;
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fm in reg FIFO_CTRL_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_fifo_mode_set(stmdev_ctx_t *ctx,
                               a3g4250d_fifo_mode_t val)
{
  a3g4250d_fifo_ctrl_reg_t fifo_ctrl_reg;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_FIFO_CTRL_REG,
                          (uint8_t *)&fifo_ctrl_reg, 1);

  if (ret == 0) {
    fifo_ctrl_reg.fm = (uint8_t)val;
    ret = a3g4250d_write_reg(ctx, A3G4250D_FIFO_CTRL_REG,
                             (uint8_t *)&fifo_ctrl_reg, 1);
  }

  return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fm in reg FIFO_CTRL_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_fifo_mode_get(stmdev_ctx_t *ctx,
                               a3g4250d_fifo_mode_t *val)
{
  a3g4250d_fifo_ctrl_reg_t fifo_ctrl_reg;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_FIFO_CTRL_REG,
                          (uint8_t *)&fifo_ctrl_reg, 1);

  switch (fifo_ctrl_reg.fm) {
    case A3G4250D_FIFO_BYPASS_MODE:
      *val = A3G4250D_FIFO_BYPASS_MODE;
      break;

    case A3G4250D_FIFO_MODE:
      *val = A3G4250D_FIFO_MODE;
      break;

    case A3G4250D_FIFO_STREAM_MODE:
      *val = A3G4250D_FIFO_STREAM_MODE;
      break;

    default:
      *val = A3G4250D_FIFO_BYPASS_MODE;
      break;
  }

  return ret;
}

/**
  * @brief  FIFO stored data level[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fss in reg FIFO_SRC_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_fifo_data_level_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_fifo_src_reg_t fifo_src_reg;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_FIFO_SRC_REG,
                          (uint8_t *)&fifo_src_reg, 1);
  *val = fifo_src_reg.fss;
  return ret;
}

/**
  * @brief  FIFOemptybit.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of empty in reg FIFO_SRC_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_fifo_empty_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_fifo_src_reg_t fifo_src_reg;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_FIFO_SRC_REG,
                          (uint8_t *)&fifo_src_reg, 1);
  *val = fifo_src_reg.empty;
  return ret;
}

/**
  * @brief  Overrun bit status.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of ovrn in reg FIFO_SRC_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t a3g4250d_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_fifo_src_reg_t fifo_src_reg;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_FIFO_SRC_REG,
                          (uint8_t *)&fifo_src_reg, 1);
  *val = fifo_src_reg.ovrn;
  return ret;
}

/**
  * @brief  Watermark status:[get]
  *                0: FIFO filling is lower than WTM level;
  *                1: FIFO filling is equal or higher than WTM level)
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of wtm in reg FIFO_SRC_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */

int32_t a3g4250d_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  a3g4250d_fifo_src_reg_t fifo_src_reg;
  int32_t ret;
  ret = a3g4250d_read_reg(ctx, A3G4250D_FIFO_SRC_REG,
                          (uint8_t *)&fifo_src_reg, 1);
  *val = fifo_src_reg.wtm;
  return ret;
}

/**
  * @}
  *
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
