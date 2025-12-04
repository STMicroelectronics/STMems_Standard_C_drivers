/*
 ******************************************************************************
 * @file    asm9g300b_reg.c
 * @author  Sensors Software Solution Team
 * @brief   ASM9G300B driver file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <string.h>
#include "asm9g300b_reg.h"

/**
  * @defgroup    ASM9G300B
  * @brief       This file provides a set of functions needed to drive the
  *              asm9g300b sensor.
  * @{
  *
  */

/**
  * @defgroup    ASM9G300B_Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device and prepare SafeSPI
  *              MISO and MOSI frames.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/*
 * Calculate the CRC3 of a frame.
 */
static uint8_t asm9g300b_calc_crc3(uint32_t data, uint8_t init, uint8_t poly)
{
  uint8_t crc = init;

  for (int i = 31; i >= 0; --i) {
    // Bring in the next data bit
    uint8_t bit = (data >> i) & 1;

    // Shift in the data bit
    crc = ((crc << 1) | bit);

    // If the new MSB (bit 3) is set, apply the polynomial
    if (crc & 0x8)
      crc ^= poly;

    // Keep CRC to 3 bits (mask off the 4th bit)
    crc &= 0x7;
  }

  return crc;
}

/*
 * Generate SafeSPI MOSI frame
 */
static uint32_t asm9g300b_gen_mosi_frame(uint8_t ta, uint8_t rw, uint16_t data)
{
  uint32_t frame, crc3;

  frame = (ta << 22) | (rw << 21) | (data << 3);

  /* poly is (x^3 + x^1 + x^0) */
  crc3 = asm9g300b_calc_crc3(frame & ~0x7, 0x05, 0xB);

  return (frame | crc3);
}

/*
 * Decode SafeSPI MISO frame
 */
static int32_t asm9g300b_dec_miso_frame(uint32_t frame, uint16_t *data, uint8_t *state)
{
  uint32_t crc3;
  uint8_t s1, s0;

  /* poly is (x^3 + x^1 + x^0) */
  crc3 = asm9g300b_calc_crc3(frame & ~0x7, 0x05, 0xB);
  if (crc3 != (frame & 0x7))
  {
    return -1;
  }

  s1 = (frame >> 20) & 0x1;
  s0 = (frame >> 3) & 0x1;
  *state = (s1 << 1) | s0;
  *data = (frame >> 4) & 0xffff;

  return 0;
}

/*
 * Read frame from SafeSPI bus
 */
static int32_t asm9g300b_read_frame(const stmdev_ctx_t *ctx, uint8_t *reg, uint8_t *data)
{
  uint32_t frame;
  uint16_t d;
  uint8_t s;
  int ret;

  if (ctx == NULL)
  {
    return -1;
  }

  /* 'reg' argument is unused */
  ret = ctx->read_reg(ctx->handle, 0, (uint8_t *)&frame, 1);
  if (ret < 0)
  {
    return -1;
  }

  ret = asm9g300b_dec_miso_frame(frame, &d, &s);
  if (ret < 0)
  {
    return -1;
  }

  data[0] = d & 0xff;
  data[1] = (d >> 8) & 0xff;

  return ret;
}

/*
 * Write frame to SafeSPI bus
 */
static int32_t asm9g300b_write_frame(const stmdev_ctx_t *ctx, uint8_t rw, uint8_t reg, uint16_t data)
{
  uint32_t fr;

  if (ctx == NULL)
  {
    return -1;
  }

  fr = asm9g300b_gen_mosi_frame(reg, rw, data);

  /* send frame. Pls note that 'reg' is already incapsulated in frame */
  return ctx->write_reg(ctx->handle, 0, (uint8_t *)&fr, 1);
}

/**
  * @brief  Read raw data from reg channel
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  raw   buffer to retrieve data(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t __weak asm9g300b_read_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint16_t *raw)
{
  int32_t ret;
  uint16_t tmp = 0;
  uint8_t sa;

  ret = asm9g300b_write_frame(ctx, 0, reg, tmp);
  if (ret == -1) {
    return -1;
  }

  return asm9g300b_read_frame(ctx, &sa, (uint8_t *)raw);
}

/**
  * @brief  Write raw data to reg channel
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to be written
  * @param  raw   data to be written
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t __weak asm9g300b_write_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint16_t raw)
{
  return asm9g300b_write_frame(ctx, 1, reg, raw);
}

/**
  * @}
  *
  */

/**
  * @brief  Set Device Command
  *
  * @param  ctx      read / write interface definitions
  * @param  cmd      Device command to be executed
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_set_command(const stmdev_ctx_t *ctx, asm9g300b_commands_t cmd)
{
  uint16_t val = (uint16_t)cmd;
  int32_t ret;

  ret = asm9g300b_write_frame(ctx, 1, ASM9G300B_CONFIG02, val);

  return ret;
}

/**
  * @brief  Device startup
  *
  * @param  ctx   communication interface handler.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_startup(const stmdev_ctx_t *ctx)
{
  int32_t ret;
  asm9g300b_config01_t cfg01 = {0};
  asm9g300b_config04_t cfg04 = {0};
  asm9g300b_config05_t cfg05 = {0};
  asm9g300b_config06_t cfg06 = {0};
  uint16_t *cfgp = (uint16_t *)&cfg01;
  uint16_t tmp;
  asm9g300b_priv_t *cfg_priv;

  if (ctx == NULL || ctx->mdelay == NULL || ctx->priv_data == NULL)
  {
    return -1;
  }
  ctx->mdelay(25);

  cfg_priv = ctx->priv_data;

  /* Perform hard reset */
  ret = asm9g300b_set_command(ctx, ASM9G300B_CMD_HARD_RESET);
  ctx->mdelay(20);

  /* Configure filters and SDO strength */
  cfg01.sdo_drv = cfg_priv->sdo_drv;
  cfg01.iir_bw_sel_ax = cfg_priv->iir_bw_sel_ax;
  cfg01.iir_bw_sel_ay = cfg_priv->iir_bw_sel_ay;
  cfg01.iir_bw_sel_az = cfg_priv->iir_bw_sel_az;
  cfg01.iir_bw_sel_rx = cfg_priv->iir_bw_sel_rx;
  cfg01.iir_bw_sel_rz = cfg_priv->iir_bw_sel_rz;
  cfgp = (uint16_t *)&cfg01;
  ret = asm9g300b_write_frame(ctx, 1, ASM9G300B_CONFIG01, *cfgp);
  if (ret == -1) {
    return -1;
  }

  /* Configure debounce times */
  cfg04.tdebrng_ax = cfg_priv->t_debounce_ax;
  cfg04.tdebrng_ay = cfg_priv->t_debounce_ay;
  cfg04.tdebrng_az = cfg_priv->t_debounce_az;
  cfg04.tdebrng_rx = cfg_priv->t_debounce_rx;
  cfg04.tdebrng_rz = cfg_priv->t_debounce_rz;
  cfgp = (uint16_t *)&cfg04;
  ret = asm9g300b_write_frame(ctx, 1, ASM9G300B_CONFIG04, *cfgp);
  if (ret == -1) {
    return -1;
  }

  /* Configure Z clamp */
  cfg05.zclampconfig = cfg_priv->z_clamp;
  cfgp = (uint16_t *)&cfg05;
  ret = asm9g300b_write_frame(ctx, 1, ASM9G300B_CONFIG05, *cfgp);
  if (ret == -1) {
    return -1;
  }

  /* Configure leftovers */
  cfg06.iir_bw_sel_ry = cfg_priv->iir_bw_sel_ry;
  cfg06.tdebrng_ry = cfg_priv->t_debounce_ry;
  cfgp = (uint16_t *)&cfg06;
  ret = asm9g300b_write_frame(ctx, 1, ASM9G300B_CONFIG06, *cfgp);
  if (ret == -1) {
    return -1;
  }

  ctx->mdelay(10);

  /* wake-up sensor */
  ret = asm9g300b_set_command(ctx, ASM9G300B_CMD_WAKE_UP);
  if (ret == -1) {
    return -1;
  }
  ctx->mdelay(350);

  /* read all status register to clear the events */
  asm9g300b_read_reg(ctx, ASM9G300B_SSUMOK, &tmp);
  asm9g300b_read_reg(ctx, ASM9G300B_SSUMRNG, &tmp);
  asm9g300b_read_reg(ctx, ASM9G300B_STATARSX, &tmp);
  asm9g300b_read_reg(ctx, ASM9G300B_STATARSY, &tmp);
  asm9g300b_read_reg(ctx, ASM9G300B_STATARSZ, &tmp);
  asm9g300b_read_reg(ctx, ASM9G300B_STATACCX, &tmp);
  asm9g300b_read_reg(ctx, ASM9G300B_STATACCY, &tmp);
  asm9g300b_read_reg(ctx, ASM9G300B_STATACCZ, &tmp);
  asm9g300b_read_reg(ctx, ASM9G300B_STATCOM1, &tmp);
  asm9g300b_read_reg(ctx, ASM9G300B_STATCOM2, &tmp);
  ctx->mdelay(10);

  if (cfg_priv->disable_auto_self_test == 0) {
    /* Invoke Acc/Ars Auto self-test */
    ret = asm9g300b_set_command(ctx, ASM9G300B_CMD_ALL_AUTO_SELF_TEST);
    if (ret == -1) {
      return -1;
    }

    /* check self-test result */
    uint32_t st_status = 0, retry = 0;
    do {
      ctx->mdelay(50);
      asm9g300b_st_status_get(ctx, &st_status);
    } while((st_status & ASM9G300B_ST_STATUS_MASK) != ASM9G300B_ST_STATUS_OK && retry++ < 5);

    if (retry >= 5)
    {
      return -1;
    }
  }

  /* Put sensor in Normal mode */
  ret = asm9g300b_set_command(ctx, ASM9G300B_CMD_EOI);
  if (ret == -1) {
    return -1;
  }

  return ret;
}

/**
  * @brief  Check  communication on SPI bus
  *
  * @param  ctx   communication interface handler.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_check_spi_communication(const stmdev_ctx_t *ctx)
{
  int32_t ret;
  uint16_t val1, val2 = 0;
  uint8_t reg = 0;

  val1 = 0x1234;

  ret = asm9g300b_write_frame(ctx, 1, ASM9G300B_TEST_REG, val1);
  if (ret == -1) {
    return -1;
  }

  val2 = 0;
  ret = asm9g300b_read_frame(ctx, &reg, (uint8_t *)&val2);
  if (ret == -1) {
    return -1;
  }

  if (val1 != val2)
  {
    return -1;
  }

  return ret;
}

/**
  * @brief  Get Device Summary Status register
  *
  * @param  ctx    communication interface handler.(ptr)
  * @param  status Summary status register content .(ptr)
  * @retval        interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_summary_status_get(const stmdev_ctx_t *ctx, uint16_t *status)
{
  return asm9g300b_read_reg(ctx, ASM9G300B_SSUMOK, status);
}

/**
  * @brief  Get Device Summary Signal Range Status register
  *
  * @param  ctx    communication interface handler.(ptr)
  * @param  status Summary signal range status register content .(ptr)
  * @retval        interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_summary_sig_range_status_get(const stmdev_ctx_t *ctx, uint16_t *status)
{
  return asm9g300b_read_reg(ctx, ASM9G300B_SSUMRNG, status);
}

/**
  * @brief  Get ARS Status registers
  *
  * @param  ctx    communication interface handler.(ptr)
  * @param  status ARS status registers content .(ptr to 3 uint16_t)
  * @retval        interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_ars_status_get(const stmdev_ctx_t *ctx, uint16_t *status)
{
  int32_t ret;

  ret = asm9g300b_read_reg(ctx, ASM9G300B_STATARSX, &status[0]);
  ret += asm9g300b_read_reg(ctx, ASM9G300B_STATARSY, &status[1]);
  ret += asm9g300b_read_reg(ctx, ASM9G300B_STATARSZ, &status[2]);

  return ret;
}

/**
  * @brief  Get ACC Status registers
  *
  * @param  ctx    communication interface handler.(ptr)
  * @param  status ACC status registers content .(ptr to 3 uint16_t)
  * @retval        interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_acc_status_get(const stmdev_ctx_t *ctx, uint16_t *status)
{
  int32_t ret;

  ret = asm9g300b_read_reg(ctx, ASM9G300B_STATACCX, &status[0]);
  ret += asm9g300b_read_reg(ctx, ASM9G300B_STATACCY, &status[1]);
  ret += asm9g300b_read_reg(ctx, ASM9G300B_STATACCZ, &status[2]);

  return ret;
}

/**
  * @brief  Get Common Mode Status register
  *         It chains StatCOM1 and StatCOM2 together
  *
  * @param  ctx    communication interface handler.(ptr)
  * @param  status Summary status register content .(ptr)
  * @retval        interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_com_status_get(const stmdev_ctx_t *ctx, uint32_t *status)
{
  int32_t ret;
  uint16_t temp1, temp2;

  ret = asm9g300b_read_reg(ctx, ASM9G300B_STATCOM1, &temp1);
  ret += asm9g300b_read_reg(ctx, ASM9G300B_STATCOM2, &temp2);

  *status = (temp2 << 16) | temp1;

  return ret;
}

/**
  * @brief  Get Self-Test Status registers
  *         It chains ST_STATUS_ARS and ST_STATUS_ACC together
  *
  * @param  ctx    communication interface handler.(ptr)
  * @param  status Summary status register content .(ptr)
  * @retval        interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_st_status_get(const stmdev_ctx_t *ctx, uint32_t *status)
{
  int32_t ret;
  uint16_t temp1, temp2;

  ret = asm9g300b_read_reg(ctx, ASM9G300B_ST_STATUS_ARS, &temp1);
  ret += asm9g300b_read_reg(ctx, ASM9G300B_ST_STATUS_ACC, &temp2);

  *status = (temp2 << 16) | temp1;

  return ret;
}

int32_t asm9g300b_from_ars_lsb_to_mdps(int16_t lsb)
{
  return 20 * lsb; /* unit is mdps */
}

int32_t asm9g300b_from_acc_lsb_to_mms2(int16_t lsb)
{
  return 5 * lsb; /* unit is mm/s^2 */
}

int32_t asm9g300b_from_mgp_lsb_to_mms2(int16_t lsb)
{
  return 40 * lsb; /* unit is mm/s^2 */
}

int32_t asm9g300b_from_temp_lsb_to_celsius(int16_t lsb)
{
  return (25000 + 40 * lsb); /* unit is milliCelsius degrees */
}

/**
  * @brief  Angular rate data.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  raw   lsb data retrived from the sensor.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_ars_data_get(const stmdev_ctx_t *ctx, int16_t *raw)
{
  int32_t ret;

  ret = asm9g300b_read_reg(ctx, ASM9G300B_ARSX, (uint16_t *)&raw[0]);
  ret += asm9g300b_read_reg(ctx, ASM9G300B_ARSY, (uint16_t *)&raw[1]);
  ret += asm9g300b_read_reg(ctx, ASM9G300B_ARSZ, (uint16_t *)&raw[2]);

  return ret;
}

/**
  * @brief  Low-g Accelerometer data.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  raw   lsb data retrived from the sensor.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_acc_data_get(const stmdev_ctx_t *ctx, int16_t *raw)
{
  int32_t ret;

  ret = asm9g300b_read_reg(ctx, ASM9G300B_ACCX, (uint16_t *)&raw[0]);
  ret += asm9g300b_read_reg(ctx, ASM9G300B_ACCY, (uint16_t *)&raw[1]);
  ret += asm9g300b_read_reg(ctx, ASM9G300B_ACCZ, (uint16_t *)&raw[2]);

  return ret;
}

/**
  * @brief  Mid-g Accelerometer data.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  raw   lsb data retrived from the sensor.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_mgp_data_get(const stmdev_ctx_t *ctx, int16_t *raw)
{
  int32_t ret;

  ret = asm9g300b_read_reg(ctx, ASM9G300B_MGPX, (uint16_t *)&raw[0]);
  ret += asm9g300b_read_reg(ctx, ASM9G300B_MGPY, (uint16_t *)&raw[1]);
  ret += asm9g300b_read_reg(ctx, ASM9G300B_MGPZ, (uint16_t *)&raw[2]);

  return ret;
}

/**
  * @brief  Temperature data.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  raw   lsb data retrived from the sensor.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_temp_data_get(const stmdev_ctx_t *ctx, int16_t *raw)
{
  return asm9g300b_read_reg(ctx, ASM9G300B_TEMP, (uint16_t *)raw);
}

/**
  * @brief  Get Device serial number.[get]
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  s_num Serial number.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm9g300b_getSerialNum(const stmdev_ctx_t *ctx, uint32_t *s_num)
{
  uint16_t tmp0 = 0, tmp1 = 0;
  int32_t ret;

  ret = asm9g300b_read_reg(ctx, ASM9G300B_ASIC_ID, &tmp0);
  ret += asm9g300b_read_reg(ctx, ASM9G300B_ID_TRACKING2, &tmp1);
  if (ret == -1) {
    return -1;
  }

  *s_num = ((tmp0 & 0xf800) << 5) + tmp1;

  return ret;
}

/**
  * @}
  *
  */
