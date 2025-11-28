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

/**
  * @brief  Generate SafeSPI MOSI frame
  *
  * @param  ta    register to access
  * @param  read  0: read request, 1: write request
  * @param  data  data to be sent
  * @retval       the MOSI frame
  *
  */
uint32_t asm9g300b_gen_mosi_frame(uint8_t ta, uint8_t rw, uint16_t data)
{
  uint32_t frame, crc3;

  frame = (ta << 22) | (rw << 21) | (data << 3);

  /* poly is (x^3 + x^1 + x^0) */
  crc3 = asm9g300b_calc_crc3(frame & ~0x7, 0x05, 0xB);

  return (frame | crc3);
}

/**
  * @brief  Decode SafeSPI MISO frame
  *
  * @param  ta    register to access
  * @param  read  0: read request, 1: write request
  * @param  data  data to be sent
  * @retval       the MOSI frame
  *
  */
int32_t asm9g300b_dec_miso_frame(uint32_t frame, uint16_t *data)
{
  uint32_t crc3;

  /* poly is (x^3 + x^1 + x^0) */
  crc3 = asm9g300b_calc_crc3(frame & ~0x7, 0x05, 0xB);
  if (crc3 != (frame & 0x7))
  {
    return -1;
  }

  *data = (frame >> 4) & 0xffff;

  return 0;
}

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
int32_t __weak asm9g300b_read_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data)
{
  uint32_t frame;
  uint16_t d;
  int ret;

  if (ctx == NULL)
  {
    return -1;
  }

  ret = ctx->read_reg(ctx->handle, reg, (uint8_t *)&frame, 1);
  if (ret < 0)
  {
    return -1;
  }

  ret = asm9g300b_dec_miso_frame(frame, &d);
  if (ret < 0)
  {
    return -1;
  }

  data[0] = d & 0xff;
  data[1] = (d >> 8) & 0xff;

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
int32_t __weak asm9g300b_write_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint16_t data)
{
  uint32_t fr;

  if (ctx == NULL)
  {
    return -1;
  }

  fr = asm9g300b_gen_mosi_frame(reg, 1, data);

  /* send frame. Pls note that 'reg' is already incapsulated in frame */
  return ctx->write_reg(ctx->handle, 0, (uint8_t *)&fr, 1);
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

  ret = asm9g300b_write_reg(ctx, ASM9G300B_CONFIG02, val);

  return ret;
}

int32_t asm9g300b_startup(const stmdev_ctx_t *ctx)
{
  int32_t ret;
  asm9g300b_config01_t cfg = {0};
  uint16_t *cfgp = (uint16_t *)&cfg;
  uint16_t tmp;

  if (ctx == NULL || ctx->mdelay == NULL)
  {
    return -1;
  }

  ctx->mdelay(25);

  /* Perform hard reset */
  ret = asm9g300b_set_command(ctx, ASM9G300B_CMD_HARD_RESET);
  ctx->mdelay(20);

  /* Configure filters and SDO strength  */
  cfg.sdo_drv = 0;
  cfg.iir_bw_sel_ax = ASM9G300B_IIR_FILTER_60HZ;
  cfg.iir_bw_sel_ay = ASM9G300B_IIR_FILTER_60HZ;
  cfg.iir_bw_sel_az = ASM9G300B_IIR_FILTER_60HZ;
  cfg.iir_bw_sel_rx = ASM9G300B_IIR_FILTER_60HZ;
  cfg.iir_bw_sel_rz = ASM9G300B_IIR_FILTER_60HZ;
  ret = asm9g300b_write_reg(ctx, ASM9G300B_CONFIG01, *cfgp);
  if (ret == -1) {
    return -1;
  }

  ctx->mdelay(10);

  /* wake-up sensor */
  ret = asm9g300b_set_command(ctx, ASM9G300B_CMD_WAKE_UP);
  ctx->mdelay(350);

  /* read all status register to clear the events */
  asm9g300b_read_reg(ctx, ASM9G300B_SSUMOK, (uint8_t *)&tmp);
  asm9g300b_read_reg(ctx, ASM9G300B_SSUMRNG, (uint8_t *)&tmp);

  /* Put sensor in Normal mode */
  ret = asm9g300b_set_command(ctx, ASM9G300B_CMD_EOI);
  ctx->mdelay(350);

  return ret;
}

int32_t asm9g300b_check_spi_communication(const stmdev_ctx_t *ctx)
{
  int32_t ret;
  uint16_t val1, val2 = 0;
  uint8_t i;

  val1 = 0x1234;

  do {
    ret = asm9g300b_write_reg(ctx, ASM9G300B_TEST_REG, val1);
    if (ret == -1) {
      return -1;
    }

    val2 = 0;
    ret = asm9g300b_read_reg(ctx, ASM9G300B_TEST_REG, (uint8_t *)&val2);
    if (ret == -1) {
      return -1;
    }

    if (val1 != val2)
    {
      return -1;
    }

    val1 += 0x1111;
  } while (i++ < 5);

  return ret;
}

/**
  * @}
  *
  */
