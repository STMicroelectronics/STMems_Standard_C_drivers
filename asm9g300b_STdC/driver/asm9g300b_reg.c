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
int32_t __weak asm9g300b_read_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data,
                                  uint16_t len)
{
  uint32_t frame;
  uint16_t d;
  int ret;

  if (ctx == NULL)
  {
    return -1;
  }

  ret = ctx->read_reg(ctx->handle, reg, (uint8_t *)&frame, len);
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
int32_t __weak asm9g300b_write_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint16_t *data,
                                   uint16_t len)
{
  uint32_t fr;

  if (ctx == NULL)
  {
    return -1;
  }

  fr = asm9g300b_gen_mosi_frame(reg, 1, *data);

  /* send frame. Pls note that 'reg' is already incapsulated in frame */
  return ctx->write_reg(ctx->handle, 0, (uint8_t *)&fr, 2);
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

  ret = asm9g300b_write_reg(ctx, ASM9G300B_CONFIG02, &val, 1);

  return ret;
}

int32_t asm9g300b_startup(const stmdev_ctx_t *ctx)
{
  int32_t ret;

  if (ctx == NULL || ctx->mdelay == NULL)
  {
    return -1;
  }

  ctx->mdelay(25);

  /* Perform hard reset */
  ret = asm9g300b_set_command(ctx, ASM9G300B_CMD_HARD_RESET);
  ctx->mdelay(20);

  asm9g300b_check_spi_communication(ctx);

  /* Configure Combo4  */
  //Combo4_WriteRegister(imu, FILTER_CONTROL, Filter); /* (Set Filter to 60Hz and SDO Drive=0x1C00 is default) */
  ctx->mdelay(10);

  return ret;
}

int32_t asm9g300b_check_spi_communication(const stmdev_ctx_t *ctx)
{
  int32_t ret;
  uint16_t val1, val2;

  val1 = 0x1234;
  ret = asm9g300b_write_reg(ctx, ASM9G300B_TEST_REG, &val1, 1);
  if (ret == -1) {
    return -1;
  }

  ret = asm9g300b_read_reg(ctx, ASM9G300B_TEST_REG, (uint8_t *)&val2, 1);
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
  * @}
  *
  */
