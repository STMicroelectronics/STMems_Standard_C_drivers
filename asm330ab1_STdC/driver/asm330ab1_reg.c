/*
 ******************************************************************************
 * @file    asm330ab1_reg.c
 * @author  Sensors Software Solution Team
 * @brief   ASM330AB1 driver file
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
#include "asm330ab1_reg.h"

/**
  * @defgroup    ASM330AB1
  * @brief       This file provides a set of functions needed to drive the
  *              asm330ab1 sensor.
  * @{
  *
  */

/**
  * @defgroup    ASM330AB1_Interfaces_Functions
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
static uint8_t asm330ab1_calc_crc3(uint32_t data, uint8_t init, uint8_t poly)
{
  uint8_t crc = init;

  for (int i = 31; i >= 0; --i)
  {
    // Bring in the next data bit
    uint8_t bit = (data >> i) & 1;

    // Shift in the data bit
    crc = ((crc << 1) | bit);

    // If the new MSB (bit 3) is set, apply the polynomial
    if (crc & 0x8)
    {
      crc ^= poly;
    }

    // Keep CRC to 3 bits (mask off the 4th bit)
    crc &= 0x7;
  }

  return crc;
}

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint32_t crc3   :  3;
  uint32_t data   :  8;
  uint32_t xxx    :  8;
  uint32_t unused :  1;
  uint32_t cap    :  1;
  uint32_t rw     :  1;
  uint32_t ta     : 10;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint32_t ta     : 10;
  uint32_t rw     :  1;
  uint32_t cap    :  1;
  uint32_t unused :  1;
  uint32_t xxx    :  8;
  uint32_t data   :  8;
  uint32_t crc3   :  3;
#endif /* DRV_BYTE_ORDER */
} safespi_mosi_frame_t;

/*
 * Generate SafeSPI MOSI frame
 *
 * 31                22  21 20  19  18            11 10            3 2     0
 * +-------------------+---+---+---+----------------+---------------+------+
 * |      TA9:0        |RW |CAP| 0 |      XXX       |   DATAI7:0    | C2:0 |
 * +-------------------+---+---+---+----------------+---------------+------+
 *
 */
static uint32_t asm330ab1_gen_mosi_frame(uint16_t ta, uint8_t rw, uint16_t data)
{
  uint32_t frame = 0;
  safespi_mosi_frame_t *fp = (safespi_mosi_frame_t *)&frame;

  fp->ta = ta;
  fp->rw = rw;
  fp->data = data;

  /* poly is (x^3 + x^1 + x^0) */
  fp->crc3 = asm330ab1_calc_crc3(frame, 0x05, 0xB);

  return frame;
}

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint32_t crc3   :  3;
  uint32_t s0     :  1;
  uint32_t data   : 16;
  uint32_t s1     :  1;
  uint32_t sa     : 10;
  uint32_t d      :  1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint32_t d      :  1;
  uint32_t sa     : 10;
  uint32_t s1     :  1;
  uint32_t data   : 16;
  uint32_t s0     :  1;
  uint32_t crc3   :  3;
#endif /* DRV_BYTE_ORDER */
} safespi_miso_frame_t;

/*
 * Decode SafeSPI MISO frame
 *
 *  31  30                21 20  19                            4  3  2     0
 * +---+--------------------+---+-------------------------------+---+------+
 * | D |        SA9:0       |S1 |           DATAI15:0           | S0| C2:0 |
 * +---+--------------------+---+-------------------------------+---+------+
 */
static int32_t asm330ab1_dec_miso_frame(uint32_t frame, uint16_t *data, uint8_t *state)
{
  uint32_t crc3;
  safespi_miso_frame_t *fp = (safespi_miso_frame_t *)&frame;

  /* poly is (x^3 + x^1 + x^0) */
  crc3 = asm330ab1_calc_crc3(frame & ~0x7, 0x05, 0xB);
  if (crc3 != fp->crc3)
  {
    return -1;
  }

  *state = (fp->s1 << 1) | fp->s0;
  *data = fp->data;

  return 0;
}

/*
 * Read frame from SafeSPI bus
 */
static int32_t asm330ab1_recv_rsp_frame(const stmdev_ctx_t *ctx, uint8_t *reg, uint8_t *data)
{
  uint32_t frame = 0;
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

  ret = asm330ab1_dec_miso_frame(frame, &d, &s);
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
static int32_t asm330ab1_send_req_frame(const stmdev_ctx_t *ctx, uint8_t rw, uint8_t reg,
                                        uint16_t data)
{
  uint32_t fr;
  uint16_t ta = reg;
  asm330ab1_priv_t *priv = ctx->priv_data;

  if (ctx == NULL)
  {
    return -1;
  }

  if (priv && priv->ta9 == 1)
  {
    ta |= 0x200;
  }

  fr = asm330ab1_gen_mosi_frame(ta, rw, data);

  /* send frame. Pls note that 'reg' is already incapsulated in frame */
  return ctx->write_reg(ctx->handle, 0, (uint8_t *)&fr, 1);
}

/**
  * @brief  Read raw data from reg channel
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t __weak asm330ab1_read_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                                  uint8_t *data,
                                  uint16_t len)
{
  asm330ab1_priv_t *cfg;

  if (ctx == NULL || ctx->priv_data == NULL)
  {
    return -1;
  }

  cfg = ctx->priv_data;

  if (cfg->use_safespi_bus)
  {
    uint16_t tmp = 0;
    uint8_t sa;
    int32_t ret;

    ret = asm330ab1_send_req_frame(ctx, 0, reg, tmp);
    if (ret == -1)
    {
      return -1;
    }

    return asm330ab1_recv_rsp_frame(ctx, &sa, data);
  }
  else
  {
    return ctx->read_reg(ctx->handle, reg, data, len);
  }
}

/**
  * @brief  Write raw data to reg channel
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to be written
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t __weak asm330ab1_write_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                                   uint8_t *data,
                                   uint16_t len)
{
  asm330ab1_priv_t *cfg;

  if (ctx == NULL || ctx->priv_data == NULL)
  {
    return -1;
  }

  cfg = ctx->priv_data;

  if (cfg->use_safespi_bus)
  {
    uint16_t raw = 0;

    memcpy((uint8_t *)&raw, data, sizeof(uint16_t));
    return asm330ab1_send_req_frame(ctx, 1, reg, raw);
  }
  else
  {
    return ctx->write_reg(ctx->handle, reg, data, len);
  }
}

/**
  * @}
  *
  */

/**
  * @defgroup  ASM330AB1_Private_functions
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

/**
  * @defgroup  Sensitivity
  * @brief     These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t asm330ab1_from_fs2g_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.061f;
}

float_t asm330ab1_from_fs4g_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.122f;
}

float_t asm330ab1_from_fs8g_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.244f;
}

float_t asm330ab1_from_fs16g_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.488f;
}

float_t asm330ab1_from_fs125_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 4.370f;
}

float_t asm330ab1_from_fs250_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 8.750f;
}

float_t asm330ab1_from_fs500_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 17.5f;
}

float_t asm330ab1_from_fs1000_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 35.0f;
}

float_t asm330ab1_from_fs2000_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 70.0f;
}

float_t asm330ab1_from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb / 256.0f) + 25.0f);
}

/**
  * @}
  *
  */

/**
  * @defgroup  Basic functions
  * @brief     This section groups all the functions concerning
  *            device basic configuration.
  * @{
  *
  */

/**
  * @brief  Device Who am I.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  id     Buffer to read the who_am_i identificator
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_device_id_get(const stmdev_ctx_t *ctx, uint8_t *id)
{
  return asm330ab1_read_reg(ctx, ASM330AB1_WHO_AM_I, id, 1);
}

/**
  * @brief  Perform reboot of the device.
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_reboot(const stmdev_ctx_t *ctx)
{
  asm330ab1_ctrl3_t ctrl3 = {0};
  uint8_t retry = 0;
  int32_t ret;

  if (ctx->mdelay == NULL)
  {
    ret = -1;
    goto exit;
  }

  ret = asm330ab1_read_reg(ctx, ASM330AB1_CTRL3, (uint8_t *)&ctrl3, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* Set the BOOT bit of the CTRL3 register to 1. */
  ctrl3.boot = 1;
  ret = asm330ab1_write_reg(ctx, ASM330AB1_CTRL3, (uint8_t *)&ctrl3, 1);

  /* Poll the BOOT bit of the CTRL3 register until it returns to 0. */
  do
  {
    ret += asm330ab1_read_reg(ctx, ASM330AB1_CTRL3, (uint8_t *)&ctrl3, 1);
    if (ret != 0)
    {
      goto exit;
    }

    ctx->mdelay(1);
  } while (ctrl3.boot == 1 && retry++ < 3);

  ret = (ctrl3.boot == 0) ? 0 : -1;

exit:
  return ret;
}

/**
  * @brief Global reset of the device: power-on reset.
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_sw_por(const stmdev_ctx_t *ctx)
{
  asm330ab1_page_sel_t page_sel = {0};
  int32_t ret;

  if (ctx == NULL || ctx->mdelay == NULL)
  {
    ret = -1;
    goto exit;
  }

  /* 1. Set the SW_POR bit to 1. */
  page_sel.sw_por = 1;
  ret = asm330ab1_write_reg(ctx, ASM330AB1_PAGE_SEL, (uint8_t *)&page_sel, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* 2. Wait 40 ms. */
  ctx->mdelay(40);

exit:
  return ret;
}

/**
  * @brief  s/w reset of the device
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_sw_reset(const stmdev_ctx_t *ctx)
{
  asm330ab1_ctrl3_t ctrl3 = {0};
  uint8_t retry = 0;
  int32_t ret;

  if (ctx->mdelay == NULL)
  {
    ret = -1;
    goto exit;
  }

  /* Set the SW_RESET bit of the CTRL3 register to 1. */
  ctrl3.sw_reset = 1;
  ret = asm330ab1_write_reg(ctx, ASM330AB1_CTRL3, (uint8_t *)&ctrl3, 1);

  /* Poll the SW_RESET bit of the CTRL3 register until it returns to 0. */
  do
  {
    ret += asm330ab1_read_reg(ctx, ASM330AB1_CTRL3, (uint8_t *)&ctrl3, 1);
    if (ret != 0)
    {
      goto exit;
    }

    ctx->mdelay(1);
  } while (ctrl3.sw_reset == 1 && retry++ < 3);

  ret = (ctrl3.sw_reset == 0) ? 0 : -1;

exit:
  return ret;
}

/**
  * @brief  Page selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  page   page to be set (ASM330AB1_MAIN_PAGE/ASM330AB1_FUSA_PAGE)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_page_sel_set(const stmdev_ctx_t *ctx, uint8_t page)
{
  asm330ab1_page_sel_t reg;
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_PAGE_SEL, (uint8_t *)&reg, 1);
  if (ret != 0)
  {
    goto exit;
  }

  reg.page_sel = page;

  ret = asm330ab1_write_reg(ctx, ASM330AB1_PAGE_SEL, (uint8_t *)&reg, 1);

exit:
  return ret;
}

/**
  * @brief  Page selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  page   page to be set (ASM330AB1_MAIN_PAGE/ASM330AB1_FUSA_PAGE)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_page_sel_get(const stmdev_ctx_t *ctx, uint8_t *page)
{
  asm330ab1_page_sel_t reg;
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_PAGE_SEL, (uint8_t *)&reg, 1);
  *page = reg.page_sel;

  return ret;
}

/**
  * @brief  Set End-Of-Interrupt condition
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_eoi_set(const stmdev_ctx_t *ctx)
{
  asm330ab1_page_sel_t reg;
  asm330ab1_fusa_status_t status;
  uint8_t retry = 0;
  int32_t ret;

  if (ctx->mdelay == NULL)
  {
    ret = -1;
    goto exit;
  }

  ret = asm330ab1_read_reg(ctx, ASM330AB1_PAGE_SEL, (uint8_t *)&reg, 1);
  if (ret != 0)
  {
    goto exit;
  }

  reg.eoi = 1;

  ret = asm330ab1_write_reg(ctx, ASM330AB1_PAGE_SEL, (uint8_t *)&reg, 1);

  /* Poll the eoi_fusa_status bit of the ASM330AB1_FUSA_STATUS register until op is completed. */
  do
  {
    ctx->mdelay(100);

    ret += asm330ab1_read_reg(ctx, ASM330AB1_FUSA_STATUS, (uint8_t *)&status, 1);
    if (ret != 0)
    {
      goto exit;
    }
  } while (status.eoi_fusa_status == 0 && retry++ < 3);

  ret = (status.eoi_fusa_status == 1) ? 0 : -1;

exit:
  return ret;
}

/**
  * @brief  Lock/unlock device user pages.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    0: unlock pages, 1: lock pages
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_pages_lock(const stmdev_ctx_t *ctx, uint8_t val)
{
  asm330ab1_page_sel_t reg;
  asm330ab1_fusa_status_t status;
  uint8_t retry = 0;
  int32_t ret;

  if (ctx->mdelay == NULL)
  {
    ret = -1;
    goto exit;
  }

  ret = asm330ab1_read_reg(ctx, ASM330AB1_PAGE_SEL, (uint8_t *)&reg, 1);
  if (ret != 0)
  {
    goto exit;
  }

  reg.lock = val;

  ret = asm330ab1_write_reg(ctx, ASM330AB1_PAGE_SEL, (uint8_t *)&reg, 1);

  /* Poll the lock_fusa_status bit of the ASM330AB1_FUSA_STATUS register until op is completed. */
  do
  {
    ctx->mdelay(1);

    ret += asm330ab1_read_reg(ctx, ASM330AB1_FUSA_STATUS, (uint8_t *)&status, 1);
    if (ret != 0)
    {
      goto exit;
    }
  } while (status.lock_fusa_status != val && retry++ < 3);

  ret = (status.lock_fusa_status == val) ? 0 : -1;

exit:
  return ret;
}

/**
  * @brief  Check  communication on SPI bus
  *
  * @param  ctx   communication interface handler.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_check_spi_communication(const stmdev_ctx_t *ctx)
{
  int32_t ret;
  uint16_t val1, val2 = 0;
  uint8_t reg = 0;

  val1 = 0xAB;

  ret = asm330ab1_send_req_frame(ctx, 1, ASM330AB1_TEST_IF, val1);
  if (ret == -1)
  {
    return -1;
  }

  val2 = 0;
  ret = asm330ab1_recv_rsp_frame(ctx, &reg, (uint8_t *)&val2);
  if (ret == -1)
  {
    return -1;
  }

  if (val1 != val2)
  {
    return -1;
  }

  return ret;
}

/**
  * @brief  Configure PIN characteristics.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    pin configuration structure.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_pin_conf_set(const stmdev_ctx_t *ctx, const asm330ab1_pin_conf_t *val)
{
  asm330ab1_pin_ctrl0_t ctrl0;
  asm330ab1_pin_ctrl1_t ctrl1;
  uint8_t reg[2];

  ctrl0.scl_sda_pd_dis = val->scl_sda_pd_dis;
  ctrl0.int1_pd_dis    = val->int1_pd_dis;
  ctrl0.int2_pd_dis    = val->int2_pd_dis;
  ctrl0.int3_pd_dis    = val->int3_pd_dis;
  ctrl0.scl_sda_pu_en  = val->scl_sda_pu_en;
  ctrl0.cs_pu_dis      = val->cs_pu_dis;
  ctrl0.ssd_pu_dis     = val->ssd_pu_dis;

  ctrl1.pu_dis_ta0     = val->pu_dis_ta0;
  ctrl1.ibhr_por_en    = val->ibhr_por_en;
  ctrl1.sdo_pu_en      = val->sdo_pu_en;
  ctrl1.sdo_drive      = val->sdo_drive;

  bytecpy(&reg[0], (uint8_t *)&ctrl0);
  bytecpy(&reg[1], (uint8_t *)&ctrl1);

  return asm330ab1_write_reg(ctx, ASM330AB1_PIN_CTRL0, reg, 2);
}

/**
  * @brief  Configure PIN characteristics.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    pin configuration structure.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_pin_conf_get(const stmdev_ctx_t *ctx, asm330ab1_pin_conf_t *val)
{
  asm330ab1_pin_ctrl0_t ctrl0;
  asm330ab1_pin_ctrl1_t ctrl1;
  uint8_t reg[2];
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_PIN_CTRL0, reg, 2);
  if (ret != 0)
  {
    goto exit;
  }

  bytecpy((uint8_t *)&ctrl0, &reg[0]);
  bytecpy((uint8_t *)&ctrl1, &reg[1]);

  val->scl_sda_pd_dis = ctrl0.scl_sda_pd_dis;
  val->int1_pd_dis    = ctrl0.int1_pd_dis;
  val->int2_pd_dis    = ctrl0.int2_pd_dis;
  val->int3_pd_dis    = ctrl0.int3_pd_dis;
  val->scl_sda_pu_en  = ctrl0.scl_sda_pu_en;
  val->cs_pu_dis      = ctrl0.cs_pu_dis;
  val->ssd_pu_dis     = ctrl0.ssd_pu_dis;
  val->pu_dis_ta0     = ctrl1.pu_dis_ta0;
  val->ibhr_por_en    = ctrl1.ibhr_por_en;
  val->sdo_pu_en      = ctrl1.sdo_pu_en;

  switch (ctrl1.sdo_drive)
  {
    case 0:
      val->sdo_drive = ASM330AB1_SDO_DRIVE_10PF_14PF;
      break;
    case 2:
      val->sdo_drive = ASM330AB1_SDO_DRIVE_15PF_29PF;
      break;
    case 1:
      val->sdo_drive = ASM330AB1_SDO_DRIVE_20PF_54PF;
      break;
    default:
    case 3:
      val->sdo_drive = ASM330AB1_SDO_DRIVE_55PF_100PF;
      break;
  }

exit:
  return ret;
}

/**
  * @brief  Provide CRC for a SPI write operation
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    CRC value
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_crc_spi_write_ops(const stmdev_ctx_t *ctx, uint8_t *crc)
{
  return asm330ab1_write_reg(ctx, ASM330AB1_CRC_W, crc, 1);
}

/**
  * @brief  Retrieve CRC in a SPI read operation
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    CRC value
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_crc_spi_read_ops(const stmdev_ctx_t *ctx, uint8_t *crc)
{
  return asm330ab1_read_reg(ctx, ASM330AB1_CRC_R, crc, 1);
}

/**
  * @brief  XL data rate.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Data rate value: struct asm330ab1_odr_t
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_xl_data_rate_set(const stmdev_ctx_t *ctx, asm330ab1_odr_t val)
{
  asm330ab1_ctrl1_t ctrl1;
  uint8_t cfg = (val & 0xf0) >> 4;
  uint8_t odr = val & 0x0f;
  int32_t ret;

  ret = asm330ab1_haodr_cfg_set(ctx, (asm330ab1_haodr_cfg_val_t)cfg);
  if (ret != 0)
  {
    goto exit;
  }

  ctrl1.op_mode_xl = ASM330AB1_HA_MODE;
  ctrl1.odr_xl = odr;
  ret = asm330ab1_write_reg(ctx, ASM330AB1_CTRL1, (uint8_t *)&ctrl1, 1);

exit:
  return ret;
}

/**
  * @brief  XL data rate.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Data rate value: struct asm330ab1_odr_t
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_xl_data_rate_get(const stmdev_ctx_t *ctx, asm330ab1_odr_t *val)
{
  asm330ab1_ctrl1_t ctrl1;
  asm330ab1_haodr_cfg_val_t cfg;
  int32_t ret;

  ret = asm330ab1_haodr_cfg_get(ctx, &cfg);
  ret += asm330ab1_read_reg(ctx, ASM330AB1_CTRL1, (uint8_t *)&ctrl1, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* build up the asm330ab1_odr_t value */
  switch ((cfg << 4) | ctrl1.odr_xl)
  {
    case 0x00:
      *val = ASM330AB1_HA00_ODR_POWER_DOWN;
      break;
    case 0x03:
      *val = ASM330AB1_HA00_ODR_AT_15Hz;
      break;
    case 0x04:
      *val = ASM330AB1_HA00_ODR_AT_30Hz;
      break;
    case 0x05:
      *val = ASM330AB1_HA00_ODR_AT_60Hz;
      break;
    case 0x06:
      *val = ASM330AB1_HA00_ODR_AT_120Hz;
      break;
    case 0x07:
      *val = ASM330AB1_HA00_ODR_AT_240Hz;
      break;
    case 0x08:
      *val = ASM330AB1_HA00_ODR_AT_480Hz;
      break;
    case 0x09:
      *val = ASM330AB1_HA00_ODR_AT_960Hz;
      break;
    case 0x0A:
      *val = ASM330AB1_HA00_ODR_AT_1920Hz;
      break;
    case 0x0B:
      *val = ASM330AB1_HA00_ODR_AT_3840Hz;
      break;

    case 0x20:
      *val = ASM330AB1_HA02_ODR_POWER_DOWN;
      break;
    case 0x23:
      *val = ASM330AB1_HA02_ODR_AT_12Hz5;
      break;
    case 0x24:
      *val = ASM330AB1_HA02_ODR_AT_25Hz;
      break;
    case 0x25:
      *val = ASM330AB1_HA02_ODR_AT_50Hz;
      break;
    case 0x26:
      *val = ASM330AB1_HA02_ODR_AT_100Hz;
      break;
    case 0x27:
      *val = ASM330AB1_HA02_ODR_AT_200Hz;
      break;
    case 0x28:
      *val = ASM330AB1_HA02_ODR_AT_400Hz;
      break;
    case 0x29:
      *val = ASM330AB1_HA02_ODR_AT_800Hz;
      break;
    case 0x2A:
      *val = ASM330AB1_HA02_ODR_AT_1600Hz;
      break;
    case 0x2B:
      *val = ASM330AB1_HA02_ODR_AT_3200Hz;
      break;

    case 0x30:
      *val = ASM330AB1_HA03_ODR_POWER_DOWN;
      break;
    case 0x33:
      *val = ASM330AB1_HA03_ODR_AT_13Hz;
      break;
    case 0x34:
      *val = ASM330AB1_HA03_ODR_AT_26Hz;
      break;
    case 0x35:
      *val = ASM330AB1_HA03_ODR_AT_52Hz;
      break;
    case 0x36:
      *val = ASM330AB1_HA03_ODR_AT_104Hz;
      break;
    case 0x37:
      *val = ASM330AB1_HA03_ODR_AT_208Hz;
      break;
    case 0x38:
      *val = ASM330AB1_HA03_ODR_AT_417Hz;
      break;
    case 0x39:
      *val = ASM330AB1_HA03_ODR_AT_833Hz;
      break;
    case 0x3A:
      *val = ASM330AB1_HA03_ODR_AT_1667Hz;
      break;
    case 0x3B:
      *val = ASM330AB1_HA03_ODR_AT_3333Hz;
      break;

    default:
      ret = -1;
      break;
  }

exit:
  return ret;
}

/**
  * @brief  GY data rate.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Data rate value: struct asm330ab1_odr_t
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_gy_data_rate_set(const stmdev_ctx_t *ctx, asm330ab1_odr_t val)
{
  asm330ab1_ctrl2_t ctrl2;
  uint8_t cfg = (val & 0xf0) >> 4;
  uint8_t odr = val & 0x0f;
  int32_t ret;

  ret = asm330ab1_haodr_cfg_set(ctx, (asm330ab1_haodr_cfg_val_t)cfg);
  if (ret != 0)
  {
    goto exit;
  }

  ctrl2.op_mode_g = ASM330AB1_HA_MODE;
  ctrl2.odr_g = odr;
  ret = asm330ab1_write_reg(ctx, ASM330AB1_CTRL2, (uint8_t *)&ctrl2, 1);

exit:
  return ret;
}

/**
  * @brief  GY data rate.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Data rate value: struct asm330ab1_odr_t
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_gy_data_rate_get(const stmdev_ctx_t *ctx, asm330ab1_odr_t *val)
{
  asm330ab1_ctrl2_t ctrl2;
  asm330ab1_haodr_cfg_val_t cfg;
  int32_t ret;

  ret = asm330ab1_haodr_cfg_get(ctx, &cfg);
  ret += asm330ab1_read_reg(ctx, ASM330AB1_CTRL2, (uint8_t *)&ctrl2, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* build up the asm330ab1_odr_t value */
  switch ((cfg << 4) | ctrl2.odr_g)
  {
    case 0x00:
      *val = ASM330AB1_HA00_ODR_POWER_DOWN;
      break;
    case 0x03:
      *val = ASM330AB1_HA00_ODR_AT_15Hz;
      break;
    case 0x04:
      *val = ASM330AB1_HA00_ODR_AT_30Hz;
      break;
    case 0x05:
      *val = ASM330AB1_HA00_ODR_AT_60Hz;
      break;
    case 0x06:
      *val = ASM330AB1_HA00_ODR_AT_120Hz;
      break;
    case 0x07:
      *val = ASM330AB1_HA00_ODR_AT_240Hz;
      break;
    case 0x08:
      *val = ASM330AB1_HA00_ODR_AT_480Hz;
      break;
    case 0x09:
      *val = ASM330AB1_HA00_ODR_AT_960Hz;
      break;
    case 0x0A:
      *val = ASM330AB1_HA00_ODR_AT_1920Hz;
      break;
    case 0x0B:
      *val = ASM330AB1_HA00_ODR_AT_3840Hz;
      break;

    case 0x20:
      *val = ASM330AB1_HA02_ODR_POWER_DOWN;
      break;
    case 0x23:
      *val = ASM330AB1_HA02_ODR_AT_12Hz5;
      break;
    case 0x24:
      *val = ASM330AB1_HA02_ODR_AT_25Hz;
      break;
    case 0x25:
      *val = ASM330AB1_HA02_ODR_AT_50Hz;
      break;
    case 0x26:
      *val = ASM330AB1_HA02_ODR_AT_100Hz;
      break;
    case 0x27:
      *val = ASM330AB1_HA02_ODR_AT_200Hz;
      break;
    case 0x28:
      *val = ASM330AB1_HA02_ODR_AT_400Hz;
      break;
    case 0x29:
      *val = ASM330AB1_HA02_ODR_AT_800Hz;
      break;
    case 0x2A:
      *val = ASM330AB1_HA02_ODR_AT_1600Hz;
      break;
    case 0x2B:
      *val = ASM330AB1_HA02_ODR_AT_3200Hz;
      break;

    case 0x30:
      *val = ASM330AB1_HA03_ODR_POWER_DOWN;
      break;
    case 0x33:
      *val = ASM330AB1_HA03_ODR_AT_13Hz;
      break;
    case 0x34:
      *val = ASM330AB1_HA03_ODR_AT_26Hz;
      break;
    case 0x35:
      *val = ASM330AB1_HA03_ODR_AT_52Hz;
      break;
    case 0x36:
      *val = ASM330AB1_HA03_ODR_AT_104Hz;
      break;
    case 0x37:
      *val = ASM330AB1_HA03_ODR_AT_208Hz;
      break;
    case 0x38:
      *val = ASM330AB1_HA03_ODR_AT_417Hz;
      break;
    case 0x39:
      *val = ASM330AB1_HA03_ODR_AT_833Hz;
      break;
    case 0x3A:
      *val = ASM330AB1_HA03_ODR_AT_1667Hz;
      break;
    case 0x3B:
      *val = ASM330AB1_HA03_ODR_AT_3333Hz;
      break;

    default:
      ret = -1;
      break;
  }

exit:
  return ret;
}

/**
  * @brief  High Accuracy configuration.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Configuration value: 0, 2, 3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_haodr_cfg_set(const stmdev_ctx_t *ctx, asm330ab1_haodr_cfg_val_t val)
{
  asm330ab1_haodr_cfg_t cfg = {0};

  cfg.haodr_sel = (uint8_t)val;
  return asm330ab1_write_reg(ctx, ASM330AB1_HAODR_CFG, (uint8_t *)&cfg, 1);
}

/**
  * @brief  High Accuracy configuration.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Configuration value: 0, 2, 3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_haodr_cfg_get(const stmdev_ctx_t *ctx, asm330ab1_haodr_cfg_val_t *val)
{
  asm330ab1_haodr_cfg_t cfg = {0};
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_HAODR_CFG, (uint8_t *)&cfg, 1);

  *val = (asm330ab1_haodr_cfg_val_t)cfg.haodr_sel;

  return ret;
}

/**
  * @brief  Accelerometer full scale configuration.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Full scale enum values (asm330ab1_xl_full_scale_t)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_xl_full_scale_set(const stmdev_ctx_t *ctx, asm330ab1_xl_full_scale_t val)
{
  asm330ab1_ctrl8_t ctrl8 = {0};
  int ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_CTRL8, (uint8_t *)&ctrl8, 1);
  ctrl8.fs_xl = (uint8_t)val;
  ret += asm330ab1_write_reg(ctx, ASM330AB1_CTRL8, (uint8_t *)&ctrl8, 1);

  return ret;
}

/**
  * @brief  Accelerometer full scale configuration.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Full scale enum values (asm330ab1_xl_full_scale_t)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_xl_full_scale_get(const stmdev_ctx_t *ctx, asm330ab1_xl_full_scale_t *val)
{
  asm330ab1_ctrl8_t ctrl8 = {0};
  int ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_CTRL8, (uint8_t *)&ctrl8, 1);

  switch (ctrl8.fs_xl)
  {
    case 0:
      *val = ASM330AB1_2g;
      break;
    case 1:
      *val = ASM330AB1_4g;
      break;
    case 2:
      *val = ASM330AB1_8g;
      break;
    default:
    case 3:
      *val = ASM330AB1_16g;
      break;
  }

  return ret;
}

/**
  * @brief  Gyroscope full scale configuration.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Full scale enum values (asm330ab1_gy_full_scale_t)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_gy_full_scale_set(const stmdev_ctx_t *ctx, asm330ab1_gy_full_scale_t val)
{
  asm330ab1_ctrl6_t ctrl6 = {0};
  int ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_CTRL6, (uint8_t *)&ctrl6, 1);
  ctrl6.fs_g = (uint8_t)val;
  ret += asm330ab1_write_reg(ctx, ASM330AB1_CTRL6, (uint8_t *)&ctrl6, 1);

  return ret;
}

/**
  * @brief  Gyroscope full scale configuration.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Full scale enum values (asm330ab1_gy_full_scale_t)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_gy_full_scale_get(const stmdev_ctx_t *ctx, asm330ab1_gy_full_scale_t *val)
{
  asm330ab1_ctrl6_t ctrl6 = {0};
  int ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_CTRL6, (uint8_t *)&ctrl6, 1);

  switch (ctrl6.fs_g)
  {
    case 0:
      *val = ASM330AB1_125dps;
      break;
    case 1:
      *val = ASM330AB1_250dps;
      break;
    case 2:
      *val = ASM330AB1_500dps;
      break;
    case 3:
      *val = ASM330AB1_1000dps;
      break;
    default:
    case 4:
      *val = ASM330AB1_2000dps;
      break;
  }

  return ret;
}

/**
  * @brief   Select the signals that need to be routed on int1 pad.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    the drdy event to route on int1 pin.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_pin_int1_route_set(const stmdev_ctx_t *ctx, asm330ab1_pin_int_route_t *val)
{
  asm330ab1_int1_ctrl_t           int1_ctrl;
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_INT1_CTRL, (uint8_t *)&int1_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  int1_ctrl.int1_drdy_xl   = val->drdy_xl;
  int1_ctrl.int1_drdy_g    = val->drdy_g;
  int1_ctrl.int1_drdy_temp = val->drdy_temp;
  int1_ctrl.int1_drdy_fusa = val->drdy_fusa;

  return asm330ab1_write_reg(ctx, ASM330AB1_INT1_CTRL, (uint8_t *)&int1_ctrl, 1);
}

/**
  * @brief   Select the signals that need to be routed on int1 pad.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    the drdy event to route on int1 pin.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_pin_int1_route_get(const stmdev_ctx_t *ctx, asm330ab1_pin_int_route_t *val)
{
  asm330ab1_int1_ctrl_t           int1_ctrl;
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_INT1_CTRL, (uint8_t *)&int1_ctrl, 1);

  val->drdy_xl   = int1_ctrl.int1_drdy_xl;
  val->drdy_g    = int1_ctrl.int1_drdy_g;
  val->drdy_temp = int1_ctrl.int1_drdy_temp;
  val->drdy_fusa = int1_ctrl.int1_drdy_fusa;

  return ret;
}

/**
  * @brief   Select the signals that need to be routed on int2 pad.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    the drdy event to route on int2 pin.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_pin_int2_route_set(const stmdev_ctx_t *ctx, asm330ab1_pin_int_route_t *val)
{
  asm330ab1_int2_ctrl_t           int2_ctrl;
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_INT2_CTRL, (uint8_t *)&int2_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  int2_ctrl.int2_drdy_xl   = val->drdy_xl;
  int2_ctrl.int2_drdy_g    = val->drdy_g;
  int2_ctrl.int2_drdy_temp = val->drdy_temp;
  int2_ctrl.int2_timestamp = val->timestamp;
  int2_ctrl.int2_in_lh     = val->in_lh;
  int2_ctrl.int2_cap_en    = val->cap_en;

  return asm330ab1_write_reg(ctx, ASM330AB1_INT2_CTRL, (uint8_t *)&int2_ctrl, 1);
}

/**
  * @brief   Select the signals that need to be routed on int2 pad.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    the drdy event to route on int2 pin.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_pin_int2_route_get(const stmdev_ctx_t *ctx, asm330ab1_pin_int_route_t *val)
{
  asm330ab1_int2_ctrl_t           int2_ctrl;
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_INT2_CTRL, (uint8_t *)&int2_ctrl, 1);

  val->drdy_xl   = int2_ctrl.int2_drdy_xl;
  val->drdy_g    = int2_ctrl.int2_drdy_g;
  val->drdy_temp = int2_ctrl.int2_drdy_temp;
  val->timestamp = int2_ctrl.int2_timestamp;
  val->in_lh     = int2_ctrl.int2_in_lh;
  val->cap_en    = int2_ctrl.int2_cap_en;

  return ret;
}

/**
  * @brief   Select the signals that need to be routed on int3 pad.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    the drdy event to route on int3 pin.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_pin_int3_route_set(const stmdev_ctx_t *ctx, asm330ab1_pin_int_route_t *val)
{
  asm330ab1_int3_ctrl_t           int3_ctrl;
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_INT3_CTRL, (uint8_t *)&int3_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  int3_ctrl.int3_drdy_xl   = val->drdy_xl;
  int3_ctrl.int3_drdy_g    = val->drdy_g;
  int3_ctrl.int3_drdy_temp = val->drdy_temp;
  int3_ctrl.int3_drdy_fusa = val->drdy_fusa;

  return asm330ab1_write_reg(ctx, ASM330AB1_INT3_CTRL, (uint8_t *)&int3_ctrl, 1);
}

/**
  * @brief   Select the signals that need to be routed on int3 pad.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    the drdy event to route on int3 pin.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_pin_int3_route_get(const stmdev_ctx_t *ctx, asm330ab1_pin_int_route_t *val)
{
  asm330ab1_int3_ctrl_t           int3_ctrl;
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_INT3_CTRL, (uint8_t *)&int3_ctrl, 1);

  val->drdy_xl   = int3_ctrl.int3_drdy_xl;
  val->drdy_g    = int3_ctrl.int3_drdy_g;
  val->drdy_temp = int3_ctrl.int3_drdy_temp;
  val->drdy_fusa = int3_ctrl.int3_drdy_fusa;

  return ret;
}

/**
  * @brief   Data ready configuration.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    the drdy mode structure.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_data_ready_mode_set(const stmdev_ctx_t *ctx, asm330ab1_data_ready_mode_t val)
{
  asm330ab1_ctrl4_t           ctrl4;
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_CTRL4, (uint8_t *)&ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }

  ctrl4.drdy_pulsed = val.mode;
  ctrl4.drdy_mask   = val.mask;

  return asm330ab1_write_reg(ctx, ASM330AB1_CTRL4, (uint8_t *)&ctrl4, 1);
}

/**
  * @brief   Data ready configuration.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    the drdy mode structure.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_data_ready_mode_get(const stmdev_ctx_t *ctx, asm330ab1_data_ready_mode_t *val)
{
  asm330ab1_ctrl4_t           ctrl4;
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_CTRL4, (uint8_t *)&ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl4.drdy_pulsed)
  {
    case 1:
      val->mode = ASM330AB1_DRDY_PULSED;
      break;
    default:
    case 0:
      val->mode = ASM330AB1_DRDY_LATCHED;
      break;
  }

  switch (ctrl4.drdy_mask)
  {
    case 1:
      val->mask = ASM330AB1_DRDY_MASKED;
      break;
    default:
    case 0:
      val->mask = ASM330AB1_DRDY_NOT_MASKED;
      break;
  }

  return 0;
}

/**
  * @brief  Temperature data output register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Temperature data output register
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_temperature_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_OUT_TEMP_L, &buff[0], 2);
  if (ret != 0)
  {
    return ret;
  }

  *val = (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  return ret;
}

/**
  * @brief  Angular rate sensor.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Angular rate sensor.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_angular_rate_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_OUTX_L_G, &buff[0], 6);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

/**
  * @brief  Linear acceleration sensor.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Linear acceleration sensor.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_acceleration_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_OUTX_L_A, &buff[0], 6);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

/**
  * @brief  Timestamp enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      0: timestamp disable, 1: timestamp enable
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_timestamp_enable(const stmdev_ctx_t *ctx, uint8_t val)
{
  asm330ab1_ctrl3_t           ctrl3;
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_CTRL3, (uint8_t *)&ctrl3, 1);
  if (ret != 0)
  {
    return ret;
  }

  ctrl3.timestamp_en = val;

  return asm330ab1_write_reg(ctx, ASM330AB1_CTRL3, (uint8_t *)&ctrl3, 1);
}

/**
  * @brief  Timestamp at last data output.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      timestamp in usec
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_timestamp_us_get(const stmdev_ctx_t *ctx, uint64_t *val)
{
  uint8_t buff[4];
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_TIMESTAMP0, &buff[0], 4);
  if (ret != 0)
  {
    return ret;
  }

  *val = (int16_t)buff[3];
  *val = (*val * 256) + (int16_t)buff[2];
  *val = (*val * 256) + (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  *val *= 21; /* 1LSB is 21 usec */

  return ret;
}

/**
  * @brief  Timestamp at last data ready event.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      timestamp in usec
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_timestamp_odr_us_get(const stmdev_ctx_t *ctx, uint64_t *val)
{
  uint8_t buff[4];
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_TIMESTAMP_ODR0, &buff[0], 4);
  if (ret != 0)
  {
    return ret;
  }

  *val = (int16_t)buff[3];
  *val = (*val * 256) + (int16_t)buff[2];
  *val = (*val * 256) + (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  *val *= 21; /* 1LSB is 21 usec */

  return ret;
}

/**
  * @brief  Timestamp at last external trigger event.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      timestamp in usec
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_timestamp_ext_us_get(const stmdev_ctx_t *ctx, uint64_t *val)
{
  uint8_t buff[4];
  int32_t ret;

  ret = asm330ab1_read_reg(ctx, ASM330AB1_TIMESTAMP_EXT0, &buff[0], 4);
  if (ret != 0)
  {
    return ret;
  }

  *val = (int16_t)buff[3];
  *val = (*val * 256) + (int16_t)buff[2];
  *val = (*val * 256) + (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  *val *= 21; /* 1LSB is 21 usec */

  return ret;
}

/**
  * @brief  Device Status.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  status Buffer to read the device_status register
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_device_status_get(const stmdev_ctx_t *ctx, asm330ab1_device_status_t *status)
{
  return asm330ab1_read_reg(ctx, ASM330AB1_DEVICE_STATUS, (uint8_t *)status, 1);
}

/**
  * @brief  Status.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  status Buffer to read the status register
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t asm330ab1_status_get(const stmdev_ctx_t *ctx, asm330ab1_status_t *status)
{
  return asm330ab1_read_reg(ctx, ASM330AB1_STATUS, (uint8_t *)status, 1);
}

/**
  * @}
  *
  */

/**
  * @defgroup  Device startup
  * @brief     This section groups all the functions concerning
  *            device startup  as explained in Safety Manual section 6.1
  * @{
  *
  */

/**
  * @brief  Power-down the sensors (See Safety Manual section 6)
  *
  * @param  ctx   communication interface handler.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_sensor_power_down(const stmdev_ctx_t *ctx)
{
  uint8_t reg_tmp, val;
  int32_t ret;

  /* set power down */
  ret = asm330ab1_page_sel_set(ctx, ASM330AB1_MAIN_PAGE);
  ret += asm330ab1_xl_data_rate_set(ctx, ASM330AB1_HA00_ODR_POWER_DOWN);
  ret += asm330ab1_gy_data_rate_set(ctx, ASM330AB1_HA00_ODR_POWER_DOWN);

  /* Step 2.2 */
  val = 0xC3;
  ret += asm330ab1_write_reg(ctx, 0x06, &val, 1);

  /* Step 2.3 */
  val = 0x75;
  ret += asm330ab1_write_reg(ctx, 0x36, &val, 1);

  /* Step 2.4 */
  ret += asm330ab1_page_sel_set(ctx, ASM330AB1_RESERVED_PAGE_16);

  /* Step 2.5 */
  val = 0xC8;
  ret += asm330ab1_write_reg(ctx, 0x76, &val, 1);

  /* Step 2.6 */
  ret += asm330ab1_page_sel_set(ctx, ASM330AB1_RESERVED_PAGE_1);

  /* Step 2.7 */
  ret += asm330ab1_read_reg(ctx, 0x60, &reg_tmp, 1);
  reg_tmp &= 0x7F;
  ret += asm330ab1_write_reg(ctx, 0x60, &reg_tmp, 1);

  /* Step 2.8 */
  ret += asm330ab1_page_sel_set(ctx, ASM330AB1_MAIN_PAGE);

  return ret;
}

/**
  * @brief  Start-up the sensors (See Safety Manual section 6)
  *
  * @param  ctx   communication interface handler.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_sensor_start_up(const stmdev_ctx_t *ctx)
{
  uint8_t reg_tmp, val;
  int32_t ret;

  /* Step 6.1 */
  ret = asm330ab1_page_sel_set(ctx, ASM330AB1_RESERVED_PAGE_1);

  /* Step 6.2 */
  ret += asm330ab1_read_reg(ctx, 0x60, &reg_tmp, 1);
  reg_tmp |= 0x80;
  ret += asm330ab1_write_reg(ctx, 0x60, &reg_tmp, 1);

  /* Step 6.3 */
  ret += asm330ab1_page_sel_set(ctx, ASM330AB1_RESERVED_PAGE_16);

  /* Step 6.4 */
  val = 0x8;
  ret += asm330ab1_write_reg(ctx, 0x76, &val, 1);

  /* Step 6.5 */
  ret += asm330ab1_page_sel_set(ctx, ASM330AB1_MAIN_PAGE);

  /* Step 6.6 */
  val = 0x0;
  ret = asm330ab1_write_reg(ctx, 0x06, &val, 1);

  /* Step 6.7 */
  val = 0x0;
  ret += asm330ab1_write_reg(ctx, 0x36, &val, 1);

  return ret;
}

/**
  * @brief  Check device initialization faults
  *
  * @param  ctx   communication interface handler.(ptr)
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_check_faults(const stmdev_ctx_t *ctx)
{
  uint8_t status;
  uint8_t retry = 0;
  int32_t ret = 0;

  if (ctx->mdelay == NULL)
  {
    ret = -1;
    goto exit;
  }

  /* Step 11 */
  asm330ab1_fusa_fault_clear(ctx, 7);

  /* Check FUSA_STATUS */
  do
  {
    ctx->mdelay(10);

    ret += asm330ab1_read_reg(ctx, ASM330AB1_FUSA_STATUS, &status, 1);
    if (ret != 0)
    {
      goto exit;
    }
  } while (status != 0xFF && retry++ < 3);

  ret = (status == 0xFF) ? 0 : -1;

exit:
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Fu.Sa. page handling
  * @brief     This section groups all the functions concerning
  *            Fu.Sa. page
  * @{
  *
  */

/**
  * @brief  Clear faults belonging to a specific group
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  group group selector
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_fusa_fault_clear(const stmdev_ctx_t *ctx, uint8_t group)
{
  asm330ab1_clear_t clear;
  int32_t ret;

  ret = asm330ab1_page_sel_set(ctx, ASM330AB1_FUSA_PAGE);
  if (ret != 0)
  {
    goto exit;
  }

  clear.group = group;
  ret = asm330ab1_write_reg(ctx, ASM330AB1_CLEAR, (uint8_t *)&clear, 1);

  ret += asm330ab1_page_sel_set(ctx, ASM330AB1_MAIN_PAGE);

exit:
  return ret;
}

/**
  * @brief  Read the Fu.Sa. fault status details
  *
  * @param  ctx    communication interface handler.(ptr)
  * @param  status fault status details
  * @retval        interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_fusa_status_read(const stmdev_ctx_t *ctx, asm330ab1_fusa_faults_t *status)
{
  asm330ab1_sum_status_t sum_status;
  asm330ab1_sum_range_t sum_range;
  uint8_t reg[2];
  int32_t ret;

  ret = asm330ab1_page_sel_set(ctx, ASM330AB1_FUSA_PAGE);
  if (ret != 0)
  {
    goto exit;
  }

  ret = asm330ab1_read_reg(ctx, ASM330AB1_SUM_STATUS, reg, 2);

  bytecpy((uint8_t *)&sum_status, &reg[0]);
  bytecpy((uint8_t *)&sum_range, &reg[1]);

  ret += asm330ab1_page_sel_set(ctx, ASM330AB1_MAIN_PAGE);
  if (ret != 0)
  {
    goto exit;
  }

  status->xl_status_x = sum_status.xl_status_x;
  status->xl_status_y = sum_status.xl_status_y;
  status->xl_status_z = sum_status.xl_status_z;
  status->gy_status_x = sum_status.gy_status_x;
  status->gy_status_y = sum_status.gy_status_y;
  status->gy_status_z = sum_status.gy_status_z;
  status->if_status   = sum_status.if_status;
  status->com_status  = sum_status.com_status;
  status->xl_range_x  = sum_range.xl_range_x;
  status->xl_range_y  = sum_range.xl_range_y;
  status->xl_range_z  = sum_range.xl_range_z;
  status->gy_range_x  = sum_range.gy_range_x;
  status->gy_range_y  = sum_range.gy_range_y;
  status->gy_range_z  = sum_range.gy_range_z;

exit:
  return ret;
}

/**
  * @brief  Set the XL/GY data_n_dump for self-test
  *
  * @param  ctx    communication interface handler.(ptr)
  * @param  ndump  XL/GY data_n_dump value
  * @retval        interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_data_n_dump_set(const stmdev_ctx_t *ctx, asm330ab1_data_n_dump_t ndump)
{
  asm330ab1_st_auto_cfg_t st_auto_cfg = { 0 };
  int32_t ret;

  ret = asm330ab1_page_sel_set(ctx, ASM330AB1_FUSA_PAGE);
  if (ret != 0)
  {
    goto exit;
  }

  st_auto_cfg.gy_data_n_dump = ndump.gy_data_n_dump;
  st_auto_cfg.xl_data_n_dump = ndump.xl_data_n_dump;
  ret = asm330ab1_write_reg(ctx, ASM330AB1_ST_AUTO_CFG, (uint8_t *)&st_auto_cfg, 1);

  ret += asm330ab1_page_sel_set(ctx, ASM330AB1_MAIN_PAGE);

exit:
  return ret;
}

/**
  * @brief  Start/Stop GY self-test automatic procedure
  *
  * @param  ctx    communication interface handler.(ptr)
  * @param  val    0: stop, 1: start
  * @retval        interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_st_auto_gy_start(const stmdev_ctx_t *ctx, uint8_t val)
{
  asm330ab1_st_auto_cfg_t st_auto_cfg = { 0 };
  int32_t ret;

  ret = asm330ab1_page_sel_set(ctx, ASM330AB1_FUSA_PAGE);
  if (ret != 0)
  {
    goto exit;
  }

  ret = asm330ab1_read_reg(ctx, ASM330AB1_ST_AUTO_CFG, (uint8_t *)&st_auto_cfg, 1);
  st_auto_cfg.st_auto_gy_start = val;
  ret += asm330ab1_write_reg(ctx, ASM330AB1_ST_AUTO_CFG, (uint8_t *)&st_auto_cfg, 1);

  ret += asm330ab1_page_sel_set(ctx, ASM330AB1_MAIN_PAGE);

exit:
  return ret;
}

/**
  * @brief  Start/Stop XL self-test automatic procedure
  *
  * @param  ctx    communication interface handler.(ptr)
  * @param  val    0: stop, 1: start
  * @retval        interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_st_auto_xl_start(const stmdev_ctx_t *ctx, uint8_t val)
{
  asm330ab1_st_auto_cfg_t st_auto_cfg = { 0 };
  int32_t ret;

  ret = asm330ab1_page_sel_set(ctx, ASM330AB1_FUSA_PAGE);
  if (ret != 0)
  {
    goto exit;
  }

  ret = asm330ab1_read_reg(ctx, ASM330AB1_ST_AUTO_CFG, (uint8_t *)&st_auto_cfg, 1);
  st_auto_cfg.st_auto_xl_start = val;
  ret += asm330ab1_write_reg(ctx, ASM330AB1_ST_AUTO_CFG, (uint8_t *)&st_auto_cfg, 1);

  ret += asm330ab1_page_sel_set(ctx, ASM330AB1_MAIN_PAGE);

exit:
  return ret;
}

/**
  * @brief  Get self-test auto sum_status register
  *
  * @param  ctx         communication interface handler.(ptr)
  * @param  sum_status  sum_status register
  * @retval             interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t asm330ab1_st_auto_sum_status_get(const stmdev_ctx_t *ctx,
                                         asm330ab1_st_auto_sum_status_t *sum_status)
{
  int32_t ret;

  ret = asm330ab1_page_sel_set(ctx, ASM330AB1_FUSA_PAGE);
  if (ret != 0)
  {
    goto exit;
  }

  ret = asm330ab1_read_reg(ctx, ASM330AB1_ST_AUTO_SUM_STATUS, (uint8_t *)sum_status, 1);

  ret += asm330ab1_page_sel_set(ctx, ASM330AB1_MAIN_PAGE);

exit:
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
