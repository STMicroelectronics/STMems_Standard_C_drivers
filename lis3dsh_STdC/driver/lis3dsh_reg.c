/*
 ******************************************************************************
 * @file    lis3dsh_reg.c
 * @author  Sensors Software Solution Team
 * @brief   LIS3DSH driver file
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

#include "lis3dsh_reg.h"

/**
  * @defgroup  LIS3DSH
  * @brief     This file provides a set of functions needed to drive the
  *            lis3dsh enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup  LIS3DSH_Interfaces_Functions
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
int32_t lis3dsh_read_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
int32_t lis3dsh_write_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
  * @defgroup  LIS3DSH_Private_functions
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
  * @defgroup  LIS3DSH_Sensitivity
  * @brief     These functions convert raw-data into engineering units.
  * @{
  *
  */
float_t lis3dsh_from_fs2_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.06f;
}

float_t lis3dsh_from_fs4_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.12f;
}

float_t lis3dsh_from_fs6_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.18f;
}

float_t lis3dsh_from_fs8_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.24f;
}

float_t lis3dsh_from_fs16_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.73f;
}

float_t lis3dsh_from_lsb_to_celsius(int8_t lsb)
{
  return (((float_t)lsb / 256.0f) + 25.0f);
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
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          ID values.(ptr)
  *
  */
int32_t lis3dsh_id_get(stmdev_ctx_t *ctx, lis3dsh_id_t *val)
{
  uint8_t reg[3];
  int32_t ret;
  
  ret = lis3dsh_read_reg(ctx, LIS3DSH_INFO1,reg, 3);
  val->info1  = reg[0];
  val->info2  = reg[1];
  val->whoami = reg[2];
  
  return ret;
}

/**
  * @brief  Configures the bus operating mode.[set]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          configures the bus operating mode.(ptr)
  *
  */
int32_t lis3dsh_bus_mode_set(stmdev_ctx_t *ctx, lis3dsh_bus_mode_t *val)
{
  lis3dsh_ctrl_reg5_t ctrl_reg5;
  int32_t ret;

  ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
  if (ret == 0) {
    ctrl_reg5.sim = (uint8_t)*val;
    ret = lis3dsh_write_reg(ctx, LIS3DSH_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
  }

  return ret;

}

/**
  * @brief  Get the bus operating mode.[get]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          retrieves the bus operating.(ptr)
  *
  */
int32_t lis3dsh_bus_mode_get(stmdev_ctx_t *ctx, lis3dsh_bus_mode_t *val)
{
  lis3dsh_ctrl_reg5_t ctrl_reg5;
  int32_t ret;

  ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
  switch ( ctrl_reg5.sim ) {
    case LIS3DSH_SEL_BY_HW:
      *val = LIS3DSH_SEL_BY_HW;
      break;
    case LIS3DSH_SPI_3W:
      *val = LIS3DSH_SPI_3W;
      break;
    default:
      *val = LIS3DSH_SEL_BY_HW;
      break;
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
  *                      procedure.(ptr)
  *
  */
int32_t lis3dsh_init_set(stmdev_ctx_t *ctx, lis3dsh_init_t val)
{
  lis3dsh_ctrl_reg3_t ctrl_reg3;
  lis3dsh_ctrl_reg4_t ctrl_reg4;
  lis3dsh_ctrl_reg6_t ctrl_reg6;

  int32_t ret;

  switch ( val ) {
    case LIS3DSH_BOOT:
      ctrl_reg6.boot = (uint8_t)val & (uint8_t)LIS3DSH_BOOT;
      ret = lis3dsh_write_reg(ctx, LIS3DSH_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
      break;
    case LIS3DSH_RESET:
      ctrl_reg3.strt = ( (uint8_t)val & (uint8_t)LIS3DSH_RESET) >> 1;
      ret = lis3dsh_write_reg(ctx, LIS3DSH_CTRL_REG6, (uint8_t*)&ctrl_reg6, 1);
      break;
    case LIS3DSH_DRV_RDY:
      ctrl_reg4.xen = PROPERTY_ENABLE;
      ctrl_reg4.yen = PROPERTY_ENABLE;
      ctrl_reg4.zen = PROPERTY_ENABLE;
      ctrl_reg4.bdu = PROPERTY_ENABLE;
      ctrl_reg6.add_inc = PROPERTY_ENABLE;
      ret = lis3dsh_write_reg(ctx, LIS3DSH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
      if (ret == 0) {
        ret = lis3dsh_write_reg(ctx, LIS3DSH_CTRL_REG6, (uint8_t*)&ctrl_reg6, 1);
      }
      break;
    default:
      ctrl_reg3.strt = ( (uint8_t)val & (uint8_t)LIS3DSH_RESET) >> 1;
      ret = lis3dsh_write_reg(ctx, LIS3DSH_CTRL_REG6, (uint8_t*)&ctrl_reg6, 1);
      break;
  }
  return ret;
}

/**
  * @brief  Get the status of the device.[get]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the status of the device.(ptr)
  *
  */
int32_t lis3dsh_status_get(stmdev_ctx_t *ctx, lis3dsh_status_var_t *val)
{
  lis3dsh_ctrl_reg3_t ctrl_reg3;
  lis3dsh_ctrl_reg6_t ctrl_reg6;
  lis3dsh_stat_t stat;
  int32_t ret;

  ret = lis3dsh_read_reg(ctx, LIS3DSH_STAT, (uint8_t*)&stat, 1);
  if (ret == 0) {
    ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  }
  if (ret == 0) {
    ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG6, (uint8_t*)&ctrl_reg6, 1);
  }

  val->sw_reset = ctrl_reg3.strt;
  val->boot     = ctrl_reg6.boot;
  val->drdy_xl  = stat.drdy;
  val->ovrn_xl  = stat.dor;

  return ret;
}

/**
  * @brief  Interrupt pins hardware signal configuration.[set]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the pins hardware signal settings.(ptr)
  *
  */
int32_t lis3dsh_interrupt_mode_set(stmdev_ctx_t *ctx, lis3dsh_int_mode_t *val)
{
  lis3dsh_ctrl_reg3_t ctrl_reg3;
  int32_t ret;

  ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  if (ret == 0) {
    ctrl_reg3.iel = ~(val->latched);
    ctrl_reg3.iea = ~(val->active_low);
    ret = lis3dsh_write_reg(ctx, LIS3DSH_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
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
int32_t lis3dsh_interrupt_mode_get(stmdev_ctx_t *ctx, lis3dsh_int_mode_t *val)
{
  lis3dsh_ctrl_reg3_t ctrl_reg3;
  int32_t ret;
    
  ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  val->latched = ~(ctrl_reg3.iel);
  val->active_low = ~(ctrl_reg3.iea);

  return ret;
}

/**
  * @brief  Route interrupt signals on int1 pin.[set]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the signals to route on int1 pin.(ptr)
  *
  */
int32_t lis3dsh_pin_int1_route_set(stmdev_ctx_t *ctx,
                                   lis3dsh_pin_int1_route_t *val)
{
  lis3dsh_ctrl_reg1_t ctrl_reg1;
  lis3dsh_ctrl_reg2_t ctrl_reg2;
  lis3dsh_ctrl_reg3_t ctrl_reg3;
  lis3dsh_ctrl_reg6_t ctrl_reg6;
  uint8_t reg[5];
  int32_t ret;

  ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG1, (uint8_t*)&reg, 5);
  bytecpy(( uint8_t*)&ctrl_reg1, &reg[0]);
  bytecpy(( uint8_t*)&ctrl_reg2, &reg[1]);
  bytecpy(( uint8_t*)&ctrl_reg3, &reg[2]);
  bytecpy(( uint8_t*)&ctrl_reg6, &reg[4]);

  ctrl_reg1.sm1_pin    = ~(val->fsm1);
  ctrl_reg2.sm2_pin    = ~(val->fsm2);
  ctrl_reg3.dr_en      = val->drdy_xl;
  ctrl_reg6.p1_wtm     = val->fifo_th;
  ctrl_reg6.p1_empty   = val->fifo_empty;
  ctrl_reg6.p1_overrun = val->fifo_full;

  if ( (val->fsm1 | val->fsm2 | val->drdy_xl | 
        val->fifo_empty | val->fifo_th | 
        val->fifo_full) == PROPERTY_ENABLE ){
    ctrl_reg3.int1_en = PROPERTY_ENABLE;
  }
  else {
    ctrl_reg3.int1_en = PROPERTY_DISABLE;
  }

  bytecpy(&reg[0], ( uint8_t*)&ctrl_reg1);
  bytecpy(&reg[1], ( uint8_t*)&ctrl_reg2);
  bytecpy(&reg[2], ( uint8_t*)&ctrl_reg3);
  bytecpy(&reg[4], ( uint8_t*)&ctrl_reg6);
  if (ret == 0) {
    ret = lis3dsh_write_reg(ctx, LIS3DSH_CTRL_REG1, (uint8_t*)&reg, 5);
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
int32_t lis3dsh_pin_int1_route_get(stmdev_ctx_t *ctx,
                                   lis3dsh_pin_int1_route_t *val)
{
  lis3dsh_ctrl_reg1_t ctrl_reg1;
  lis3dsh_ctrl_reg2_t ctrl_reg2;
  lis3dsh_ctrl_reg3_t ctrl_reg3;
  lis3dsh_ctrl_reg6_t ctrl_reg6;
  uint8_t reg[5];
  int32_t ret;

  ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG1, (uint8_t*)&reg, 5);
  bytecpy(( uint8_t*)&ctrl_reg1, &reg[0]);
  bytecpy(( uint8_t*)&ctrl_reg2, &reg[1]);
  bytecpy(( uint8_t*)&ctrl_reg3, &reg[2]);
  bytecpy(( uint8_t*)&ctrl_reg6, &reg[4]);

  if ( ctrl_reg3.int1_en == PROPERTY_ENABLE ){
    val->fsm1       = ~(ctrl_reg1.sm1_pin); 
    val->fsm2       = ~(ctrl_reg2.sm2_pin); 
    val->drdy_xl    = ctrl_reg3.dr_en; 
    val->fifo_th    = ctrl_reg6.p1_wtm;
    val->fifo_empty = ctrl_reg6.p1_empty;
    val->fifo_full  = ctrl_reg6.p1_overrun;
  }
  else {
    val->fsm1       = PROPERTY_DISABLE;
    val->fsm2       = PROPERTY_DISABLE;
    val->drdy_xl    = PROPERTY_DISABLE;
    val->fifo_th    = PROPERTY_DISABLE;
    val->fifo_empty = PROPERTY_DISABLE;
    val->fifo_full  = PROPERTY_DISABLE;
  }

  return ret;
}

/**
  * @brief  Route interrupt signals on int2 pin.[set]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the signals to route on int2 pin.(ptr)
  *
  */
int32_t lis3dsh_pin_int2_route_set(stmdev_ctx_t *ctx, 
                                   lis3dsh_pin_int2_route_t *val)
{
  lis3dsh_ctrl_reg1_t ctrl_reg1;
  lis3dsh_ctrl_reg2_t ctrl_reg2;
  lis3dsh_ctrl_reg3_t ctrl_reg3;
  lis3dsh_ctrl_reg6_t ctrl_reg6;
  uint8_t reg[5];
  int32_t ret;

  ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG1, (uint8_t*)&reg, 5);
  bytecpy(( uint8_t*)&ctrl_reg1, &reg[0]);
  bytecpy(( uint8_t*)&ctrl_reg2, &reg[1]);
  bytecpy(( uint8_t*)&ctrl_reg3, &reg[2]);
  bytecpy(( uint8_t*)&ctrl_reg6, &reg[4]);

  ctrl_reg1.sm1_pin    = val->fsm1;
  ctrl_reg2.sm2_pin    = val->fsm2;
  ctrl_reg6.p2_boot    = val->boot;

  if ( (val->fsm1 | val->fsm2 | val->boot) ==  PROPERTY_ENABLE){
    ctrl_reg3.int1_en = PROPERTY_ENABLE;
  }
  else {
    ctrl_reg3.int1_en = PROPERTY_DISABLE;
  }

  bytecpy(&reg[0], (uint8_t*)&ctrl_reg1);
  bytecpy(&reg[1], (uint8_t*)&ctrl_reg2);
  bytecpy(&reg[2], (uint8_t*)&ctrl_reg3);
  bytecpy(&reg[4], (uint8_t*)&ctrl_reg6);
  if (ret == 0) {
    ret = lis3dsh_write_reg(ctx, LIS3DSH_CTRL_REG1, (uint8_t*)&reg, 5);
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
int32_t lis3dsh_pin_int2_route_get(stmdev_ctx_t *ctx,
                                    lis3dsh_pin_int2_route_t *val)
{
  lis3dsh_ctrl_reg1_t ctrl_reg1;
  lis3dsh_ctrl_reg2_t ctrl_reg2;
  lis3dsh_ctrl_reg3_t ctrl_reg3;
  lis3dsh_ctrl_reg6_t ctrl_reg6;
  uint8_t reg[5];
  int32_t ret;

  ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG1, (uint8_t*)&reg, 5);
  bytecpy(( uint8_t*)&ctrl_reg1, &reg[0]);
  bytecpy(( uint8_t*)&ctrl_reg2, &reg[1]);
  bytecpy(( uint8_t*)&ctrl_reg3, &reg[2]);
  bytecpy(( uint8_t*)&ctrl_reg6, &reg[4]);

  if ( ctrl_reg3.int1_en == PROPERTY_ENABLE ){
    val->fsm1       = ctrl_reg1.sm1_pin; 
    val->fsm2       = ctrl_reg2.sm2_pin; 
    val->boot       = ctrl_reg6.p2_boot; 

  }
  else {
    val->fsm1       = PROPERTY_DISABLE;
    val->fsm2       = PROPERTY_DISABLE;
    val->boot       = PROPERTY_DISABLE; 
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
int32_t lis3dsh_all_sources_get(stmdev_ctx_t *ctx, lis3dsh_all_sources_t *val)
{

  lis3dsh_fifo_src_t fifo_src;
  lis3dsh_stat_t stat;
  int32_t ret;

  ret = lis3dsh_read_reg(ctx, LIS3DSH_STAT, (uint8_t*)&stat, 1);
  if (ret == 0) {
    ret = lis3dsh_read_reg(ctx, LIS3DSH_FIFO_SRC, (uint8_t*)&fifo_src, 1);
  }
  
  val->drdy_xl        = stat.drdy; 
  val->ovrn_xl        = stat.dor; 
  val->fsm_lc         = stat.l_count; 
  val->fsm_ext_sync   = stat.syncw; 
  val->fsm1_wait_fsm2 = stat.sync1; 
  val->fsm2_wait_fsm1 = stat.sync2; 
  val->fsm1           = stat.int_sm1; 
  val->fsm2           = stat.int_sm2; 
  val->fifo_ovr       = fifo_src.ovrn_fifo;
  val->fifo_empty     = fifo_src.empty;
  val->fifo_full      = fifo_src.ovrn_fifo;
  val->fifo_th        = fifo_src.wtm;

  return ret;
}

/**
  * @brief  Sensor conversion parameters selection.[set]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          set the sensor conversion parameters by checking
  *                      the constraints of the device.(ptr)
  *
  */
int32_t lis3dsh_mode_set(stmdev_ctx_t *ctx, lis3dsh_md_t *val)
{
  lis3dsh_ctrl_reg4_t ctrl_reg4;
  lis3dsh_ctrl_reg5_t ctrl_reg5;
  int32_t ret;

  ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  if (ret == 0) {
    ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
  }

  ctrl_reg4.odr = (uint8_t)val->odr;
  ctrl_reg5.fscale = (uint8_t)val->fs;

  if (ret == 0) {
    ret = lis3dsh_write_reg(ctx, LIS3DSH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  }
  if (ret == 0) {
    ret = lis3dsh_write_reg(ctx, LIS3DSH_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
  }

  return ret;
}

/**
  * @brief  Sensor conversion parameters selection.[get]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          get the sensor conversion parameters.(ptr)
  *
  */
int32_t lis3dsh_mode_get(stmdev_ctx_t *ctx, lis3dsh_md_t *val)
{
  
  lis3dsh_ctrl_reg4_t ctrl_reg4;
  lis3dsh_ctrl_reg5_t ctrl_reg5;
  int32_t ret;

  ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  if (ret == 0) {
    ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
  }

  switch (ctrl_reg4.odr) {
    case LIS3DSH_OFF:
      val->odr = LIS3DSH_OFF;
      break;
    case LIS3DSH_3Hz125:
      val->odr = LIS3DSH_3Hz125;
      break;
    case LIS3DSH_6Hz25:
      val->odr = LIS3DSH_6Hz25;
      break;
    case LIS3DSH_12Hz5:
      val->odr = LIS3DSH_12Hz5;
      break;
    case LIS3DSH_25Hz:
      val->odr = LIS3DSH_25Hz;
      break;
    case LIS3DSH_50Hz:
      val->odr = LIS3DSH_50Hz;
      break;
    case LIS3DSH_100Hz:
      val->odr = LIS3DSH_100Hz;
      break;
    case LIS3DSH_400Hz:
      val->odr = LIS3DSH_400Hz;
      break;
    case LIS3DSH_800Hz:
      val->odr = LIS3DSH_800Hz;
      break;
    case LIS3DSH_1kHz6:
      val->odr = LIS3DSH_1kHz6;
      break;
    default:
      val->odr = LIS3DSH_OFF;
      break;
  }

  switch (ctrl_reg5.fscale) {
    case LIS3DSH_2g:
      val->fs = LIS3DSH_2g;
      break;
    case LIS3DSH_4g:
      val->fs = LIS3DSH_4g;
      break;
    case LIS3DSH_6g:
      val->fs = LIS3DSH_6g;
      break;
    case LIS3DSH_8g:
      val->fs = LIS3DSH_8g;
      break;
    case LIS3DSH_16g:
      val->fs = LIS3DSH_16g;
      break;
    default:
      val->fs = LIS3DSH_2g;
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
int32_t lis3dsh_data_get(stmdev_ctx_t *ctx, lis3dsh_md_t *md,
                         lis3dsh_data_t *data)
{
  uint8_t buff[6];
  int32_t ret;
  uint8_t i;
  uint8_t j;
  
  ret = lis3dsh_read_reg(ctx, LIS3DSH_OUT_T, (uint8_t*)&data->heat.raw, 1);
  if (ret == 0) {
    ret = lis3dsh_read_reg(ctx, LIS3DSH_OUT_X_L, (uint8_t*)&buff, 6);
  }
  

  /* temperature conversion */
  data->heat.deg_c = lis3dsh_from_lsb_to_celsius(data->heat.raw);

  /* acceleration conversion */
  j = 0U;
  for (i = 0U; i < 3U; i++) {
    data->xl.raw[i] = (int16_t)buff[j+1U];
    data->xl.raw[i] = (data->xl.raw[i] * 256) + (int16_t) buff[j];
    j+=2U;
    switch ( md->fs ) {
      case LIS3DSH_2g:
        data->xl.mg[i] =lis3dsh_from_fs2_to_mg(data->xl.raw[i]);
        break;
      case LIS3DSH_4g:
        data->xl.mg[i] =lis3dsh_from_fs4_to_mg(data->xl.raw[i]);
        break;
      case LIS3DSH_6g:
        data->xl.mg[i] =lis3dsh_from_fs6_to_mg(data->xl.raw[i]);
        break;
      case LIS3DSH_8g:
        data->xl.mg[i] =lis3dsh_from_fs8_to_mg(data->xl.raw[i]);
        break;
      case LIS3DSH_16g:
        data->xl.mg[i] =lis3dsh_from_fs16_to_mg(data->xl.raw[i]);
        break;
      default:
        data->xl.mg[i] = 0.0f;
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
  * @brief  Configures the self test.[set]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          Self test mode mode.(ptr)
  *
  */
int32_t lis3dsh_self_test_set(stmdev_ctx_t *ctx, lis3dsh_st_t val)
{
  lis3dsh_ctrl_reg5_t ctrl_reg5;
  int32_t ret;

  ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
  if (ret == 0) {
    ctrl_reg5.st = (uint8_t) val;
    ret = lis3dsh_write_reg(ctx, LIS3DSH_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
  }
  return ret;
}

/**
  * @brief  Get self test configuration.[set]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          Self test mode mode.(ptr)
  *
  */
int32_t lis3dsh_self_test_get(stmdev_ctx_t *ctx, lis3dsh_st_t *val)
{
  lis3dsh_ctrl_reg5_t ctrl_reg5;
  int32_t ret;

  ret = lis3dsh_read_reg(ctx, LIS3DSH_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);

  switch (ctrl_reg5.st)
  {
    case LIS3DSH_ST_DISABLE:
      *val = LIS3DSH_ST_DISABLE;
      break;
    case LIS3DSH_ST_POSITIVE:
      *val = LIS3DSH_ST_POSITIVE;
      break;
    case LIS3DSH_ST_NEGATIVE:
      *val = LIS3DSH_ST_NEGATIVE;
      break;
    default:
      *val = LIS3DSH_ST_DISABLE;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
