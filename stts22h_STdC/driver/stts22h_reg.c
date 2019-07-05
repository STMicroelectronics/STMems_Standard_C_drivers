/*
 ******************************************************************************
 * @file    stts22h_reg.c
 * @author  Sensors Software Solution Team
 * @brief   STTS22H driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "stts22h_reg.h"

/**
  * @defgroup    STTS22H
  * @brief       This file provides a set of functions needed to drive the
  *              stts22h enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup    STTS22H_Interfaces_Functions
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
int32_t stts22h_read_reg(stts22h_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
int32_t stts22h_write_reg(stts22h_ctx_t* ctx, uint8_t reg, uint8_t* data,
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
  * @defgroup    STTS22H_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t stts22h_from_lsb_to_celsius(int16_t lsb)
{
  return ((float_t)lsb /100.0f);
}

/**
  * @}
  *
  */

/**
  * @defgroup    STTS22H_Data_generation
  * @brief       This section groups all the functions concerning
  *              data generation.
  * @{
  *
  */

/**
  * @brief  Temperature sensor data rate selection..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "one_shot" in reg STTS22H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_temp_data_rate_set(stts22h_ctx_t *ctx, stts22h_odr_temp_t val)
{
  stts22h_ctrl_t ctrl;
  int32_t ret;

  ret = stts22h_read_reg(ctx, STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  if(ret == 0){
    ctrl.one_shot = (uint8_t)val & 0x01U;
    ctrl.freerun = ((uint8_t)val & 0x02U) >> 1;
    ctrl.low_odr_en = ((uint8_t)val & 0x04U) >> 2;
    ctrl.avg = ((uint8_t)val & 0x30U) >> 4;
    ret = stts22h_write_reg(ctx, STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  }
  return ret;
}

/**
  * @brief  Temperature sensor data rate selection..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of one_shot in reg CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_temp_data_rate_get(stts22h_ctx_t *ctx,
                                   stts22h_odr_temp_t *val)
{
  stts22h_ctrl_t ctrl;
  int32_t ret;

  ret = stts22h_read_reg(ctx, STTS22H_CTRL,
                            (uint8_t*)&ctrl, 1);
  switch ( ctrl.one_shot | (ctrl.freerun << 1) | (ctrl.low_odr_en << 2) |
           (ctrl.avg << 4)){
    case STTS22H_POWER_DOWN:
      *val = STTS22H_POWER_DOWN;
      break;
    case STTS22H_ONE_SHOT:
      *val = STTS22H_ONE_SHOT;
      break;
    case STTS22H_1Hz:
      *val = STTS22H_1Hz;
      break;
    case STTS22H_25Hz:
      *val = STTS22H_25Hz;
      break;
    case STTS22H_50Hz:
      *val = STTS22H_50Hz;
      break;
    case STTS22H_100Hz:
      *val = STTS22H_100Hz;
      break;
    case STTS22H_200Hz:
      *val = STTS22H_200Hz;
      break;
    default:
      *val = STTS22H_POWER_DOWN;
      break;
  }
  return ret;
}

/**
  * @brief  Block data update.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of bdu in reg CTRL.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_block_data_update_set(stts22h_ctx_t *ctx, uint8_t val)
{
  stts22h_ctrl_t ctrl;
  int32_t ret;

  ret = stts22h_read_reg(ctx, STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  if(ret == 0){
    ctrl.bdu = val;
    ret = stts22h_write_reg(ctx, STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  }
  return ret;
}

/**
  * @brief  Block data update.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of bdu in reg CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_block_data_update_get(stts22h_ctx_t *ctx, uint8_t *val)
{
  int32_t ret;
  ret = stts22h_read_reg(ctx, STTS22H_CTRL, (uint8_t*)val, 1);
  return ret;
}

/**
  * @brief    New data available from temperature sensor..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Return an option of "stts22h_uint8_t".(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_temp_flag_data_ready_get(stts22h_ctx_t *ctx, uint8_t *val)
{
  stts22h_status_t status;
  int32_t ret;
  ret = stts22h_read_reg(ctx, STTS22H_STATUS, (uint8_t*)&status, 1);
  if (status.busy == PROPERTY_DISABLE){
    *val = PROPERTY_ENABLE;
  }
  else{
    *val = PROPERTY_DISABLE;   
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    STTS22H_Dataoutput
  * @brief       This section groups all the data output functions.
  * @{
  *
  */

/**
  * @brief   Temperature data output register(r). L and H registers
  *          together express a 16-bit word in two’s complement..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_temperature_raw_get(stts22h_ctx_t *ctx, int16_t *buff)
{  
  uint16_t temperature;
  uint8_t temperature_low;
  int32_t ret;

  ret = stts22h_read_reg(ctx, STTS22H_TEMP_H_OUT,
                         (uint8_t*)&temperature, 1);
  if (ret == 0) {
    ret = stts22h_read_reg(ctx, STTS22H_TEMP_L_OUT,
                           &temperature_low, 1);

    temperature  = (temperature << 8) + temperature_low;
    *buff = (int16_t)temperature;
  }  
  
  return ret;
}
/**
  * @}
  *
  */

/**
  * @defgroup    STTS22H_Common
  * @brief       This section groups common usefull functions.
  * @{
  *
  */

/**
  * @brief  Device Who am I..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_dev_id_get(stts22h_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = stts22h_read_reg(ctx, STTS22H_WHOAMI, buff, 1);
  return ret;
}
/**
  * @brief   Device status register.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    In one-shot mode this bit is high when the
  *                conversion is in progress..(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_dev_status_get(stts22h_ctx_t *ctx, stts22h_dev_status_t *val)
{
  stts22h_status_t status;
  int32_t ret;

  ret = stts22h_read_reg(ctx, STTS22H_STATUS, (uint8_t*)&status, 1);
  val->busy = status.busy;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    STTS22H_Serial_interface
  * @brief       This section groups all the functions concerning main
  *              serial interface management.
  * @{
  *
  */

/**
  * @brief  SMBus mode..[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "time_out_dis" in reg STTS22H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_smbus_interface_set(stts22h_ctx_t *ctx,
                                    stts22h_smbus_md_t val)
{
  stts22h_ctrl_t ctrl;
  int32_t ret;

  ret = stts22h_read_reg(ctx, STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  if(ret == 0){
    ctrl.time_out_dis = (uint8_t)val;
    ret = stts22h_write_reg(ctx, STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  }
  return ret;
}

/**
  * @brief  SMBus mode..[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of time_out_dis in reg CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_smbus_interface_get(stts22h_ctx_t *ctx, 
                                    stts22h_smbus_md_t *val)
{
  stts22h_ctrl_t ctrl;
  int32_t ret;

  ret = stts22h_read_reg(ctx, STTS22H_CTRL,
                            (uint8_t*)&ctrl, 1);
  switch (ctrl.time_out_dis){
    case STTS22H_SMBUS_TIMEOUT_ENABLE:
      *val = STTS22H_SMBUS_TIMEOUT_ENABLE;
      break;
    case STTS22H_SMBUS_TIMEOUT_DISABLE:
      *val = STTS22H_SMBUS_TIMEOUT_DISABLE;
      break;
    default:
      *val = STTS22H_SMBUS_TIMEOUT_ENABLE;
      break;
  }
  return ret;
}

/**
  * @brief  Register address automatically incremented during a multiple
  *         byte access with a serial interface.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "if_add_inc" in reg STTS22H.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_auto_increment_set(stts22h_ctx_t *ctx, uint8_t val)
{
  stts22h_ctrl_t ctrl;
  int32_t ret;

  ret = stts22h_read_reg(ctx, STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  if(ret == 0){
    ctrl.if_add_inc = (uint8_t)val;
    ret = stts22h_write_reg(ctx, STTS22H_CTRL, (uint8_t*)&ctrl, 1);
  }
  return ret;
}

/**
  * @brief   Register address automatically incremented during a multiple
  *          byte access with a serial interface.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of if_add_inc in reg CTRL.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_auto_increment_get(stts22h_ctx_t *ctx, uint8_t *val)
{
  int32_t ret;
  ret = stts22h_read_reg(ctx, STTS22H_CTRL, (uint8_t*)&val, 1);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    STTS22H_ Interrupt_on_threshold
  * @brief       This section group all the functions concerning the
  *              interrupt on threshold configuration.
  * @{
  *
  */

/**
  * @brief  Over temperature interrupt value. ( degC / 0.64 ) + 64.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of thl in reg TEMP_H_LIMIT.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_temp_trshld_high_set(stts22h_ctx_t *ctx, uint8_t val)
{
  stts22h_temp_h_limit_t temp_h_limit;
  int32_t ret;

  ret = stts22h_read_reg(ctx, STTS22H_TEMP_H_LIMIT,
                         (uint8_t*)&temp_h_limit, 1);
  if(ret == 0){
    temp_h_limit.thl = val;
    ret = stts22h_write_reg(ctx, STTS22H_TEMP_H_LIMIT,
                            (uint8_t*)&temp_h_limit, 1);
  }
  return ret;
}

/**
  * @brief  Over temperature interrupt value. ( degC / 0.64 ) + 64.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of thl in reg TEMP_H_LIMIT.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_temp_trshld_high_get(stts22h_ctx_t *ctx, uint8_t *val)
{
  stts22h_temp_h_limit_t temp_h_limit;
  int32_t ret;

  ret = stts22h_read_reg(ctx, STTS22H_TEMP_H_LIMIT,
                         (uint8_t*)&temp_h_limit, 1);
  *val = temp_h_limit.thl;

  return ret;
}

/**
  * @brief   Under temperature interrupt value. ( degC / 0.64 ) + 64.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of tll in reg TEMP_L_LIMIT.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_temp_trshld_low_set(stts22h_ctx_t *ctx, uint8_t val)
{
  stts22h_temp_l_limit_t temp_l_limit;
  int32_t ret;

  ret = stts22h_read_reg(ctx, STTS22H_TEMP_L_LIMIT,
                         (uint8_t*)&temp_l_limit, 1);
  if(ret == 0){
    temp_l_limit.tll = val;
    ret = stts22h_write_reg(ctx, STTS22H_TEMP_L_LIMIT,
                            (uint8_t*)&temp_l_limit, 1);
  }
  return ret;
}

/**
  * @brief   Under temperature interrupt value. ( degC / 0.64 ) + 64.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of tll in reg TEMP_L_LIMIT.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_temp_trshld_low_get(stts22h_ctx_t *ctx, uint8_t *val)
{
  stts22h_temp_l_limit_t temp_l_limit;
  int32_t ret;

  ret = stts22h_read_reg(ctx, STTS22H_TEMP_L_LIMIT,
                            (uint8_t*)&temp_l_limit, 1);
  *val = temp_l_limit.tll;

  return ret;
}

/**
  * @brief   Temperature interrupt on threshold source.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val     Low limit temperature exceeded..(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t stts22h_temp_trshld_src_get(stts22h_ctx_t *ctx,
                                    stts22h_temp_trlhd_src_t *val)
{
  stts22h_status_t status;
  int32_t ret;

  ret = stts22h_read_reg(ctx, STTS22H_STATUS, (uint8_t*)&status, 1);
  val->under_thl = status.under_thl;
  val->over_thh = status.over_thh;

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