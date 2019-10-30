/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
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

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A2
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 *
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A2 - Host side: UART(COM) to USB bridge
 *                                       - I2C(Default)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read` and
 * `tx_com` is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */
#define NUCLEO_F411RE_X_NUCLEO_IKS01A2

#ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
/* NUCLEO_F411RE_X_NUCLEO_IKS01A2: Define communication interface */
#define SENSOR_BUS hi2c1
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stts22h_reg.h"
#include "gpio.h"
#include "i2c.h"
#ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
#include "usart.h"
#endif

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

static axis1bit16_t data_raw_temperature;
static float temperature_degC;
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);

/* Main Example --------------------------------------------------------------*/
void example_main_stts22h(void)
{
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Check device ID */
  stts22h_dev_id_get(&dev_ctx, &whoamI);
  if (whoamI != STTS22H_ID)
    while(1); /* manage here device not found */

  /* Enable interrupt on high(=49.5 degC)/low(=2.5 degC) temperature. */
  //float temperature_high_limit = 49.5f;
  //stts22h_temp_trshld_high_set(&dev_ctx, (int8_t)(temperature_high_limit / 64.0f) + 64 );

  //float temperature_low_limit = 2.5f;
  //stts22h_temp_trshld_low_set(&dev_ctx, (int8_t)(temperature_low_limit / 64.0f) + 64 );

  /* Set Output Data Rate */
  stts22h_temp_data_rate_set(&dev_ctx, STTS22H_1Hz);

  /* Read samples in polling mode */
  while(1)
  {
    /*
     * Read output only if not busy
     * WARNING: _flag_data_ready_get works only when the device is in single
     *          mode or with data rate set at 1Hz (this function use the busy
     *          bit in status register please see the DS for details)
     */
    uint8_t flag;
    stts22h_temp_flag_data_ready_get(&dev_ctx, &flag);
    if (flag)
    {
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0, sizeof(int16_t));
      stts22h_temperature_raw_get(&dev_ctx, &data_raw_temperature.i16bit);
      temperature_degC = stts22h_from_lsb_to_celsius(data_raw_temperature.i16bit);

      sprintf((char*)tx_buffer, "Temperature [degC]:%3.2f\r\n", temperature_degC);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
    }
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
#ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
  /** I2C Device Address 8 bit format **/
  HAL_I2C_Mem_Write(handle, STTS22H_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#endif
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
#ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
  /** I2C Device Address 8 bit format **/
  HAL_I2C_Mem_Read(handle, STTS22H_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#endif
  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to trasmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  #ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
  #endif
}

