/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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
 * - STEVAL_MKI109V3
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Supported) / I2C(Default)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */
#define STEVAL_MKI109V3

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hi2c1

/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#else
#error "Please leave STEVAL_MKI109V3 defined: this sensor support I2C only."
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stts751_reg.h"
#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#endif

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static axis1bit16_t data_raw_temperature;
static float temperature_degC;
static stts751_id_t whoamI;
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
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void example_main_stts751(void)
{
  /* Initialize mems driver interface */
  stts751_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Initialize platform specific hardware */
  platform_init();

  /* Check device ID */
  stts751_device_id_get(&dev_ctx, &whoamI);
  if ( (whoamI.product_id != STTS751_ID_0xxxx) ||
     //(whoamI.product_id != STTS751_ID_1xxxx) ||
       (whoamI.manufacturer_id != STTS751_ID_MAN) ||
       (whoamI.revision_id != STTS751_REV) )
    while(1); /* manage here device not found */

  /* Enable interrupt on high(=49.5 degC)/low(=-4.5 degC) temperature. */
  float temperature_high_limit = 49.5f;
  stts751_high_temperature_threshold_set(&dev_ctx, stts751_from_celsius_to_lsb(temperature_high_limit));

  float temperature_low_limit = -4.5f;
  stts751_low_temperature_threshold_set(&dev_ctx, stts751_from_celsius_to_lsb(temperature_low_limit));

  stts751_pin_event_route_set(&dev_ctx,  PROPERTY_ENABLE);

  /* Set Output Data Rate */
  stts751_temp_data_rate_set(&dev_ctx, STTS751_TEMP_ODR_1Hz);

  /* Set Resolution */
  stts751_resolution_set(&dev_ctx, STTS751_11bit);

  /* Read samples in polling mode */
  while(1)
  {
    /* Read output only if not busy */
    uint8_t flag;
    stts751_flag_busy_get(&dev_ctx, &flag);
    if (flag)
    {
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0, sizeof(int16_t));
      stts751_temperature_raw_get(&dev_ctx, &data_raw_temperature.i16bit);
      temperature_degC = stts751_from_lsb_to_celsius(data_raw_temperature.i16bit);

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
#ifdef STEVAL_MKI109V3
  /** I2C Device Address 8 bit format **/
  HAL_I2C_Mem_Write(handle, STTS751_0xxxx_ADD_7K5, reg,
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
#ifdef STEVAL_MKI109V3
  /** I2C Device Address 8 bit format **/
  HAL_I2C_Mem_Read(handle, STTS751_0xxxx_ADD_7K5, reg,
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
  #ifdef STEVAL_MKI109V3
  CDC_Transmit_FS(tx_buffer, len);
  #endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#ifdef STEVAL_MKI109V3
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_Delay(1000);
#endif
}
