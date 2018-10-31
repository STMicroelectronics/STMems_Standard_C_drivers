/*
 ******************************************************************************
 * @file    wake_up.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to detect wake-up from sensor.
 *          This sample show how to use HP filter.
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
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A2
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A2 - Host side: UART(COM) to USB bridge
 *                                       - I2C(Default) / SPI(N/A)
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
//#define STEVAL_MKI109V3
#define NUCLEO_F411RE_X_NUCLEO_IKS01A2

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2

/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
/* NUCLEO_F411RE_X_NUCLEO_IKS01A2: Define communication interface */
#define SENSOR_BUS hi2c1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "stm32f4xx_hal.h"
#include "iis2dh_reg.h"
#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
#include "usart.h"
#endif

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
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
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void example_wake_up_iis2dh(void)
{
  /*
   *  Initialize mems driver interface
   */
  iis2dh_ctx_t dev_ctx;
  iis2dh_int1_cfg_t int1_cfg;
  iis2dh_ctrl_reg3_t ctrl_reg3;
  uint8_t dummy;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /*
   * Initialize platform specific hardware
   */
  platform_init();

  /*
   *  Check device ID
   */
  iis2dh_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != IIS2DH_ID)
  {
    while(1)
    {
      /* manage here device not found */
    }
  }

  /*
   * High-pass filter enabled on interrupt activity 1
   */
  iis2dh_high_pass_int_conf_set(&dev_ctx, IIS2DH_ON_INT1_GEN);

  /*
   * Enable HP filter for wake-up event detection
   *
   * Use this setting to remove gravity on data output
   */
  iis2dh_high_pass_on_outputs_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Enable AOI1 on int1 pin
   */
  iis2dh_pin_int1_config_get(&dev_ctx, &ctrl_reg3);
  ctrl_reg3.i1_aoi1 = PROPERTY_ENABLE;
  iis2dh_pin_int1_config_set(&dev_ctx, &ctrl_reg3);

  /*
   * Interrupt 1 pin latched
   */
  iis2dh_int1_pin_notification_mode_set(&dev_ctx,
		  	  	  	  IIS2DH_INT1_LATCHED);

  /*
   * Set full scale to 2 g
   */
  iis2dh_full_scale_set(&dev_ctx, IIS2DH_2g);

  /*
   * Set interrupt threshold to 0x10 -> 250 mg
   */
  iis2dh_int1_gen_threshold_set(&dev_ctx, 0x10);

  /*
   * Set no time duration
   */
  iis2dh_int1_gen_duration_set(&dev_ctx, 0);

  /*
   * Dummy read to force the HP filter to current acceleration value.
   */
  iis2dh_filter_reference_get(&dev_ctx, &dummy);

  /*
   * Configure wake-up interrupt event on all axis
   */
  iis2dh_int1_gen_conf_get(&dev_ctx, &int1_cfg);
  int1_cfg.zhie = PROPERTY_ENABLE;
  int1_cfg.yhie = PROPERTY_ENABLE;
  int1_cfg.xhie = PROPERTY_ENABLE;
  int1_cfg.aoi = PROPERTY_DISABLE;
  iis2dh_int1_gen_conf_set(&dev_ctx, &int1_cfg);

  /*
   * Set device in HR mode
   */
  iis2dh_operating_mode_set(&dev_ctx, IIS2DH_HR_12bit);

  /*
   * Set Output Data Rate to 100 Hz
   */
  iis2dh_data_rate_set(&dev_ctx, IIS2DH_ODR_100Hz);

  while(1)
  {
    iis2dh_int1_src_t src;

    /*
     * Read INT pin 1 in polling mode
     * or read src status register
     */
    iis2dh_int1_gen_source_get(&dev_ctx, &src);
    if (src.xh || src.yh || src.zh)
    {
      iis2dh_int1_gen_source_get(&dev_ctx, &src);
      sprintf((char*)tx_buffer, "wake-up detected : "
              "x %d, y %d, z %d\r\n",
              src.xh, src.yh, src.zh);
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
  if (handle == &hi2c1)
  {
    /* Write multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Write(handle, IIS2DH_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
    /* Write multiple command */
    reg |= 0x40;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }
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
  if (handle == &hi2c1)
  {
    /* Read multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Read(handle, IIS2DH_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
    /* Read multiple command */
    reg |= 0xC0;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }
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
