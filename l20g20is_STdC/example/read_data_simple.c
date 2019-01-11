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
 *                    - Sensor side: I2C(Default) / SPI(supported)
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
/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "stm32f4xx_hal.h"
#include "l20g20is_reg.h"
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
static axis3bit16_t data_raw_angular_rate;
static float angular_rate_mdps[3];
static l20g20is_dev_status_t reg;
static uint8_t rst, whoami;
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
void example_main_l20g20is(void)
{
  /* Initialize platform specific hardware */
  platform_init();

  /* Initialize inertial sensors (IMU) driver interface */
  l20g20is_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = (void*)&hspi2;

  /* Check device ID */
  l20g20is_dev_id_get(&dev_ctx, &whoami);
  if ( whoami != L20G20IS_ID ){
    while(1){
      /* manage here device not found */
    }
  }

  /* Restore default configuration */
  l20g20is_dev_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    l20g20is_dev_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  l20g20is_gy_orient_t l20g20is_gy_orient;
  l20g20is_gy_orient.orient = 0; /* no swap axis */
  l20g20is_gy_orient.signy = 0;  /* no sign inversion on Y axis */
  l20g20is_gy_orient.signy = 0;  /* no sign inversion on X axis */
  l20g20is_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set Orientation */
  l20g20is_gy_orient_set(&dev_ctx, l20g20is_gy_orient);
  
  /* Set full scale */
  l20g20is_gy_full_scale_set(&dev_ctx, L20G20IS_200dps);
  
  /* Configure filtering chain - See datasheet for filtering chain details */
  /* Gyroscope filtering chain */
  l20g20is_gy_filter_lp_bandwidth_set(&dev_ctx, L20G20IS_LPF_BW_160Hz);
  l20g20is_gy_filter_hp_bandwidth_set(&dev_ctx, L20G20IS_HPF_BYPASS);

  /* Set Output Data Rate / Power mode */
  l20g20is_gy_data_rate_set(&dev_ctx, L20G20IS_GY_9k33Hz);

  /* Read samples in polling mode (no int) */
  while(1)
  {
    /* Read device status register */
    l20g20is_dev_status_get(&dev_ctx, &reg);

    if ( reg.xyda_ois )
    {
      /* Read imu data */
      memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));

      l20g20is_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);

      angular_rate_mdps[0] = l20g20is_from_fs200dps_to_mdps(data_raw_angular_rate.i16bit[0]);
      angular_rate_mdps[1] = l20g20is_from_fs200dps_to_mdps(data_raw_angular_rate.i16bit[1]);
      angular_rate_mdps[2] = 0x00;

      sprintf((char*)tx_buffer, "[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
    }


  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor I2C address.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
#ifdef STEVAL_MKI109V3
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#endif
return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor I2C address.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
#ifdef STEVAL_MKI109V3
  /* Read command */
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#endif
  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
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
