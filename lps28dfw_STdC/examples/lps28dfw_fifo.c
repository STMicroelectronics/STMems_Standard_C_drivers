/*
 ******************************************************************************
 * @file    fifo.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - NUCLEO_F401RE
 * - DISCOVERY_SPC584B
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(N/A)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
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

//#define NUCLEO_F401RE    /* little endian */
//#define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */


#if defined(NUCLEO_F401RE)
/* NUCLEO_F401RE: Define communication interface */
#define SENSOR_BUS hi2c1

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS I2CD1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lps28dfw_reg.h"

#if defined(NUCLEO_F401RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#elif defined(SPC584B_DIS)
#include "components.h"
#endif

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME         10 //ms

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[1000];
static lps28dfw_fifo_data_t data[32];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lps28dfw_fifo(void)
{
  lps28dfw_all_sources_t all_sources;
  lps28dfw_bus_mode_t bus_mode;
  lps28dfw_stat_t status;
  stmdev_ctx_t dev_ctx;
  lps28dfw_id_t id;
  lps28dfw_md_t md;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Initialize platform specific hardware */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  lps28dfw_id_get(&dev_ctx, &id);
  if (id.whoami != LPS28DFW_ID)
    while(1);

  /* Restore default configuration */
  lps28dfw_init_set(&dev_ctx, LPS28DFW_RESET);
  do {
    lps28dfw_status_get(&dev_ctx, &status);
  } while (status.sw_reset);

  /* Set bdu and if_inc recommended for driver usage */
  lps28dfw_init_set(&dev_ctx, LPS28DFW_DRV_RDY);

  /* Select bus interface */
  bus_mode.filter = LPS28DFW_AUTO;
  lps28dfw_bus_mode_set(&dev_ctx, &bus_mode);

  /* Set Output Data Rate */
  md.odr = LPS28DFW_10Hz;
  md.avg = LPS28DFW_16_AVG;
  md.lpf = LPS28DFW_LPF_ODR_DIV_4;
  md.fs = LPS28DFW_1260hPa;
  lps28dfw_mode_set(&dev_ctx, &md);

  /* Enable FIFO */
  lps28dfw_fifo_mode_set(&dev_ctx, LPS28DFW_STREAM);
  lps28dfw_fifo_stop_on_wtm_set(&dev_ctx, LPS28DFW_FIFO_EV_WTM);
  lps28dfw_fifo_watermark_set(&dev_ctx, 32);

  /* Read samples in polling mode (no int) */
  while(1)
  {
    uint8_t level, i;

    /* Read output only if new values are available */
    lps28dfw_all_sources_get(&dev_ctx, &all_sources);
    if (all_sources.fifo_th) {
      lps28dfw_fifo_level_get(&dev_ctx, &level);
      lps28dfw_fifo_data_get(&dev_ctx, level, &md, data);

      snprintf((char*)tx_buffer, sizeof(tx_buffer), "--- FIFO samples\r\n");
      tx_com(tx_buffer, strlen((char const*)tx_buffer));

      for (i = 0; i < level; i++) {
        snprintf((char*)tx_buffer, sizeof(tx_buffer),
                "%02d: pressure [hPa]:%6.2f\r\n", i, data[i].hpa);

        tx_com(tx_buffer, strlen((char const*)tx_buffer));
      }

      snprintf((char*)tx_buffer, sizeof(tx_buffer), "\r\n");
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
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Write(handle, LPS28DFW_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t *) bufp, len, 1000);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LPS28DFW_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Read(handle, LPS28DFW_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LPS28DFW_I2C_ADD_L & 0xFE, reg, bufp, len);
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
#if defined(NUCLEO_F401RE)
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
#elif defined(SPC584B_DIS)
  sd_lld_write(&SD2, tx_buffer, len);
#endif
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
#if defined(NUCLEO_F401RE)
  HAL_Delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
}
