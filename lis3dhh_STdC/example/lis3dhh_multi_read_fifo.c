/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
 * evaluation boards:2
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI180V1
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI
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

#define STEVAL_MKI109V3  /* little endian */


/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2
/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lis3dhh_reg.h"

#if defined(STEVAL_MKI109V3)
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "spi.h"

#endif

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME   10 //ms

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lis3dhh_multi_read_fifo(void)
{
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;
  /* Initialize platform specific hardware */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lis3dhh_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LIS3DHH_ID)
    while (1); /*manage here device not found */

  /* Restore default configuration */
  lis3dhh_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lis3dhh_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lis3dhh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lis3dhh_data_rate_set(&dev_ctx, LIS3DHH_1kHz1);
  /* Set FIFO watermark to 16 samples (half FIFO) */
  lis3dhh_fifo_watermark_set(&dev_ctx, 16);
  /* Set FIFO to Stream mode */
  lis3dhh_fifo_mode_set(&dev_ctx, LIS3DHH_DYNAMIC_STREAM_MODE);
  /* Enable FIFO */
  lis3dhh_fifo_set(&dev_ctx, PROPERTY_ENABLE);

  /* Uncomment to use Int1 as interrupt on FIFO watermark */
  //lis3dhh_fifo_threshold_on_int1_set(&dev_ctx, PROPERTY_ENABLE);

  /* Read samples in polling mode (no int) */
  while (1) {
    uint8_t wtm;
    uint8_t num = 0;
    /* Read output only if new value is available */
    lis3dhh_fifo_fth_flag_get(&dev_ctx, &wtm);

    if (wtm) {
      /* Read number of sample in FIFO */
      lis3dhh_fifo_full_flag_get(&dev_ctx, &num);

      while (num-- > 0) {
        /* Read XL samples */
        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        lis3dhh_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
        acceleration_mg[0] = lis3dhh_from_lsb_to_mg(data_raw_acceleration[0]);
        acceleration_mg[1] = lis3dhh_from_lsb_to_mg(data_raw_acceleration[1]);
        acceleration_mg[2] = lis3dhh_from_lsb_to_mg(data_raw_acceleration[2]);
        sprintf((char *)tx_buffer,
                "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
      }
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
static int32_t platform_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len)
{
#ifdef STEVAL_MKI109V3

  if (handle == &hspi2) {
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
#ifdef STEVAL_MKI109V3

  if (handle == &hspi2) {
    /* Read command */
    reg |= 0x80;
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
 * @param  tx_buffer     buffer to transmit
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
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
#ifdef STEVAL_MKI109V3
  HAL_Delay(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#if defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}
