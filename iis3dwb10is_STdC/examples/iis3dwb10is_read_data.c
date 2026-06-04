/*
 ******************************************************************************
 * @file    read_data.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI208V1K
 * - STEVAL_MKI109D  + STEVAL-MKI208V1K
 *
 * Used interfaces:
 *
 * STEVAL_MKI109D     - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default)
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default)
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

//#define STEVAL_MKI109V3  /* little endian */
//#define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#if defined(STEVAL_MKI109D)
/* MKI109D: Define communication interface */
#define SENSOR_BUS hspi1

/* MKI109D: Vdd and Vddio power supply values */
#define ISS3DWB10IS_VDD 3.3f
#define ISS3DWB10IS_VDDIO 3.3f
#elif defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2
/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "iis3dwb10is_reg.h"

#if defined(STEVAL_MKI109D)
#include "board.h"
#include "usbd_cdc_if.h"
#elif defined(STEVAL_MKI109V3)
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#endif

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME        20 //ms

/* Private variables ---------------------------------------------------------*/
static int32_t data_raw_acceleration[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t temperature_degC;
static uint8_t whoamI;
static uint8_t tx_buffer[1000];
static iis3dwb10is_pin_int_route_t route = {0};

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

static uint64_t xl_event_num = 0, temp_event_num = 0;
static uint8_t xl_event = 0, temp_event = 0;
static stmdev_ctx_t dev_ctx;

void iis3dwb10is_read_data_handler(void)
{
  iis3dwb10is_data_ready_t drdy;

  /* Read output only if new xl value is available */
  iis3dwb10is_data_ready_get(&dev_ctx, &drdy);
  if (drdy.drdy_xl) {
    xl_event = 1;
    xl_event_num++;
    iis3dwb10is_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
  }

  if (drdy.drdy_temp) {
    temp_event = 1;
    temp_event_num++;
    iis3dwb10is_temperature_raw_get(&dev_ctx, &data_raw_temperature);
  }
}

/* Main Example --------------------------------------------------------------*/
void iis3dwb10is_read_data(void)
{
  iis3dwb10is_data_rate_t rate;
  iis3dwb10is_xl_data_cfg_t xl_cfg;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  iis3dwb10is_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != IIS3DWB10IS_ID)
    while (1);

  /* Restore default configuration */
  iis3dwb10is_sw_por(&dev_ctx);

  /* Enable Block Data Update */
  iis3dwb10is_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  iis3dwb10is_xl_data_config_get(&dev_ctx, &xl_cfg);
  xl_cfg.rounding = IIS3DWB10IS_WRAPAROUND_DISABLED;
  iis3dwb10is_xl_data_config_set(&dev_ctx, xl_cfg);

  /* Set full scale */
  iis3dwb10is_xl_full_scale_set(&dev_ctx, IIS3DWB10IS_50g);

  iis3dwb10is_pin_int1_route_get(&dev_ctx, &route);
  route.drdy_xl = 1;
  route.drdy_temp = 1;
  iis3dwb10is_pin_int1_route_set(&dev_ctx, route);

  /* Set Output Data Rate */
  rate.burst = IIS3DWB10IS_CONTINUOS_MODE;
  rate.odr = IIS3DWB10IS_ODR_10KHz;
  iis3dwb10is_xl_data_rate_set(&dev_ctx, rate);

  /* Read samples in polling mode */
  while (1) {
    if (xl_event && xl_event_num%100 == 0) {
      xl_event = 0;

      acceleration_mg[0] =
        iis3dwb10is_from_fs50g_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
        iis3dwb10is_from_fs50g_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
        iis3dwb10is_from_fs50g_to_mg(data_raw_acceleration[2]);

      sprintf((char *)tx_buffer,
              "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (temp_event && temp_event_num%52 == 0) {
      temp_event = 0;

      /* Read temperature data */
      temperature_degC = iis3dwb10is_from_lsb_to_celsius(data_raw_temperature);

      sprintf((char *)tx_buffer, "Temperature [degC]:%6.2f\r\n", temperature_degC);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
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
#if defined(STEVAL_MKI109D)
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
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
#if defined(STEVAL_MKI109D)
  reg |= 0x80;
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(handle, bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
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
#if defined(STEVAL_MKI109D)
  CDC_Transmit_FS(tx_buffer, len);
#elif defined(STEVAL_MKI109V3)
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
#if defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(STEVAL_MKI109D)
  delay(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#if defined(STEVAL_MKI109D)
  struct spi_conf spi_conf;

  /* init SPI bus communication */
  spi_conf.wire = WIRE_4;
  spi_init(&spi_conf);

  set_range_100x(1);

  /* set VDD/VDDIO on DIL24 */
  set_vdd(ISS3DWB10IS_VDD);
  set_vddio(ISS3DWB10IS_VDDIO);
  delay(100);
#elif defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}
