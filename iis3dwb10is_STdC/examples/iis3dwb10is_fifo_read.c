/*
 ******************************************************************************
 * @file    fifo_read.c
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
 * - DISCOVERY_SPC584B + STEVAL-MKI208V1K
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
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

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2
/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS NULL
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "iis3dwb10is_reg.h"

#if defined(NUCLEO_F411RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#elif defined(STEVAL_MKI109V3)
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"

#elif defined(SPC584B_DIS)
#include "components.h"
#endif

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME        20 //ms
#define    FIFO_WATERMARK   1000
#define    FIFO_MAX_SIZE    2048

/* Private variables ---------------------------------------------------------*/
static float acceleration_mg[3];
static float temperature_degC;
static uint64_t timestamp_us;
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

static stmdev_ctx_t dev_ctx;
static uint64_t fifo_event_num = 0;
static uint8_t fifo_event = 0;
static uint8_t fifo_buf[FIFO_ROW_LEN * FIFO_MAX_SIZE];
static iis3dwb10is_fifo_out_raw_t fifo_data;

void iis3dwb10is_fifo_read_handler(void)
{
  fifo_event = 1;
}

static void iis3dwb10is_fifo_read_thread(void)
{
  uint16_t i;
  iis3dwb10is_fifo_out_raw_t *fdatap;
  iis3dwb10is_fifo_status_t fifo_status;
  uint16_t fifo_level = 0;
  uint8_t do_print = 0;

  /* Read watermark flag */
  iis3dwb10is_fifo_status_get(&dev_ctx, &fifo_status);

  if (fifo_status.fifo_th) {
    if (fifo_event_num++%10 == 0) {
      do_print = 1;
    }

    fifo_level = fifo_status.fifo_level;
  }

  /* spurious interrupt */
  if (fifo_level == 0)
    return;

  iis3dwb10is_fifo_out_raw_get(&dev_ctx, fifo_buf, fifo_level);

  if (do_print) {
    sprintf((char *)tx_buffer, "%lld: FIFO level %d\r\n", fifo_event_num, fifo_level);
    tx_com(tx_buffer, strlen((char const *)tx_buffer));

    for (i = 0; i < fifo_level; i++) {
      iis3dwb10is_fifo_process(&fifo_buf[i*FIFO_ROW_LEN], &fifo_data);
      fdatap = &fifo_data;

      switch(fdatap->tag) {
        case IIS3DWB10IS_TAG_EMPTY:
          break;

        case IIS3DWB10IS_TAG_TS:
          timestamp_us = iis3dwb10is_from_lsb_to_us(fdatap->ts_raw);

          sprintf((char *)tx_buffer, "Timestamp [us]:%lld\r\n", timestamp_us);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          break;

        case IIS3DWB10IS_TAG_XL:
          acceleration_mg[0] =
            iis3dwb10is_from_fs50g_to_mg(fdatap->xl.x_raw);
          acceleration_mg[1] =
            iis3dwb10is_from_fs50g_to_mg(fdatap->xl.y_raw);
          acceleration_mg[2] =
            iis3dwb10is_from_fs50g_to_mg(fdatap->xl.z_raw);

          sprintf((char *)tx_buffer,
                  "%d: Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n", i,
                  acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          break;

        case IIS3DWB10IS_TAG_TEMP:
          temperature_degC = iis3dwb10is_from_lsb_to_celsius(fdatap->temp_raw);

          sprintf((char *)tx_buffer, "Temperature [degC]:%6.2f\r\n", temperature_degC);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          break;

        case IIS3DWB10IS_TAG_TEMP_QVAR:
          temperature_degC = iis3dwb10is_from_lsb_to_celsius(fdatap->temp_raw);

          sprintf((char *)tx_buffer, "Temperature [degC]:%6.2f - QVAR [mV]:%04x\r\n", temperature_degC, fdatap->qvar);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          break;

        default:
          sprintf((char *)tx_buffer, "unhandled tag %02x\r\n", fdatap->tag);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          break;
      }
    }

    sprintf((char *)tx_buffer, "-- DONE %d\r\n\r\n", i);
    tx_com(tx_buffer, strlen((char const *)tx_buffer));

    do_print = 0;
  }
}

/* Main Example --------------------------------------------------------------*/
void iis3dwb10is_fifo_read(void)
{
  iis3dwb10is_reset_t rst;
  iis3dwb10is_data_rate_t rate;
  iis3dwb10is_fifo_sensor_batch_t fifo_batch;
  iis3dwb10is_qvar_mode_t qvar_mode;
  iis3dwb10is_int_pin_t int_pin;

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
  rst.boot = 1;
  rst.sw_rst = 1;
  iis3dwb10is_reset_set(&dev_ctx, rst);

  do {
    iis3dwb10is_reset_get(&dev_ctx, &rst);
  } while (rst.sw_rst);

  /*
   * Set FIFO watermark (number of unread sensor data TAG + 9 bytes
   * stored in FIFO) to FIFO_WATERMARK samples
   */

  iis3dwb10is_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);
  iis3dwb10is_fifo_stop_on_wtm_set(&dev_ctx, 1);

  iis3dwb10is_fifo_batch_get(&dev_ctx,  &fifo_batch);
  fifo_batch.batch_xl = 1;
  fifo_batch.batch_temp = 1;
  fifo_batch.batch_ts = IIS3DWB10IS_TMSTMP_NOT_BATCHED;
  fifo_batch.batch_qvar = 1;
  iis3dwb10is_fifo_batch_set(&dev_ctx,  fifo_batch);

  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  iis3dwb10is_fifo_mode_set(&dev_ctx, IIS3DWB10IS_STREAM_MODE);

  /* Enable Block Data Update */
  iis3dwb10is_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set full scale */
  iis3dwb10is_xl_full_scale_set(&dev_ctx, IIS3DWB_50g);

  iis3dwb10is_interrupt_pin_mode_get(&dev_ctx, &int_pin);
  int_pin.pp_od = IIS3DWB10IS_PUSH_PULL;
  int_pin.strength = IIS3DWB10IS_PAD_STRENGTH_LOWER;
  iis3dwb10is_interrupt_pin_mode_set(&dev_ctx, int_pin);

  iis3dwb10is_pin_int_route_get(&dev_ctx, &route);
  route.int1_fifo_th = 1;
  iis3dwb10is_pin_int_route_set(&dev_ctx, route);

  iis3dwb10is_qvar_mode_get(&dev_ctx, &qvar_mode);
  qvar_mode.qvar_en = 1;
  qvar_mode.qvar1_pad_en = 1;
  qvar_mode.qvar2_pad_en = 1;
  qvar_mode.c_zin = IIS3DWB10IS_QVAR_GAIN_2X;
  iis3dwb10is_qvar_mode_set(&dev_ctx, qvar_mode);

  /* Set Output Data Rate */
  rate.burst = IIS3DWB10IS_CONTINUOS_MODE;
  rate.odr = IIS3DWB10IS_ODR_10KHz;
  iis3dwb10is_xl_data_rate_set(&dev_ctx, rate);

  /* Read samples in polling mode */
  while (1) {
    if (fifo_event) {
      fifo_event = 0;
      iis3dwb10is_fifo_read_thread();
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
#ifdef STEVAL_MKI109V3

  if (handle == &hspi2) {
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }

#elif defined(SPC584B_DIS)
  /* Add here the SPC5 write SPI interface */
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

#elif defined(SPC584B_DIS)
  /* Add here the SPC5 read SPI interface */
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
#if defined(NUCLEO_F411RE) | defined(STEVAL_MKI109V3)
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
#if defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}
