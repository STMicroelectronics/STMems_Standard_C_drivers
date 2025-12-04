/*
 ******************************************************************************
 * @file    read.c
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
#include "asm9g300b_reg.h"

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
#define    BOOT_TIME        25 //ms

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[1000];

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
static void check_device_status(stmdev_ctx_t *ctx)
{
    uint16_t status;

    asm9g300b_summary_status_get(ctx, &status);
    sprintf((char *)tx_buffer, "Summary status %04x\r\n", status);
    //tx_com(tx_buffer, strlen((char const *)tx_buffer));

    asm9g300b_summary_sig_range_status_get(ctx, &status);
    sprintf((char *)tx_buffer, "Summary signal range status %04x\r\n", status);
    //tx_com(tx_buffer, strlen((char const *)tx_buffer));

#if 0
    if ((status & 0x400) == 0)
    {
      /* S_OK_C is zero */
      uint32_t com_status;

      asm9g300b_com_status_get(ctx, &com_status);
      sprintf((char *)tx_buffer, "COMMON status %08x\r\n", com_status);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
#endif
}

static uint32_t num_samples = 0;

static void asm9g300b_read_all_sensors_task(stmdev_ctx_t *ctx)
{
    int16_t raw[3];
    int32_t accel[3], gyro[3], temp;
    uint8_t *tx_p;
    uint16_t num;

    check_device_status(ctx);
    tx_p = tx_buffer;

    num = sprintf((char *)tx_p, "NUM %d\r\n", num_samples);
    tx_p += num;

    /* Get low-g accel data */
    asm9g300b_acc_data_get(ctx, raw);

    accel[0] = asm9g300b_from_acc_lsb_to_mms2(raw[0]);
    accel[1] = asm9g300b_from_acc_lsb_to_mms2(raw[1]);
    accel[2] = asm9g300b_from_acc_lsb_to_mms2(raw[2]);

    num = sprintf((char *)tx_p, "LOW-G %d (mm/s^2) %d (mm/s^2) %d (mm/s^2)\r\n",
                                accel[0], accel[1], accel[2]);
    tx_p += num;

    /* Get angular rate data */
    asm9g300b_ars_data_get(ctx, raw);

    gyro[0] = asm9g300b_from_ars_lsb_to_mdps(raw[0]);
    gyro[1] = asm9g300b_from_ars_lsb_to_mdps(raw[1]);
    gyro[2] = asm9g300b_from_ars_lsb_to_mdps(raw[2]);

    num = sprintf((char *)tx_p, "GYRO %d (mdps) %d (mdps) %d (mdps)\r\n",
                                gyro[0], gyro[1], gyro[2]);
    tx_p += num;

    /* Get Mid-g accel data */
    asm9g300b_mgp_data_get(ctx, raw);
    accel[0] = asm9g300b_from_mgp_lsb_to_mms2(raw[0]);
    accel[1] = asm9g300b_from_mgp_lsb_to_mms2(raw[1]);
    accel[2] = asm9g300b_from_mgp_lsb_to_mms2(raw[2]);

    num = sprintf((char *)tx_p, "MID-G %d (mm/s^2) %d (mm/s^2) %d (mm/s^2)\r\n",
                                accel[0], accel[1], accel[2]);
    tx_p += num;

    /* Get TEMPERATURE data */
    asm9g300b_temp_data_get(ctx, &raw[0]);
    temp = asm9g300b_from_temp_lsb_to_celsius(raw[0]);

    sprintf((char *)tx_p, "TEMP %d (milli Celsius)\r\n\r\n", temp);
}

static asm9g300b_priv_t asm9g300b_config =
  {
    .iir_bw_sel_ax = ASM9G300B_IIR_FILTER_60HZ,
    .iir_bw_sel_ay = ASM9G300B_IIR_FILTER_60HZ,
    .iir_bw_sel_az = ASM9G300B_IIR_FILTER_60HZ,
    .iir_bw_sel_rx = ASM9G300B_IIR_FILTER_60HZ,
    .iir_bw_sel_ry = ASM9G300B_IIR_FILTER_60HZ,
    .iir_bw_sel_rz = ASM9G300B_IIR_FILTER_60HZ,

    .t_debounce_ax = ASM9G300B_DEBOUNCE_TIME_0MS,
    .t_debounce_ay = ASM9G300B_DEBOUNCE_TIME_0MS,
    .t_debounce_az = ASM9G300B_DEBOUNCE_TIME_0MS,
    .t_debounce_rx = ASM9G300B_DEBOUNCE_TIME_0MS,
    .t_debounce_ry = ASM9G300B_DEBOUNCE_TIME_0MS,
    .t_debounce_rz = ASM9G300B_DEBOUNCE_TIME_0MS,

    .z_clamp = ASM9G300B_NVM_CLIPPING_DEFAULT,

    .sdo_drv = 0,
    .disable_auto_self_test = 0,
  };

void asm9g300b_read(void)
{
  stmdev_ctx_t dev_ctx;
  int ret;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  dev_ctx.priv_data = &asm9g300b_config;

  /* Init test platform */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  ret = asm9g300b_startup(&dev_ctx);
  if (ret < 0)
  {
    while(1); /* device not started */
  }

  if (asm9g300b_check_spi_communication(&dev_ctx) < 0)
  {
    while(1); /* stop here if SPI communication is not established */
  }

  uint32_t serial_num;
  asm9g300b_getSerialNum(&dev_ctx, &serial_num);

  sprintf((char *)tx_buffer, "serial_num %08x\r\n", serial_num);
  tx_com(tx_buffer, strlen((char const *)tx_buffer));

  uint32_t tick_start = HAL_GetTick();

  while(1)
  {
    uint32_t tick_curr;

    tick_curr = HAL_GetTick();

    /* simulate a thread scheduled every 1ms (1kHz) */
    if ((tick_curr - tick_start) >= 1)
    {
      tick_start = tick_curr;
      asm9g300b_read_all_sensors_task(&dev_ctx);

      /* prints out only every 1000 samples */
      if ((num_samples++ % 1000) == 999)
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write (unused)
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  (void)reg; /* 'reg' argument is anused (it's already embedded in bufp) */

#ifdef STEVAL_MKI109V3
  uint8_t txBuf[4];

  if (handle == &hspi2) {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
      /* on SPI bus dta is sent big endian */
      txBuf[0] = bufp[3];
      txBuf[1] = bufp[2];
      txBuf[2] = bufp[1];
      txBuf[3] = bufp[0];
#endif /* DRV_BYTE_ORDER */

      HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
      uint8_t status = (HAL_SPI_Transmit(handle, txBuf, 4, HAL_MAX_DELAY) == HAL_OK);
      while(HAL_SPI_GetState(handle) != HAL_SPI_STATE_READY);
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
  (void)reg; /* 'reg' argument is anused (it's already embedded in bufp) */

#ifdef STEVAL_MKI109V3
  uint8_t txBuf[4];
  uint8_t rxBuf[4];

  if (handle == &hspi2) {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
      /* on SPI bus dta is sent big endian */
      txBuf[0] = bufp[3];
      txBuf[1] = bufp[2];
      txBuf[2] = bufp[1];
      txBuf[3] = bufp[0];
#endif /* DRV_BYTE_ORDER */

      HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(handle, txBuf,rxBuf, 4, HAL_MAX_DELAY);
      while(HAL_SPI_GetState(handle) != HAL_SPI_STATE_READY);
      HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);

#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
      /* on SPI bus dta is sent big endian */
      bufp[0] = rxBuf[3];
      bufp[1] = rxBuf[2];
      bufp[2] = rxBuf[1];
      bufp[3] = rxBuf[0];
#endif /* DRV_BYTE_ORDER */
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
