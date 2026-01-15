/*
 ******************************************************************************
 * @file    read_fifo.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to extract data from the sensor in
 *          polling mode.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
 * - STEVAL_MKI109V3 + STEVAL-
 * - NUCLEO_F401RE + STEVAL-
 * - DISCOVERY_SPC584B + STEVAL-
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
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

//#define STEVAL_MKI109V3  /* little endian */
//#define NUCLEO_F401RE    /* little endian */
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

#elif defined(NUCLEO_F401RE)
/* NUCLEO_F401RE: Define communication interface */
#define SENSOR_BUS hi2c1

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS I2CD1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "iis2dulpx_reg.h"

#if defined(NUCLEO_F401RE)
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

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME         10 //ms


/* Private variables ---------------------------------------------------------*/
static iis2dulpx_md_t md;

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[1000];
static iis2dulpx_priv_t priv_data;

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

#define NUM_FIFO_ENTRY  33 /* 32 samples + timestamp */

static stmdev_ctx_t dev_ctx;
static iis2dulpx_fifo_mode_t fifo_mode;
static iis2dulpx_fifo_batch_t batch;
static uint8_t fifo_wtm_event = 0;

void iis2dulpx_read_fifo_handler(void)
{
  uint8_t wmflag = 0;

  iis2dulpx_fifo_wtm_flag_get(&dev_ctx, &wmflag);

  if (wmflag > 0) {
    fifo_wtm_event = 1;
  }
}

/* Main Example --------------------------------------------------------------*/
void iis2dulpx_read_fifo(void)
{
  iis2dulpx_pin_int1_route_t int1_route;
  uint8_t id;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  dev_ctx.priv_data = &priv_data;

  /* Initialize platform specific hardware */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  iis2dulpx_exit_deep_power_down(&dev_ctx);

  /* Check device ID */
  iis2dulpx_device_id_get(&dev_ctx, &id);
  if (id != IIS2DULPX_ID)
    while(1);

  /* Restore default configuration */
  iis2dulpx_sw_reset(&dev_ctx);

  /* init bdu and add_inc */
  iis2dulpx_init_set(&dev_ctx);

  /* Set FIFO watermark to 32 sample(s) */
  fifo_mode.store = IIS2DULPX_FIFO_1X;
  fifo_mode.xl_only = 0;
  fifo_mode.operation = IIS2DULPX_STREAM_MODE;
  iis2dulpx_fifo_mode_set(&dev_ctx, fifo_mode);
  iis2dulpx_fifo_watermark_set(&dev_ctx, NUM_FIFO_ENTRY);
  batch.dec_ts = IIS2DULPX_DEC_TS_32;
  batch.bdr_xl = IIS2DULPX_BDR_XL_ODR;
  iis2dulpx_fifo_batch_set(&dev_ctx, batch);
  iis2dulpx_fifo_stop_on_wtm_set(&dev_ctx, IIS2DULPX_FIFO_EV_WTM);

  iis2dulpx_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

  /* Configure interrupt pins */
  int1_route.fifo_th = PROPERTY_ENABLE;
  iis2dulpx_pin_int1_route_set(&dev_ctx, &int1_route);

  /* Set Output Data Rate */
  md.fs =  IIS2DULPX_4g;
  md.bw = IIS2DULPX_ODR_div_4;
  md.odr = IIS2DULPX_25Hz_LP;
  iis2dulpx_mode_set(&dev_ctx, &md);

  /* wait forever (FIFO samples read with irq) */
  while (1) {
    uint16_t num = 0;
    uint32_t ts;
    iis2dulpx_fifo_data_t fdata;

    if (fifo_wtm_event) {
      fifo_wtm_event = 0;

      /* Read number of samples in FIFO */
      iis2dulpx_fifo_data_level_get(&dev_ctx, &num);

      snprintf((char *)tx_buffer, sizeof(tx_buffer), "-- %d in FIFO\r\n", num);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
      while (num--) {
        iis2dulpx_fifo_data_get(&dev_ctx, &md, &fifo_mode, &fdata);

        switch (fdata.tag) {
        case IIS2DULPX_XL_ONLY_2X_TAG:
          snprintf((char*)tx_buffer, sizeof(tx_buffer), "%02d_0: Acceleration [0][mg]:\t%4.2f\t%4.2f\t%4.2f\r\n",
                  NUM_FIFO_ENTRY - num, fdata.xl[0].mg[0], fdata.xl[0].mg[1], fdata.xl[0].mg[2]);
          tx_com(tx_buffer, strlen((char const*)tx_buffer));
          snprintf((char*)tx_buffer, sizeof(tx_buffer), "%02d_1: Acceleration [1][mg]:\t%4.2f\t%4.2f\t%4.2f\r\n",
                  NUM_FIFO_ENTRY - num, fdata.xl[1].mg[0], fdata.xl[1].mg[1], fdata.xl[1].mg[2]);
          tx_com(tx_buffer, strlen((char const*)tx_buffer));
          break;
        case IIS2DULPX_XL_TEMP_TAG:
          if (fifo_mode.xl_only == 0) {
            snprintf((char *)tx_buffer, sizeof(tx_buffer),
                    "%02d: Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\tTemp[degC]:%3.2f\r\n", NUM_FIFO_ENTRY - num,
                    fdata.xl[0].mg[0], fdata.xl[0].mg[1], fdata.xl[0].mg[2], fdata.heat.deg_c);
          } else {
            snprintf((char *)tx_buffer, sizeof(tx_buffer),
                    "%02d: Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n", NUM_FIFO_ENTRY - num,
                    fdata.xl[0].mg[0], fdata.xl[0].mg[1], fdata.xl[0].mg[2]);
          }
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          break;
        case IIS2DULPX_TIMESTAMP_TAG:
          ts = fdata.cfg_chg.timestamp / 100;
          snprintf((char*)tx_buffer, sizeof(tx_buffer), "Timestamp:\t%lu ms\t\r\n", ts);
          tx_com(tx_buffer, strlen((char const*)tx_buffer));
          break;
       default:
          snprintf((char*)tx_buffer, sizeof(tx_buffer), "unknown TAG (%02x)\t\r\n", fdata.tag);
          tx_com(tx_buffer, strlen((char const*)tx_buffer));
          break;
        }
      }
      snprintf((char *)tx_buffer, sizeof(tx_buffer),"\r\n");
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
static int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *bufp, uint16_t len)
{
#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Write(handle, IIS2DULPX_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  IIS2DULPX_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, IIS2DULPX_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, IIS2DULPX_I2C_ADD_L & 0xFE, reg, bufp, len);
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
#if defined(NUCLEO_F401RE)
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
#elif defined(STEVAL_MKI109V3)
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
#if defined(NUCLEO_F401RE) | defined(STEVAL_MKI109V3)
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
