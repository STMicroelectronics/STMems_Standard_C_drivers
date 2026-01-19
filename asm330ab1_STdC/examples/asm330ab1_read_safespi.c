/*
 ******************************************************************************
 * @file    read_safespi.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
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

#if defined(STEVAL_MKI109D)
/* MKI109D: Define communication interface */
#define SENSOR_BUS hspi1

/* MKI109D: Vdd and Vddio power supply values */
#define ASM330AB1_VDD 3.3f
#define ASM330AB1_VDDIO 3.3f

#elif defined(STEVAL_MKI109V3)
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
#include "asm330ab1_reg.h"

#if defined(NUCLEO_F411RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#elif defined(STEVAL_MKI109D)
#include "board.h"
#include "usbd_cdc_if.h"

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
static asm330ab1_priv_t asm330ab1_config =
  {
    .use_safespi_bus = 1,
  };

void asm330ab1_read_safespi(void)
{
  stmdev_ctx_t dev_ctx;
  int ret;
  uint8_t whoamI;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  dev_ctx.priv_data = &asm330ab1_config;

  /* Init test platform */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  asm330ab1_check_spi_communication(&dev_ctx);
  while(1);

  static uint8_t test = 0xf1;
  asm330ab1_write_reg(&dev_ctx, 0x7, &test, 1);
  test = 0;
  asm330ab1_read_reg(&dev_ctx, 0x7, &test, 1);

  /* Check device ID */
  asm330ab1_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != ASM330AB1_ID)
    while (1);
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

#if defined(STEVAL_MKI109D)
  uint8_t txBuf[4];

  if (handle == &hspi1) {

#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    /* on SPI bus dta is sent big endian */
    txBuf[0] = bufp[3];
    txBuf[1] = bufp[2];
    txBuf[2] = bufp[1];
    txBuf[3] = bufp[0];
#else
    memcpy(txBuf, bufp, 4);
#endif /* DRV_BYTE_ORDER */

    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    uint8_t status = (HAL_SPI_Transmit(handle, txBuf, 4, HAL_MAX_DELAY) == HAL_OK);
    while(HAL_SPI_GetState(handle) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
  }

#elif defined(STEVAL_MKI109V3)
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

#if defined(STEVAL_MKI109D)
  uint8_t txBuf[4];
  uint8_t rxBuf[4];

  if (handle == &hspi1) {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    /* on SPI bus dta is sent big endian */
    txBuf[0] = bufp[3];
    txBuf[1] = bufp[2];
    txBuf[2] = bufp[1];
    txBuf[3] = bufp[0];
#else
    memcpy(txBuf, bufp, 4);
#endif /* DRV_BYTE_ORDER */

    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(handle, txBuf,rxBuf, 4, HAL_MAX_DELAY);
    while(HAL_SPI_GetState(handle) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    /* on SPI bus dta is sent big endian */
    bufp[0] = rxBuf[3];
    bufp[1] = rxBuf[2];
    bufp[2] = rxBuf[1];
    bufp[3] = rxBuf[0];
#else
    memcpy(bufp, rxBuf, 4);
#endif /* DRV_BYTE_ORDER */
 }

#elif defined(STEVAL_MKI109V3)
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
#if defined(STEVAL_MKI109D)
  CDC_Transmit_FS(tx_buffer, len);
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
#if defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(NUCLEO_F411RE) | defined(STEVAL_MKI109V3)
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
#if defined(STEVAL_MKI109D)
  struct spi_conf spi_conf;

  /* init SPI bus communication */
  spi_conf.wire = WIRE_4;
  spi_init(&spi_conf);

  /* set VDD/VDDIO on DIL24 */
  set_vdd(ASM330AB1_VDD);
  set_vddio(ASM330AB1_VDDIO);
  delay(100);

#elif defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}
