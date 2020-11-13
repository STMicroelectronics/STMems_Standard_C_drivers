/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "lps25hb_reg.h"
#include <string.h>
#include <stdio.h>

//#define MKI109V2
#define NUCLEO_STM32F411RE + IKS01A2

#ifdef MKI109V2
#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"
#include "spi.h"
#include "i2c.h"
#endif

#ifdef NUCLEO_STM32F411RE
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#endif

typedef union {
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

typedef union {
  int32_t i32bit;
  uint8_t u8bit[4];
} axis1bit32_t;

/* Private macro -------------------------------------------------------------*/
#ifdef MKI109V2
#define CS_SPI2_GPIO_Port   CS_DEV_GPIO_Port
#define CS_SPI2_Pin         CS_DEV_Pin
#define CS_SPI1_GPIO_Port   CS_RF_GPIO_Port
#define CS_SPI1_Pin         CS_RF_Pin
#endif

#ifdef NUCLEO_STM32F411RE
/* N/A on NUCLEO_STM32F411RE + IKS01A1 */
/* N/A on NUCLEO_STM32F411RE + IKS01A2 */
#define CS_SPI2_GPIO_Port   0
#define CS_SPI2_Pin         0
#define CS_SPI1_GPIO_Port   0
#define CS_SPI1_Pin         0
#endif

#define TX_BUF_DIM          1000

/* Private variables ---------------------------------------------------------*/
static axis1bit32_t data_raw_pressure;
static axis1bit16_t data_raw_temperature;
static float pressure_hPa;
static float temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[TX_BUF_DIM];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   Replace the functions "platform_write" and "platform_read" with your
 *   platform specific read and write function.
 *   This example use an STM32 evaluation board and CubeMX tool.
 *   In this case the "*handle" variable is useful in order to select the
 *   correct interface but the usage uf "*handle" is not mandatory.
 */

static int32_t platform_write(void *handle, uint8_t Reg,
                              uint8_t *Bufp,
                              uint16_t len)
{
  if (handle == &hi2c1) {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80;
    HAL_I2C_Mem_Write(handle, LPS25HB_I2C_ADD_L, Reg,
                      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }

#ifdef MKI109V2

  else if (handle == &hspi2) {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x40;
    HAL_GPIO_WritePin(CS_SPI2_GPIO_Port, CS_SPI2_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Transmit(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_SPI2_GPIO_Port, CS_SPI2_Pin, GPIO_PIN_SET);
  }

  else if (handle == &hspi1) {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x40;
    HAL_GPIO_WritePin(CS_SPI1_GPIO_Port, CS_SPI1_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Transmit(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_SPI1_GPIO_Port, CS_SPI1_Pin, GPIO_PIN_SET);
  }

#endif
  return 0;
}

static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  if (handle == &hi2c1) {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80;
    HAL_I2C_Mem_Read(handle, LPS25HB_I2C_ADD_L, Reg,
                     I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }

#ifdef MKI109V2

  else if (handle == &hspi2) {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0xC0;
    HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Receive(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_SET);
  }

  else {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0xC0;
    HAL_GPIO_WritePin(CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Receive(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_SET);
  }

#endif
  return 0;
}

/*
 *  Function to print messages
 */
void tx_com( uint8_t *tx_buffer, uint16_t len )
{
#ifdef NUCLEO_STM32F411RE
  HAL_UART_Transmit( &huart2, tx_buffer, len, 1000 );
#endif
#ifdef MKI109V2
  CDC_Transmit_FS( tx_buffer, len );
#endif
}

/* Main Example --------------------------------------------------------------*/

void example_main(void)
{
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;
  /* Check device ID */
  whoamI = 0;
  lps25hb_device_id_get(&dev_ctx, &whoamI);

  if ( whoamI != LPS25HB_ID )
    while (1); /*manage here device not found */

  /* Restore default configuration */
  lps25hb_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lps25hb_reset_get(&dev_ctx, &rst);
  } while (rst);

  /*  Enable Block Data Update */
  lps25hb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lps25hb_data_rate_set(&dev_ctx, LPS25HB_ODR_1Hz);

  /* Read samples in polling mode (no int) */
  while (1) {
    /* Read output only if new value is available */
    lps25hb_reg_t reg;
    lps25hb_status_get(&dev_ctx, &reg.status_reg);

    if (reg.status_reg.p_da) {
      memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
      lps25hb_pressure_raw_get(&dev_ctx, data_raw_pressure.u8bit);
      pressure_hPa = lps25hb_from_lsb_to_hpa( data_raw_pressure.i32bit);
      sprintf((char *)tx_buffer, "pressure [hPa]:%6.2f\r\n", pressure_hPa);
      tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
    }

    if (reg.status_reg.t_da) {
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      lps25hb_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
      temperature_degC = lps25hb_from_lsb_to_degc(
                           data_raw_temperature.i16bit);
      sprintf((char *)tx_buffer, "temperature [degC]:%6.2f\r\n",
              temperature_degC);
      tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
    }
  }
}

