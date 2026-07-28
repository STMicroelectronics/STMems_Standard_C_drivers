/*
 ******************************************************************************
 * @file    fxls8974cf_read_data_fifo.c
 * @author  Sensors Software Solution Team
 * @brief   This file show an example for FXLS8974CF to configure in FIFO mode
 *          and read buffered samples when configured FIFO_DEPTH is reached.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 * opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

 /*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - NUCLEO_F401RE + FRDM-STBI-A8974
 * - STEVAL_MKI109D + FXLS8974 (DIL-24)
 * - DISCOVERY_SPC584B + FRDM-STBI-A8974
 *
 * Used interfaces:
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default)
 *
 * STEVAL_MKI109D     - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
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

//#define STEVAL_MKI109D   /* little endian */
//#define NUCLEO_F401RE    /* little endian */
//#define SPC584B_DIS      /* big endian */


/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#if defined(STEVAL_MKI109D)
/* MKI109D: Define communication interface */
#define SENSOR_BUS hspi1

/* MKI109D: Vdd and Vddio power supply values */
#define FXLS8974CF_VDD 3.3f
#define FXLS8974CF_VDDIO 3.3f

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

#if defined(NUCLEO_F401RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#elif defined(STEVAL_MKI109D)
#include "board.h"
#include "usbd_cdc_if.h"

#elif defined(SPC584B_DIS)
#include "components.h"

#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fxls8974cf_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

#define BOOT_TIME         10

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN 1 */
fxls8974cf_fifo_data_t raw;
uint8_t fifo_status;
float x_g, y_g, z_g;
  
/* USER CODE BEGIN PV */
static uint8_t tx_buffer[1000];
stmdev_ctx_t dev_ctx;
fxls8974cf_fs_t fs;
fxls8974cf_odr_t odr;
/* USER CODE END PV */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
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
static void platform_init(void *handle);
/* USER CODE END PFP */

/* Main Example --------------------------------------------------------------*/
/* Initialize FXLS8974CF Device */
int32_t fxls8974cf_init_example(void)
{
  uint8_t who;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Init test platform */
  platform_init(dev_ctx.handle);

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Reset device */
  fxls8974cf_reset(&dev_ctx);

  fxls8974cf_device_id_get(&dev_ctx, &who);
  if (FXLS8974_WHO_AM_I_VAL == who)
  {
    snprintf((char *)tx_buffer, sizeof(tx_buffer),
            "Successfully Initialized Timandra with WHO_AM_I = 0x%X\r\n",who);
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }
  else
  {
    snprintf((char *)tx_buffer, sizeof(tx_buffer),
            "Bad WHO_AM_I = 0x%X\r\n",who);
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
    return -1;
  }

  /* Set full scale range */
  fxls8974cf_full_scale_set(&dev_ctx, FXLS8974CF_2g);
  fxls8974cf_full_scale_get(&dev_ctx, &fs);

  /* Set ODR */
  fxls8974cf_wake_odr_set(&dev_ctx, FXLS8974CF_ODR_6_25HZ);
  fxls8974cf_wake_odr_get(&dev_ctx, &odr);
  fxls8974cf_fifo_mode_set(&dev_ctx, FIFO_DEPTH);

  return 0;

}

/**
  * @brief  The application entry point.
  * @retval int
  */
int32_t fxls8974cf_read_data_fifo(void)
{

  while (1)
  {
    /* USER CODE BEGIN WHILE */

    /* Wait for data ready from the FXLS8974. */
    fxls8974cf_check_fifo_status(&dev_ctx, &fifo_status);
    if ( 0 == (fifo_status & FXLS8974CF_BUF_STATUS_BUF_WMRK))
    {
      continue;
    }

    /* Read FXLS8974CF 3-axis raw data */
    fxls8974cf_acceleration_raw_fifo_get(&dev_ctx, &raw, FIFO_DEPTH);

    for (uint8_t i = 0; i < FIFO_DEPTH; i++)
    {
      /* Convert raw data to g units */
      x_g = fxls8974cf_from_raw_to_2g(raw.x[i]);
      y_g = fxls8974cf_from_raw_to_2g(raw.y[i]);
      z_g = fxls8974cf_from_raw_to_2g(raw.z[i]);

      snprintf((char *)tx_buffer, sizeof(tx_buffer),
                "\r\nAccel:\tX=%0.3f  Y=%0.3f  Z=%0.3f\r\n", x_g, y_g, z_g);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
    snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "FIFO Depth = %d Completed\r\n\r\n", FIFO_DEPTH);
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
    /* USER CODE END WHILE */
  }

  return 0;
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
int32_t platform_write(void *handle, uint8_t reg,
                       const uint8_t *bufp, uint16_t len)
{
#if defined(NUCLEO_F401RE)
  if (HAL_I2C_Mem_Write(handle, FXLS8974CF_I2C_ADD_SA0_0, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000) != HAL_OK)
  {
    return -1;
  }
#elif defined(STEVAL_MKI109D)
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY); //8CLKs dummy
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  FXLS8974CF_I2C_ADD_SA0_0 & 0xFE, reg, (uint8_t*) bufp, len);
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
static int32_t platform_read(void *handle, uint8_t reg,
                      uint8_t *bufp, uint16_t len)
{
#if defined(NUCLEO_F401RE)
  if (HAL_I2C_Mem_Read(handle, FXLS8974CF_I2C_ADD_SA0_0, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000) != HAL_OK)
  {
    return -1;
  }
#elif defined(STEVAL_MKI109D)
  reg |= 0x80;
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY); //8CLKs dummy
  HAL_SPI_Receive(handle, bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, FXLS8974CF_I2C_ADD_SA0_0 & 0xFE, reg, bufp, len);
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
#elif defined(STEVAL_MKI109D)
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
#if defined(NUCLEO_F401RE) || defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(STEVAL_MKI109D)
  delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void *handle)
{
#if defined(STEVAL_MKI109D)
  struct spi_conf spi_conf;

  /* init SPI bus communication */
  spi_conf.wire = WIRE_4_EXT;
  spi_init(&spi_conf);

  /* set VDD/VDDIO on DIL24 */
  set_vdd(FXLS8974CF_VDD);
  set_vddio(FXLS8974CF_VDDIO);
  delay(100);

#endif
}


