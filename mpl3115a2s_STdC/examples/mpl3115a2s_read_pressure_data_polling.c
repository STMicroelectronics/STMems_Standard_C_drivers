/*
 ******************************************************************************
 * @file    mpl3115a2s_read_pressure_data_polling.c
 * @author  Sensors Software Solution Team
 * @brief   This file show an example for MPL3115A2S to poll data ready event
 *          and read pressure, temperature samples on data ready event.
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
 * - NUCLEO_F401RE + FRDMSTBC-P3115
 * - DISCOVERY_SPC584B + FRDMSTBC-P3115
 *
 * Used interfaces:
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default)
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

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "stdbool.h"

#if defined(NUCLEO_F401RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#elif defined(SPC584B_DIS)
#include "components.h"

#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpl3115a2s_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BOOT_TIME         100
#define SENSOR_BUS hi2c1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t tx_buffer[1000];
stmdev_ctx_t dev_ctx;
static mpl3115a2s_pressuredata_t raw;
static uint8_t dataReady;
static int16_t tempInDegrees;
static float pressureInPascals;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int32_t mpl3115a2s_init_example(void)
{
  uint8_t who;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Reset the sensor */
  mpl3115a2s_device_reset(&dev_ctx);

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  mpl3115a2s_dummy_read(&dev_ctx);
  mpl3115a2s_device_id_get(&dev_ctx, &who);
  if ((MPL3115A2S_WHOAMI_VALUE == who) || (FXPQ3115A2S_WHOAMI_VALUE == who))
  {
    snprintf((char *)tx_buffer, sizeof(tx_buffer),
            "Successfully Initialized MPL3115 with WHO_AM_I = 0x%X\r\n",who);
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }
  else
  {
    snprintf((char *)tx_buffer, sizeof(tx_buffer),
            "Bad WHO_AM_I = 0x%X\r\n",who);
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
    return -1;
  }

  /* Enable DataReady */
  mpl3115a2s_enable_data_ready(&dev_ctx);

  /* Set Pressure (BAROMETER) Mode */
  mpl3115a2s_config_set(&dev_ctx, BAR_MODE);

  /* Set OSR to 128 */
  mpl3115a2s_osr_set(&dev_ctx, mpl3115a2s_OSR_128);

  /* Set auto-acquisition to 1 secs */
  mpl3115a2s_auto_aquisition_set(&dev_ctx, MPL3115A2S_SAMPLING_EXPONENT);

  return 0;

}

/**
  * @brief  The application entry point.
  * @retval int
  */
int32_t mpl3115a2s_read_pressure_data_polling(void)
{

  /* Infinite loop */
  while (1)
  {
    /* USER CODE END WHILE */
    /* Wait for data ready from the MPL3115A2S. */
    mpl3115a2s_check_data_ready(&dev_ctx, &dataReady);
    if (0 == (dataReady & MPL3115A2S_DR_STATUS_PTDR_DRDY))
    {
        continue;
    }

    /* Read raw pressure samples */
    mpl3115a2s_pressure_raw_get(&dev_ctx, &raw);

    /* Convert raw samples to units */
    pressureInPascals = mpl3115a2s_pressure_raw_to_pa(raw.pressure);
    tempInDegrees = mpl3115a2s_temp_raw_to_degc(raw.temperature);
    
    snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "Pressure:\t%0.2f Pa\r\n", pressureInPascals);
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
    
    snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "Temperature:\t%d DegC\r\n\r\n", tempInDegrees);
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
  if (HAL_I2C_Mem_Write(handle, MPL3115A2S_I2C_ADDRESS, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000) != HAL_OK)
  {
    return -1;
  }
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  MPL3115A2S_I2C_ADDRESS & 0xFE, reg, (uint8_t*) bufp, len);
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
  if (HAL_I2C_Mem_Read(handle, MPL3115A2S_I2C_ADDRESS, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000) != HAL_OK)
  {
    return -1;
  }
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, MPL3115A2S_I2C_ADDRESS & 0xFE, reg, bufp, len);
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
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

/* USER CODE END 0 */
