/*
 ******************************************************************************
 * @file    tilt.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to detect tilt from sensor.
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

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A2
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A2 - Host side: UART(COM) to USB bridge
 *                                       - I2C(Default) / SPI(N/A)
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
//#define STEVAL_MKI109V3
#define NUCLEO_F411RE_X_NUCLEO_IKS01A2

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2

/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
/* NUCLEO_F411RE_X_NUCLEO_IKS01A2: Define communication interface */
#define SENSOR_BUS hi2c1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <lsm6ds3_reg.h>
#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
#include "usart.h"
#endif

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void example_main_tilt_lsm6ds3(void)
{
  /*
   * Initialize mems driver interface.
   */
  stmdev_ctx_t dev_ctx;
  lsm6ds3_int1_route_t int_1_reg;

  /*
   * Uncomment if interrupt generation on Tilt INT2 pin
   */
  //lsm6ds3_int2_route_t int_2_reg;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /*
   * Initialize platform specific hardware
   */
  platform_init();

  /*
   * Check device ID
   */
  lsm6ds3_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LSM6DS3_ID)
    while(1)
    {
      /* manage here device not found */
    }

  /*
   * Restore default configuration
   */
  lsm6ds3_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6ds3_reset_get(&dev_ctx, &rst);
  } while (rst);

  /*
   * Set XL Output Data Rate:The tilt function works at 26 Hz,
   * so the accelerometer ODR must be set at 26 Hz or higher values
   */
  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_26Hz);

  /*
   * Set 2g full XL scale
   */
  lsm6ds3_xl_full_scale_set(&dev_ctx, LSM6DS3_2g);

  /*
   * Enable Tilt function
   */
  lsm6ds3_tilt_sens_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Enable interrupt generation on Tilt INT1 pin
   */
  lsm6ds3_int_notification_set(&dev_ctx, LSM6DS3_INT_LATCHED);
  lsm6ds3_tap_detection_on_z_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6ds3_tap_detection_on_y_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6ds3_tap_detection_on_x_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6ds3_pin_int1_route_get(&dev_ctx, &int_1_reg);
  int_1_reg.int1_tilt = PROPERTY_ENABLE;
  lsm6ds3_pin_int1_route_set(&dev_ctx, &int_1_reg);

  /*
   * Uncomment if interrupt generation on Tilt INT2 pin
   */
  //lsm6ds3_pin_int2_route_get(&dev_ctx, &int_2_reg);
  //int_2_reg.int2_tilt = PROPERTY_ENABLE;
  //lsm6ds3_pin_int2_route_set(&dev_ctx, &int_2_reg);

  /*
   * Wait Events
   */
  while(1)
  {
    lsm6ds3_all_src_t all_source;

    /*
     * Check Tilt events
     */
    lsm6ds3_all_sources_get(&dev_ctx, &all_source);
    if (all_source.func_src.tilt_ia)
    {
      sprintf((char*)tx_buffer, "TILT Detected\r\n");
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
    }

    /*
     * Slow polling on device
     */
    HAL_Delay(100);
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, LSM6DS3_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
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
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Read(handle, LSM6DS3_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
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
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to trasmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  #ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
  #endif
  #ifdef STEVAL_MKI109V3
  CDC_Transmit_FS(tx_buffer, len);
  #endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#ifdef STEVAL_MKI109V3
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_Delay(1000);
#endif
}
