/*
 ******************************************************************************
 * @file    single_tap.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to detect single tap from sensor.
 *
 ******************************************************************************
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
 * - STEVAL_MKI109V3 + STEVAL-MKI191V1
 * - NUCLEO_F401RE + STEVAL-MKI191V1
 * - DISCOVERY_SPC584B + STEVAL-MKI191V1
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
#include "iis2dlpc_reg.h"

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

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            20 //ms

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
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void iis2dlpc_tap(void)
{
  /*Initialize mems driver interface. */
  stmdev_ctx_t dev_ctx;
  iis2dlpc_reg_t int_route;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  /*Initialize platform specific hardware */
  platform_init();
  /*Check device ID */
  iis2dlpc_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != IIS2DLPC_ID)
    while (1) {
      /* manage here device not found */
    }

  /*Restore default configuration */
  iis2dlpc_reset_set(&dev_ctx);

  do {
    iis2dlpc_reset_get(&dev_ctx, &rst);
  } while (rst);

  /*Set full scale */
  iis2dlpc_full_scale_set(&dev_ctx, IIS2DLPC_2g);
  /*Configure power mode */
  iis2dlpc_power_mode_set(&dev_ctx,
                          IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_12bit);
  /*Set Output Data Rate */
  iis2dlpc_data_rate_set(&dev_ctx, IIS2DLPC_XL_ODR_400Hz);
  /*Enable Tap detection on X, Y, Z */
  iis2dlpc_tap_detection_on_z_set(&dev_ctx, PROPERTY_ENABLE);
  iis2dlpc_tap_detection_on_y_set(&dev_ctx, PROPERTY_ENABLE);
  iis2dlpc_tap_detection_on_x_set(&dev_ctx, PROPERTY_ENABLE);
  /*Set Tap threshold on all axis */
  iis2dlpc_tap_threshold_x_set(&dev_ctx, 9);
  iis2dlpc_tap_threshold_y_set(&dev_ctx, 9);
  iis2dlpc_tap_threshold_z_set(&dev_ctx, 9);
  /*Configure Single Tap parameter */
  iis2dlpc_tap_quiet_set(&dev_ctx, 1);
  iis2dlpc_tap_shock_set(&dev_ctx, 2);
  /*Enable Single Tap detection only */
  iis2dlpc_tap_mode_set(&dev_ctx, IIS2DLPC_ONLY_SINGLE);
  /*Enable single tap detection interrupt */
  iis2dlpc_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
  int_route.ctrl4_int1_pad_ctrl.int1_single_tap = PROPERTY_ENABLE;
  iis2dlpc_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);

  /*Wait Events */
  while (1) {
    iis2dlpc_all_sources_t all_source;
    /* Check Single Tap events */
    iis2dlpc_all_sources_get(&dev_ctx, &all_source);

    if (all_source.tap_src.single_tap) {
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "Tap Detected: Sign %s",
              all_source.tap_src.tap_sign ? "positive" : "negative");

      if (all_source.tap_src.x_tap) {
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "%s on X axis\r\n", tx_buffer);
      }

      if (all_source.tap_src.y_tap) {
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "%s on Y axis\r\n", tx_buffer);
      }

      if (all_source.tap_src.z_tap) {
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "%s on Z axis\r\n", tx_buffer);
      }

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
#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Write(handle, IIS2DLPC_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  IIS2DLPC_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, IIS2DLPC_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, IIS2DLPC_I2C_ADD_L & 0xFE, reg, bufp, len);
#endif
  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
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
  platform_delay(1000);
#endif
}
