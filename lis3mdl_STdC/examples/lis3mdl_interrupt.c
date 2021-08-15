/*
 ******************************************************************************
 * @file    interrupt_configuration_example.c
 * @author  Sensors Software Solution Team
 * @brief   This file how to set interrupt and configure interrupt threshold
 *          on magnetic field.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI137V1
 * - NUCLEO_F411RE + X-NUCLEO-IKS01A1
 * - DISCOVERY_SPC584B + STEVAL-MKI137V1
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
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
//#define NUCLEO_F411RE    /* little endian */
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

#elif defined(NUCLEO_F411RE)
/* NUCLEO_F411RE: Define communication interface */
#define SENSOR_BUS hi2c1

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS I2CD1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lis3mdl_reg.h"

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

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_magnetic[3];
static float magnetic_mG[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
/*
 * Set the magnetic field threshold (in mg) for test
 */
static uint16_t threshold = (uint16_t)(1711 * 0.192f);

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
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lis3mdl_interrupt(void)
{
  lis3mdl_int_cfg_t int_ctrl;
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;
  /* Initialize platform specific hardware */
  platform_init();
  /* Check device ID */
  lis3mdl_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LIS3MDL_ID) {
    while (1) {
      /* manage here device not found */
    }
  }

  /* Restore default configuration */
  lis3mdl_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lis3mdl_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lis3mdl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lis3mdl_data_rate_set(&dev_ctx, LIS3MDL_HP_1Hz25);
  /* Set full scale */
  lis3mdl_full_scale_set(&dev_ctx, LIS3MDL_16_GAUSS);
  /* Enable temperature sensor */
  lis3mdl_temperature_meas_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set device in continuous mode */
  lis3mdl_operating_mode_set(&dev_ctx, LIS3MDL_CONTINUOUS_MODE);
  /* Enable interrupt generation on interrupt */
  lis3mdl_int_generation_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set interrupt threshold
   *
   * The sample code exploits a threshold and notify it by
   * hardware through the INT/DRDY pin
   */
  lis3mdl_int_threshold_set(&dev_ctx, threshold);
  int_ctrl.iea = PROPERTY_ENABLE;
  int_ctrl.ien = PROPERTY_ENABLE;
  int_ctrl.zien = PROPERTY_ENABLE;
  int_ctrl.yien = PROPERTY_ENABLE;
  int_ctrl.xien = PROPERTY_ENABLE;
  lis3mdl_int_config_set(&dev_ctx, &int_ctrl);

  /* Read samples in polling mode */
  while (1) {
    uint8_t reg;
    char *exceeds_info;
    lis3mdl_int_src_t source;
    /* Read output only if new value is available
     * It's also possible to use interrupt pin for trigger
     */
    lis3mdl_mag_data_ready_get(&dev_ctx, &reg);

    if (reg) {
      /* Read magnetic field data */
      memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
      lis3mdl_magnetic_raw_get(&dev_ctx, data_raw_magnetic);
      magnetic_mG[0] = 1000 * lis3mdl_from_fs16_to_gauss(
                         data_raw_magnetic[0]);
      magnetic_mG[1] = 1000 * lis3mdl_from_fs16_to_gauss(
                         data_raw_magnetic[1]);
      magnetic_mG[2] = 1000 * lis3mdl_from_fs16_to_gauss(
                         data_raw_magnetic[2]);
      exceeds_info = NULL;
      /* Read LIS3MDL INT SRC register */
      lis3mdl_int_source_get(&dev_ctx, &source);

      if (source.nth_x) {
        exceeds_info = "X-axis neg";
      }

      else if (source.nth_y) {
        exceeds_info = "Y-axis neg";
      }

      else if (source.nth_z) {
        exceeds_info = "Z-axis neg";
      }

      else if (source.pth_x) {
        exceeds_info = "X-axis pos";
      }

      else if (source.pth_y) {
        exceeds_info = "Y-axis pos";
      }

      else if (source.pth_z) {
        exceeds_info = "Z-axis pos";
      }

      sprintf((char *)tx_buffer,
              "Magnetic [mG]:%4.2f\t%4.2f\t%4.2f (%s)\r\n",
              magnetic_mG[0], magnetic_mG[1], magnetic_mG[2],
              exceeds_info != NULL ? exceeds_info : "");
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
#if defined(NUCLEO_F411RE)
  /* Write multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Write(handle, LIS3MDL_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Write multiple command */
  reg |= 0x40;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  /* Write multiple command */
  reg |= 0x80;
  i2c_lld_write(handle,  LIS3MDL_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
#if defined(NUCLEO_F411RE)
  /* Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read(handle, LIS3MDL_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Read multiple command */
  reg |= 0xC0;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  /* Read multiple command */
  reg |= 0x80;
  i2c_lld_read(handle, LIS3MDL_I2C_ADD_L & 0xFE, reg, bufp, len);
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
#if defined(NUCLEO_F411RE)
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
  platform_delay(1000);
#endif
}

