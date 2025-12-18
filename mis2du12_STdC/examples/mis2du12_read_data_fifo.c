/*
 ******************************************************************************
 * @file    read_data_fifo.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to extract data from the fifo.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "mis2du12_reg.h"

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
static uint8_t tx_buffer[1000];
static mis2du12_fifo_data_t data;

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

static   stmdev_ctx_t dev_ctx;
static   mis2du12_md_t md;
static   mis2du12_fifo_md_t fifo_mode;
static uint8_t fifo_wtm_event = 0;

void mis2du12_read_fifo_handler(void)
{
  mis2du12_fifo_status_t status;

  mis2du12_fifo_status_get(&dev_ctx, &status);
  if (status.fifo_fth) {
    fifo_wtm_event = 1;
  }
}

/* Main Example --------------------------------------------------------------*/
void mis2du12_read_data_fifo(void)
{
  mis2du12_pin_int_route_t int_route;
  mis2du12_status_t status;
  mis2du12_id_t id;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Initialize platform specific hardware */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  mis2du12_id_get(&dev_ctx, &id);
  if (id.whoami != MIS2DU12_ID)
    while(1);

  /* Restore default configuration */
  mis2du12_init_set(&dev_ctx, MIS2DU12_RESET);
  do {
    mis2du12_status_get(&dev_ctx, &status);
  } while (status.sw_reset);

  /* Set bdu and if_inc recommended for driver usage */
  mis2du12_init_set(&dev_ctx, MIS2DU12_DRV_RDY);


  /* Select bus interface */
  mis2du12_bus_mode_set(&dev_ctx, MIS2DU12_I3C_DISABLE);

  /* Set FIFO watermark to 32 sample(s) */
  fifo_mode.store = MIS2DU12_8_BIT;
  fifo_mode.watermark = 32;
  fifo_mode.operation = MIS2DU12_STREAM;
  mis2du12_fifo_mode_set(&dev_ctx, &fifo_mode);

  /* Set Output Data Rate */
  md.fs =  MIS2DU12_4g;
  md.odr = MIS2DU12_25Hz;
  mis2du12_mode_set(&dev_ctx, &md);

  /* Configure interrupt pins */
  mis2du12_pin_int2_route_get(&dev_ctx, &int_route);
  int_route.fifo_th = PROPERTY_ENABLE;
  mis2du12_pin_int2_route_set(&dev_ctx, &int_route);

  /* wait forever (xl FIFO samples read with drdy irq) */
  while (1) {

    if (fifo_wtm_event == 1) {
      uint8_t i;
      uint8_t fifo_level;

      fifo_wtm_event = 0;

      mis2du12_fifo_level_get(&dev_ctx, &fifo_mode, &fifo_level);

      snprintf((char*)tx_buffer, sizeof(tx_buffer), "-- FIFO level is %d\r\n", fifo_level);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));

      for (i = 0; i < fifo_level; i++) {
        mis2du12_fifo_data_get(&dev_ctx, &md, &fifo_mode, &data);

        /* print sensor data */
        if (fifo_mode.store == MIS2DU12_8_BIT) {
          snprintf((char*)tx_buffer, sizeof(tx_buffer), "%02d: Acceleration [mg]:\t%4.2f\t%4.2f\t%4.2f\tTemp[degC]:%3.2f\r\n",
                  i, data.xl[0].mg[0], data.xl[0].mg[1], data.xl[0].mg[2], data.heat.deg_c);
          tx_com(tx_buffer, strlen((char const*)tx_buffer));
        } else {
          snprintf((char*)tx_buffer, sizeof(tx_buffer), "%02d: Acceleration [0][mg]:\t%4.2f\t%4.2f\t%4.2f\r\n",
                  i, data.xl[0].mg[0], data.xl[0].mg[1], data.xl[0].mg[2]);
          tx_com(tx_buffer, strlen((char const*)tx_buffer));
          snprintf((char*)tx_buffer, sizeof(tx_buffer), "%02d: Acceleration [1][mg]:\t%4.2f\t%4.2f\t%4.2f\r\n",
                  i, data.xl[1].mg[0], data.xl[1].mg[1], data.xl[1].mg[2]);
          tx_com(tx_buffer, strlen((char const*)tx_buffer));
       }
      }
      snprintf((char*)tx_buffer, sizeof(tx_buffer), "\r\n");
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
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
  HAL_I2C_Mem_Write(handle, MIS2DU12_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  MIS2DU12_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, MIS2DU12_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, MIS2DU12_I2C_ADD_L & 0xFE, reg, bufp, len);
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
