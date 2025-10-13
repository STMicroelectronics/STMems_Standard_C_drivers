/*
 ******************************************************************************
 * @file    sixd.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to generate and handle a Free Fall event.
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
 * - STEVAL_MKI109V3 +
 * - NUCLEO_F401RE + X-NUCLEO-IKS01A3
 * - DISCOVERY_SPC584B +
 * - NUCLEO_H503RB + X-NUCLEO-IKS4A1
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
 * NUCLEO_STM32H503RG - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I3C(Default)
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

#elif defined(NUCLEO_H503RB)
/* NUCLEO_H503RB: Define communication interface */
#define SENSOR_BUS hi3c1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lsm6dsv16x_reg.h"

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

#elif defined(NUCLEO_H503RB)
#include "usart.h"
#include "i3c.h"
#include "i3c_api.h"
#include <stdio.h>

static uint8_t i3c_dyn_addr = 0x0A;
#endif

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static lsm6dsv16x_interrupt_mode_t irq;

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
static void platform_init(void *handle);

static stmdev_ctx_t dev_ctx;
static uint8_t sixd_event_catched = 0;

void lsm6dsv16x_sixd_handler(void)
{
  lsm6dsv16x_all_sources_t status;

  /* Read output only if new xl value is available */
  lsm6dsv16x_all_sources_get(&dev_ctx, &status);

  if (status.six_d) {
    sixd_event_catched = (status.six_d    << 6) |
                         (status.six_d_zh << 5) |
                         (status.six_d_zl << 4) |
                         (status.six_d_yh << 3) |
                         (status.six_d_yl << 2) |
                         (status.six_d_xh << 1) |
                         (status.six_d_xl << 0);
  }
}

/* Main Example --------------------------------------------------------------*/
void lsm6dsv16x_sixd(void)
{
  lsm6dsv16x_pin_int_route_t pin_int;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Init test platform */
  platform_init(dev_ctx.handle);

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  lsm6dsv16x_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSV16X_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsv16x_sw_por(&dev_ctx);

  /* Enable Block Data Update */
  lsm6dsv16x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

#if defined(NUCLEO_H503RB)
  /* if I3C is used then INT pin must be explicitly enabled */
  lsm6dsv16x_i3c_int_en_set(&dev_ctx, 1);
#endif

  pin_int.sixd = PROPERTY_ENABLE;
  lsm6dsv16x_pin_int1_route_set(&dev_ctx, &pin_int);
  //lsm6dsv16x_pin_int2_route_set(&dev_ctx, &pin_int);

  irq.enable = 1;
  irq.lir = 1;
  lsm6dsv16x_interrupt_enable_set(&dev_ctx, irq);

  lsm6dsv16x_filt_sixd_feed_set(&dev_ctx, LSM6DSV16X_SIXD_FEED_LOW_PASS);
  lsm6dsv16x_6d_threshold_set(&dev_ctx, LSM6DSV16X_DEG_60);

  /* Set Output Data Rate.*/
  lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_120Hz);
  /* Set full scale */
  lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_2g);

  /* wait forever (6D event handle in irq handler) */
  while (1) {
    if (sixd_event_catched & 0x40) {

      switch (sixd_event_catched) {
      case 0x48:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "6D Position A\r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      case 0x41:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "6D Position B\r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      case 0x42:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "6D Position C\r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      case 0x44:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "6D Position D\r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      case 0x60:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "6D Position E\r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      case 0x50:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "6D Position F\r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      }
      sixd_event_catched = 0;
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
  HAL_I2C_Mem_Write(handle, LSM6DSV16X_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSV16X_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
#elif defined(NUCLEO_H503RB)
  i3c_write(handle, i3c_dyn_addr, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LSM6DSV16X_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LSM6DSV16X_I2C_ADD_L & 0xFE, reg, bufp, len);
#elif defined(NUCLEO_H503RB)
  i3c_read(handle, i3c_dyn_addr, reg, bufp, len);
#endif
  return 0;
}

/*
 * @brief  platform specific outputs on terminal (platform dependent)
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
#elif defined(NUCLEO_H503RB)
  HAL_UART_Transmit(&huart3, tx_buffer, len, 1000);
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
#if defined(NUCLEO_F401RE) || defined(STEVAL_MKI109V3) || defined(NUCLEO_H503RB)
  HAL_Delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void *handle)
{
#if defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);

#elif defined(NUCLEO_H503RB)
  i3c_set_bus_frequency(handle, 1000000);
  i3c_rstdaa(handle);
  i3c_setdasa(handle, LSM6DSV16X_I2C_ADD_L, &i3c_dyn_addr, 1);
  i3c_set_bus_frequency(handle, 12500000);

#endif
}
