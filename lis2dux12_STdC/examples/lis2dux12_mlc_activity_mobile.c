/*
 ******************************************************************************
 * @file    mlc_activity_mobile.c
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
 * - NUCLEO_F401RE +
 * - DISCOVERY_SPC584B +
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
#include "lis2dux12_activity_recognition_for_mobile.h"
#include "lis2dux12_reg.h"

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
#define    BOOT_TIME            10 //ms

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

static stmdev_ctx_t dev_ctx;
static uint8_t stationary_event_catched = 0;
static uint8_t walking_event_catched = 0;
static uint8_t jogging_event_catched = 0;
static uint8_t biking_event_catched = 0;
static uint8_t driving_event_catched = 0;

void lis2dux12_mlc_activity_mobile_handler(void)
{
  lis2dux12_mlc_status_mainpage_t status;
  uint8_t mlc_out;

  /* Read output only if new xl value is available */
  lis2dux12_mlc_status_get(&dev_ctx, &status);

  if (status.is_mlc1) {
    lis2dux12_mlc_out_get(&dev_ctx, &mlc_out);

    switch(mlc_out) {
    case 0:
      stationary_event_catched = 1;
      break;
    case 1:
      walking_event_catched = 1;
      break;
    case 4:
      jogging_event_catched = 1;
      break;
    case 8:
      biking_event_catched = 1;
      break;
    case 12:
      driving_event_catched = 1;
      break;
    }
  }
}

/* Main Example --------------------------------------------------------------*/
void lis2dux12_mlc_activity_mobile(void)
{
  uint8_t id;
  uint32_t i;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  lis2dux12_exit_deep_power_down(&dev_ctx);

  /* Check device ID */
  lis2dux12_device_id_get(&dev_ctx, &id);
  if (id != LIS2DUX12_ID)
    while(1);

  /* Restore default configuration */
  lis2dux12_sw_reset(&dev_ctx);

  /* init bdu and add_inc */
  lis2dux12_init_set(&dev_ctx);

  /* Start Machine Learning Core configuration */
  for ( i = 0; i < (sizeof(lis2dux12_activity_recognition_for_mobile_conf_0) / sizeof(struct mems_conf_op) ); i++ ) {
    switch(lis2dux12_activity_recognition_for_mobile_conf_0[i].type) {
    case MEMS_CONF_OP_TYPE_DELAY:
      platform_delay(lis2dux12_activity_recognition_for_mobile_conf_0[i].data);
      break;
    case MEMS_CONF_OP_TYPE_WRITE:
      lis2dux12_write_reg(&dev_ctx, lis2dux12_activity_recognition_for_mobile_conf_0[i].address,
                           (uint8_t *)&lis2dux12_activity_recognition_for_mobile_conf_0[i].data, 1);
      break;
    }
  }

  /* wait forever (FF event handle in irq handler) */
  while (1) {
    if (stationary_event_catched) {
      stationary_event_catched = 0;

      snprintf((char *)tx_buffer, sizeof(tx_buffer), "stationary event\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
    if (walking_event_catched) {
      walking_event_catched = 0;

      snprintf((char *)tx_buffer, sizeof(tx_buffer), "walking event\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
    if (jogging_event_catched) {
      jogging_event_catched = 0;

      snprintf((char *)tx_buffer, sizeof(tx_buffer), "jogging event\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
    if (biking_event_catched) {
      biking_event_catched = 0;

      snprintf((char *)tx_buffer, sizeof(tx_buffer), "biking event\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
    if (driving_event_catched) {
      driving_event_catched = 0;

      snprintf((char *)tx_buffer, sizeof(tx_buffer), "driving event\r\n");
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
  HAL_I2C_Mem_Write(handle, LIS2DUX12_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LIS2DUX12_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LIS2DUX12_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LIS2DUX12_I2C_ADD_L & 0xFE, reg, bufp, len);
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
