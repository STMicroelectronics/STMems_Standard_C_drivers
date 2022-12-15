/*
 ******************************************************************************
 * @file    read_fifo_irq.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
 * - NUCLEO_F411RE
 * - DISCOVERY_SPC584B
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(N/A)
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
#include "lps22df_reg.h"

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
#define    BOOT_TIME         10 //ms
#define    FIFO_TH           32
/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[1000];
static lps22df_fifo_data_t data[FIFO_TH];

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
static uint8_t lps22df_fifo_wtm_event;


/* Main Example --------------------------------------------------------------*/
void lps22df_read_fifo_irq_handler(void)
{
  lps22df_fifo_wtm_event = 1;
}

void lps22df_read_fifo_irq(void)
{
  lps22df_pin_int_route_t int_route;
  lps22df_bus_mode_t bus_mode;
  lps22df_stat_t status;
  lps22df_id_t id;
  lps22df_md_t md;
  lps22df_fifo_md_t fifo_md;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Initialize platform specific hardware */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  lps22df_id_get(&dev_ctx, &id);
  if (id.whoami != LPS22DF_ID)
    while(1);

  /* Restore default configuration */
  lps22df_init_set(&dev_ctx, LPS22DF_RESET);
  do {
    lps22df_status_get(&dev_ctx, &status);
  } while (status.sw_reset);

  /* Set bdu and if_inc recommended for driver usage */
  lps22df_init_set(&dev_ctx, LPS22DF_DRV_RDY);

  /* Select bus interface */
  bus_mode.filter = LPS22DF_AUTO;
  bus_mode.interface = LPS22DF_SEL_BY_HW;
  lps22df_bus_mode_set(&dev_ctx, &bus_mode);

  /* Set Output Data Rate */
  md.odr = LPS22DF_25Hz;
  md.avg = LPS22DF_16_AVG;
  md.lpf = LPS22DF_LPF_ODR_DIV_4;
  lps22df_mode_set(&dev_ctx, &md);

  fifo_md.watermark = FIFO_TH;
  fifo_md.operation = LPS22DF_STREAM;
  lps22df_fifo_mode_set(&dev_ctx, &fifo_md);

  /* Configure inerrupt pins */
  lps22df_pin_int_route_get(&dev_ctx, &int_route);
  int_route.fifo_th   = PROPERTY_ENABLE;
  lps22df_pin_int_route_set(&dev_ctx, &int_route);

  /* Read samples on FIFO wtm event*/
  while(1) {
    if (lps22df_fifo_wtm_event) {
      lps22df_all_sources_t status;
      uint8_t num, i;

      lps22df_fifo_wtm_event = 0;

      lps22df_all_sources_get(&dev_ctx, &status);

      if (status.fifo_th) {
        lps22df_fifo_level_get(&dev_ctx, &num);

        lps22df_fifo_data_get(&dev_ctx, num, data);

        sprintf((char*)tx_buffer, "-- FIFO interrupt %d\r\n", num);
        tx_com(tx_buffer, strlen((char const*)tx_buffer));

        for (i = 0; i < num; i++) {
          sprintf((char*)tx_buffer, "pressure [hPa]:%6.2f\r\n", data[i].hpa);
          tx_com(tx_buffer, strlen((char const*)tx_buffer));
        }

        sprintf((char*)tx_buffer, "-- \r\n\n");
        tx_com(tx_buffer, strlen((char const*)tx_buffer));
      }
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
  HAL_I2C_Mem_Write(handle, LPS22DF_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LPS22DF_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LPS22DF_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LPS22DF_I2C_ADD_L & 0xFE, reg, bufp, len);
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
  HAL_Delay(1000);
#endif
}
