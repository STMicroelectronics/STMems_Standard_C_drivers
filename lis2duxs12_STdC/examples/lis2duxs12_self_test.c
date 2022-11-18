/*
 ******************************************************************************
 * @file    lis2duxs12_self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file implements the self test procedure.
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
 * - NUCLEO_F411RE + STEVAL-
 * - DISCOVERY_SPC584B + STEVAL-
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
#include "lis2duxs12_reg.h"

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

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME      10

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[1000];
static   lis2duxs12_fifo_data_t out1, out2, st_dev;

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

/* Self test limits. */
#define    ST_RANGE_DEV_X_MIN          50U
#define    ST_RANGE_DEV_X_MAX         700U
#define    ST_RANGE_DEV_Y_MIN          50U
#define    ST_RANGE_DEV_Y_MAX         700U
#define    ST_RANGE_DEV_Z_MIN         200U
#define    ST_RANGE_DEV_Z_MAX        1200U

/* Private variables ---------------------------------------------------------*/

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

enum st_test_type {
  ST_POS = 0,
  ST_NEG
};

static void lis2duxs12_st_avg_5_samples(stmdev_ctx_t *ctx,
                                      lis2duxs12_md_t *md,
                                      lis2duxs12_fifo_mode_t *fmode,
                                      lis2duxs12_fifo_data_t *fdata)
{
  uint8_t i;

  fdata->xl[0].mg[0] = 0.0f;
  fdata->xl[0].mg[1] = 0.0f;
  fdata->xl[0].mg[2] = 0.0f;

  for (i = 0; i < 5; i++) {
    lis2duxs12_fifo_data_t tmp;

   // lis2duxs12_data_get(ctx, md, &tmp);
    lis2duxs12_fifo_data_get(ctx, md, fmode, &tmp);

    fdata->xl[0].mg[0] += tmp.xl[0].mg[0];
    fdata->xl[0].mg[1] += tmp.xl[0].mg[1];
    fdata->xl[0].mg[2] += tmp.xl[0].mg[2];
  }
  fdata->xl[0].mg[0] /= 5;
  fdata->xl[0].mg[1] /= 5;
  fdata->xl[0].mg[2] /= 5;
}

/* Main Example --------------------------------------------------------------*/
void lis2duxs12_self_test(void)
{
  lis2duxs12_status_t status;
  lis2duxs12_md_t md;
  lis2duxs12_fifo_mode_t fifo_mode;
  uint8_t id;
  stmdev_ctx_t dev_ctx;
  uint8_t st_result = ST_PASS;
  uint8_t i, test;
  uint16_t num;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Initialize platform specific hardware */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  lis2duxs12_exit_deep_power_down(&dev_ctx);
  platform_delay(25); /* wait 25ms after going out DEEP power state */

  /* Check device ID */
  lis2duxs12_device_id_get(&dev_ctx, &id);
  if (id != LIS2DUXS12_ID)
    while(1);

  /* Restore default configuration */
  lis2duxs12_init_set(&dev_ctx, LIS2DUXS12_RESET);
  do {
    lis2duxs12_status_get(&dev_ctx, &status);
  } while (status.sw_reset);

  /*
   * Accelerometer Self Test
   */

  for (test = ST_POS; test <= ST_NEG; test++)
  {
    /*
     * 1. Set the device in soft power-down and wait 10ms.
     */
    md.fs =  LIS2DUXS12_8g;
    md.odr = LIS2DUXS12_OFF;
    lis2duxs12_mode_set(&dev_ctx, &md);
    platform_delay(10);

    /*
     * 2. Set the FIFO_EN bit in the CTRL4 (13h) register to 1.
     * 3. Set the XL_ONLY_FIFO bit in the FIFO_WTM (16h) register to 1.
     * 5. Set the FIFO_CTRL (15h) register to 00h to empty the FIFO
     */
    lis2duxs12_fifo_mode_get(&dev_ctx, &fifo_mode);
    fifo_mode.operation = LIS2DUXS12_BYPASS_MODE;
    fifo_mode.xl_only = 1;
    fifo_mode.store = LIS2DUXS12_FIFO_1X;
    lis2duxs12_fifo_mode_set(&dev_ctx, fifo_mode);

    /*
     * 4. Set the ST_SIGN_X and ST_SIGN_Y bits in the CTRL3 (12h) register to 1
     *    and the ST_SIGN_Z bit in the WAKE_UP_DUR (1Dh) register to 0
     *    (i.e. 001 for POSITIVE. Instead, for NEGATIVE is 010).
     */
    if (test == ST_POS)
      lis2duxs12_self_test_sign_set(&dev_ctx, LIS2DUXS12_XL_ST_POSITIVE);
    else
      lis2duxs12_self_test_sign_set(&dev_ctx, LIS2DUXS12_XL_ST_NEGATIVE);

    /*
     * 6. Set ST[1:0] to "10"
     */
    lis2duxs12_self_test_start(&dev_ctx, 2);

    /*
     * 7. Set ODR = 200 Hz, BW = ODR/2, FS = +/-8 g from the CTRL5 (14h) register
     *    and wait 50ms.
     */
    md.fs =  LIS2DUXS12_8g;
    md.odr = LIS2DUXS12_200Hz;
    md.bw = LIS2DUXS12_ODR_div_2;
    lis2duxs12_mode_set(&dev_ctx, &md);
    platform_delay(50);

    /*
     * 8. Set the FIFO_CTRL (15h) register to 01h to start filling the FIFO.
     */
    lis2duxs12_fifo_mode_get(&dev_ctx, &fifo_mode);
    fifo_mode.operation = LIS2DUXS12_FIFO_MODE;
    lis2duxs12_fifo_mode_set(&dev_ctx, fifo_mode);

    /*
     * 9. Read the first 5 samples from FIFO, compute the average for each
     *    axis, and save the result in OUT1.
     */
    do {
      lis2duxs12_fifo_data_level_get(&dev_ctx, &num);
    } while (num < 5);

    lis2duxs12_st_avg_5_samples(&dev_ctx, &md, &fifo_mode, &out1);

    /*
     * 10. Set device in Power Down mode and wait 10 ms.
     */
    md.fs =  LIS2DUXS12_8g;
    md.odr = LIS2DUXS12_OFF;
    lis2duxs12_mode_set(&dev_ctx, &md);
    platform_delay(10);

    /*
     * 11. Set the FIFO_CTRL (15h) register to 00h to empty the FIFO.
     */
    lis2duxs12_fifo_mode_get(&dev_ctx, &fifo_mode);
    fifo_mode.operation = LIS2DUXS12_BYPASS_MODE;
    lis2duxs12_fifo_mode_set(&dev_ctx, fifo_mode);

    /*
     * 12. Set ST[1:0] to "01"
     */
    lis2duxs12_self_test_start(&dev_ctx, 1);

    /*
     * 13. Set ODR = 200 Hz, BW = ODR/2, FS = +/-8 g from the CTRL5 (14h) register
     *     and wait 50ms.
     */
    md.fs =  LIS2DUXS12_8g;
    md.odr = LIS2DUXS12_200Hz;
    md.bw = LIS2DUXS12_ODR_div_2;
    lis2duxs12_mode_set(&dev_ctx, &md);
    platform_delay(50);

    /*
     * 14. Set the FIFO_CTRL (15h) register to 01h to start filling the FIFO
     *     and wait 25ms.
     */
    lis2duxs12_fifo_mode_get(&dev_ctx, &fifo_mode);
    fifo_mode.operation = LIS2DUXS12_FIFO_MODE;
    lis2duxs12_fifo_mode_set(&dev_ctx, fifo_mode);
    platform_delay(25);

    /*
     * 15. Read the first 5 samples from FIFO, compute the average for each
     *     axis, and save the result in OUT2.
     */
    do {
      lis2duxs12_fifo_data_level_get(&dev_ctx, &num);
    } while (num < 5);

   lis2duxs12_st_avg_5_samples(&dev_ctx, &md, &fifo_mode, &out2);

    /*
     * 16. Set device in Power Down mode and wait 10 ms.
     */
    md.fs =  LIS2DUXS12_8g;
    md.odr = LIS2DUXS12_OFF;
    lis2duxs12_mode_set(&dev_ctx, &md);
    platform_delay(10);

    /*
     * 17. Set the ST[1:0] bits in the SELF_TEST (32h) register to 00.
     */
   lis2duxs12_self_test_stop(&dev_ctx);

    /*
     * 18. Self-test deviation is 2 * |OUT2 - OUT1|. Compute the value for
     * each axis and verify that it falls within the range provided in the
     * datasheet
     */
    for (i = 0; i < 3; i++)
      st_dev.xl[0].mg[i] = 2 * fabs(out2.xl[0].mg[i] - out1.xl[0].mg[i]);

    /*
     * 19. Set device in Power Down mode
     */
    md.fs =  LIS2DUXS12_8g;
    md.odr = LIS2DUXS12_OFF;
    lis2duxs12_mode_set(&dev_ctx, &md);

    /* check if stdev falls into given ranges */
    st_result = ST_FAIL;
    if ((st_dev.xl[0].mg[0] >= ST_RANGE_DEV_X_MIN && st_dev.xl[0].mg[0] <= ST_RANGE_DEV_X_MAX) &&
        (st_dev.xl[0].mg[1] >= ST_RANGE_DEV_Y_MIN && st_dev.xl[0].mg[1] <= ST_RANGE_DEV_Y_MAX) &&
        (st_dev.xl[0].mg[2] >= ST_RANGE_DEV_Z_MIN && st_dev.xl[0].mg[2] <= ST_RANGE_DEV_Z_MAX))
    {
      st_result = ST_PASS;
    }

    if (st_result == ST_PASS) {
      sprintf((char *)tx_buffer, "%s Self Test - PASS\r\n", (test == ST_POS) ? "\nPOS" : "NEG");
    }
    else {
      sprintf((char *)tx_buffer, "%s Self Test - FAIL!!!!\r\n", (test == ST_POS) ? "\nPOS" : "NEG");
    }

    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }

  //while(1);
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
#if defined(NUCLEO_F411RE)
  HAL_I2C_Mem_Write(handle, LIS2DUXS12_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LIS2DUXS12_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LIS2DUXS12_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LIS2DUXS12_I2C_ADD_L & 0xFE, reg, bufp, len);
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
  HAL_Delay(1000);
#endif
}
