/*
 ******************************************************************************
 * @file    self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to get data from sensor.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI196V1
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A3
 * - DISCOVERY_SPC584B + STEVAL-MKI196V1
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
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
#include "ism330is_reg.h"

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
#define    BOOT_TIME      10

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

/* Self test limits. */
#define    ST_XL_RANGE_MG_MIN              50U
#define    ST_XL_RANGE_MG_MAX            1700U
#define    ST_GY_RANGE_MDPS_MIN        150000U
#define    ST_GY_RANGE_MDPS_MAX        700000U

enum st_test_type {
  ST_POS = 0,
  ST_NEG
};

/* Private variables ---------------------------------------------------------*/
static int16_t temp_raw[3];
static uint8_t whoamI;
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

static void ism330is_avg_5_xl_samples(stmdev_ctx_t *ctx, float out[3])
{
  uint8_t drdy, i;

  out[0] = 0.0f;
  out[1] = 0.0f;
  out[2] = 0.0f;
  for (i = 0; i < 5; i++) {
    do {
      ism330is_xl_flag_data_ready_get(ctx, &drdy);
    } while(!drdy);

    ism330is_acceleration_raw_get(ctx, temp_raw);
    out[0] += ism330is_from_fs4g_to_mg(temp_raw[0]);
    out[1] += ism330is_from_fs4g_to_mg(temp_raw[1]);
    out[2] += ism330is_from_fs4g_to_mg(temp_raw[2]);
  }

  out[0] /= 5;
  out[1] /= 5;
  out[2] /= 5;
}

static void ism330is_avg_5_gy_samples(stmdev_ctx_t *ctx, float out[3])
{
  uint8_t drdy, i;

  out[0] = 0.0f;
  out[1] = 0.0f;
  out[2] = 0.0f;
  for (i = 0; i < 5; i++) {
    do {
      ism330is_gy_flag_data_ready_get(ctx, &drdy);
    } while(!drdy);

    ism330is_angular_rate_raw_get(ctx, temp_raw);
    out[0] += ism330is_from_fs2000dps_to_mdps(temp_raw[0]);
    out[1] += ism330is_from_fs2000dps_to_mdps(temp_raw[1]);
    out[2] += ism330is_from_fs2000dps_to_mdps(temp_raw[2]);
  }

  out[0] /= 5;
  out[1] /= 5;
  out[2] /= 5;
}

/* Main Example --------------------------------------------------------------*/
void ism330is_self_test(void)
{
  stmdev_ctx_t dev_ctx;
  uint8_t drdy, i, test;
  float out_st_mg[3], out_nost_mg[3], abs_diff_mg[3];
  uint8_t st_result = ST_PASS;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  ism330is_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != ISM330IS_ID)
    while (1);

  /* Restore default configuration */
  ism330is_software_reset(&dev_ctx);

  /*
   * Accelerometer SELF-TEST
   */
  for (test = ST_POS; test <= ST_NEG; test++)
  {
    /*
     * Initialize and turn on XL sensor
     * Set BDU = 1, FS = +/- 4g, ODR = 52Hz
     */
    ism330is_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    ism330is_xl_data_rate_set(&dev_ctx, ISM330IS_XL_ODR_AT_52Hz_HP);
    ism330is_xl_full_scale_set(&dev_ctx, ISM330IS_4g);

    /*
     * Power up, wait 100ms for stable output
     * Discard data
     */
    platform_delay(100);

    do {
      ism330is_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while(!drdy);

    ism330is_acceleration_raw_get(&dev_ctx, temp_raw);
    memset(temp_raw, 0x00, 3 * sizeof(int16_t));

    /*
     * For 5 times, after checking XLDA bit, read the output registers
     * Average the stored data on each axis
     */
    ism330is_avg_5_xl_samples(&dev_ctx, out_nost_mg);

    /*
     * Enable xl self-test
     * wait 100ms for stable output
     * Discard data
     */
    if (test == ST_POS)
      ism330is_xl_self_test_set(&dev_ctx, ISM330IS_XL_ST_POSITIVE);
    else
      ism330is_xl_self_test_set(&dev_ctx, ISM330IS_XL_ST_NEGATIVE);
    platform_delay(100);

    do {
      ism330is_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while(!drdy);

    ism330is_acceleration_raw_get(&dev_ctx, temp_raw);
    memset(temp_raw, 0x00, 3 * sizeof(int16_t));

    /*
     * For 5 times, after checking XLDA bit, read the output registers
     * Average the stored data on each axis
     */
    ism330is_avg_5_xl_samples(&dev_ctx, out_st_mg);

    /*
     * Disable self-test, disable XL sensor
     */
    ism330is_xl_self_test_set(&dev_ctx, ISM330IS_XL_ST_DISABLE);
    ism330is_xl_data_rate_set(&dev_ctx, ISM330IS_XL_ODR_OFF);

    /*
     * Test if data in range
     */
    st_result = ST_PASS;
    for (i = 0; i < 3; i++) {
      abs_diff_mg[i] = fabs(out_st_mg[i] - out_nost_mg[i]);

      if (abs_diff_mg[i] < ST_XL_RANGE_MG_MIN || abs_diff_mg[i] > ST_XL_RANGE_MG_MAX)
        st_result = ST_FAIL;
    }

    if (st_result == ST_PASS) {
      sprintf((char *)tx_buffer, "%s XL Self Test - PASS\r\n", (test == ST_POS) ? "\nPOS" : "NEG");
    } else {
      sprintf((char *)tx_buffer, "%s XL Self Test - FAIL!!!!\r\n", (test == ST_POS) ? "\nPOS" : "NEG");
    }
    tx_com(tx_buffer, strlen((char const *)tx_buffer));

  }

  /*
   * Gyro SELF-TEST
   */
  for (test = ST_POS; test <= ST_NEG; test++)
  {
    /*
     * Initialize and turn on GY sensor
     * Set BDU = 1, FS = +/- 2000dps, ODR = 208Hz
     */
    ism330is_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    ism330is_gy_data_rate_set(&dev_ctx, ISM330IS_GY_ODR_AT_208Hz_HP);
    ism330is_gy_full_scale_set(&dev_ctx, ISM330IS_2000dps);

    /*
     * Power up, wait 100ms for stable output
     * Discard data
     */
    platform_delay(100);

    do {
      ism330is_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while(!drdy);

    ism330is_angular_rate_raw_get(&dev_ctx, temp_raw);
    memset(temp_raw, 0x00, 3 * sizeof(int16_t));

    /*
     * For 5 times, after checking GDA bit, read the output registers
     * Average the stored data on each axis
     */
    ism330is_avg_5_gy_samples(&dev_ctx, out_nost_mg);

    /*
     * Enable gy self-test
     * wait 100ms for stable output
     * Discard data
     */
    if (test == ST_POS)
      ism330is_gy_self_test_set(&dev_ctx, ISM330IS_GY_ST_POSITIVE);
    else
      ism330is_gy_self_test_set(&dev_ctx, ISM330IS_GY_ST_NEGATIVE);
    platform_delay(100);

    do {
      ism330is_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while(!drdy);

    ism330is_angular_rate_raw_get(&dev_ctx, temp_raw);
    memset(temp_raw, 0x00, 3 * sizeof(int16_t));

    /*
     * For 5 times, after checking GDA bit, read the output registers
     * Average the stored data on each axis
     */
    ism330is_avg_5_gy_samples(&dev_ctx, out_st_mg);

    /*
     * Disable self-test, disable sensor
     */
    ism330is_gy_self_test_set(&dev_ctx, ISM330IS_GY_ST_DISABLE);
    ism330is_gy_data_rate_set(&dev_ctx, ISM330IS_GY_ODR_OFF);

    /*
     * Test if data in range
     */
    st_result = ST_PASS;
    for (i = 0; i < 3; i++) {
      abs_diff_mg[i] = fabs(out_st_mg[i] - out_nost_mg[i]);

      if (abs_diff_mg[i] < ST_GY_RANGE_MDPS_MIN || abs_diff_mg[i] > ST_GY_RANGE_MDPS_MAX)
        st_result = ST_FAIL;
    }

    if (st_result == ST_PASS) {
      sprintf((char *)tx_buffer, "%s GY Self Test - PASS\r\n", (test == ST_POS) ? "POS" : "NEG");
    } else {
      sprintf((char *)tx_buffer, "%s GY Self Test - FAIL!!!!\r\n", (test == ST_POS) ? "POS" : "NEG");
    }
    tx_com(tx_buffer, strlen((char const *)tx_buffer));

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
  HAL_I2C_Mem_Write(handle, ISM330IS_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  ISM330IS_I2C_ADD_H & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, ISM330IS_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, ISM330IS_I2C_ADD_H & 0xFE, reg, bufp, len);
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
