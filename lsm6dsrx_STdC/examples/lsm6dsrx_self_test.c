/*
 ******************************************************************************
 * @file    lsm6dsrx_self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file implement the self test procedure.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI195V1
 * - NUCLEO_F401RE + STEVAL-MKI195V1
 * - DISCOVERY_SPC584B + STEVAL-MKI195V1
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
#include <lsm6dsrx_reg.h>

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

/* Self test limits. */
#define    MIN_ST_LIMIT_mg        40.0f
#define    MAX_ST_LIMIT_mg      1700.0f
#define    MIN_ST_LIMIT_mdps   150000.0f
#define    MAX_ST_LIMIT_mdps   700000.0f

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

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

/* Main Example --------------------------------------------------------------*/
void lsm6dsrx_self_test(void)
{
  stmdev_ctx_t dev_ctx;
  float_t val_st_off[3];
  float_t val_st_on[3];
  float_t test_val[3];
  int16_t data_raw[3];
  uint8_t tx_buffer[1000];
  uint8_t st_result;
  uint8_t whoamI;
  uint8_t drdy;
  uint8_t rst;
  uint8_t i;
  uint8_t j;
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
  lsm6dsrx_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSRX_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsrx_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsrx_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dsrx_i3c_disable_set(&dev_ctx, LSM6DSRX_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dsrx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Accelerometer Self Test */
  /* Set Output Data Rate */
  lsm6dsrx_xl_data_rate_set(&dev_ctx, LSM6DSRX_XL_ODR_52Hz);
  /* Set full scale */
  lsm6dsrx_xl_full_scale_set(&dev_ctx, LSM6DSRX_4g);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dsrx_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6dsrx_acceleration_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsrx_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dsrx_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsrx_from_fs4g_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm6dsrx_xl_self_test_set(&dev_ctx, LSM6DSRX_XL_ST_POSITIVE);
  //lsm6dsrx_xl_self_test_set(&dev_ctx, LIS2DH12_XL_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dsrx_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6dsrx_acceleration_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsrx_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dsrx_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsrx_from_fs4g_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabsf((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  st_result = ST_PASS;

  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mg > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mg)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm6dsrx_xl_self_test_set(&dev_ctx, LSM6DSRX_XL_ST_DISABLE);
  /* Disable sensor. */
  lsm6dsrx_xl_data_rate_set(&dev_ctx, LSM6DSRX_XL_ODR_OFF);
  /* Gyroscope Self Test */
  /* Set Output Data Rate */
  lsm6dsrx_gy_data_rate_set(&dev_ctx, LSM6DSRX_GY_ODR_208Hz);
  /* Set full scale */
  lsm6dsrx_gy_full_scale_set(&dev_ctx, LSM6DSRX_2000dps);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dsrx_gy_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6dsrx_angular_rate_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsrx_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dsrx_angular_rate_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsrx_from_fs2000dps_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm6dsrx_gy_self_test_set(&dev_ctx, LSM6DSRX_GY_ST_POSITIVE);
  //lsm6dsrx_gy_self_test_set(&dev_ctx, LIS2DH12_GY_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(100);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsrx_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dsrx_angular_rate_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsrx_from_fs2000dps_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabsf((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mdps > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mdps)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm6dsrx_gy_self_test_set(&dev_ctx, LSM6DSRX_GY_ST_DISABLE);
  /* Disable sensor. */
  lsm6dsrx_xl_data_rate_set(&dev_ctx, LSM6DSRX_XL_ODR_OFF);

  if (st_result == ST_PASS) {
    snprintf((char *)tx_buffer, sizeof(tx_buffer), "Self Test - PASS\r\n" );
  }

  else {
    snprintf((char *)tx_buffer, sizeof(tx_buffer), "Self Test - FAIL\r\n" );
  }

  tx_com(tx_buffer, strlen((char const *)tx_buffer));
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
  HAL_I2C_Mem_Write(handle, LSM6DSRX_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSRX_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LSM6DSRX_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LSM6DSRX_I2C_ADD_L & 0xFE, reg, bufp, len);
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
  HAL_Delay(1000);
#endif
}
