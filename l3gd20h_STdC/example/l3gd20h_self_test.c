/*
 ******************************************************************************
 * @file    self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file implements the self test procedure.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI136V1
 * - NUCLEO_F411RE + STEVAL-MKI136V1
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
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
#define NUCLEO_F411RE

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2

/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F411RE)
/* NUCLEO_F411RE: Define communication interface */
#define SENSOR_BUS hi2c1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <l3gd20h_reg.h>
#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#elif defined(NUCLEO_F411RE)
#include "usart.h"
#endif

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

/* Private macro -------------------------------------------------------------*/

#define    BOOT_TIME        10 //ms
#define    WAIT_TIME       300 //ms


/* Self test limits. */
#define    MIN_ST_LIMIT_mdps   200000.0f
#define    MAX_ST_LIMIT_mdps   800000.0f

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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void l3gd20h_self_test(void)
{
  uint8_t tx_buffer[1000];
  axis3bit16_t data_raw;
  stmdev_ctx_t dev_ctx;
  float val_st_off[3];
  float val_st_on[3];
  float test_val[3];
  uint8_t st_result;
  uint8_t whoamI;
  uint8_t drdy;
  uint8_t rst;
  uint8_t i;
  uint8_t j;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Init test platform */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  l3gd20h_dev_id_get(&dev_ctx, &whoamI);
  if (whoamI != L3GD20H_ID)
    while(1);

  /* Restore default configuration */
  l3gd20h_dev_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    l3gd20h_dev_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  l3gd20h_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Gyroscope Self Test
   */

  /* Set Output Data Rate */
  l3gd20h_gy_data_rate_set(&dev_ctx, L3GD20H_200Hz);
  /* Set full scale */
  l3gd20h_gy_full_scale_set(&dev_ctx, L3GD20H_2000dps);

  /* Wait stable output */
  platform_delay(WAIT_TIME);

  /* Check if new value available */
  do {
    l3gd20h_gy_flag_data_ready_get(&dev_ctx, &drdy);
  } while(!drdy);
  /* Read dummy data and discard it */
  l3gd20h_angular_rate_raw_get(&dev_ctx, data_raw.u8bit);

  /* Read 5 sample and get the average vale for each axis */

  memset(val_st_off, 0x00, 3*sizeof(float));
  for (i = 0; i < 5; i++){
    /* Check if new value available */
    do {
      l3gd20h_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while(!drdy);
    /* Read data and accumulate the mg value */
    l3gd20h_angular_rate_raw_get(&dev_ctx, data_raw.u8bit);
    for (j = 0; j < 3; j++){
      val_st_off[j] += l3gd20h_from_fs2000_to_mdps(data_raw.i16bit[j]);
    }
  }
  /* Calculate the mg average values */
  for (i = 0; i < 3; i++){
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  l3gd20h_gy_self_test_set(&dev_ctx, L3GD20H_ST_POSITIVE);
  //l3gd20h_gy_self_test_set(&dev_ctx, LIS2DH12_ST_NEGATIVE);

  /* Wait stable output */
  platform_delay(WAIT_TIME);

  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3*sizeof(float));
  for (i = 0; i < 5; i++){
    /* Check if new value available */
    do {
      l3gd20h_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while(!drdy);
    /* Read data and accumulate the mg value */
    l3gd20h_angular_rate_raw_get(&dev_ctx, data_raw.u8bit);
    for (j = 0; j < 3; j++){
      val_st_on[j] += l3gd20h_from_fs2000_to_mdps(data_raw.i16bit[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++){
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++){
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }
  /* Check self test limit */
  for (i = 0; i < 3; i++){
    if (( MIN_ST_LIMIT_mdps > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mdps)){
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  l3gd20h_gy_self_test_set(&dev_ctx, L3GD20H_ST_DISABLE);
  /* Disable sensor. */
  l3gd20h_gy_data_rate_set(&dev_ctx, L3GD20H_POWER_DOWN);

  if (st_result == ST_PASS) {
    sprintf((char*)tx_buffer, "Self Test - PASS\r\n" );
  }
  else {
    sprintf((char*)tx_buffer, "Self Test - FAIL\r\n" );
  }
  tx_com(tx_buffer, strlen((char const*)tx_buffer));
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
    HAL_I2C_Mem_Write(handle, L3GD20H_I2C_ADD_L, reg,
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
    HAL_I2C_Mem_Read(handle, L3GD20H_I2C_ADD_L, reg,
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
  #ifdef NUCLEO_F411RE
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
  #endif
  #ifdef STEVAL_MKI109V3
  CDC_Transmit_FS(tx_buffer, len);
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
  HAL_Delay(ms);
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
