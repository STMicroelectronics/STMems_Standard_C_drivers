/*
 ******************************************************************************
 * @file    read_data_simple_timestamp.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor (with
 *          timestamp)
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A2
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A2 - Host side: UART(COM) to USB bridge
 *                                       - I2C(Default) / SPI(N/A)
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
#define NUCLEO_F411RE_X_NUCLEO_IKS01A2

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2

/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
/* NUCLEO_F411RE_X_NUCLEO_IKS01A2: Define communication interface */
#define SENSOR_BUS hi2c1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <asm330lhh_reg.h>
#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
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

/* Private variables ---------------------------------------------------------*/
typedef union {
	uint8_t byte[4];
	uint32_t val;
} timestamp_t;

static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void example_main_read_simple_timestamp_asm330lhh(void)
{
  stmdev_ctx_t dev_ctx;
  int8_t fine_frq_correction;
  float ts_res;

  /*
   *  Initialize mems driver interface
   */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /*
   * Init test platform
   */
  platform_init();

  /*
   *  Check device ID
   */
  asm330lhh_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != ASM330LHH_ID)
    while(1);

  /*
   *  Restore default configuration
   */
  asm330lhh_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    asm330lhh_reset_get(&dev_ctx, &rst);
  } while (rst);

  /*
   *  Enable Block Data Update
   */
  asm330lhh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set Output Data Rate
   */
  asm330lhh_xl_data_rate_set(&dev_ctx, ASM330LHH_XL_ODR_12Hz5);
  asm330lhh_gy_data_rate_set(&dev_ctx, ASM330LHH_GY_ODR_12Hz5);
  
  /*
   * ODR correction factor: Difference in percentage of the effective 
   *                        ODR (and timestamp rate) with respect
   *                        to the typical.[set]
   *                        Step:  0.15%. 8-bit format, 2's complement.
   * The formula below can be used to calculate a better estimation of
   * the actual timestamp:
   * TS_Res = 1 / (40000 + (0.0015 * INTERNAL_FREQ_FINE * 40000))
   */  
  asm330lhh_odr_cal_reg_get(&dev_ctx, &fine_frq_correction);
  ts_res = 1 / (40000 + ( 0.0015 * fine_frq_correction * 40000) );

  /*
   * Set full scale
   */
  asm330lhh_xl_full_scale_set(&dev_ctx, ASM330LHH_2g);
  asm330lhh_gy_full_scale_set(&dev_ctx, ASM330LHH_2000dps);

  /*
   * Enable timestamp
   */
  asm330lhh_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Configure filtering chain(No aux interface)
   *
   * Accelerometer - LPF1 + LPF2 path
   */
  asm330lhh_xl_hp_path_on_out_set(&dev_ctx, ASM330LHH_LP_ODR_DIV_100);
  asm330lhh_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Read samples in polling mode (no int)
   */
  while(1)
  {
    asm330lhh_reg_t reg;
    timestamp_t timestamp;
    float timestamp_us;

    /*
     * Read output only if new value is available
     */
    asm330lhh_status_reg_get(&dev_ctx, &reg.status_reg);

    if (reg.status_reg.xlda || reg.status_reg.gda || reg.status_reg.tda){
      asm330lhh_timestamp_raw_get(&dev_ctx, timestamp.byte);
      timestamp_us = timestamp.val * ts_res;
    }
    if (reg.status_reg.xlda)
    {
      /*
       * Read acceleration field data
       */
      memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
      asm330lhh_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
      acceleration_mg[0] =
        asm330lhh_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] =
        asm330lhh_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] =
        asm330lhh_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);

      sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f %.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
              timestamp_us);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
    }

    if (reg.status_reg.gda)
    {
      /*
       * Read angular rate field data
       */
      memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
      asm330lhh_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
      angular_rate_mdps[0] =
        asm330lhh_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
      angular_rate_mdps[1] =
        asm330lhh_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
      angular_rate_mdps[2] =
        asm330lhh_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

      sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f %f\r\n",
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2],
      timestamp_us);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
    }

    if (reg.status_reg.tda)
    {
      /*
       * Read temperature data
       */
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      asm330lhh_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
      temperature_degC = asm330lhh_from_lsb_to_celsius(data_raw_temperature.i16bit);

      sprintf((char*)tx_buffer,
              "Temperature [degC]:%6.2f %f\r\n", temperature_degC, timestamp_us);
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, ASM330LHH_I2C_ADD_L, reg,
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
    HAL_I2C_Mem_Read(handle, ASM330LHH_I2C_ADD_L, reg,
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
  #ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
  #endif
  #ifdef STEVAL_MKI109V3
  CDC_Transmit_FS( tx_buffer, len );
  #endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#ifdef STEVAL_MKI109V3
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_Delay(1000);
#endif
}
