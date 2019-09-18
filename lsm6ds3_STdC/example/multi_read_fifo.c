/*
 ******************************************************************************
 * @file    multi_read_fifo.c
 * @author  Sensors Software Solution Team
 * @brief   This file show a little bit more complete way to get data
 *          from sensor by using FIFO.
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
#include <lsm6ds3_reg.h>
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

/* Private macro -------------------------------------------------------------*/
#define MIN_ODR(x, y) 				(x < y ? x : y)
#define MAX_ODR(x, y) 				(x > y ? x : y)
#define MAX_PATTERN_NUM				FIFO_THRESHOLD / 6
#define LSM6DS3_ODR_LSB_TO_HZ(_odr)	(_odr ? (13 << (_odr - 1)) : 0)

/* Private types ---------------------------------------------------------*/
typedef struct {
  uint8_t enable;
  lsm6ds3_odr_xl_t odr;
  uint16_t odr_hz_val;
  lsm6ds3_xl_fs_t fs;
  uint8_t decimation;
  uint8_t samples_num_in_pattern;
} sensor_lsm6ds3_xl;

typedef struct {
  uint8_t enable;
  lsm6ds3_odr_g_t odr;
  uint16_t odr_hz_val;
  lsm6ds3_fs_g_t fs;
  uint8_t decimation;
  uint8_t samples_num_in_pattern;
} sensor_lsm6ds3_gy;

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

/*
 * 6ds3 Accelerometer test parameters
 */
static sensor_lsm6ds3_xl test_6ds3_xl = {
  PROPERTY_ENABLE,
  LSM6DS3_XL_ODR_52Hz,
  0,
  LSM6DS3_2g,
  0,
  0,
};

/*
 * 6ds3 Gyroscope test parameters
 */
static sensor_lsm6ds3_gy test_6ds3_gyro = {
  PROPERTY_ENABLE,
  LSM6DS3_GY_ODR_26Hz,
  0,
  LSM6DS3_2000dps,
  0,
  0,
};

static uint16_t pattern_len;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static stmdev_ctx_t dev_ctx;

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

/*
 * Following routine read a pattern from FIFO
 */
static void LSM6DS3_Read_FIFO_Pattern(void)
{
  uint8_t gy_num = test_6ds3_gyro.samples_num_in_pattern;
  uint8_t xl_num = test_6ds3_xl.samples_num_in_pattern;

  /*
   * FIFO pattern is composed by gy_num gyroscope triplets and
   * xl_num accelerometer triplets. The sequence has always following order:
   * gyro first, accelerometer second
   */
  while(gy_num > 0 || xl_num > 0)
  {
    /*
     * Read gyro samples
     */
    if (test_6ds3_gyro.enable && gy_num > 0)
    {
      lsm6ds3_fifo_raw_data_get(&dev_ctx, data_raw_angular_rate.u8bit,
                                3 * sizeof(int16_t));
      angular_rate_mdps[0] =
        lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
      angular_rate_mdps[1] =
        lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
      angular_rate_mdps[2] =
        lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

      sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
      gy_num--;
    }

    /*
     * Read XL samples
     */
    if (test_6ds3_xl.enable && xl_num > 0)
    {
      lsm6ds3_fifo_raw_data_get(&dev_ctx, data_raw_acceleration.u8bit,
                                3 * sizeof(int16_t));
      acceleration_mg[0] =
        lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] =
        lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] =
        lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);

      sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
      xl_num--;
    }
  }
}

/*
 * Callback to handle the XL and Gyro event
 *
 * Samples acquisition is triggered by FIFO threshold event
 */
static void LSM6DS3_ACC_GYRO_sample_Callback_fifo(void)
{
  uint16_t num = 0;
  uint16_t num_pattern = 0;

  /*
   * Read number of word in FIFO
   */
  lsm6ds3_fifo_data_level_get(&dev_ctx, &num);
  num_pattern = num / pattern_len;

  while (num_pattern-- > 0)
    LSM6DS3_Read_FIFO_Pattern();
}

/*
 * Following routine calculate the FIFO pattern composition based
 * on gyro and acc enable state and ODR freq
 */
static uint16_t LSM6DS3_Calculate_FIFO_Pattern(uint16_t *min_odr, uint16_t *max_odr)
{
  uint16_t fifo_samples_tot_num = 0;

  /*
   * Calculate min_odr and max_odr for current configuration
   */
  if (test_6ds3_gyro.enable)
  {
    test_6ds3_gyro.odr_hz_val = LSM6DS3_ODR_LSB_TO_HZ(test_6ds3_gyro.odr);
    *max_odr = MAX_ODR(*max_odr, test_6ds3_gyro.odr_hz_val);
    *min_odr = MIN_ODR(*min_odr, test_6ds3_gyro.odr_hz_val);
  }

  if (test_6ds3_xl.enable)
  {
    test_6ds3_xl.odr_hz_val = LSM6DS3_ODR_LSB_TO_HZ(test_6ds3_xl.odr);
    *max_odr = MAX_ODR(*max_odr, test_6ds3_xl.odr_hz_val);
    *min_odr = MIN_ODR(*min_odr, test_6ds3_xl.odr_hz_val);
  }

  /*
   * Calculate how many samples for each sensor are in current FIFO pattern
   */
  if (test_6ds3_gyro.enable)
  {
    test_6ds3_gyro.samples_num_in_pattern = test_6ds3_gyro.odr_hz_val / *min_odr;
    test_6ds3_gyro.decimation =  *max_odr / test_6ds3_gyro.odr_hz_val;
    fifo_samples_tot_num += test_6ds3_gyro.samples_num_in_pattern;
  }

  if (test_6ds3_xl.enable)
  {
    test_6ds3_xl.samples_num_in_pattern = test_6ds3_xl.odr_hz_val / *min_odr;
    test_6ds3_xl.decimation =  *max_odr / test_6ds3_xl.odr_hz_val;
    fifo_samples_tot_num += test_6ds3_xl.samples_num_in_pattern;
  }

  /*
   * Return the total number of 16-bit samples in the pattern
   */
  return(6 * fifo_samples_tot_num);
}

/* Main Example --------------------------------------------------------------*/
void example_main_fifo_lsm6ds3(void)
{
  uint16_t max_odr = 0, min_odr = 0xffff;

  /*
   * Interrupt generation on FIFO watermark INT1/INT2 pin
   */
  //lsm6ds3_int2_route_t int_2_reg;
  //lsm6ds3_int1_route_t int_1_reg;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /*
   * Initialize platform specific hardware
   */
  platform_init();

  /*
   * Check device ID
   */
  lsm6ds3_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LSM6DS3_ID)
    while(1)
    {
      /* manage here device not found */
    }

  /*
   * Restore default configuration
   */
  lsm6ds3_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
	  lsm6ds3_reset_get(&dev_ctx, &rst);
  } while (rst);

  /*
   * Set XL and Gyro Output Data Rate
   */
  lsm6ds3_xl_data_rate_set(&dev_ctx, test_6ds3_xl.odr);
  lsm6ds3_gy_data_rate_set(&dev_ctx, test_6ds3_gyro.odr);

  /*
   * Set XL full scale and Gyro full scale
   */
  lsm6ds3_xl_full_scale_set(&dev_ctx, test_6ds3_xl.fs);
  lsm6ds3_gy_full_scale_set(&dev_ctx, test_6ds3_gyro.fs);

  /*
   * Calculate number of sensors samples in each FIFO pattern
   */
  pattern_len = LSM6DS3_Calculate_FIFO_Pattern(&min_odr, &max_odr);

  /*
   * Set FIFO watermark to a multiple of a pattern
   */
  lsm6ds3_fifo_watermark_set(&dev_ctx, pattern_len);

  /*
   * Set FIFO mode to Stream mode
   */
  lsm6ds3_fifo_mode_set(&dev_ctx, LSM6DS3_STREAM_MODE);

  /*
   * Enable FIFO watermark interrupt generation on INT1 pin
   */
  //lsm6ds3_pin_int1_route_get(&dev_ctx, &int_1_reg);
  //int_1_reg.int1_fth = PROPERTY_ENABLE;
  //lsm6ds3_pin_int1_route_set(&dev_ctx, &int_1_reg);

  /*
   * FIFO watermark interrupt on INT2 pin
   */
  //lsm6ds3_pin_int2_route_get(&dev_ctx, &int_2_reg);
  //int_2_reg.int2_fth = PROPERTY_ENABLE;
  //lsm6ds3_pin_int2_route_set(&dev_ctx, &int_2_reg);

  /*
   * Set ODR FIFO
   */
  lsm6ds3_fifo_data_rate_set(&dev_ctx, LSM6DS3_FIFO_416Hz);

  /*
   * Set FIFO sensor decimator
   */
  lsm6ds3_fifo_xl_batch_set(&dev_ctx, (lsm6ds3_dec_fifo_xl_t)test_6ds3_xl.decimation);
  lsm6ds3_fifo_gy_batch_set(&dev_ctx, (lsm6ds3_dec_fifo_gyro_t)test_6ds3_gyro.decimation);

  while(1)
  {
    uint8_t wt;

    /*
     * Read FIFO watermark flag in polling mode
     */
    lsm6ds3_fifo_wtm_flag_get(&dev_ctx, &wt);
    if (wt)
    	LSM6DS3_ACC_GYRO_sample_Callback_fifo();
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
    HAL_I2C_Mem_Write(handle, LSM6DS3_I2C_ADD_L, reg,
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
    HAL_I2C_Mem_Read(handle, LSM6DS3_I2C_ADD_L, reg,
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
  CDC_Transmit_FS(tx_buffer, len);
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
