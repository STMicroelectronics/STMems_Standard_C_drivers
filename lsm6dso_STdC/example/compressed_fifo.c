/*
 ******************************************************************************
 * @file    compressed_fifo.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to configure compressed FIFO and
 *          to retrieve acc and gyro data. This sample use a fifo utility
 *          library tool for FIFO decompression.
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
#include <lsm6dso_reg.h>
#include <fifo_utility.h>
#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
#include "usart.h"
#endif

/* Private macro -------------------------------------------------------------*/
/*
 * Select FIFO samples watermark, max value is 512
 * in FIFO are stored acc, gyro and timestamp samples
 */
#define FIFO_WATERMARK    10
#define FIFO_COMPRESSION  3
#define SLOT_NUMBER      (FIFO_WATERMARK * FIFO_COMPRESSION)

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
static st_fifo_raw_slot raw_slot[SLOT_NUMBER];
static st_fifo_out_slot out_slot[SLOT_NUMBER];
static st_fifo_out_slot acc_slot[SLOT_NUMBER];
static st_fifo_out_slot gyr_slot[SLOT_NUMBER];

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

sensor_data_t sensor_data;

/* Main Example --------------------------------------------------------------*/
void example_compressed_fifo_simple_lsm6dso(void)
{
  stmdev_ctx_t dev_ctx;
  uint16_t out_slot_size;

  /* Uncomment to configure INT 1 */
  //lsm6dso_pin_int1_route_t int1_route;

  /* Uncomment to configure INT 2 */
  //lsm6dso_pin_int2_route_t int2_route;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /* Init test platform */
  platform_init();

  /* Init utility for FIFO decompression */
  st_fifo_init(0, 0, 0);

  /* Check device ID */
  lsm6dso_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LSM6DSO_ID)
    while(1);

  /* Restore default configuration */
  lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6dso_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);

  /* Enable Block Data Update */
  lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set full scale */
  lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
  lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps);

  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to FIFO_WATERMARK samples
   */
  lsm6dso_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);

  /* Set FIFO batch XL/Gyro ODR to 12.5Hz */
  lsm6dso_fifo_xl_batch_set(&dev_ctx, LSM6DSO_XL_BATCHED_AT_12Hz5);
  lsm6dso_fifo_gy_batch_set(&dev_ctx, LSM6DSO_GY_BATCHED_AT_12Hz5);

  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dso_fifo_mode_set(&dev_ctx, LSM6DSO_STREAM_MODE);

  /* Enable FIFO compression on all samples */
  lsm6dso_compression_algo_set(&dev_ctx, LSM6DSO_CMP_DISABLE);

  /* Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed */
  //lsm6dso_data_ready_mode_set(&dev_ctx, LSM6DSO_DRDY_PULSED);

  /*
   * FIFO watermark interrupt routed on INT1 pin
   * WARNING: INT1 pin is used by sensor to switch in I3C mode.
   */
  //lsm6dso_pin_int1_route_get(&dev_ctx, &int1_route);
  //int1_route.reg.int1_ctrl.int1_fifo_th = PROPERTY_ENABLE;
  //lsm6dso_pin_int1_route_set(&dev_ctx, &int1_route);

  /* FIFO watermark interrupt routed on INT2 pin */
  //lsm6dso_pin_int2_route_get(&dev_ctx, &int2_route);
  //int2_route.reg.int2_ctrl.int2_fifo_th = PROPERTY_ENABLE;
  //lsm6dso_pin_int2_route_set(&dev_ctx, &int2_route);

  /* Set Output Data Rate */
  lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_12Hz5);
  lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_12Hz5);
  lsm6dso_fifo_timestamp_decimation_set(&dev_ctx, LSM6DSO_DEC_1);
  lsm6dso_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

  /* Wait samples */
  while(1)
  {
    uint16_t num = 0;
    uint8_t wmflag = 0;
    uint16_t slots = 0;
    uint16_t acc_samples;
    uint16_t gyr_samples;

  /* Read watermark flag */
    lsm6dso_fifo_wtm_flag_get(&dev_ctx, &wmflag);
    if (wmflag > 0)
    {
      /* Read number of samples in FIFO */
      lsm6dso_fifo_data_level_get(&dev_ctx, &num);
      while(num--)
      {
        /*
         * Read FIFO sensor tag
         *
         * To reorder data samples in FIFO is needed the register
         * LSM6DSO_FIFO_DATA_OUT_TAG, including tag counter and parity.
         */
        lsm6dso_read_reg(&dev_ctx, LSM6DSO_FIFO_DATA_OUT_TAG,
                         (uint8_t *)&raw_slot[slots].fifo_data_out[0], 1);

        /* Read FIFO sensor value */
        lsm6dso_fifo_out_raw_get(&dev_ctx, &raw_slot[slots].fifo_data_out[1]);
        slots++;
      }

      /* Uncompress FIFO samples and filter based on sensor type */
      st_fifo_decompress(out_slot, raw_slot, &out_slot_size, slots);
      st_fifo_sort(out_slot, out_slot_size);
      acc_samples = st_fifo_get_sensor_occurrence(out_slot,
                                                  out_slot_size,
                                                  ST_FIFO_ACCELEROMETER);
      gyr_samples = st_fifo_get_sensor_occurrence(out_slot,
                                                  out_slot_size,
                                                  ST_FIFO_GYROSCOPE);
      /* Count how many acc and gyro samples */
      st_fifo_extract_sensor(acc_slot, out_slot,out_slot_size,
                             ST_FIFO_ACCELEROMETER);
      st_fifo_extract_sensor(gyr_slot, out_slot, out_slot_size,
                             ST_FIFO_GYROSCOPE);

      for (int i = 0; i < acc_samples; i++)
      {
        memcpy( sensor_data.raw_data, acc_slot[i].raw_data, sizeof(sensor_data) );

        sprintf((char*)tx_buffer, "ACC:\t%u\t%d\t%4.2f\t%4.2f\t%4.2f\r\n",
                (unsigned int)acc_slot[i].timestamp,
                 acc_slot[i].sensor_tag,
                 lsm6dso_from_fs2_to_mg(sensor_data.data[0]),
                 lsm6dso_from_fs2_to_mg(sensor_data.data[1]),
                 lsm6dso_from_fs2_to_mg(sensor_data.data[2]));
        tx_com(tx_buffer, strlen((char const*)tx_buffer));
      }

      for (int i = 0; i < gyr_samples; i++)
      {
        memcpy( sensor_data.raw_data, gyr_slot[i].raw_data, sizeof(sensor_data) );

        sprintf((char*)tx_buffer, "GYR:\t%u\t%d\t%4.2f\t%4.2f\t%4.2f\r\n",
                (unsigned int)gyr_slot[i].timestamp,
                gyr_slot[i].sensor_tag,
                lsm6dso_from_fs2000_to_mdps(sensor_data.data[0]),
                lsm6dso_from_fs2000_to_mdps(sensor_data.data[1]),
                lsm6dso_from_fs2000_to_mdps(sensor_data.data[2]));
        tx_com(tx_buffer, strlen((char const*)tx_buffer));
      }

      slots = 0;
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
    HAL_I2C_Mem_Write(handle, LSM6DSO_I2C_ADD_H, reg,
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
    HAL_I2C_Mem_Read(handle, LSM6DSO_I2C_ADD_H, reg,
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
