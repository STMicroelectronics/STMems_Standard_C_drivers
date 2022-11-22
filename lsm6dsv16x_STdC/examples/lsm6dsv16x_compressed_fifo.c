/*
 ******************************************************************************
 * @file    compressed_fifo.c
 * @author  Sensors Software Solution Team
 * @brief   This file how to configure compressed FIFO and to retrieve acc
 *          and gyro data. This sample use a fifo utility library tool
 *          for FIFO decompression.
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
#include "lsm6dsv16x_reg.h"
#include "st_fifo.h"

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
/*
 * Select FIFO samples watermark, max value is 512
 * in FIFO are stored acc, gyro and timestamp samples
 */
#define BOOT_TIME         10
#define FIFO_WATERMARK    32
#define FIFO_COMPRESSION   3
#define SLOT_NUMBER      (FIFO_WATERMARK * FIFO_COMPRESSION)

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
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
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lsm6dsv16x_compressed_fifo(void)
{
  lsm6dsv16x_fifo_status_t fifo_status;
  uint16_t out_slot_size;
  stmdev_ctx_t dev_ctx;
  st_fifo_conf conf;
  lsm6dsv16x_reset_t rst;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  lsm6dsv16x_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSV16X_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsv16x_reset_set(&dev_ctx, LSM6DSV16X_RESTORE_CTRL_REGS);
  do {
    lsm6dsv16x_reset_get(&dev_ctx, &rst);
  } while (rst != LSM6DSV16X_READY);

  /* Init utility for FIFO decompression */
  conf.device = ST_FIFO_LSM6DSV16X;
  conf.bdr_xl = 15.0f;
  conf.bdr_gy = 15.0f;
  conf.bdr_vsens = 0;

  st_fifo_init(&conf);

  /* Enable Block Data Update */
  lsm6dsv16x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_2g);
  lsm6dsv16x_gy_full_scale_set(&dev_ctx, LSM6DSV16X_2000dps);
  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to FIFO_WATERMARK samples
   */
  lsm6dsv16x_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);
  /* Set FIFO batch XL/Gyro ODR to 15Hz */
  lsm6dsv16x_fifo_xl_batch_set(&dev_ctx, LSM6DSV16X_XL_BATCHED_AT_15Hz);
  lsm6dsv16x_fifo_gy_batch_set(&dev_ctx, LSM6DSV16X_GY_BATCHED_AT_15Hz);
  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dsv16x_fifo_mode_set(&dev_ctx, LSM6DSV16X_STREAM_MODE);
  /* Enable FIFO compression on all samples */
  lsm6dsv16x_fifo_compress_algo_set(&dev_ctx, LSM6DSV16X_CMP_8_TO_1);
  lsm6dsv16x_fifo_compress_algo_real_time_set(&dev_ctx, 1);

  /* Set Output Data Rate */
  lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_15Hz);
  lsm6dsv16x_gy_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_15Hz);
  lsm6dsv16x_fifo_timestamp_batch_set(&dev_ctx, LSM6DSV16X_TMSTMP_DEC_8);
  lsm6dsv16x_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

  /* Wait samples */
  while (1) {
    uint16_t num = 0;
    uint16_t slots = 0;
    uint16_t acc_samples;
    uint16_t gyr_samples;
    /* Read watermark flag */
    lsm6dsv16x_fifo_status_get(&dev_ctx, &fifo_status);

    if (fifo_status.fifo_th == 1) {
      num = fifo_status.fifo_level;

      sprintf((char *)tx_buffer, "-- FIFO num %d \r\n", num);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));

      while (num--) {
        lsm6dsv16x_fifo_out_raw_t f_data;
        /* Read FIFO sensor value */
        lsm6dsv16x_fifo_out_raw_get(&dev_ctx, &f_data);
        raw_slot[slots].fifo_data_out[0] = (f_data.tag << 3) | (f_data.cnt << 1);
        memcpy(&raw_slot[slots].fifo_data_out[1], f_data.data, 6);

        slots++;
      }

      /* Uncompress FIFO samples and filter based on sensor type */
      st_fifo_decode(out_slot, raw_slot, &out_slot_size, slots);

      st_fifo_sort(out_slot, out_slot_size);
      acc_samples = st_fifo_get_sensor_occurrence(out_slot,
                                                  out_slot_size,
                                                  ST_FIFO_ACCELEROMETER);
      gyr_samples = st_fifo_get_sensor_occurrence(out_slot,
                                                  out_slot_size,
                                                  ST_FIFO_GYROSCOPE);
      /* Count how many acc and gyro samples */
      st_fifo_extract_sensor(acc_slot, out_slot, out_slot_size,
                             ST_FIFO_ACCELEROMETER);
      st_fifo_extract_sensor(gyr_slot, out_slot, out_slot_size,
                             ST_FIFO_GYROSCOPE);

     for (int i = 0; i < acc_samples; i++) {
        sprintf((char *)tx_buffer, "ACC:\t%u\t%d\t%4.2f\t%4.2f\t%4.2f\r\n",
                (unsigned int)acc_slot[i].timestamp,
                acc_slot[i].sensor_tag,
                lsm6dsv16x_from_fs2_to_mg(acc_slot[i].sensor_data.x),
                lsm6dsv16x_from_fs2_to_mg(acc_slot[i].sensor_data.y),
                lsm6dsv16x_from_fs2_to_mg(acc_slot[i].sensor_data.z));
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
      }

      for (int i = 0; i < gyr_samples; i++) {
        sprintf((char *)tx_buffer, "GYR:\t%u\t%d\t%4.2f\t%4.2f\t%4.2f\r\n",
                (unsigned int)gyr_slot[i].timestamp,
                gyr_slot[i].sensor_tag,
                lsm6dsv16x_from_fs2000_to_mdps(gyr_slot[i].sensor_data.x),
                lsm6dsv16x_from_fs2000_to_mdps(gyr_slot[i].sensor_data.y),
                lsm6dsv16x_from_fs2000_to_mdps(gyr_slot[i].sensor_data.z));
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
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
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
#if defined(NUCLEO_F411RE)
  HAL_I2C_Mem_Write(handle, LSM6DSV16X_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSV16X_I2C_ADD_H & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LSM6DSV16X_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LSM6DSV16X_I2C_ADD_H & 0xFE, reg, bufp, len);
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
