/*
 ******************************************************************************
 * @file    read_fifo.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to get data from sensor.
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
#include "lsm6dsv320x_reg.h"

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
#define    FIFO_WATERMARK       128
#define    CNT_FOR_OUTPUT       100

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static int16_t *datax;
static int16_t *datay;
static int16_t *dataz;
static int32_t *ts;
static lsm6dsv320x_filt_settling_mask_t filt_settling_mask;

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

static   stmdev_ctx_t dev_ctx;
static double_t lowg_xl_sum[3], hg_xl_sum[3], gyro_sum[3];
static uint16_t lowg_xl_cnt = 0, hg_xl_cnt = 0, gyro_cnt = 0;

static   uint8_t fifo_thread_run = 0;

static void lsm6dsv320x_fifo_thread(void)
{
  float_t acceleration_mg[3];
  float_t angular_rate_mdps[3];

  if (fifo_thread_run) {
    lsm6dsv320x_fifo_status_t fifo_status;
    uint16_t num;

    fifo_thread_run = 0;

    /* Read watermark flag */
    lsm6dsv320x_fifo_status_get(&dev_ctx, &fifo_status);

    num = fifo_status.fifo_level;

    while (num--) {
      lsm6dsv320x_fifo_out_raw_t f_data;

      /* Read FIFO sensor value */
      lsm6dsv320x_fifo_out_raw_get(&dev_ctx, &f_data);
      datax = (int16_t *)&f_data.data[0];
      datay = (int16_t *)&f_data.data[2];
      dataz = (int16_t *)&f_data.data[4];
      ts = (int32_t *)&f_data.data[0];

      switch (f_data.tag) {
      case LSM6DSV320X_XL_NC_TAG:
        /* Read acceleration field data */
        acceleration_mg[0] = lsm6dsv320x_from_fs2_to_mg(*datax);
        acceleration_mg[1] = lsm6dsv320x_from_fs2_to_mg(*datay);
        acceleration_mg[2] = lsm6dsv320x_from_fs2_to_mg(*dataz);

        lowg_xl_sum[0] += acceleration_mg[0];
        lowg_xl_sum[1] += acceleration_mg[1];
        lowg_xl_sum[2] += acceleration_mg[2];
        lowg_xl_cnt++;
        break;

      case LSM6DSV320X_XL_HG_TAG:
        acceleration_mg[0] = lsm6dsv320x_from_fs256_to_mg(*datax);
        acceleration_mg[1] = lsm6dsv320x_from_fs256_to_mg(*datay);
        acceleration_mg[2] = lsm6dsv320x_from_fs256_to_mg(*dataz);

        hg_xl_sum[0] += acceleration_mg[0];
        hg_xl_sum[1] += acceleration_mg[1];
        hg_xl_sum[2] += acceleration_mg[2];
        hg_xl_cnt++;
        break;

      case LSM6DSV320X_GY_NC_TAG:
        angular_rate_mdps[0] = lsm6dsv320x_from_fs2000_to_mdps(*datax);
        angular_rate_mdps[1] = lsm6dsv320x_from_fs2000_to_mdps(*datay);
        angular_rate_mdps[2] = lsm6dsv320x_from_fs2000_to_mdps(*dataz);

        gyro_sum[0] += angular_rate_mdps[0];
        gyro_sum[1] += angular_rate_mdps[1];
        gyro_sum[2] += angular_rate_mdps[2];
        gyro_cnt++;
        break;

      case LSM6DSV320X_TIMESTAMP_TAG:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "TIMESTAMP [ms] %d\r\n", *ts);
        tx_com(tx_buffer, strlen((char const *)tx_buffer));

        /* print media low-g xl data */
        if (lowg_xl_cnt > 0) {
          acceleration_mg[0] = lowg_xl_sum[0] / lowg_xl_cnt;
          acceleration_mg[1] = lowg_xl_sum[1] / lowg_xl_cnt;
          acceleration_mg[2] = lowg_xl_sum[2] / lowg_xl_cnt;

          snprintf((char *)tx_buffer, sizeof(tx_buffer), "lg xl (media of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                  lowg_xl_cnt, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          lowg_xl_sum[0] = lowg_xl_sum[1] = lowg_xl_sum[2] = 0.0;
          lowg_xl_cnt = 0;
        }

        /* print media high-g xl data */
        if (hg_xl_cnt > 0) {
          acceleration_mg[0] = hg_xl_sum[0] / hg_xl_cnt;
          acceleration_mg[1] = hg_xl_sum[1] / hg_xl_cnt;
          acceleration_mg[2] = hg_xl_sum[2] / hg_xl_cnt;

          snprintf((char *)tx_buffer, sizeof(tx_buffer), "hg xl (media of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                  hg_xl_cnt, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          hg_xl_sum[0] = hg_xl_sum[1] = hg_xl_sum[2] = 0.0;
          hg_xl_cnt = 0;
        }

        /* print media gyro data */
        if (gyro_cnt > 0) {
          angular_rate_mdps[0] = gyro_sum[0] / gyro_cnt;
          angular_rate_mdps[1] = gyro_sum[1] / gyro_cnt;
          angular_rate_mdps[2] = gyro_sum[2] / gyro_cnt;

          snprintf((char *)tx_buffer, sizeof(tx_buffer), "gyro (media of %d samples) [mdps]:%4.2f\t%4.2f\t%4.2f\r\n\r\n",
                  gyro_cnt, angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0;
          gyro_cnt = 0;
        }
        break;

      default:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "[%02x] UNHANDLED TAG \r\n", f_data.tag);
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      }
    }
  }
}

void lsm6dsv320x_read_fifo_handler(void)
{
  fifo_thread_run = 1;
}

/* Main Example --------------------------------------------------------------*/
void lsm6dsv320x_read_fifo(void)
{
  lsm6dsv320x_reset_t rst;
  lsm6dsv320x_pin_int_route_t pin_int = { 0 };

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
  lsm6dsv320x_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSV320X_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsv320x_reset_set(&dev_ctx, LSM6DSV320X_RESTORE_CTRL_REGS);
  do {
    lsm6dsv320x_reset_get(&dev_ctx, &rst);
  } while (rst != LSM6DSV320X_READY);

  /* Enable Block Data Update */
  lsm6dsv320x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to FIFO_WATERMARK samples
   */
  lsm6dsv320x_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);
  /* Set FIFO batch XL/Gyro ODR */
  lsm6dsv320x_fifo_xl_batch_set(&dev_ctx, LSM6DSV320X_XL_BATCHED_AT_60Hz);
  lsm6dsv320x_fifo_hg_xl_batch_set(&dev_ctx, 1);
  lsm6dsv320x_fifo_gy_batch_set(&dev_ctx, LSM6DSV320X_GY_BATCHED_AT_120Hz);
  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dsv320x_fifo_mode_set(&dev_ctx, LSM6DSV320X_STREAM_MODE);
  lsm6dsv320x_fifo_timestamp_batch_set(&dev_ctx, LSM6DSV320X_TMSTMP_DEC_32);
  lsm6dsv320x_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  lsm6dsv320x_xl_data_rate_set(&dev_ctx, LSM6DSV320X_ODR_AT_60Hz);
  lsm6dsv320x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV320X_HG_XL_ODR_AT_480Hz, 0);
  lsm6dsv320x_gy_data_rate_set(&dev_ctx, LSM6DSV320X_ODR_AT_120Hz);

  /* Set full scale */
  lsm6dsv320x_xl_full_scale_set(&dev_ctx, LSM6DSV320X_2g);
  lsm6dsv320x_hg_xl_full_scale_set(&dev_ctx, LSM6DSV320X_320g);
  lsm6dsv320x_gy_full_scale_set(&dev_ctx, LSM6DSV320X_2000dps);

  /* Configure filtering chain */
  filt_settling_mask.drdy = PROPERTY_ENABLE;
  filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  filt_settling_mask.irq_g = PROPERTY_ENABLE;
  lsm6dsv320x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
  lsm6dsv320x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv320x_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV320X_GY_ULTRA_LIGHT);
  lsm6dsv320x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv320x_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV320X_XL_STRONG);

  /* enable fifo_th on High-G XL (sensor at highest frequency) */
  pin_int.fifo_th = PROPERTY_ENABLE;
  lsm6dsv320x_pin_int1_route_set(&dev_ctx, &pin_int);
  //lsm6dsv320x_pin_int2_route_set(&dev_ctx, &pin_int);

  /* Read samples in polling mode (no int) */
  while (1) {
    lsm6dsv320x_fifo_thread();
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
  HAL_I2C_Mem_Write(handle, LSM6DSV320X_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSV320X_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LSM6DSV320X_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LSM6DSV320X_I2C_ADD_L & 0xFE, reg, bufp, len);
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
