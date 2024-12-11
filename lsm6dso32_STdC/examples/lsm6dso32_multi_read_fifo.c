/*
 ******************************************************************************
 * @file    multi_read_fifo.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to get data from sensor FIFO.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI215V1
 * - NUCLEO_F401RE + STEVAL-MKI215V1
 * - DISCOVERY_SPC584B + STEVAL-MKI215V1
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
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
#include "lsm6dso32_reg.h"

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

typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union {
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME              10

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* WARNING:
 * Functions declare in this section are defined at the end of this file
 * and are strictly related to the hardware platform used.
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lsm6dso32_multi_read_fifo(void)
{
  stmdev_ctx_t dev_ctx;
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
  lsm6dso32_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSO32_ID)
    while (1);

  /* Restore default configuration */
  lsm6dso32_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dso32_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dso32_i3c_disable_set(&dev_ctx, LSM6DSO32_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dso32_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lsm6dso32_xl_full_scale_set(&dev_ctx, LSM6DSO32_4g);
  lsm6dso32_gy_full_scale_set(&dev_ctx, LSM6DSO32_2000dps);
  /* Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to 10 samples
   */
  lsm6dso32_fifo_watermark_set(&dev_ctx, 10);
  /* Set FIFO batch XL / Gyro ODR to 12.5Hz / 26Hz */
  lsm6dso32_fifo_xl_batch_set(&dev_ctx, LSM6DSO32_XL_BATCHED_AT_12Hz5);
  lsm6dso32_fifo_gy_batch_set(&dev_ctx, LSM6DSO32_GY_BATCHED_AT_26Hz);
  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dso32_fifo_mode_set(&dev_ctx, LSM6DSO32_STREAM_MODE);
  /* Set Output Data Rate */
  /* Set ODR (Output Data Rate) and power mode*/
  lsm6dso32_xl_data_rate_set(&dev_ctx, LSM6DSO32_XL_ODR_12Hz5_LOW_PW);
  lsm6dso32_gy_data_rate_set(&dev_ctx, LSM6DSO32_GY_ODR_26Hz_HIGH_PERF);

  /* Wait samples */
  while (1) {
    uint16_t num = 0;
    uint8_t wmflag = 0;
    lsm6dso32_fifo_tag_t reg_tag;
    axis3bit16_t dummy;
    /* Read watermark flag */
    lsm6dso32_fifo_wtm_flag_get(&dev_ctx, &wmflag);

    if (wmflag > 0) {
      /* Read number of samples in FIFO */
      lsm6dso32_fifo_data_level_get(&dev_ctx, &num);

      while (num--) {
        /* Read FIFO tag */
        lsm6dso32_fifo_sensor_tag_get(&dev_ctx, &reg_tag);

        switch (reg_tag) {
          case LSM6DSO32_XL_NC_TAG:
            memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dso32_fifo_out_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
            acceleration_mg[0] =
              lsm6dso32_from_fs4_to_mg(data_raw_acceleration.i16bit[0]);
            acceleration_mg[1] =
              lsm6dso32_from_fs4_to_mg(data_raw_acceleration.i16bit[1]);
            acceleration_mg[2] =
              lsm6dso32_from_fs4_to_mg(data_raw_acceleration.i16bit[2]);
            snprintf((char *)tx_buffer, sizeof(tx_buffer),
                    "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                    acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
            break;

          case LSM6DSO32_GYRO_NC_TAG:
            memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dso32_fifo_out_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
            angular_rate_mdps[0] =
              lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
            angular_rate_mdps[1] =
              lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
            angular_rate_mdps[2] =
              lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);
            snprintf((char *)tx_buffer, sizeof(tx_buffer),
                    "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                    angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
            break;

          default:
            /* Flush unused samples */
            memset(dummy.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dso32_fifo_out_raw_get(&dev_ctx, dummy.u8bit);
            break;
        }
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
#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Write(handle, LSM6DSO32_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSO32_I2C_ADD_H & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LSM6DSO32_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LSM6DSO32_I2C_ADD_H & 0xFE, reg, bufp, len);
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
