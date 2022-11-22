/*
 ******************************************************************************
 * @file    sensor_fusion.c
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

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

/* Private variables ---------------------------------------------------------*/
static lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;

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

/*
 * Original conversion routines taken from: https://github.com/numpy/numpy
 *
 * uint32_t npy_halfbits_to_floatbits(uint16_t h);
 * float_t npy_half_to_float(uint16_t h);
 *
 * Released under BSD-3-Clause License
 */
static uint32_t npy_halfbits_to_floatbits(uint16_t h)
{
    uint16_t h_exp, h_sig;
    uint32_t f_sgn, f_exp, f_sig;

    h_exp = (h&0x7c00u);
    f_sgn = ((uint32_t)h&0x8000u) << 16;
    switch (h_exp) {
        case 0x0000u: /* 0 or subnormal */
            h_sig = (h&0x03ffu);
            /* Signed zero */
            if (h_sig == 0) {
                return f_sgn;
            }
            /* Subnormal */
            h_sig <<= 1;
            while ((h_sig&0x0400u) == 0) {
                h_sig <<= 1;
                h_exp++;
            }
            f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
            f_sig = ((uint32_t)(h_sig&0x03ffu)) << 13;
            return f_sgn + f_exp + f_sig;
        case 0x7c00u: /* inf or NaN */
            /* All-ones exponent and a copy of the significand */
            return f_sgn + 0x7f800000u + (((uint32_t)(h&0x03ffu)) << 13);
        default: /* normalized */
            /* Just need to adjust the exponent and shift */
            return f_sgn + (((uint32_t)(h&0x7fffu) + 0x1c000u) << 13);
    }
}

static float_t npy_half_to_float(uint16_t h)
{
    union { float_t ret; uint32_t retbits; } conv;
    conv.retbits = npy_halfbits_to_floatbits(h);
    return conv.ret;
}

static void sflp2q(float quat[4], uint16_t sflp[3])
{
  float sumsq = 0;

  quat[0] = npy_half_to_float(sflp[0]);
  quat[1] = npy_half_to_float(sflp[1]);
  quat[2] = npy_half_to_float(sflp[2]);

  for (uint8_t i = 0; i < 3; i++)
    sumsq += quat[i] * quat[i];

  if (sumsq > 1.0f)
    sumsq = 1.0f;

  quat[3] = sqrtf(1.0f - sumsq);
}

/* Main Example --------------------------------------------------------------*/
void lsm6dsv16x_sensor_fusion(void)
{
  lsm6dsv16x_fifo_status_t fifo_status;
  stmdev_ctx_t dev_ctx;
  lsm6dsv16x_reset_t rst;
  lsm6dsv16x_sflp_gbias_t gbias;

  /* Uncomment to configure INT 1 */
  //lsm6dsv16x_pin_int1_route_t int1_route;
  /* Uncomment to configure INT 2 */
  //lsm6dsv16x_pin_int2_route_t int2_route;
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
  lsm6dsv16x_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSV16X_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsv16x_reset_set(&dev_ctx, LSM6DSV16X_RESTORE_CTRL_REGS);
  do {
    lsm6dsv16x_reset_get(&dev_ctx, &rst);
  } while (rst != LSM6DSV16X_READY);

  /* Enable Block Data Update */
  lsm6dsv16x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_4g);
  lsm6dsv16x_gy_full_scale_set(&dev_ctx, LSM6DSV16X_2000dps);

  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to FIFO_WATERMARK samples
   */
  lsm6dsv16x_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);

  /* Set FIFO batch of sflp data */
  fifo_sflp.game_rotation = 1;
  fifo_sflp.gravity = 1;
  fifo_sflp.gbias = 1;
  lsm6dsv16x_fifo_sflp_batch_set(&dev_ctx, fifo_sflp);

  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dsv16x_fifo_mode_set(&dev_ctx, LSM6DSV16X_STREAM_MODE);

  /* Set Output Data Rate */
  lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_30Hz);
  lsm6dsv16x_gy_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_30Hz);
  lsm6dsv16x_sflp_data_rate_set(&dev_ctx, LSM6DSV16X_SFLP_30Hz);

  lsm6dsv16x_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * here application may initialize offset with latest values
   * calculated from previous run and saved to non volatile memory.
   */
  gbias.gbias_x = 0.0f;
  gbias.gbias_y = 0.0f;
  gbias.gbias_z = 0.0f;
  lsm6dsv16x_sflp_game_gbias_set(&dev_ctx, &gbias);

  /* Wait samples */
  while (1) {
    uint16_t num = 0;

    /* Read watermark flag */
    lsm6dsv16x_fifo_status_get(&dev_ctx, &fifo_status);

    if (fifo_status.fifo_th == 1) {
      num = fifo_status.fifo_level;

      sprintf((char *)tx_buffer, "-- FIFO num %d \r\n", num);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));

      while (num--) {
        lsm6dsv16x_fifo_out_raw_t f_data;
        int16_t *axis;
        float quat[4];
        float gravity_mg[3];
        float gbias_mdps[3];

        /* Read FIFO sensor value */
        lsm6dsv16x_fifo_out_raw_get(&dev_ctx, &f_data);

        switch (f_data.tag) {
        case LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG:
          axis = (int16_t *)&f_data.data[0];
          gbias_mdps[0] = lsm6dsv16x_from_fs125_to_mdps(axis[0]);
          gbias_mdps[1] = lsm6dsv16x_from_fs125_to_mdps(axis[1]);
          gbias_mdps[2] = lsm6dsv16x_from_fs125_to_mdps(axis[2]);
          sprintf((char *)tx_buffer, "GBIAS [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                         gbias_mdps[0], gbias_mdps[1], gbias_mdps[2]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          break;
        case LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG:
          axis = (int16_t *)&f_data.data[0];
          gravity_mg[0] = lsm6dsv16x_from_sflp_to_mg(axis[0]);
          gravity_mg[1] = lsm6dsv16x_from_sflp_to_mg(axis[1]);
          gravity_mg[2] = lsm6dsv16x_from_sflp_to_mg(axis[2]);
          sprintf((char *)tx_buffer, "Gravity [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                         gravity_mg[0], gravity_mg[1], gravity_mg[2]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          break;
        case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
          sflp2q(quat, (uint16_t *)&f_data.data[0]);
          sprintf((char *)tx_buffer, "Game Rotation \tX: %2.3f\tY: %2.3f\tZ: %2.3f\tW: %2.3f\r\n",
                  quat[0], quat[1], quat[2], quat[3]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          break;
        default:
         break;
        }
      }

      sprintf((char *)tx_buffer, "------ \r\n\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
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
