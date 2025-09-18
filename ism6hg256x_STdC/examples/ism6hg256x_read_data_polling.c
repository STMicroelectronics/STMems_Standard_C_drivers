/*
 ******************************************************************************
 * @file    read_data_polling.c
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
#include "ism6hg256x_reg.h"

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
#define    CNT_FOR_OUTPUT       100

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_motion[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
static float_t temperature_degC;
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static ism6hg256x_filt_settling_mask_t filt_settling_mask;

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
void ism6hg256x_read_data_polling(void)
{
  ism6hg256x_reset_t rst;
  stmdev_ctx_t dev_ctx;
  double_t lowg_xl_sum[3], hg_xl_sum[3], gyro_sum[3], temp_sum;
  uint16_t lowg_xl_cnt = 0, hg_xl_cnt = 0, gyro_cnt = 0, temp_cnt = 0;

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
  ism6hg256x_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != ISM6HG256X_ID)
    while (1);

  /* Restore default configuration */
  ism6hg256x_reset_set(&dev_ctx, ISM6HG256X_RESTORE_CTRL_REGS);
  do {
    ism6hg256x_reset_get(&dev_ctx, &rst);
  } while (rst != ISM6HG256X_READY);

  /* Enable Block Data Update */
  ism6hg256x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  ism6hg256x_xl_data_rate_set(&dev_ctx, ISM6HG256X_ODR_AT_60Hz);
  ism6hg256x_hg_xl_data_rate_set(&dev_ctx, ISM6HG256X_HG_XL_ODR_AT_960Hz, 1);
  ism6hg256x_gy_data_rate_set(&dev_ctx, ISM6HG256X_ODR_AT_120Hz);

  /* Set full scale */
  ism6hg256x_xl_full_scale_set(&dev_ctx, ISM6HG256X_2g);
  ism6hg256x_hg_xl_full_scale_set(&dev_ctx, ISM6HG256X_256g);
  ism6hg256x_gy_full_scale_set(&dev_ctx, ISM6HG256X_2000dps);

  /* Configure filtering chain */
  filt_settling_mask.drdy = PROPERTY_ENABLE;
  filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  filt_settling_mask.irq_g = PROPERTY_ENABLE;
  ism6hg256x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
  ism6hg256x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
  ism6hg256x_filt_gy_lp1_bandwidth_set(&dev_ctx, ISM6HG256X_GY_ULTRA_LIGHT);
  ism6hg256x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
  ism6hg256x_filt_xl_lp2_bandwidth_set(&dev_ctx, ISM6HG256X_XL_STRONG);

  lowg_xl_sum[0] = lowg_xl_sum[1] = lowg_xl_sum[2] = 0.0;
  hg_xl_sum[0] = hg_xl_sum[1] = hg_xl_sum[2] = 0.0;
  gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0;
  temp_sum = 0.0;

  /* Read samples in polling mode (no int) */
  while (1) {
    ism6hg256x_data_ready_t drdy;

    /* Read output only if new xl value is available */
    ism6hg256x_flag_data_ready_get(&dev_ctx, &drdy);

    if (drdy.drdy_xl) {
      /* Read acceleration field data */
      memset(data_raw_motion, 0x00, 3 * sizeof(int16_t));
      ism6hg256x_acceleration_raw_get(&dev_ctx, data_raw_motion);
      acceleration_mg[0] = ism6hg256x_from_fs2_to_mg(data_raw_motion[0]);
      acceleration_mg[1] = ism6hg256x_from_fs2_to_mg(data_raw_motion[1]);
      acceleration_mg[2] = ism6hg256x_from_fs2_to_mg(data_raw_motion[2]);

      lowg_xl_sum[0] += acceleration_mg[0];
      lowg_xl_sum[1] += acceleration_mg[1];
      lowg_xl_sum[2] += acceleration_mg[2];
      lowg_xl_cnt++;
    }

    if (drdy.drdy_hgxl) {
      /* Read acceleration field data */
      memset(data_raw_motion, 0x00, 3 * sizeof(int16_t));
      ism6hg256x_hg_acceleration_raw_get(&dev_ctx, data_raw_motion);
      acceleration_mg[0] = ism6hg256x_from_fs256_to_mg(data_raw_motion[0]);
      acceleration_mg[1] = ism6hg256x_from_fs256_to_mg(data_raw_motion[1]);
      acceleration_mg[2] = ism6hg256x_from_fs256_to_mg(data_raw_motion[2]);

      hg_xl_sum[0] += acceleration_mg[0];
      hg_xl_sum[1] += acceleration_mg[1];
      hg_xl_sum[2] += acceleration_mg[2];
      hg_xl_cnt++;
    }

    /* Read output only if new xl value is available */
    if (drdy.drdy_gy) {
      /* Read angular rate field data */
      memset(data_raw_motion, 0x00, 3 * sizeof(int16_t));
      ism6hg256x_angular_rate_raw_get(&dev_ctx, data_raw_motion);
      angular_rate_mdps[0] = ism6hg256x_from_fs2000_to_mdps(data_raw_motion[0]);
      angular_rate_mdps[1] = ism6hg256x_from_fs2000_to_mdps(data_raw_motion[1]);
      angular_rate_mdps[2] = ism6hg256x_from_fs2000_to_mdps(data_raw_motion[2]);

      gyro_sum[0] += angular_rate_mdps[0];
      gyro_sum[1] += angular_rate_mdps[1];
      gyro_sum[2] += angular_rate_mdps[2];
      gyro_cnt++;
    }

    if (drdy.drdy_temp) {
      /* Read temperature data */
      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
      ism6hg256x_temperature_raw_get(&dev_ctx, &data_raw_temperature);
      temperature_degC = ism6hg256x_from_lsb_to_celsius(data_raw_temperature);
      temp_sum += temperature_degC;
      temp_cnt++;
    }

    if (lowg_xl_cnt >= CNT_FOR_OUTPUT) {
      /* print media low-g xl data */
      acceleration_mg[0] = lowg_xl_sum[0] / lowg_xl_cnt;
      acceleration_mg[1] = lowg_xl_sum[1] / lowg_xl_cnt;
      acceleration_mg[2] = lowg_xl_sum[2] / lowg_xl_cnt;

      snprintf((char *)tx_buffer, sizeof(tx_buffer), "lg xl (media of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              lowg_xl_cnt, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
      lowg_xl_sum[0] = lowg_xl_sum[1] = lowg_xl_sum[2] = 0.0;
      lowg_xl_cnt = 0;

      /* print media high-g xl data */
      acceleration_mg[0] = hg_xl_sum[0] / hg_xl_cnt;
      acceleration_mg[1] = hg_xl_sum[1] / hg_xl_cnt;
      acceleration_mg[2] = hg_xl_sum[2] / hg_xl_cnt;

      snprintf((char *)tx_buffer, sizeof(tx_buffer), "hg xl (media of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              hg_xl_cnt, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
      hg_xl_sum[0] = hg_xl_sum[1] = hg_xl_sum[2] = 0.0;
      hg_xl_cnt = 0;

      /* print media gyro data */
      angular_rate_mdps[0] = gyro_sum[0] / gyro_cnt;
      angular_rate_mdps[1] = gyro_sum[1] / gyro_cnt;
      angular_rate_mdps[2] = gyro_sum[2] / gyro_cnt;

      snprintf((char *)tx_buffer, sizeof(tx_buffer), "gyro (media of %d samples) [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              gyro_cnt, angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
      gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0;
      gyro_cnt = 0;

      /* print media temperature data */
      temperature_degC = temp_sum / temp_cnt;
      snprintf((char *)tx_buffer, sizeof(tx_buffer),"Temperature (media of %d samples) [degC]:%6.2f\r\n\r\n",
              temp_cnt, temperature_degC);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
      temp_cnt = 0;
      temp_sum = 0.0;
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
  HAL_I2C_Mem_Write(handle, ISM6HG256X_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  ISM6HG256X_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, ISM6HG256X_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, ISM6HG256X_I2C_ADD_L & 0xFE, reg, bufp, len);
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
