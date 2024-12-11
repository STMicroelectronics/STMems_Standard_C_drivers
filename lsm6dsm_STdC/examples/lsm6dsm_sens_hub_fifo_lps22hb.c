/*
 ******************************************************************************
 * @file    sensor_hub_fifo_lps22hb_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to read LPS22HB press.
 *          connected to LSM6DSM I2C master interface (with FIFO support).
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
 * - STEVAL_MKI109V3 + STEVAL-MKI189V1
 * - NUCLEO_F401RE + STEVAL-MKI189V1
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
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
#define NUCLEO_F401RE_X_NUCLEO_IKS01A2

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2

/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F401RE_X_NUCLEO_IKS01A2)
/* NUCLEO_F401RE_X_NUCLEO_IKS01A2: Define communication interface */
#define SENSOR_BUS hi2c1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lsm6dsm_reg.h"
#include "lps22hb_reg.h"


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

typedef union {
  int32_t i32bit;
  uint8_t u8bit[4];
} axis1bit32_t;

/* Private macro -------------------------------------------------------------*/
#define OUT_XYZ_SIZE    6
#define PRESS_OUT_XYZ_SIZE  4
#define TEMP_OUT_XYZ_SIZE  2

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI, rst;
static float_t pressure_hPa;
static float_t temperature_degC;
static float_t angular_rate_mdps[3];
static float_t acceleration_mg[3];
static axis1bit32_t data_raw_pressure;
static axis1bit16_t data_raw_temperature;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static stmdev_ctx_t dev_ctx;
static stmdev_ctx_t press_ctx;
static uint8_t tx_buffer[1000];

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
 * Read data byte from internal register of a slave device connected
 * to master I2C interface
 */
static int32_t lsm6dsm_read_lps22hb_cx(void *ctx, uint8_t reg,
                                       uint8_t *data, uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t mm_error;
  uint8_t drdy;
  lsm6dsm_func_src1_t func_src1;
  lsm6dsm_sh_cfg_read_t val = {
    .slv_add = LPS22HB_I2C_ADD_H,
    .slv_subadd = reg,
    .slv_len = len,
  };
  (void)ctx;
  /* Configure Sensor Hub to read LPS22HB */
  mm_error = lsm6dsm_sh_slv0_cfg_read(&dev_ctx, &val);
  lsm6dsm_sh_num_of_dev_connected_set(&dev_ctx, LSM6DSM_SLV_0_1);
  /* Enable I2C Master and I2C master Pull Up */
  lsm6dsm_func_en_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsm_sh_master_set(&dev_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_104Hz);
  /* Wait Sensor Hub operation flag set. */
  lsm6dsm_acceleration_raw_get(&dev_ctx, data_raw_acceleration.i16bit);

  do {
    lsm6dsm_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  do {
    lsm6dsm_read_reg(&dev_ctx, LSM6DSM_FUNC_SRC1, (uint8_t *)&func_src1,
                     1);
  } while (!func_src1.sensorhub_end_op);

  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_OFF);
  lsm6dsm_sh_read_data_raw_get(&dev_ctx,
                               (lsm6dsm_emb_sh_read_t *)&data);
  lsm6dsm_func_en_set(&dev_ctx, PROPERTY_DISABLE);
  lsm6dsm_sh_master_set(&dev_ctx, PROPERTY_DISABLE);
  return mm_error;
}

/*
 * Write data byte to internal register of a slave device connected
 * to master I2C interface
 */
static int32_t lsm6dsm_write_lps22hb_cx(void *ctx, uint8_t reg,
                                        const uint8_t *data, uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t mm_error;
  uint8_t drdy;
  lsm6dsm_func_src1_t func_src1;
  lsm6dsm_sh_cfg_write_t val = {
    .slv0_add = LPS22HB_I2C_ADD_H,
    .slv0_subadd = reg,
    .slv0_data = *data,
  };
  (void)ctx;
  (void)len;
  /* Disable accelerometer */
  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_OFF);
  /* Configure Sensor Hub to write */
  mm_error = lsm6dsm_sh_cfg_write(&dev_ctx, &val);
  /* Enable I2C Master and I2C master Pull Up */
  lsm6dsm_func_en_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsm_sh_master_set(&dev_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation */
  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_104Hz);
  /* Wait Sensor Hub operation flag set */
  lsm6dsm_acceleration_raw_get(&dev_ctx, data_raw_acceleration.i16bit);

  do {
    lsm6dsm_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  do {
    lsm6dsm_read_reg(&dev_ctx, LSM6DSM_FUNC_SRC1, (uint8_t *)&func_src1,
                     1);
  } while (!func_src1.sensorhub_end_op);

  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_OFF);
  lsm6dsm_func_en_set(&dev_ctx, PROPERTY_DISABLE);
  lsm6dsm_sh_master_set(&dev_ctx, PROPERTY_DISABLE);
  return mm_error;
}

/*
 * Configure LPS22HB barometer sensor over I2C master line (Slave1)
 *
 * Enable LPS22HB barometer:
 * set ODR to 50 Hz
 * enable BDU
 */
static void configure_lps22hb(stmdev_ctx_t *ctx)
{
  lsm6dsm_sh_cfg_read_t val = {
    .slv_add = LPS22HB_I2C_ADD_H,
    .slv_subadd = LPS22HB_STATUS,
    .slv_len = OUT_XYZ_SIZE,
  };
  lps22hb_data_rate_set(ctx, LPS22HB_ODR_50_Hz);
  lps22hb_block_data_update_set(ctx, PROPERTY_ENABLE);
  /* Prepare sensor hub to read data from external Slave1 */
  lsm6dsm_sh_slv0_cfg_read(&dev_ctx, &val);
}

/* Main Example --------------------------------------------------------------*/
void example_fifo_lps22hb_simple_lsm6dsm(void)
{
  uint16_t pattern_len;
  //lsm6dsm_int1_route_t int_1_reg;
  //lsm6dsm_int2_route_t int_2_reg;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  /* Configure low level function to access to external device */
  press_ctx.read_reg = lsm6dsm_read_lps22hb_cx;
  press_ctx.write_reg = lsm6dsm_write_lps22hb_cx;
  dev_ctx.handle = &SENSOR_BUS;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(15);
  /* Check device ID */
  lsm6dsm_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSM_ID)
    while (1) {
      /* manage here device not found */
    }

  /* Restore default configuration */
  lsm6dsm_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsm_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Some hardware require to enable pull up on master I2C interface */
  //lsm6dsm_sh_pin_mode_set(&dev_ctx, LSM6DSM_INTERNAL_PULL_UP);
  /* Check if LPS22HB connected to Sensor Hub. */
  lps22hb_device_id_get(&press_ctx, &whoamI);

  if (whoamI != LPS22HB_ID) {
    while (1) {
      /* manage here device not found */
    }
  }

  /* Configure LPS22HB on the I2C master line */
  configure_lps22hb(&press_ctx);
  /* Configure Sensor Hub to read one slave */
  lsm6dsm_sh_num_of_dev_connected_set(&dev_ctx, LSM6DSM_SLV_0);
  /* Set XL full scale and Gyro full scale */
  lsm6dsm_xl_full_scale_set(&dev_ctx, LSM6DSM_2g);
  lsm6dsm_gy_full_scale_set(&dev_ctx, LSM6DSM_2000dps);
  /* Enable Block Data Update (BDU) when FIFO support selected */
  lsm6dsm_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set FIFO watermark to a multiple of a pattern
   * in this example we set watermark to 10 pattern
   * which means ten sequence of:
   * (GYRO + XL + BARO_TEMP) = 12 bytes
   */
  pattern_len = 12;
  lsm6dsm_fifo_watermark_set(&dev_ctx, 10 * pattern_len);
  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dsm_fifo_mode_set(&dev_ctx, LSM6DSM_STREAM_MODE);
  /* Uncomment to enable FIFO watermark interrupt generation
   * on INT1 pin
   */
  //lsm6dsm_pin_int1_route_get(&dev_ctx, &int_1_reg);
  //int_1_reg.int1_fth = PROPERTY_ENABLE;
  //lsm6dsm_pin_int1_route_set(&dev_ctx, int_1_reg);
  /* Uncomment to enable FIFO watermark interrupt generation
   * on INT2 pin
   */
  //lsm6dsm_pin_int2_route_get(&dev_ctx, &int_2_reg);
  //int_2_reg.int2_fth = PROPERTY_ENABLE;
  //lsm6dsm_pin_int2_route_set(&dev_ctx, int_2_reg);
  /* Set FIFO sensor decimator. */
  lsm6dsm_fifo_xl_batch_set(&dev_ctx, LSM6DSM_FIFO_XL_NO_DEC);
  lsm6dsm_fifo_gy_batch_set(&dev_ctx, LSM6DSM_FIFO_GY_NO_DEC);
  lsm6dsm_fifo_dataset_3_batch_set(&dev_ctx, LSM6DSM_FIFO_DS3_NO_DEC);
  /* Enable master and XL trigger */
  lsm6dsm_func_en_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsm_sh_master_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set ODR FIFO */
  lsm6dsm_fifo_data_rate_set(&dev_ctx, LSM6DSM_FIFO_52Hz);
  /* Set XL and Gyro Output Data Rate:
   * in this example we set 52 Hz for Accelerometer and
   * 52 Hz for Gyroscope
   */
  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_52Hz);
  lsm6dsm_gy_data_rate_set(&dev_ctx, LSM6DSM_GY_ODR_52Hz);

  while (1) {
    uint16_t num = 0;
    uint16_t num_pattern = 0;
    uint8_t dummy[OUT_XYZ_SIZE];
    uint8_t waterm = 0;
    /* Read LSM6DSM watermark flag */
    lsm6dsm_fifo_wtm_flag_get(&dev_ctx, &waterm);

    if (waterm) {
      /* Read number of word in FIFO */
      lsm6dsm_fifo_data_level_get(&dev_ctx, &num);
      num_pattern = num / pattern_len;

      while (num_pattern-- > 0) {
        /* Following the sensors ODR configuration, FIFO pattern is composed
         * by this sequence of samples: GYRO, XL, BARO_TEMP
         */
        lsm6dsm_fifo_raw_data_get(&dev_ctx,
                                  data_raw_angular_rate.u8bit,
                                  OUT_XYZ_SIZE);
        angular_rate_mdps[0] =
          lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
        angular_rate_mdps[1] =
          lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
        angular_rate_mdps[2] =
          lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);
        snprintf((char *)tx_buffer, sizeof(tx_buffer),
                "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                angular_rate_mdps[0],
                angular_rate_mdps[1],
                angular_rate_mdps[2]);
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        lsm6dsm_fifo_raw_data_get(&dev_ctx,
                                  data_raw_acceleration.u8bit,
                                  OUT_XYZ_SIZE);
        acceleration_mg[0] =
          lsm6dsm_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
        acceleration_mg[1] =
          lsm6dsm_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
        acceleration_mg[2] =
          lsm6dsm_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);
        snprintf((char *)tx_buffer, sizeof(tx_buffer),
                "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                acceleration_mg[0],
                acceleration_mg[1],
                acceleration_mg[2]);
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        /* Barometer and Temperature sensor share the same slave because is
         * a combo sensor, we use a single FIFO slot to encapsulate both
         * samples data. One byte in FIFO left unused.
         */
        lsm6dsm_fifo_raw_data_get(&dev_ctx, dummy, OUT_XYZ_SIZE);
        memcpy(data_raw_pressure.u8bit, &dummy[0], PRESS_OUT_XYZ_SIZE);
        memcpy(data_raw_temperature.u8bit, &dummy[3], TEMP_OUT_XYZ_SIZE);
        /* Please note: the conversion function lps22hb_from_lsb_to_hpa
         * work on uint32_t format left-aligned (not 24 bit), so an
         * additional 8-bit shift is required (4096.0f * 256 = 1048576.0f
         * sensitivity apply by the function).
         * In this case status register is in the lower position and
         * is used to perform the 8 bit shift.
         * */
        data_raw_pressure.u8bit[0] = 0x00; // set to zero the status register
        pressure_hPa = lps22hb_from_lsb_to_hpa(data_raw_pressure.i32bit);
        temperature_degC =
          lps22hb_from_lsb_to_degc(data_raw_temperature.i16bit);
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "Press [hPa]:%4.2f\t\r\n", pressure_hPa);
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "Temp [C]:%4.2f\t\r\n", temperature_degC);
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
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
  HAL_I2C_Mem_Write(handle, LSM6DSM_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSM_I2C_ADD_H & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LSM6DSM_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LSM6DSM_I2C_ADD_H & 0xFE, reg, bufp, len);
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
