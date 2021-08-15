/*
 ******************************************************************************
 * @file    sensor_hub_fifo_lis2mdl_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to read LIS2MDL mag
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
 * - NUCLEO_F411RE + STEVAL-MKI189V1
 * - DISCOVERY_SPC584B + STEVAL-MKI189V1
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
#include "lsm6dsm_reg.h"
#include "lis2mdl_reg.h"

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

typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME          15 //ms

/* Private variables ---------------------------------------------------------*/
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_mG[3];
static axis3bit16_t data_raw_magnetic;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static stmdev_ctx_t dev_ctx;
static stmdev_ctx_t mag_ctx;
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
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/*
 * Read data byte from internal register of a slave device connected
 * to master I2C interface.
 */
static int32_t lsm6dsm_read_lis2mdl_cx(void *ctx, uint8_t reg,
                                       uint8_t *data,
                                       uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t mm_error;
  uint8_t drdy;
  lsm6dsm_func_src1_t func_src1;
  lsm6dsm_sh_cfg_read_t val = {
    .slv_add = LIS2MDL_I2C_ADD,
    .slv_subadd = reg,
    .slv_len = len,
  };
  (void)ctx;
  /* Disable accelerometer */
  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_OFF);
  /* Configure Sensor Hub to read LIS2MDL */
  mm_error = lsm6dsm_sh_slv0_cfg_read(&dev_ctx, &val);
  lsm6dsm_sh_num_of_dev_connected_set(&dev_ctx, LSM6DSM_SLV_0_1);
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
  lsm6dsm_sh_read_data_raw_get(&dev_ctx,
                               (lsm6dsm_emb_sh_read_t *)&data);
  lsm6dsm_func_en_set(&dev_ctx, PROPERTY_DISABLE);
  lsm6dsm_sh_master_set(&dev_ctx, PROPERTY_DISABLE);
  return mm_error;
}

/*
 * Write data byte to internal register of a slave device connected
 * to master I2C interface.
 */
static int32_t lsm6dsm_write_lis2mdl_cx(void *ctx, uint8_t reg,
                                        const uint8_t *data, uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t mm_error;
  uint8_t drdy;
  lsm6dsm_func_src1_t func_src1;
  lsm6dsm_sh_cfg_write_t val = {
    .slv0_add = LIS2MDL_I2C_ADD,
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
 * Configure LIS2MDL magnetometer sensor over I2C master line (Slave0)
 *
 * Enable LIS2MDL Magnetometer:
 * set continuous mode
 * set temperature compensation
 * enable BDU
 * set ODR to 50 Hz
 */
static void configure_lis2mdl(stmdev_ctx_t *ctx)
{
  lsm6dsm_sh_cfg_read_t val = {
    .slv_add = LIS2MDL_I2C_ADD,
    .slv_subadd = LIS2MDL_OUTX_L_REG,
    .slv_len = 3 * sizeof(int16_t),
  };
  lis2mdl_operating_mode_set(ctx, LIS2MDL_CONTINUOUS_MODE);
  lis2mdl_offset_temp_comp_set(ctx, PROPERTY_ENABLE);
  lis2mdl_block_data_update_set(ctx, PROPERTY_ENABLE);
  lis2mdl_data_rate_set(ctx, LIS2MDL_ODR_50Hz);
  /* Prepare sensor hub to read data from external Slave0 */
  lsm6dsm_sh_slv0_cfg_read(&dev_ctx, &val);
}

/* Main Example --------------------------------------------------------------*/
void lsm6dsm_sens_hub_fifo_lis2mdl(void)
{
  uint16_t pattern_len;
  //lsm6dsm_int1_route_t int_1_reg;
  //lsm6dsm_int2_route_t int_2_reg;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;
  /* Configure low level function to access to external device */
  mag_ctx.read_reg = lsm6dsm_read_lis2mdl_cx;
  mag_ctx.write_reg = lsm6dsm_write_lis2mdl_cx;
  dev_ctx.handle = &SENSOR_BUS;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
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
  lsm6dsm_int_notification_set(&dev_ctx, LSM6DSM_INT_LATCHED);
  /* Check if LIS2MDL connected to Sensor Hub */
  lis2mdl_device_id_get(&mag_ctx, &whoamI);

  if (whoamI != LIS2MDL_ID) {
    while (1) {
      /* manage here device not found */
    }
  }

  /* Configure LIS2MDL on the I2C master line */
  configure_lis2mdl(&mag_ctx);
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
   * (GYRO + XL + MAG) = 18 bytes
   */
  pattern_len = 18;
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
  /* Set FIFO sensor decimator */
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
    uint8_t waterm = 0;
    /* Read LSM6DSM watermark flag */
    lsm6dsm_fifo_wtm_flag_get(&dev_ctx, &waterm);

    if (waterm) {
      /* Read number of word in FIFO */
      lsm6dsm_fifo_data_level_get(&dev_ctx, &num);
      num_pattern = num / pattern_len;

      while (num_pattern-- > 0) {
        /* Following the sensors ODR configuration, FIFO pattern is composed
         * by this sequence of samples: GYRO, XL, MAG
         */
        lsm6dsm_fifo_raw_data_get(&dev_ctx,
                                  data_raw_angular_rate.u8bit,
                                  3 * sizeof(int16_t));
        angular_rate_mdps[0] =
          lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
        angular_rate_mdps[1] =
          lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
        angular_rate_mdps[2] =
          lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);
        sprintf((char *)tx_buffer,
                "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                angular_rate_mdps[0],
                angular_rate_mdps[1],
                angular_rate_mdps[2]);
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        lsm6dsm_fifo_raw_data_get(&dev_ctx,
                                  data_raw_acceleration.u8bit,
                                  3 * sizeof(int16_t));
        acceleration_mg[0] =
          lsm6dsm_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
        acceleration_mg[1] =
          lsm6dsm_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
        acceleration_mg[2] =
          lsm6dsm_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);
        sprintf((char *)tx_buffer,
                "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                acceleration_mg[0],
                acceleration_mg[1],
                acceleration_mg[2]);
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        lsm6dsm_fifo_raw_data_get(&dev_ctx,
                                  data_raw_magnetic.u8bit,
                                  3 * sizeof(int16_t));
        magnetic_mG[0] =
          lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[0]);
        magnetic_mG[1] =
          lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[1]);
        magnetic_mG[2] =
          lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[2]);
        sprintf((char *)tx_buffer,
                "Mag [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
                magnetic_mG[0],
                magnetic_mG[1],
                magnetic_mG[2]);
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
#if defined(NUCLEO_F411RE)
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
#if defined(NUCLEO_F411RE)
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
