/*
 ******************************************************************************
 * @file    sensor_hub_lis2mdl_lps22hb_no_fifo_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way enable a LIS2MDL mag and
 *          LPS22HB connected to LSM6DS3 I2C master interface (no FIFO support).
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
#include <lps22hb_reg.h>
#include <lis2mdl_reg.h>
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
  int32_t i32bit;
  uint8_t u8bit[4];
} axis1bit32_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;
  
/* Private macro -------------------------------------------------------------*/
#define OUT_XYZ_SIZE		6
#define PRESS_OUT_XYZ_SIZE	3
#define TEMP_OUT_XYZ_SIZE	2

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI, rst;
static float pressure_hPa;
static float temperature_degC;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_mG[3];
static axis3bit16_t data_raw_magnetic;
static axis1bit32_t data_raw_pressure;
static axis1bit16_t data_raw_temperature;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static stmdev_ctx_t dev_ctx;
static stmdev_ctx_t press_ctx;
static stmdev_ctx_t mag_ctx;
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

/*
 * Read data byte from internal register of a slave device connected
 * to master I2C interface
 */
static int32_t lsm6ds3_read_lps22hb_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t mm_error;
  uint8_t drdy;
  uint8_t endop;
  lsm6ds3_sh_cfg_read_t val =
  {
    .slv_add = LPS22HB_I2C_ADD_H,
    .slv_subadd = reg,
    .slv_len = len,
  };

  (void)ctx;

  /*
   * Configure Sensor Hub to read LPS22HB
   */
  mm_error = lsm6ds3_sh_slv0_cfg_read(&dev_ctx, &val);
  lsm6ds3_sh_num_of_dev_connected_set(&dev_ctx, LSM6DS3_SLV_0_1);

  /*
   * Enable I2C Master and I2C Master Pull Up
   */
  lsm6ds3_sh_master_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Enable accelerometer to trigger Sensor Hub operation
   */
  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_104Hz);

  /*
   * Wait Sensor Hub operation flag set
   */
  lsm6ds3_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
  do
  {
    lsm6ds3_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  do
  {
    lsm6ds3_sh_end_op_flag_get(&dev_ctx, &endop);
  } while (!endop);

  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_OFF);
  lsm6ds3_sh_read_data_raw_get(&dev_ctx, (lsm6ds3_sh_read_t*)&data);

  lsm6ds3_sh_master_set(&dev_ctx, PROPERTY_DISABLE);

  return mm_error;
}

/*
 * Write data byte to internal register of a slave device connected
 * to master I2C interface
 */
static int32_t lsm6ds3_write_lps22hb_cx(void* ctx, uint8_t reg, uint8_t* data,
                                        uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t mm_error;
  uint8_t drdy;
  uint8_t endop;
  lsm6ds3_sh_cfg_write_t val = {
    .slv0_add = LPS22HB_I2C_ADD_H,
    .slv0_subadd = reg,
    .slv0_data = *data,
  };

  (void)ctx;
  (void)len;

  /*
   * Disable accelerometer
   */
  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_OFF);

  /*
   * Configure Sensor Hub to write
   */
  mm_error = lsm6ds3_sh_cfg_write(&dev_ctx, &val);

  /*
   * Enable I2C Master and I2C master Pull Up
   */
  lsm6ds3_sh_master_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Enable accelerometer to trigger Sensor Hub operation
   */
  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_104Hz);

  /*
   * Wait Sensor Hub operation flag set
   */
  lsm6ds3_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
  do
  {
    lsm6ds3_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  do
  {
    lsm6ds3_sh_end_op_flag_get(&dev_ctx, &endop);
  } while (!endop);

  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_OFF);

  lsm6ds3_sh_master_set(&dev_ctx, PROPERTY_DISABLE);

  return mm_error;
}

/*
 * Read data byte from internal register of a slave device connected
 * to master I2C interface
 */
static int32_t lsm6ds3_read_lis2mdl_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t mm_error;
  uint8_t drdy;
  uint8_t endop;
  lsm6ds3_sh_cfg_read_t val = {
    .slv_add = LIS2MDL_I2C_ADD,
    .slv_subadd = reg,
    .slv_len = len,
  };

  (void)ctx;

  /*
   * Configure Sensor Hub to read LIS2MDL
   */
  mm_error = lsm6ds3_sh_slv0_cfg_read(&dev_ctx, &val);
  lsm6ds3_sh_num_of_dev_connected_set(&dev_ctx, LSM6DS3_SLV_0_1);

  /*
   * Enable I2C Master and I2C master Pull Up
   */
  lsm6ds3_sh_master_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Enable accelerometer to trigger Sensor Hub operation
   */
  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_104Hz);

  /*
   * Wait Sensor Hub operation flag set
   */
  lsm6ds3_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
  do {
    lsm6ds3_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  do
  {
    lsm6ds3_sh_end_op_flag_get(&dev_ctx, &endop);
  } while (!endop);

  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_OFF);
  lsm6ds3_sh_read_data_raw_get(&dev_ctx, (lsm6ds3_sh_read_t*)&data);

  lsm6ds3_sh_master_set(&dev_ctx, PROPERTY_DISABLE);

  return mm_error;
}

/*
 * Write data byte to internal register of a slave device connected
 * to master I2C interface
 */
static int32_t lsm6ds3_write_lis2mdl_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t mm_error;
  uint8_t drdy;
  uint8_t endop;
  lsm6ds3_sh_cfg_write_t val = {
    .slv0_add = LIS2MDL_I2C_ADD,
    .slv0_subadd = reg,
    .slv0_data = *data,
  };

  (void)ctx;
  (void)len;

  /*
   * Disable accelerometer
   */
  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_OFF);

  /*
   * Configure Sensor Hub to write
   */
  mm_error = lsm6ds3_sh_cfg_write(&dev_ctx, &val);

  /*
   * Enable I2C Master and I2C master Pull Up
   */
  lsm6ds3_sh_master_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Enable accelerometer to trigger Sensor Hub operation
   */
  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_104Hz);

  /*
   * Wait Sensor Hub operation flag set
   */
  lsm6ds3_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
  do {
    lsm6ds3_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  do
  {
    lsm6ds3_sh_end_op_flag_get(&dev_ctx, &endop);
  } while (!endop);

  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_OFF);

  lsm6ds3_sh_master_set(&dev_ctx, PROPERTY_DISABLE);

  return mm_error;
}

/* Main Example --------------------------------------------------------------*/
void example_sensor_hub_lis2mdl_lps22hb_no_fifo_lsm6ds3(void)
{
  lsm6ds3_sh_cfg_read_t lis2mdl_conf = {
    .slv_add = LIS2MDL_I2C_ADD,
    .slv_subadd = LIS2MDL_OUTX_L_REG,
    .slv_len = OUT_XYZ_SIZE,
  };
  lsm6ds3_sh_cfg_read_t lps22hb_conf =
  {
    .slv_add = LPS22HB_I2C_ADD_H,
    .slv_subadd = LPS22HB_PRESS_OUT_XL,
    .slv_len = PRESS_OUT_XYZ_SIZE + TEMP_OUT_XYZ_SIZE,
  };

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /*
   * Configure low level function to access to external device
    */
  press_ctx.read_reg = lsm6ds3_read_lps22hb_cx;
  press_ctx.write_reg = lsm6ds3_write_lps22hb_cx;
  press_ctx.handle = &hi2c1;
  mag_ctx.read_reg = lsm6ds3_read_lis2mdl_cx;
  mag_ctx.write_reg = lsm6ds3_write_lis2mdl_cx;
  mag_ctx.handle = &hi2c1;

  /*
   * Initialize platform specific hardware
   */
  platform_init();

  /*
   * Check device ID
   */
  lsm6ds3_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LSM6DS3_ID)
  {
    while(1)
    {
      /* manage here device not found */
    }
  }

  /*
   * Restore default configuration
   */
  lsm6ds3_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6ds3_reset_get(&dev_ctx, &rst);
  } while (rst);

  /*
   * Some hardware require to enable pull up on master I2C interface
   */
  //lsm6ds3_sh_pin_mode_set(&dev_ctx, LSM6DS3_INTERNAL_PULL_UP);

  /*
   * Check if LPS22HB connected to Sensor Hub
   */
  lps22hb_device_id_get(&press_ctx, &whoamI);
  if (whoamI != LPS22HB_ID)
  {
    while(1)
    {
      /* manage here device not found */
    }
  }

  /*
   * Check if LIS2MDL connected to Sensor Hub
   */
  lis2mdl_device_id_get(&mag_ctx, &whoamI);
  if (whoamI != LIS2MDL_ID)
  {
    while(1)
    {
      /* manage here device not found */
    }
  }

  /*
   * Set XL full scale and Gyro full scale
   */
  lsm6ds3_xl_full_scale_set(&dev_ctx, LSM6DS3_2g);
  lsm6ds3_gy_full_scale_set(&dev_ctx, LSM6DS3_2000dps);

  /*
   * Configure LPS22HB on the I2C master line
   */
  lps22hb_data_rate_set(&press_ctx, LPS22HB_ODR_50_Hz);
  lps22hb_block_data_update_set(&press_ctx, PROPERTY_ENABLE);

  /*
   * Prepare sensor hub to read data from external Slave1
   */
  lsm6ds3_sh_slv1_cfg_read(&dev_ctx, &lps22hb_conf);

  /*
   * Configure LIS2MDL on the I2C master line
   */
  lis2mdl_operating_mode_set(&mag_ctx, LIS2MDL_CONTINUOUS_MODE);
  lis2mdl_offset_temp_comp_set(&mag_ctx, PROPERTY_ENABLE);
  lis2mdl_block_data_update_set(&mag_ctx, PROPERTY_ENABLE);
  lis2mdl_data_rate_set(&mag_ctx, LIS2MDL_ODR_50Hz);

  /*
   * Prepare sensor hub to read data from external Slave0
   */
  lsm6ds3_sh_slv0_cfg_read(&dev_ctx, &lis2mdl_conf);

  /*
   * Configure Sensor Hub to read two slaves
   */
  lsm6ds3_sh_num_of_dev_connected_set(&dev_ctx, LSM6DS3_SLV_0_1);

  /*
   * Enable master and XL trigger
   */
  lsm6ds3_sh_master_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set XL and Gyro Output Data Rate
   */
  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_52Hz);
  lsm6ds3_gy_data_rate_set(&dev_ctx, LSM6DS3_GY_ODR_26Hz);

  while(1)
  {
    uint8_t drdy;
    lsm6ds3_sh_read_t sh_reg;

    /*
     * Read output only if new value is available
     */
    lsm6ds3_xl_flag_data_ready_get(&dev_ctx, &drdy);
    if (drdy)
    {
      /*
       * Read acceleration field data
       */
      memset(data_raw_acceleration.u8bit, 0x0, 3 * sizeof(int16_t));
      lsm6ds3_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
      acceleration_mg[0] =
        lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] =
        lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] =
        lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);

      sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));

      /*
       * Read magnetic data from sensor hub register: XL trigger a new read to
       * mag sensor
       */
      lsm6ds3_sh_read_data_raw_get(&dev_ctx, &sh_reg);
      memcpy((uint8_t *)&data_raw_magnetic,
             (uint8_t *)&sh_reg.sh_byte_1,
             OUT_XYZ_SIZE);
      magnetic_mG[0] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[0]);
      magnetic_mG[1] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[1]);
      magnetic_mG[2] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[2]);

      sprintf((char*)tx_buffer, "Mag [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
              magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));

      /*
       * Read pressure data from sensor hub register: XL trigger a new read to
       * barom. sensor. Barometer and Temperature sensor share the same slave
       * because is a combo sensor
       */
      memcpy(data_raw_pressure.u8bit,
             (uint8_t *)&sh_reg.sh_byte_7,
             PRESS_OUT_XYZ_SIZE);
      memcpy(data_raw_temperature.u8bit,
             (uint8_t *)&sh_reg.sh_byte_10,
             TEMP_OUT_XYZ_SIZE);
      pressure_hPa = lps22hb_from_lsb_to_hpa(data_raw_pressure.i32bit);
      temperature_degC = lps22hb_from_lsb_to_degc(data_raw_temperature.i16bit);

      sprintf((char*)tx_buffer, "Press [hPa]:%4.2f\t\r\n", pressure_hPa);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
      sprintf((char*)tx_buffer, "Temp [C]:%4.2f\t\r\n", temperature_degC);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
    }

    lsm6ds3_gy_flag_data_ready_get(&dev_ctx, &drdy);
    if (drdy)
    {
      /*
       * Read angular rate field data
       */
      memset(data_raw_angular_rate.u8bit, 0x0, 3 * sizeof(int16_t));
      lsm6ds3_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
      angular_rate_mdps[0] =
        lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
      angular_rate_mdps[1] =
        lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
      angular_rate_mdps[2] =
        lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

      sprintf((char*)tx_buffer,
              "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              angular_rate_mdps[0],
              angular_rate_mdps[1],
              angular_rate_mdps[2]);
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
