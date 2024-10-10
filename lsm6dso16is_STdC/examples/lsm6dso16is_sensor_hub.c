/*
 ******************************************************************************
 * @file    sensor_hub.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
 * - NUCLEO_F401RE + X_NUCLEO_IKS01A4
 * - DISCOVERY_SPC584B +
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
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
#include "lsm6dso16is_reg.h"
#include "lis2mdl_reg.h"
#include "lps22df_reg.h"

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
/*
 * Select FIFO samples watermark, max value is 512
 * in FIFO are stored acc, gyro and timestamp samples
 */
#define BOOT_TIME         10
#define FIFO_WATERMARK    64

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

/* Private variables ---------------------------------------------------------*/
static int16_t *magx;
static int16_t *magy;
static int16_t *magz;
static int16_t data_raw_acceleration[3];
static uint8_t data_raw_sensor_hub[12];
static float acceleration_mg[3];
static int32_t baro;
static int16_t temp;

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

static stmdev_ctx_t lsm6dso16is_ctx;
static stmdev_ctx_t lis2mdl_ctx;
static stmdev_ctx_t lps22df_ctx;

/* This routines use sensorhub SLV0 to configure the external targets */
static int32_t lsm6dso16is_write_lis2mdl_cx(void *ctx, uint8_t reg,
                                           const uint8_t *data, uint16_t len);
static int32_t lsm6dso16is_read_lis2mdl_cx(void *ctx, uint8_t reg,
                                          uint8_t *data, uint16_t len);
static int32_t lsm6dso16is_write_lps22df_cx(void *ctx, uint8_t reg,
                                           const uint8_t *data, uint16_t len);
static int32_t lsm6dso16is_read_lps22df_cx(void *ctx, uint8_t reg,
                                          uint8_t *data, uint16_t len);

static uint8_t drdy_event = 0;
void lsm6dso16is_sensor_hub_handler(void)
{
  drdy_event = 1;
}

/* Main Example --------------------------------------------------------------*/
void lsm6dso16is_sensor_hub(void)
{
  lsm6dso16is_pin_int1_route_t pin_int = {0};
  lsm6dso16is_sh_cfg_read_t sh_cfg_read;
  lps22df_id_t id;
  lps22df_bus_mode_t bus_mode;
  lps22df_stat_t status;
  lps22df_md_t md;
  uint8_t lis2mdl_rst;

  /* Initialize mems driver interface */
  lsm6dso16is_ctx.write_reg = platform_write;
  lsm6dso16is_ctx.read_reg = platform_read;
  lsm6dso16is_ctx.handle = &SENSOR_BUS;

  /* Initialize lis2mdl driver interface */
  lis2mdl_ctx.read_reg = lsm6dso16is_read_lis2mdl_cx;
  lis2mdl_ctx.write_reg = lsm6dso16is_write_lis2mdl_cx;
  lis2mdl_ctx.handle = &SENSOR_BUS;

  /* Initialize lps22df driver interface */
  lps22df_ctx.read_reg = lsm6dso16is_read_lps22df_cx;
  lps22df_ctx.write_reg = lsm6dso16is_write_lps22df_cx;
  lps22df_ctx.handle = &SENSOR_BUS;

  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm6dso16is_device_id_get(&lsm6dso16is_ctx, &whoamI);

  if (whoamI != LSM6DSO16IS_ID)
    while (1);

  /* Restore default configuration */
  lsm6dso16is_software_reset(&lsm6dso16is_ctx);

  /* Enable Block Data Update */
  lsm6dso16is_block_data_update_set(&lsm6dso16is_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lsm6dso16is_xl_full_scale_set(&lsm6dso16is_ctx, LSM6DSO16IS_2g);

  /*
   * Configure LIS2MDL target.
   */

  /* Check if LIS2MDL connected to Sensor Hub. */
  lis2mdl_device_id_get(&lis2mdl_ctx, &whoamI);

  if (whoamI != LIS2MDL_ID)
    while (1);

  /* Restore default configuration */
  lis2mdl_reset_set(&lis2mdl_ctx, PROPERTY_ENABLE);

  do {
    lis2mdl_reset_get(&lis2mdl_ctx, &lis2mdl_rst);
  } while (lis2mdl_rst);

  lis2mdl_block_data_update_set(&lis2mdl_ctx, PROPERTY_ENABLE);
  lis2mdl_offset_temp_comp_set(&lis2mdl_ctx, PROPERTY_ENABLE);
  lis2mdl_operating_mode_set(&lis2mdl_ctx, LIS2MDL_CONTINUOUS_MODE);
  lis2mdl_data_rate_set(&lis2mdl_ctx, LIS2MDL_ODR_20Hz);

  /*
   * Configure LPS22DF target.
   */

  /* Check if LPS22DF connected to Sensor Hub. */
  lps22df_id_get(&lps22df_ctx, &id);

  if (id.whoami != LPS22DF_ID)
    while (1);

  /* Restore default configuration */
  lps22df_init_set(&lps22df_ctx, LPS22DF_RESET);
  do {
    lps22df_status_get(&lps22df_ctx, &status);
  } while (status.sw_reset);

  /* Set bdu and if_inc recommended for driver usage */
  lps22df_init_set(&lps22df_ctx, LPS22DF_DRV_RDY);

  /* Select bus interface */
  bus_mode.filter = LPS22DF_FILTER_AUTO;
  bus_mode.interface = LPS22DF_SEL_BY_HW;
  lps22df_bus_mode_set(&lps22df_ctx, &bus_mode);

  /* Set Output Data Rate */
  md.odr = LPS22DF_4Hz;
  md.avg = LPS22DF_16_AVG;
  md.lpf = LPS22DF_LPF_ODR_DIV_4;
  lps22df_mode_set(&lps22df_ctx, &md);

  pin_int.drdy_xl = PROPERTY_ENABLE;
  lsm6dso16is_pin_int1_route_set(&lsm6dso16is_ctx, pin_int);
  //lsm6dso16is_pin_int2_route_set(&lsm6dso16is_ctx, pin_int);

  /* Set Output Data Rate */
  lsm6dso16is_xl_data_rate_set(&lsm6dso16is_ctx, LSM6DSO16IS_XL_ODR_AT_26H_HP);
  lsm6dso16is_timestamp_set(&lsm6dso16is_ctx, PROPERTY_ENABLE);

  /* Set full scale */
  lsm6dso16is_xl_full_scale_set(&lsm6dso16is_ctx, LSM6DSO16IS_2g);

  lsm6dso16is_xl_data_rate_set(&lsm6dso16is_ctx, LSM6DSO16IS_XL_ODR_OFF);

  /*
  * Prepare sensor hub to read data from external slave0 (lis2mdl) and
  * slave1 (lps22df) continuously in order to store data in FIFO.
  */
  sh_cfg_read.slv_add = (LIS2MDL_I2C_ADD & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_read.slv_subadd = LIS2MDL_OUTX_L_REG;
  sh_cfg_read.slv_len = 6;
  lsm6dso16is_sh_slv_cfg_read(&lsm6dso16is_ctx, 0, &sh_cfg_read);

  sh_cfg_read.slv_add = (LPS22DF_I2C_ADD_H & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_read.slv_subadd = LPS22DF_PRESS_OUT_XL;
  sh_cfg_read.slv_len = 6;
  lsm6dso16is_sh_slv_cfg_read(&lsm6dso16is_ctx, 1, &sh_cfg_read);

  /* Configure Sensor Hub data rate */
  lsm6dso16is_sh_data_rate_set(&lsm6dso16is_ctx, LSM6DSO16IS_SH_52Hz);

  /* Configure Sensor Hub to read one slave. */
  lsm6dso16is_sh_slave_connected_set(&lsm6dso16is_ctx, LSM6DSO16IS_SLV_0_1);

  /* Enable I2C Master. */
  lsm6dso16is_sh_master_set(&lsm6dso16is_ctx, PROPERTY_ENABLE);

  /* Set Output Data Rate.
  * Selected data rate have to be equal or greater with respect
  * with MLC data rate.
  */
  lsm6dso16is_xl_data_rate_set(&lsm6dso16is_ctx, LSM6DSO16IS_XL_ODR_AT_26H_HP);

  /* wait forever (xl samples read with drdy irq) */
  while (1) {
    if (drdy_event > 0) {
      /* Read acceleration field data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lsm6dso16is_acceleration_raw_get(&lsm6dso16is_ctx, data_raw_acceleration);
      acceleration_mg[0] =
        lsm6dso16is_from_fs2g_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
        lsm6dso16is_from_fs2g_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
        lsm6dso16is_from_fs2g_to_mg(data_raw_acceleration[2]);
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));

      lsm6dso16is_sh_read_data_raw_get(&lsm6dso16is_ctx, data_raw_sensor_hub, 12);

      /* magnetometer conversion */
      magx = (int16_t *)&data_raw_sensor_hub[0];
      magy = (int16_t *)&data_raw_sensor_hub[2];
      magz = (int16_t *)&data_raw_sensor_hub[4];
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "LIS2MDL [mGa]:\t%4.2f\t%4.2f\t%4.2f\r\n",
              lis2mdl_from_lsb_to_mgauss(*magx),
              lis2mdl_from_lsb_to_mgauss(*magy),
              lis2mdl_from_lsb_to_mgauss(*magz));
      tx_com(tx_buffer, strlen((char const *)tx_buffer));

      /* pressure conversion */
      baro = (int32_t)data_raw_sensor_hub[8];
      baro = (baro * 256) + (int32_t) data_raw_sensor_hub[7];
      baro = (baro * 256) + (int32_t) data_raw_sensor_hub[6];
      baro = baro * 256;
      /* temperature conversion */
      temp = (int16_t)data_raw_sensor_hub[10];
      temp = (temp * 256) + (int16_t) data_raw_sensor_hub[9];
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "LPS22DF [hPa]:%6.2f [degC]:%6.2f\r\n\r\n",
              lps22df_from_lsb_to_hPa(baro),
              lps22df_from_lsb_to_celsius(temp));
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
#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Write(handle, LSM6DSO16IS_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSO16IS_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LSM6DSO16IS_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LSM6DSO16IS_I2C_ADD_L & 0xFE, reg, bufp, len);
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

static int32_t lsm6dso16is_write_target_cx(void *ctx, uint8_t i2c_add, uint8_t reg,
                                          const uint8_t *data, uint16_t len)
{
  int16_t raw_xl[3];
  int32_t ret;
  uint8_t drdy;
  lsm6dso16is_status_master_t master_status;
  lsm6dso16is_sh_cfg_write_t sh_cfg_write;

  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_write.slv0_add = (i2c_add & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_write.slv0_subadd = reg,
  sh_cfg_write.slv0_data = *data,
  ret = lsm6dso16is_sh_cfg_write(&lsm6dso16is_ctx, &sh_cfg_write);
  /* Disable accelerometer. */
  lsm6dso16is_xl_data_rate_set(&lsm6dso16is_ctx, LSM6DSO16IS_XL_ODR_OFF);
  /* Enable I2C Master. */
  lsm6dso16is_sh_master_set(&lsm6dso16is_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dso16is_xl_data_rate_set(&lsm6dso16is_ctx, LSM6DSO16IS_XL_ODR_AT_26H_HP);
  /* Wait Sensor Hub operation flag set. */
  lsm6dso16is_acceleration_raw_get(&lsm6dso16is_ctx, raw_xl);

  do {
    HAL_Delay(20);
    lsm6dso16is_xl_flag_data_ready_get(&lsm6dso16is_ctx, &drdy);
  } while (!drdy);

  do {
    HAL_Delay(20);
    lsm6dso16is_sh_status_get(&lsm6dso16is_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL (trigger). */
  lsm6dso16is_sh_master_set(&lsm6dso16is_ctx, PROPERTY_DISABLE);
  lsm6dso16is_xl_data_rate_set(&lsm6dso16is_ctx, LSM6DSO16IS_XL_ODR_OFF);
  return ret;
}

/*
 * @brief  Write lsm2mdl device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t lsm6dso16is_write_lis2mdl_cx(void *ctx, uint8_t reg,
                                           const uint8_t *data, uint16_t len)
{
  return lsm6dso16is_write_target_cx(ctx, LIS2MDL_I2C_ADD, reg, data, len);
}

/*
 * @brief  Write lps22df device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t lsm6dso16is_write_lps22df_cx(void *ctx, uint8_t reg,
                                           const uint8_t *data, uint16_t len)
{
  return lsm6dso16is_write_target_cx(ctx, LPS22DF_I2C_ADD_H, reg, data, len);
}

static int32_t lsm6dso16is_read_target_cx(void *ctx, uint8_t i2c_add, uint8_t reg,
                                         uint8_t *data, uint16_t len)
{
  lsm6dso16is_sh_cfg_read_t sh_cfg_read;
  int16_t raw_xl[3];
  int32_t ret;
  uint8_t drdy;
  lsm6dso16is_status_master_t master_status;

  /* Disable accelerometer. */
  lsm6dso16is_xl_data_rate_set(&lsm6dso16is_ctx, LSM6DSO16IS_XL_ODR_OFF);
  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_read.slv_add = (i2c_add & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_read.slv_subadd = reg;
  sh_cfg_read.slv_len = len;
  ret = lsm6dso16is_sh_slv_cfg_read(&lsm6dso16is_ctx, 0, &sh_cfg_read);
  lsm6dso16is_sh_slave_connected_set(&lsm6dso16is_ctx, LSM6DSO16IS_SLV_0_1);
  /* Enable I2C Master and I2C master. */
  lsm6dso16is_sh_master_set(&lsm6dso16is_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dso16is_xl_data_rate_set(&lsm6dso16is_ctx, LSM6DSO16IS_XL_ODR_AT_26H_HP);
  /* Wait Sensor Hub operation flag set. */
  lsm6dso16is_acceleration_raw_get(&lsm6dso16is_ctx, raw_xl);

  do {
    HAL_Delay(20);
    lsm6dso16is_xl_flag_data_ready_get(&lsm6dso16is_ctx, &drdy);
  } while (!drdy);

  do {
    //HAL_Delay(20);
    lsm6dso16is_sh_status_get(&lsm6dso16is_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL(trigger). */
  lsm6dso16is_sh_master_set(&lsm6dso16is_ctx, PROPERTY_DISABLE);
  lsm6dso16is_xl_data_rate_set(&lsm6dso16is_ctx, LSM6DSO16IS_XL_ODR_OFF);
  /* Read SensorHub registers. */
  lsm6dso16is_sh_read_data_raw_get(&lsm6dso16is_ctx, data, len);
  return ret;
}

/*
 * @brief  Read lsm2mdl device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t lsm6dso16is_read_lis2mdl_cx(void *ctx, uint8_t reg,
                                          uint8_t *data, uint16_t len)
{
  return lsm6dso16is_read_target_cx(ctx, LIS2MDL_I2C_ADD, reg, data, len);
}

/*
 * @brief  Read lps22df device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t lsm6dso16is_read_lps22df_cx(void *ctx, uint8_t reg,
                                          uint8_t *data, uint16_t len)
{
  return lsm6dso16is_read_target_cx(ctx, LPS22DF_I2C_ADD_H, reg, data, len);
}