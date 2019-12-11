/*
 ******************************************************************************
 * @file    sensor_hub_fifo_lps22hh.c
 * @author  Sensor Solutions Software Team
 * @brief   This file show the simplest way to read LPS22HH mag
 *          connected to LSM6DSO I2C master interface (with FIFO support).
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
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A3 + STEVAL-MKI197V1
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 * - STM32F411 -> PC:         UART(COM) to USB bridge(via ST-link)
 * - LSM6DSO  -> STM32F411:  I2C
 * - LPS22HH   -> LSM6DSO:   I2C
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read` and
 * `tx_com` is required.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <lsm6dso_reg.h>
#include <lps22hh_reg.h>
#include <string.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* useful union to manage IMU data */
typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

/* useful union to manage LPS22HH data */
typedef union{
  struct {
    uint32_t u32bit; /* pressure plus status register */
    int16_t  i16bit; /* temperature */
  } p_and_t;
  uint8_t u8bit[6];
} p_and_t_byte_t;

/* Private macro -------------------------------------------------------------*/
#define TX_BUF_DIM                       1000

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[TX_BUF_DIM];

static stmdev_ctx_t press_ctx;
static stmdev_ctx_t ag_ctx;

static float angular_rate_mdps[3];
static float acceleration_mg[3];
static float temperature_degC;
static float pressure_hPa;


/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

static int32_t lsm6dso_write_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data,
                                         uint16_t len);

static int32_t lsm6dso_read_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data,
                                        uint16_t len);

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

/* Main Example --------------------------------------------------------------*/

/*
 * Main Example
 *
 * Configure low level function to access to external device
 * Check if LPS22HH connected to Sensor Hub
 * Configure lps22hh for data acquisition
 * Configure Sensor Hub to read one slave with XL trigger
 * Set FIFO watermark
 * Set FIFO mode to Stream mode
 * Enable FIFO batching of Slave0 + ACC + Gyro samples
 * Poll for FIFO watermark interrupt and read samples
 */
void lsm6dso_hub_fifo_lps22hh(void)
{
  uint8_t whoamI, rst, wtm_flag;

  lsm6dso_pin_int1_route_t int1_route;
  lsm6dso_sh_cfg_read_t sh_cfg_read;
  p_and_t_byte_t data_raw_press_temp;
  axis3bit16_t data_raw_acceleration;
  axis3bit16_t data_raw_angular_rate;
  axis3bit16_t dummy;

  /* Initialize lsm6dso driver interface */
  ag_ctx.write_reg = platform_write;
  ag_ctx.read_reg = platform_read;
  ag_ctx.handle = &hi2c1;

  /* Initialize lps22hh driver interface */
  press_ctx.read_reg = lsm6dso_read_lps22hh_cx;
  press_ctx.write_reg = lsm6dso_write_lps22hh_cx;
  press_ctx.handle = &hi2c1;

  /*
   * Check Connected devices.
   */
  /* Check lsm6dso ID. */
  lsm6dso_device_id_get(&ag_ctx, &whoamI);
  if (whoamI != LSM6DSO_ID)
    while(1);

  /* Restore default configuration. */
  lsm6dso_reset_set(&ag_ctx, PROPERTY_ENABLE);
  do {
    lsm6dso_reset_get(&ag_ctx, &rst);
  } while (rst);

  /* Disable I3C interface.*/
  lsm6dso_i3c_disable_set(&ag_ctx, LSM6DSO_I3C_DISABLE);

  /* Some hardware require to enable pull up on master I2C interface. */
 //lsm6dso_sh_pin_mode_set(&ag_ctx, LSM6DSO_INTERNAL_PULL_UP);

  /* Check if LPS22HH connected to Sensor Hub. */
  lps22hh_device_id_get(&press_ctx, &whoamI);
  if ( whoamI != LPS22HH_ID )
    while(1); /*manage here device not found */

  /* Configure LPS22HH. */
  lps22hh_block_data_update_set(&press_ctx, PROPERTY_ENABLE);
  lps22hh_data_rate_set(&press_ctx, LPS22HH_10_Hz_LOW_NOISE);

  /*
   * Configure LSM6DSO FIFO.
   *
   *
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to 15 samples. 5 * (Acc + Gyro + Pressure)
   */
  lsm6dso_fifo_watermark_set(&ag_ctx, 15);

  /* Set FIFO mode to Stream mode (aka Continuous Mode). */
  lsm6dso_fifo_mode_set(&ag_ctx, LSM6DSO_STREAM_MODE);

  /* Enable latched interrupt notification. */
  lsm6dso_int_notification_set(&ag_ctx, LSM6DSO_ALL_INT_LATCHED);

  /* Enable drdy 75 us pulse: uncomment if interrupt must be pulsed. */
  //lsm6dso_data_ready_mode_set(&ag_ctx, LSM6DSO_DRDY_PULSED);

  /*
   * FIFO watermark interrupt routed on INT1 pin.
   * Remember that INT1 pin is used by sensor to switch in I3C mode.
   */
  lsm6dso_pin_int1_route_get(&ag_ctx, &int1_route);
  int1_route.int1_ctrl.int1_fifo_th = PROPERTY_ENABLE;
  lsm6dso_pin_int1_route_set(&ag_ctx, &int1_route);

  /*
   * Enable FIFO batching of Slave0.
   * ODR batching is 13 Hz.
   */
  lsm6dso_sh_batch_slave_0_set(&ag_ctx, PROPERTY_ENABLE);
  lsm6dso_sh_data_rate_set(&ag_ctx, LSM6DSO_SH_ODR_13Hz);

  /* Set FIFO batch XL/Gyro ODR to 12.5Hz. */
  lsm6dso_fifo_xl_batch_set(&ag_ctx, LSM6DSO_XL_BATCHED_AT_12Hz5);
  lsm6dso_fifo_gy_batch_set(&ag_ctx, LSM6DSO_GY_BATCHED_AT_12Hz5);

  /*
   * Prepare sensor hub to read data from external Slave0 continuously
   * in order to store data in FIFO.
   */
  sh_cfg_read.slv_add = (LPS22HH_I2C_ADD_H & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_read.slv_subadd = LPS22HH_STATUS;
  sh_cfg_read.slv_len = 6;
  lsm6dso_sh_slv0_cfg_read(&ag_ctx, &sh_cfg_read);
  /* Configure Sensor Hub to read one slave. */
  lsm6dso_sh_slave_connected_set(&ag_ctx, LSM6DSO_SLV_0);
  /* Enable I2C Master. */
  lsm6dso_sh_master_set(&ag_ctx, PROPERTY_ENABLE);

  /* Configure LSM6DSO. */
  lsm6dso_xl_full_scale_set(&ag_ctx, LSM6DSO_2g);
  lsm6dso_gy_full_scale_set(&ag_ctx, LSM6DSO_2000dps);
  lsm6dso_block_data_update_set(&ag_ctx, PROPERTY_ENABLE);
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_12Hz5);
  lsm6dso_gy_data_rate_set(&ag_ctx, LSM6DSO_GY_ODR_12Hz5);

  while(1) {
    uint16_t num = 0;
    lsm6dso_fifo_tag_t reg_tag;

    /* Read watermark flag. */
    lsm6dso_fifo_wtm_flag_get(&ag_ctx, &wtm_flag);
    if ( wtm_flag ) {
      /* Read number of samples in FIFO. */
      lsm6dso_fifo_data_level_get(&ag_ctx, &num);

      while(num--) {

        /* Read FIFO tag. */
        lsm6dso_fifo_sensor_tag_get(&ag_ctx, &reg_tag);

        switch(reg_tag)
        {
          case LSM6DSO_XL_NC_TAG:
            memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dso_fifo_out_raw_get(&ag_ctx, data_raw_acceleration.u8bit);

            acceleration_mg[0] = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
            acceleration_mg[1] = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
            acceleration_mg[2] = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);

            sprintf( (char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                     acceleration_mg[0],
                     acceleration_mg[1],
                     acceleration_mg[2] );
            tx_com(tx_buffer, strlen((char const*)tx_buffer));
            break;

          case LSM6DSO_GYRO_NC_TAG:
            memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dso_fifo_out_raw_get(&ag_ctx, data_raw_angular_rate.u8bit);

            angular_rate_mdps[0] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
            angular_rate_mdps[1] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
            angular_rate_mdps[2] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);

            sprintf( (char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                     angular_rate_mdps[0],
                     angular_rate_mdps[1],
                     angular_rate_mdps[2]);
            tx_com(tx_buffer, strlen((char const*)tx_buffer));
            break;

          case LSM6DSO_SENSORHUB_SLAVE0_TAG:
            memset(data_raw_press_temp.u8bit, 0x00, sizeof(p_and_t_byte_t));
            lsm6dso_fifo_out_raw_get(&ag_ctx, data_raw_press_temp.u8bit);

            data_raw_press_temp.u8bit[0] = 0x00; /* remove status register */
            pressure_hPa = lps22hh_from_lsb_to_hpa( data_raw_press_temp.p_and_t.u32bit);
            temperature_degC = lps22hh_from_lsb_to_celsius( data_raw_press_temp.p_and_t.i16bit );

            sprintf( (char*)tx_buffer, "Press [hPa]:%4.2f\r\nTemp [degC]:%4.2f\r\n",
                     pressure_hPa, temperature_degC);
            tx_com(tx_buffer, strlen((char const*)tx_buffer));
            break;

          default:
          /* Flush unused samples. */
            memset(dummy.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dso_fifo_out_raw_get(&ag_ctx, dummy.u8bit);
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
static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, LSM6DSO_I2C_ADD_H, Reg,
                      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
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
static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  if (handle == &hi2c1)
  {
      HAL_I2C_Mem_Read(handle, LSM6DSO_I2C_ADD_H, Reg,
                       I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
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
  HAL_UART_Transmit( &huart2, tx_buffer, len, 1000 );
}

/*
 * @brief  Write lps22hh device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t lsm6dso_write_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t ret;
  uint8_t drdy;
  lsm6dso_status_master_t master_status;
  lsm6dso_sh_cfg_write_t sh_cfg_write;

  /* Configure Sensor Hub to read LPS22HH. */
  sh_cfg_write.slv0_add = (LPS22HH_I2C_ADD_H & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_write.slv0_subadd = reg,
  sh_cfg_write.slv0_data = *data,
  ret = lsm6dso_sh_cfg_write(&ag_ctx, &sh_cfg_write);

  /* Disable accelerometer. */
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_OFF);

  /* Enable I2C Master. */
  lsm6dso_sh_master_set(&ag_ctx, PROPERTY_ENABLE);

  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_104Hz);

  /* Wait Sensor Hub operation flag set. */
  lsm6dso_acceleration_raw_get(&ag_ctx, data_raw_acceleration.u8bit);
  do
  {
    HAL_Delay(20);
  lsm6dso_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while (!drdy);

  do
  {
    HAL_Delay(20);
    lsm6dso_sh_status_get(&ag_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL (trigger). */
  lsm6dso_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_OFF);

  return ret;
}

/*
 * @brief  Read lps22hh device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t lsm6dso_read_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len)
{
  lsm6dso_sh_cfg_read_t sh_cfg_read;
  axis3bit16_t data_raw_acceleration;
  int32_t ret;
  uint8_t drdy;
  lsm6dso_status_master_t master_status;

  /* Disable accelerometer. */
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_OFF);

  /* Configure Sensor Hub to read LPS22HH. */
  sh_cfg_read.slv_add = (LPS22HH_I2C_ADD_H & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_read.slv_subadd = reg;
  sh_cfg_read.slv_len = len;
  ret = lsm6dso_sh_slv0_cfg_read(&ag_ctx, &sh_cfg_read);
  lsm6dso_sh_slave_connected_set(&ag_ctx, LSM6DSO_SLV_0);

  /* Enable I2C Master and I2C master. */
   lsm6dso_sh_master_set(&ag_ctx, PROPERTY_ENABLE);

  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_104Hz);

  /* Wait Sensor Hub operation flag set. */
  lsm6dso_acceleration_raw_get(&ag_ctx, data_raw_acceleration.u8bit);
  do {
    HAL_Delay(20);
  lsm6dso_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while (!drdy);

  do {
    //HAL_Delay(20);
    lsm6dso_sh_status_get(&ag_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL(trigger). */
  lsm6dso_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_OFF);

  /* Read SensorHub registers. */
  lsm6dso_sh_read_data_raw_get(&ag_ctx, data, len);

  return ret;
}
