/*
 ******************************************************************************
 * @file    sh_fifo_lis2mdl_timestamp.c
 * @author  Sensor Solutions Software Team
 * @brief   This file shows how to to read LIS2MDL mag connected to LSM6DSOX
 *          I2C sensor hub with FIFO and Timestamp support.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI217V1
 * - NUCLEO_F411RE + STEVAL-MKI217V1
 * - DISCOVERY_SPC584B + STEVAL-MKI217V1
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
#include "lsm6dsox_reg.h"
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

/* Private macro -------------------------------------------------------------*/
#define TX_BUF_DIM          1000
#define    BOOT_TIME          10 //ms

/* Define number of byte for each sensor sample. */
#define OUT_XYZ_SIZE    6

typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union {
  struct {
    uint32_t tick;
    uint16_t unused;
  } reg;
  uint8_t byte[6];
} timestamp_sample_t;

/* Private variables ---------------------------------------------------------*/
static stmdev_ctx_t mag_ctx;
static stmdev_ctx_t ag_ctx;

/* Extern variables ----------------------------------------------------------*/

/* Platform Functions --------------------------------------------------------*/

static int32_t lsm6dsox_write_lis2mdl_cx(void *ctx, uint8_t reg,
                                         const uint8_t *data, uint16_t len);

static int32_t lsm6dsox_read_lis2mdl_cx(void *ctx, uint8_t reg,
                                        uint8_t *data, uint16_t len);

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
static void platform_delay(uint32_t ms);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/*
 * example_main - Main Example
 *
 * Configure low level function to access to external device
 * Check if LIS2MDL connected to Sensor Hub
 * Configure lis2mdl in CONTINUOUS_MODE with 20 ODR
 * Configure Sensor Hub to read one slave with XL trigger
 * Set FIFO watermark
 * Set FIFO mode to Stream mode
 * Enable FIFO batching of Slave0 + ACC + Gyro samples
 * Enable batch FIFO of hw timestamp (LSB 25 us)
 * Poll for FIFO watermark interrupt and read samples
 */
void lsm6dsox_sh_fifo_lis2mdl_timestamp(void)
{
  lsm6dsox_pin_int1_route_t int1_route;
  lsm6dsox_sh_cfg_read_t sh_cfg_read;
  uint8_t tx_buffer[TX_BUF_DIM];
  float angular_rate_mdps[3];
  float acceleration_mg[3];
  float magnetic_mG[3];
  axis3bit16_t data_raw_magnetic;
  axis3bit16_t data_raw_acceleration;
  axis3bit16_t data_raw_angular_rate;
  axis3bit16_t dummy;
  uint8_t whoamI, rst, wtm_flag;
  /* Initialize lsm6dsox driver interface */
  ag_ctx.write_reg = platform_write;
  ag_ctx.read_reg = platform_read;
  ag_ctx.handle = &SENSOR_BUS;
  /* Initialize lis2mdl driver interface */
  mag_ctx.read_reg = lsm6dsox_read_lis2mdl_cx;
  mag_ctx.write_reg = lsm6dsox_write_lis2mdl_cx;
  mag_ctx.handle = &SENSOR_BUS;
  /* Init test platform. */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID. */
  lsm6dsox_device_id_get(&ag_ctx, &whoamI);

  if (whoamI != LSM6DSOX_ID)
    while (1);

  /* Restore default configuration. */
  lsm6dsox_reset_set(&ag_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsox_reset_get(&ag_ctx, &rst);
  } while (rst);

  /* Disable I3C interface. */
  lsm6dsox_i3c_disable_set(&ag_ctx, LSM6DSOX_I3C_DISABLE);
  /* Some hardware require to enable pull up on master I2C interface. */
  //lsm6dsox_sh_pin_mode_set(&ag_ctx, LSM6DSOX_INTERNAL_PULL_UP);
  /* Check if LIS2MDL connected to Sensor Hub. */
  lis2mdl_device_id_get(&mag_ctx, &whoamI);

  if (whoamI != LIS2MDL_ID)
    while (1);

  /* Configure LIS2MDL. */
  lis2mdl_block_data_update_set(&mag_ctx, PROPERTY_ENABLE);
  lis2mdl_offset_temp_comp_set(&mag_ctx, PROPERTY_ENABLE);
  lis2mdl_operating_mode_set(&mag_ctx, LIS2MDL_CONTINUOUS_MODE);
  lis2mdl_data_rate_set(&mag_ctx, LIS2MDL_ODR_20Hz);
  /* Configure LSM6DSOX FIFO.
   *
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to 15 samples. 5 * (Acc + Gyro + Mag)
   */
  lsm6dsox_fifo_watermark_set(&ag_ctx, 15);
  /* Set FIFO mode to Stream mode (aka Continuous Mode). */
  lsm6dsox_fifo_mode_set(&ag_ctx, LSM6DSOX_STREAM_MODE);
  /* Enable latched interrupt notification. */
  lsm6dsox_int_notification_set(&ag_ctx, LSM6DSOX_ALL_INT_LATCHED);
  /* Enable drdy 75 us pulse: uncomment if interrupt must be pulsed. */
  //lsm6dsox_data_ready_mode_set(&ag_ctx, LSM6DSOX_DRDY_PULSED);
  /*
   * FIFO watermark interrupt routed on INT1 pin.
   * Remember that INT1 pin is used by sensor to switch in I3C mode.
   */
  lsm6dsox_pin_int1_route_get(&ag_ctx, &int1_route);
  int1_route.fifo_th = PROPERTY_ENABLE;
  lsm6dsox_pin_int1_route_set(&ag_ctx, int1_route);
  /*
   * Enable FIFO batching of Slave0.
   * ODR batching is 13 Hz.
   */
  lsm6dsox_sh_batch_slave_0_set(&ag_ctx, PROPERTY_ENABLE);
  lsm6dsox_sh_data_rate_set(&ag_ctx, LSM6DSOX_SH_ODR_13Hz);
  /* Set FIFO batch XL/Gyro ODR to 12.5Hz. */
  lsm6dsox_fifo_xl_batch_set(&ag_ctx, LSM6DSOX_XL_BATCHED_AT_12Hz5);
  lsm6dsox_fifo_gy_batch_set(&ag_ctx, LSM6DSOX_GY_BATCHED_AT_12Hz5);
  /* Enable time-stamp batching. */
  lsm6dsox_fifo_timestamp_decimation_set(&ag_ctx, LSM6DSOX_DEC_1);
  lsm6dsox_timestamp_set(&ag_ctx, PROPERTY_ENABLE);
  /*
   * Prepare sensor hub to read data from external Slave0 continuously
   * in order to store data in FIFO.
   */
  sh_cfg_read.slv_add = (LIS2MDL_I2C_ADD & 0xFEU) >>
                        1; /* 7bit I2C address */
  sh_cfg_read.slv_subadd = LIS2MDL_OUTX_L_REG;
  sh_cfg_read.slv_len = 6;
  lsm6dsox_sh_slv0_cfg_read(&ag_ctx, &sh_cfg_read);
  /* Configure Sensor Hub to read one slave. */
  lsm6dsox_sh_slave_connected_set(&ag_ctx, LSM6DSOX_SLV_0);
  /* Enable I2C Master. */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_ENABLE);
  /* Configure LSM6DSOX. */
  lsm6dsox_xl_full_scale_set(&ag_ctx, LSM6DSOX_2g);
  lsm6dsox_gy_full_scale_set(&ag_ctx, LSM6DSOX_2000dps);
  lsm6dsox_block_data_update_set(&ag_ctx, PROPERTY_ENABLE);
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_12Hz5);
  lsm6dsox_gy_data_rate_set(&ag_ctx, LSM6DSOX_GY_ODR_12Hz5);

  while (1) {
    uint16_t num = 0;
    lsm6dsox_fifo_tag_t reg_tag;
    timestamp_sample_t ts_tick;
    uint32_t timestamp = 0.0f;
    /* Read watermark flag. */
    lsm6dsox_fifo_wtm_flag_get(&ag_ctx, &wtm_flag);

    if ( wtm_flag ) {
      /* Read number of samples in FIFO. */
      lsm6dsox_fifo_data_level_get(&ag_ctx, &num);

      while (num--) {
        /* Read FIFO tag. */
        lsm6dsox_fifo_sensor_tag_get(&ag_ctx, &reg_tag);

        switch (reg_tag) {
          case LSM6DSOX_XL_NC_TAG:
            memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dsox_fifo_out_raw_get(&ag_ctx, data_raw_acceleration.u8bit);
            acceleration_mg[0] =
              lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
            acceleration_mg[1] =
              lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
            acceleration_mg[2] =
              lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);
            sprintf((char *)tx_buffer,
                    "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f T %u.%u ms\r\n",
                    acceleration_mg[0], acceleration_mg[1],
                    acceleration_mg[2], (unsigned int)(timestamp / 1000ULL),
                    (unsigned int)(timestamp % 1000ULL));
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
            break;

          case LSM6DSOX_GYRO_NC_TAG:
            memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dsox_fifo_out_raw_get(&ag_ctx, data_raw_angular_rate.u8bit);
            angular_rate_mdps[0] =
              lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
            angular_rate_mdps[1] =
              lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
            angular_rate_mdps[2] =
              lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);
            sprintf((char *)tx_buffer,
                    "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f T %u.%u ms\r\n",
                    angular_rate_mdps[0], angular_rate_mdps[1],
                    angular_rate_mdps[2], (unsigned int)(timestamp / 1000ULL),
                    (unsigned int)(timestamp % 1000ULL));
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
            break;

          case LSM6DSOX_SENSORHUB_SLAVE0_TAG:
            memset(data_raw_magnetic.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dsox_fifo_out_raw_get(&ag_ctx, data_raw_magnetic.u8bit);
            magnetic_mG[0] =
              lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[0]);
            magnetic_mG[1] =
              lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[1]);
            magnetic_mG[2] =
              lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[2]);
            sprintf((char *)tx_buffer,
                    "Mag [mG]:%4.2f\t%4.2f\t%4.2f T %u.%u ms\r\n",
                    magnetic_mG[0], magnetic_mG[1], magnetic_mG[2],
                    (unsigned int)(timestamp / 1000ULL),
                    (unsigned int)(timestamp % 1000ULL));
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
            break;

          case LSM6DSOX_TIMESTAMP_TAG:
            lsm6dsox_fifo_out_raw_get(&ag_ctx, ts_tick.byte);
            timestamp = (unsigned int)lsm6dsox_from_lsb_to_nsec(ts_tick.reg.tick);
            break;

          default:
            /* Flush unused samples. */
            memset(dummy.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm6dsox_fifo_out_raw_get(&ag_ctx, dummy.u8bit);
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
#if defined(NUCLEO_F411RE)
  HAL_I2C_Mem_Write(handle, LSM6DSOX_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSOX_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LSM6DSOX_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LSM6DSOX_I2C_ADD_L & 0xFE, reg, bufp, len);
#endif
  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
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
static int32_t lsm6dsox_write_lis2mdl_cx(void *ctx, uint8_t reg,
                                         const uint8_t *data,
                                         uint16_t len)
{
  int16_t data_raw_acceleration[3];
  int32_t ret;
  uint8_t drdy;
  lsm6dsox_status_master_t master_status;
  lsm6dsox_sh_cfg_write_t sh_cfg_write;
  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_write.slv0_add = (LIS2MDL_I2C_ADD & 0xFEU) >>
                          1; /* 7bit I2C address */
  sh_cfg_write.slv0_subadd = reg,
  sh_cfg_write.slv0_data = *data,
  ret = lsm6dsox_sh_cfg_write(&ag_ctx, &sh_cfg_write);
  /* Disable accelerometer. */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_OFF);
  /* Enable I2C Master. */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_104Hz);
  /* Wait Sensor Hub operation flag set. */
  lsm6dsox_acceleration_raw_get(&ag_ctx, data_raw_acceleration);

  do {
    platform_delay(20);
    lsm6dsox_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while (!drdy);

  do {
    platform_delay(20);
    lsm6dsox_sh_status_get(&ag_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL (trigger). */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_OFF);
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
static int32_t lsm6dsox_read_lis2mdl_cx(void *ctx, uint8_t reg,
                                        uint8_t *data,
                                        uint16_t len)
{
  lsm6dsox_sh_cfg_read_t sh_cfg_read;
  int16_t data_raw_acceleration[3];
  int32_t ret;
  uint8_t drdy;
  lsm6dsox_status_master_t master_status;
  /* Disable accelerometer. */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_OFF);
  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_read.slv_add = (LIS2MDL_I2C_ADD & 0xFEU) >>
                        1; /* 7bit I2C address */
  sh_cfg_read.slv_subadd = reg;
  sh_cfg_read.slv_len = len;
  ret = lsm6dsox_sh_slv0_cfg_read(&ag_ctx, &sh_cfg_read);
  lsm6dsox_sh_slave_connected_set(&ag_ctx, LSM6DSOX_SLV_0);
  /* Enable I2C Master and I2C master. */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_104Hz);
  /* Wait Sensor Hub operation flag set. */
  lsm6dsox_acceleration_raw_get(&ag_ctx, data_raw_acceleration);

  do {
    platform_delay(20);
    lsm6dsox_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while (!drdy);

  do {
    //platform_delay(20);
    lsm6dsox_sh_status_get(&ag_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL(trigger). */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_OFF);
  /* Read SensorHub registers. */
  lsm6dsox_sh_read_data_raw_get(&ag_ctx, (lsm6dsox_emb_sh_read_t *)data,
                                len);
  return ret;
}
