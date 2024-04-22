/*
 ******************************************************************************
 * @file    ism330dhcx_sensor_hub_iis2mdc_fifo_timestamp.c
 * @author  Sensor Solutions Software Team
 * @brief   This file show how to get data from magnetometer sensor through
 *          sensor hub using fifo and timestamp.
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
 * - STEVAL_MKI109V3 + X-NUCLEO-IKS02A1
 * - NUCLEO_F411RE + X-NUCLEO-IKS02A1
 * - DISCOVERY_SPC584B + X-NUCLEO-IKS02A1
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

#include "ism330dhcx_reg.h"
#include "iis2mdc_reg.h"

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
/* Wait sensor boot time */
#define BOOT_TIME        10 //ms
#define TX_BUF_DIM     1000
#define TIME_OPS      10000

/*
 * Define number of byte for each sensor sample.
 */
#define OUT_XYZ_SIZE    6

typedef union {
  struct {
    uint32_t tick;
    uint16_t unused;
  } reg;
  uint8_t byte[6];
} timestamp_sample_t;

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[TX_BUF_DIM];

/* Local variables used by sample test only. */
static stmdev_ctx_t ag_ctx;
static stmdev_ctx_t mag_ctx;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

static int32_t ism330dhcx_write_iis2mdc_cx(void *ctx, uint8_t reg,
                                           const uint8_t *data, uint16_t len);

static int32_t ism330dhcx_read_iis2mdc_cx(void *ctx, uint8_t reg,
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
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void ism330dhcx_sensor_hub_iis2mdc_fifo_timestamp(void)
{
  ism330dhcx_sh_cfg_read_t sh_cfg_read;
  axis3bit16_t data_raw_acceleration;
  axis3bit16_t data_raw_angular_rate;
  axis3bit16_t data_raw_magnetic;
  uint8_t whoamI, rst, fifo_wtm;
  float angular_rate_mdps[3];
  float acceleration_mg[3];
  float magnetic_mG[3];
  axis3bit16_t dummy;
  /* Initialize ism330dhcx driver interface */
  ag_ctx.write_reg = platform_write;
  ag_ctx.read_reg = platform_read;
  ag_ctx.handle = &SENSOR_BUS;
  /* Initialize iis2mdc driver interface */
  mag_ctx.read_reg = ism330dhcx_read_iis2mdc_cx;
  mag_ctx.write_reg = ism330dhcx_write_iis2mdc_cx;
  mag_ctx.handle = &SENSOR_BUS;
  /* Init test platform. */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID. */
  ism330dhcx_device_id_get(&ag_ctx, &whoamI);

  if (whoamI != ISM330DHCX_ID) {
    /* Sensor not detected. */
    while (1);
  }

  /* Restore default configuration. */
  ism330dhcx_reset_set(&ag_ctx, PROPERTY_ENABLE);

  do {
    ism330dhcx_reset_get(&ag_ctx, &rst);
  } while (rst);

  /* Start device configuration. */
  ism330dhcx_device_conf_set(&ag_ctx, PROPERTY_ENABLE);
  /* Some hardware require to enable pull up on master I2C interface. */
  //ism330dhcx_sh_pin_mode_set(&ag_ctx, ISM330DHCX_INTERNAL_PULL_UP);
  /* Check if IIS2MDC connected to Sensor Hub. */
  iis2mdc_device_id_get(&mag_ctx, &whoamI);

  if (whoamI != IIS2MDC_ID) {
    /* Magnetometer not detected. */
    while (1);
  }

  /* Configure IIS2MDC. */
  iis2mdc_block_data_update_set(&mag_ctx, PROPERTY_ENABLE);
  iis2mdc_offset_temp_comp_set(&mag_ctx, PROPERTY_ENABLE);
  iis2mdc_operating_mode_set(&mag_ctx, IIS2MDC_CONTINUOUS_MODE);
  iis2mdc_data_rate_set(&mag_ctx, IIS2MDC_ODR_20Hz);
  /* Configure Sensor Hub to read one slave. */
  ism330dhcx_sh_data_rate_set(&ag_ctx, ISM330DHCX_SH_ODR_26Hz);
  sh_cfg_read.slv_add = IIS2MDC_I2C_ADD; /* 8bit I2C address */
  sh_cfg_read.slv_subadd = IIS2MDC_OUTX_L_REG;
  sh_cfg_read.slv_len = 6;
  ism330dhcx_sh_slv0_cfg_read(&ag_ctx, &sh_cfg_read);
  /* Configure Sensor Hub to read one slave. */
  ism330dhcx_sh_slave_connected_set(&ag_ctx, ISM330DHCX_SLV_0);
  /* Enable I2C Master. */
  ism330dhcx_sh_master_set(&ag_ctx, PROPERTY_ENABLE);
  /* Set XL full scale and Gyro full scale. */
  ism330dhcx_xl_full_scale_set(&ag_ctx, ISM330DHCX_2g);
  ism330dhcx_gy_full_scale_set(&ag_ctx, ISM330DHCX_2000dps);
  /* Enable Block Data Update. */
  ism330dhcx_block_data_update_set(&ag_ctx, PROPERTY_ENABLE);
  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to 15 samples. 5 * (Acc + Gyro + Mag)
   */
  ism330dhcx_fifo_watermark_set(&ag_ctx, 15);
  /* Set FIFO mode to Stream mode (aka Continuous Mode). */
  ism330dhcx_fifo_mode_set(&ag_ctx, ISM330DHCX_STREAM_MODE);
  /* Enable latched interrupt notification. */
  ism330dhcx_int_notification_set(&ag_ctx, ISM330DHCX_ALL_INT_LATCHED);
  /* Enable drdy 75 us pulse: uncomment if interrupt must be pulsed. */
  //ism330dhcx_data_ready_mode_set(&ag_ctx, ISM330DHCX_DRDY_PULSED);
  /* FIFO watermark interrupt routed on INT1 pin.
   * Remember that INT1 pin is used by sensor to switch in I3C mode.
   */
  // ism330dhcx_pin_int1_route_get(&ag_ctx, &int1_route);
  // int1_route.fifo_th = PROPERTY_ENABLE;
  // ism330dhcx_pin_int1_route_set(&ag_ctx, int1_route);
  /* FIFO watermark interrupt routed on INT2 pin. */
  //ism330dhcx_pin_int2_route_get(&ag_ctx, NULL, &int2_route);
  //int2_route.fifo_th = PROPERTY_ENABLE;
  //ism330dhcx_pin_int2_route_set(&ag_ctx,  NULL, int2_route);
  /* Enable FIFO batching of Slave0. */
  ism330dhcx_sh_batch_slave_0_set(&ag_ctx, PROPERTY_ENABLE);
  /*Set FIFO batch XL/Gyro ODR to 26Hz.
   * Enable Timestamp batching with no decimation.
   */
  ism330dhcx_fifo_xl_batch_set(&ag_ctx, ISM330DHCX_XL_BATCHED_AT_26Hz);
  ism330dhcx_fifo_gy_batch_set(&ag_ctx, ISM330DHCX_GY_BATCHED_AT_26Hz);
  ism330dhcx_fifo_timestamp_decimation_set(&ag_ctx, ISM330DHCX_DEC_1);
  ism330dhcx_timestamp_set(&ag_ctx, PROPERTY_ENABLE);
  /* Enable I2C Master. */
  //ism330dhcx_sh_master_set(&ag_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate. */
  ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_26Hz);
  ism330dhcx_gy_data_rate_set(&ag_ctx, ISM330DHCX_GY_ODR_26Hz);

  while (1) {
    uint16_t num = 0;
    ism330dhcx_fifo_tag_t reg_tag;
    timestamp_sample_t ts_tick;
    uint32_t timestamp = 0.0f;
    /* Read FIFO when threshold is reached. */
    ism330dhcx_fifo_wtm_flag_get(&ag_ctx, &fifo_wtm);

    if (fifo_wtm) {
      /* Read number of samples in FIFO. */
      ism330dhcx_fifo_data_level_get(&ag_ctx, &num);

      while (num--) {
        /* Read FIFO tag. */
        ism330dhcx_fifo_sensor_tag_get(&ag_ctx, &reg_tag);

        switch (reg_tag) {
          case ISM330DHCX_XL_NC_TAG:
            memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
            ism330dhcx_fifo_out_raw_get(&ag_ctx, data_raw_acceleration.u8bit);
            acceleration_mg[0] =
              ism330dhcx_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
            acceleration_mg[1] =
              ism330dhcx_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
            acceleration_mg[2] =
              ism330dhcx_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);
            sprintf((char *)tx_buffer,
                    "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f T %u.%u ms\r\n",
                    acceleration_mg[0], acceleration_mg[1],
                    acceleration_mg[2], (unsigned int)(timestamp / 1000ULL),
                    (unsigned int)(timestamp % 1000ULL));
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
            break;

          case ISM330DHCX_GYRO_NC_TAG:
            memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
            ism330dhcx_fifo_out_raw_get(&ag_ctx, data_raw_angular_rate.u8bit);
            angular_rate_mdps[0] =
              ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
            angular_rate_mdps[1] =
              ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
            angular_rate_mdps[2] =
              ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);
            sprintf((char *)tx_buffer,
                    "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f T %u.%u ms\r\n",
                    angular_rate_mdps[0], angular_rate_mdps[1],
                    angular_rate_mdps[2], (unsigned int)(timestamp / 1000ULL),
                    (unsigned int)(timestamp % 1000ULL));
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
            break;

          case ISM330DHCX_SENSORHUB_SLAVE0_TAG:
            memset(data_raw_magnetic.u8bit, 0x00, 3 * sizeof(int16_t));
            ism330dhcx_fifo_out_raw_get(&ag_ctx, data_raw_magnetic.u8bit);
            magnetic_mG[0] =
              iis2mdc_from_lsb_to_mgauss(data_raw_magnetic.i16bit[0]);
            magnetic_mG[1] =
              iis2mdc_from_lsb_to_mgauss(data_raw_magnetic.i16bit[1]);
            magnetic_mG[2] =
              iis2mdc_from_lsb_to_mgauss(data_raw_magnetic.i16bit[2]);
            sprintf((char *)tx_buffer,
                    "Mag [mG]:%4.2f\t%4.2f\t%4.2f T %u.%u ms\r\n",
                    magnetic_mG[0], magnetic_mG[1], magnetic_mG[2],
                    (unsigned int)(timestamp / 1000ULL),
                    (unsigned int)(timestamp % 1000ULL));
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
            break;

          case ISM330DHCX_TIMESTAMP_TAG:
            ism330dhcx_fifo_out_raw_get(&ag_ctx, ts_tick.byte);
            timestamp = (unsigned int)ism330dhcx_from_lsb_to_nsec(
                          ts_tick.reg.tick);
            break;

          default:
            /* Flush unused samples. */
            memset(dummy.u8bit, 0x00, 3 * sizeof(int16_t));
            ism330dhcx_fifo_out_raw_get(&ag_ctx, dummy.u8bit);
            break;
        }
      }
    }
  }
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
static int32_t ism330dhcx_write_iis2mdc_cx(void *ctx, uint8_t reg,
                                           const uint8_t *data, uint16_t len)
{
  int16_t data_raw_acceleration[3];
  int32_t ret;
  uint8_t drdy;
  ism330dhcx_status_master_t master_status;
  ism330dhcx_sh_cfg_write_t sh_cfg_write;
  /* Configure Sensor Hub to read IIS2MDC. */
  sh_cfg_write.slv0_add = IIS2MDC_I2C_ADD; /* 8bit I2C address */
  sh_cfg_write.slv0_subadd = reg,
  sh_cfg_write.slv0_data = *data,
  ret = ism330dhcx_sh_cfg_write(&ag_ctx, &sh_cfg_write);
  /* Disable accelerometer. */
  ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_OFF);
  /* Enable I2C Master. */
  ism330dhcx_sh_master_set(&ag_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation. */
  ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_104Hz);
  /* Wait Sensor Hub operation flag set. */
  ism330dhcx_acceleration_raw_get(&ag_ctx, data_raw_acceleration);

  do {
    HAL_Delay(20);
    ism330dhcx_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while (!drdy);

  do {
    HAL_Delay(20);
    ism330dhcx_sh_status_get(&ag_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL (trigger). */
  ism330dhcx_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_OFF);
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
static int32_t ism330dhcx_read_iis2mdc_cx(void *ctx, uint8_t reg,
                                          uint8_t *data,
                                          uint16_t len)
{
  ism330dhcx_sh_cfg_read_t sh_cfg_read;
  int16_t data_raw_acceleration[3];
  int32_t ret;
  uint8_t drdy;
  ism330dhcx_status_master_t master_status;
  /* Disable accelerometer. */
  ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_OFF);
  /* Configure Sensor Hub to read IIS2MDC. */
  sh_cfg_read.slv_add = IIS2MDC_I2C_ADD; /* 8bit I2C address */
  sh_cfg_read.slv_subadd = reg;
  sh_cfg_read.slv_len = len;
  ret = ism330dhcx_sh_slv0_cfg_read(&ag_ctx, &sh_cfg_read);
  ism330dhcx_sh_slave_connected_set(&ag_ctx, ISM330DHCX_SLV_0);
  /* Enable I2C Master and I2C master. */
  ism330dhcx_sh_master_set(&ag_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation. */
  ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_104Hz);
  /* Wait Sensor Hub operation flag set. */
  ism330dhcx_acceleration_raw_get(&ag_ctx, data_raw_acceleration);

  do {
    HAL_Delay(20);
    ism330dhcx_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while (!drdy);

  do {
    //HAL_Delay(20);
    ism330dhcx_sh_status_get(&ag_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL(trigger). */
  ism330dhcx_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_OFF);
  /* Read SensorHub registers. */
  ism330dhcx_sh_read_data_raw_get(&ag_ctx,
                                  (ism330dhcx_emb_sh_read_t *)data, len);
  return ret;
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
  HAL_I2C_Mem_Write(handle, ISM330DHCX_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  ISM330DHCX_I2C_ADD_H & 0xFE, reg,
               (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, ISM330DHCX_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, ISM330DHCX_I2C_ADD_H & 0xFE, reg, bufp, len);
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
