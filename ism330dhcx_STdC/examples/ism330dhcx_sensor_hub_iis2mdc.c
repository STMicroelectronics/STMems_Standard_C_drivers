/*
 ******************************************************************************
 * @file    _sensor_hub_iis2mdc.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to get data from magnetometer sensor through
 *          sensor hub.
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
 * - NUCLEO_F401RE + X-NUCLEO-IKS02A1
 * - DISCOVERY_SPC584B + X-NUCLEO-IKS02A1
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

#include "ism330dhcx_reg.h"
#include "iis2mdc_reg.h"

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

/* Private macro -------------------------------------------------------------*/
#define TX_BUF_DIM                       1000

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[TX_BUF_DIM];

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
void ism330dhcx_mlc_iis2mdc(void)
{
  ism330dhcx_pin_int1_route_t pin_int1_route;
  ism330dhcx_sh_cfg_read_t sh_cfg_read;
  ism330dhcx_status_reg_t status;
  float angular_rate_mdps[3];
  float acceleration_mg[3];
  float mag_fielg_mG[3];
  axis3bit16_t data_raw;
  uint8_t whoamI, rst;
  uint16_t dummy;
  /* Initialize ism330dhcx driver interface */
  ag_ctx.write_reg = platform_write;
  ag_ctx.read_reg = platform_read;
  ag_ctx.handle = &SENSOR_BUS;
  /* Initialize iis2mdc driver interface */
  mag_ctx.read_reg = ism330dhcx_read_iis2mdc_cx;
  mag_ctx.write_reg = ism330dhcx_write_iis2mdc_cx;
  mag_ctx.handle = &SENSOR_BUS;
  /* initialize HW */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(10);
  /* Check ism330dhcx ID. */
  ism330dhcx_device_id_get(&ag_ctx, &whoamI);

  if (whoamI != ISM330DHCX_ID)
    while (1);

  /* Restore default configuration. */
  ism330dhcx_reset_set(&ag_ctx, PROPERTY_ENABLE);

  do {
    ism330dhcx_reset_get(&ag_ctx, &rst);
  } while (rst);

  /* Start device configuration. */
  ism330dhcx_device_conf_set(&ag_ctx, PROPERTY_ENABLE);
  /* Disable I3C interface.*/
  //ism330dhcx_i3c_disable_set(&ag_ctx, ISM330DHCX_I3C_DISABLE);
  /* Turn off Sensors */
  ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_OFF);
  ism330dhcx_gy_data_rate_set(&ag_ctx, ISM330DHCX_GY_ODR_OFF);
  /* Some hardware require to enable pull up on master I2C interface. */
  //ism330dhcx_sh_pin_mode_set(&ag_ctx, ISM330DHCX_INTERNAL_PULL_UP);
  /* Check if IIS2MDC connected to Sensor Hub. */
  iis2mdc_device_id_get(&mag_ctx, &whoamI);

  if (whoamI != IIS2MDC_ID)
    while (1);

  /* Configure IIS2MDC. */
  iis2mdc_block_data_update_set(&mag_ctx, PROPERTY_ENABLE);
  iis2mdc_offset_temp_comp_set(&mag_ctx, PROPERTY_ENABLE);
  iis2mdc_operating_mode_set(&mag_ctx, IIS2MDC_CONTINUOUS_MODE);
  iis2mdc_data_rate_set(&mag_ctx, IIS2MDC_ODR_20Hz);
  /* Prepare sensor hub to read data from external Slave0 continuously */
  ism330dhcx_sh_data_rate_set(&ag_ctx, ISM330DHCX_SH_ODR_26Hz);
  sh_cfg_read.slv_add = IIS2MDC_I2C_ADD; /* 8bit I2C address */
  sh_cfg_read.slv_subadd = IIS2MDC_OUTX_L_REG;
  sh_cfg_read.slv_len = 6;
  ism330dhcx_sh_slv0_cfg_read(&ag_ctx, &sh_cfg_read);
  /* Configure Sensor Hub to read one slave. */
  ism330dhcx_sh_slave_connected_set(&ag_ctx, ISM330DHCX_SLV_0);
  /* Enable I2C Master. */
  ism330dhcx_sh_master_set(&ag_ctx, PROPERTY_ENABLE);
  /* Set xl and gy full scale */
  ism330dhcx_xl_full_scale_set(&ag_ctx, ISM330DHCX_2g);
  ism330dhcx_gy_full_scale_set(&ag_ctx, ISM330DHCX_2000dps);
  /* Turn on Sensors */
  ism330dhcx_xl_data_rate_set(&ag_ctx, ISM330DHCX_XL_ODR_26Hz);
  ism330dhcx_gy_data_rate_set(&ag_ctx, ISM330DHCX_GY_ODR_26Hz);
  /* Route signals on interrupt pin 1 */
  ism330dhcx_pin_int1_route_get(&ag_ctx, &pin_int1_route);
  //pin_int1_route.mlc1 = PROPERTY_ENABLE;
  ism330dhcx_pin_int1_route_set(&ag_ctx, &pin_int1_route);

  while (1) {
    /* Read output only if new xl and gy value is available */
    ism330dhcx_status_reg_get(&ag_ctx, &status);

    if (status.gda && status.xlda) {
      /* Read acceleration data */
      ism330dhcx_acceleration_raw_get(&ag_ctx, data_raw.i16bit);
      acceleration_mg[0] =
        ism330dhcx_from_fs2g_to_mg(data_raw.i16bit[0]);
      acceleration_mg[1] =
        ism330dhcx_from_fs2g_to_mg(data_raw.i16bit[1]);
      acceleration_mg[2] =
        ism330dhcx_from_fs2g_to_mg(data_raw.i16bit[2]);
      /* Read angular rate */
      ism330dhcx_angular_rate_raw_get(&ag_ctx, data_raw.i16bit);
      angular_rate_mdps[0] =
        ism330dhcx_from_fs2000dps_to_mdps(data_raw.i16bit[0]);
      angular_rate_mdps[1] =
        ism330dhcx_from_fs2000dps_to_mdps(data_raw.i16bit[1]);
      angular_rate_mdps[2] =
        ism330dhcx_from_fs2000dps_to_mdps(data_raw.i16bit[2]);
      /* Read mag field */
      ism330dhcx_sh_read_data_raw_get(&ag_ctx,
                                      (ism330dhcx_emb_sh_read_t *)data_raw.u8bit, 6);
      /*fix data format with endianness */
      dummy = data_raw.u8bit[1];
      data_raw.i16bit[0] = dummy * 256 + data_raw.u8bit[0];
      dummy = data_raw.u8bit[3];
      data_raw.i16bit[1] = dummy * 256 + data_raw.u8bit[2];
      dummy = data_raw.u8bit[5];
      data_raw.i16bit[2] = dummy * 256 + data_raw.u8bit[4];
      mag_fielg_mG[0] = iis2mdc_from_lsb_to_mgauss(data_raw.i16bit[0]);
      mag_fielg_mG[1] = iis2mdc_from_lsb_to_mgauss(data_raw.i16bit[1]);
      mag_fielg_mG[2] = iis2mdc_from_lsb_to_mgauss(data_raw.i16bit[2]);
      sprintf((char *)tx_buffer,
              "xl[mg]:%4.2f\t%4.2f\t%4.2f\t"
              "gy[mdps]:%4.2f\t%4.2f\t%4.2f\t"
              "mag[mG]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2],
              mag_fielg_mG[0], mag_fielg_mG[1], mag_fielg_mG[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
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
#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Write(handle, ISM330DHCX_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  ISM330DHCX_I2C_ADD_H & 0xFE,
               (uint8_t*) reg, bufp, len);
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
