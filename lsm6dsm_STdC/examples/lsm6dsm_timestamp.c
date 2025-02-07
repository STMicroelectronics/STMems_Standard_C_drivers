/*
 ******************************************************************************
 * @file    enable_timestamp_hw.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor
 *          (interrupt enabled) with timestamp.
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
 * - DISCOVERY_SPC584B + STEVAL-MKI189V1
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
#include "lsm6dsm_reg.h"

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
  uint8_t byte[4];
  uint32_t val;
} timestamp_t;

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME          15 //ms

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
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

/* Main Example --------------------------------------------------------------*/
void lsm6dsm_timestamp(void)
{
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  lsm6dsm_int1_route_t int_1_reg;
  timestamp_t timestamp;
  /* Interrupt generation on DRDY INT2 pin */
  //lsm6dsm_int2_route_t int_2_reg;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
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

  /* Enable Block Data Update */
  lsm6dsm_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lsm6dsm_xl_full_scale_set(&dev_ctx, LSM6DSM_2g);
  lsm6dsm_gy_full_scale_set(&dev_ctx, LSM6DSM_2000dps);
  /* Set High Resolution Timestamp (25 us tick) */
  lsm6dsm_timestamp_res_set(&dev_ctx, LSM6DSM_LSB_25us);
  /* Enable timestamp in HW */
  lsm6dsm_timestamp_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate for acc/gyro to 12.5 Hz */
  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_12Hz5);
  lsm6dsm_gy_data_rate_set(&dev_ctx, LSM6DSM_GY_ODR_12Hz5);
  /* Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed */
  //lsm6dsm_data_ready_mode_set(&dev_ctx, LSM6DSM_DRDY_PULSED);
  /* Enable interrupt generation on DRDY INT1 pin */
  lsm6dsm_pin_int1_route_get(&dev_ctx, &int_1_reg);
  int_1_reg.int1_drdy_g = PROPERTY_ENABLE;
  int_1_reg.int1_drdy_xl = PROPERTY_ENABLE;
  lsm6dsm_pin_int1_route_set(&dev_ctx, int_1_reg);
  /* Interrupt generation routed on DRDY INT2 pin */
  //lsm6dsm_pin_int2_route_get(&dev_ctx, &int_2_reg);
  //int_2_reg.int2_drdy_g = PROPERTY_ENABLE;
  //int_2_reg.int2_drdy_xl = PROPERTY_ENABLE;
  //lsm6dsm_pin_int2_route_set(&dev_ctx, int_2_reg);

  /* Wait samples */
  while (1) {
    lsm6dsm_reg_t reg;
    /*
     * Read status register
     */
    lsm6dsm_status_reg_get(&dev_ctx, &reg.status_reg);

    if (reg.status_reg.xlda) {
      /* Read timestamp */
      lsm6dsm_read_reg(&dev_ctx,
                       LSM6DSM_TIMESTAMP0_REG,
                       (uint8_t *)timestamp.byte, 3);
      /* Read accelerometer field data and append timestamp information
       * in us (LSB in timestamp counter is 25 us)
       */
      memset(data_raw_acceleration, 0, 3 * sizeof(int16_t));
      lsm6dsm_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] =
        lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
        lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
        lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[2]);
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\t%ld us\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
              timestamp.val * 25);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (reg.status_reg.gda) {
      /* Read timestamp */
      lsm6dsm_read_reg(&dev_ctx,
                       LSM6DSM_TIMESTAMP0_REG,
                       (uint8_t *)timestamp.byte, 3);
      /* Read gyroscope field data and append timestamp information
       * in us(LSB in timestamp counter is 25 us)
       */
      memset(data_raw_angular_rate, 0, 3 * sizeof(int16_t));
      lsm6dsm_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
      angular_rate_mdps[0] =
        lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
      angular_rate_mdps[1] =
        lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
      angular_rate_mdps[2] =
        lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\t%ld us\r\n",
              angular_rate_mdps[0], angular_rate_mdps[1],
              angular_rate_mdps[2], timestamp.val * 25);
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
