/*
 ******************************************************************************
 * @file    activity.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to detect activity/inactivity from sensor.
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
#include "ism330bx_reg.h"

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

/* Private variables ---------------------------------------------------------*/
static   ism330bx_filt_settling_mask_t filt_settling_mask;
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static   ism330bx_act_wkup_time_windows_t time_windows;

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
void ism330bx_activity(void)
{

  ism330bx_act_thresholds_t act_thresholds;
  ism330bx_all_sources_t all_sources;
  ism330bx_reset_t rst;
  stmdev_ctx_t dev_ctx;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Init test platform */
  platform_init();

  /* Wait Boot Time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  ism330bx_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != ISM330BX_ID)
    while (1);

  /* Restore default configuration */
  ism330bx_reset_set(&dev_ctx, ISM330BX_RESTORE_CTRL_REGS);
  do {
    ism330bx_reset_get(&dev_ctx, &rst);
  } while (rst != ISM330BX_READY);

  /* Enable Block Data Update */
  ism330bx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  ism330bx_xl_data_rate_set(&dev_ctx, ISM330BX_XL_ODR_AT_30Hz);
  ism330bx_gy_data_rate_set(&dev_ctx, ISM330BX_GY_ODR_AT_60Hz);

  /* Set full scale */
  ism330bx_xl_full_scale_set(&dev_ctx, ISM330BX_2g);
  ism330bx_gy_full_scale_set(&dev_ctx, ISM330BX_2000dps);

  /* Configure filtering chain */
  filt_settling_mask.drdy = PROPERTY_ENABLE;
  filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  filt_settling_mask.irq_g = PROPERTY_ENABLE;
  ism330bx_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
  ism330bx_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
  ism330bx_filt_gy_lp1_bandwidth_set(&dev_ctx, ISM330BX_GY_ULTRA_LIGHT);
  ism330bx_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
  ism330bx_filt_xl_lp2_bandwidth_set(&dev_ctx, ISM330BX_XL_STRONG);

  /* Set duration for Wake up detection to (= 2 * 1 / ODR_XL_LOW) */
  time_windows.shock = 2;

  /* Set duration for Inactivity detection to (= 1 * 512 / ODR_XL) */
  time_windows.quiet = 2;
  ism330bx_act_wkup_time_windows_set(&dev_ctx, time_windows);

  /* Set Wake-Up / Inactivity threshold */
  act_thresholds.wk_ths_mg = 100;
  act_thresholds.inact_ths_mg = 50;
  ism330bx_act_thresholds_set(&dev_ctx, act_thresholds);

  /* Change ODR immediately when detect wuakeup */
  ism330bx_act_from_sleep_to_act_dur_set(&dev_ctx, ISM330BX_SLEEP_TO_ACT_AT_1ST_SAMPLE);

  /* Inactivity configuration: XL to LP, gyro to Power-Down */
  ism330bx_act_sleep_xl_odr_set(&dev_ctx, ISM330BX_15Hz);
  ism330bx_act_mode_set(&dev_ctx, ISM330BX_XL_LOW_POWER_GY_POWER_DOWN);

  sprintf((char *)tx_buffer, "Ready\r\n");
  tx_com(tx_buffer, strlen((char const *)tx_buffer));

  /* Wait Events */
  while (1) {
    /* Check if Activity/Inactivity events */
    ism330bx_all_sources_get(&dev_ctx, &all_sources);

    if (all_sources.sleep_state) {
      sprintf((char *)tx_buffer, "Inactivity Detected\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (all_sources.wake_up) {
      sprintf((char *)tx_buffer, "Activity Detected\r\n");
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
  HAL_I2C_Mem_Write(handle, ISM330BX_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  ISM330BX_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, ISM330BX_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, ISM330BX_I2C_ADD_L & 0xFE, reg, bufp, len);
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
