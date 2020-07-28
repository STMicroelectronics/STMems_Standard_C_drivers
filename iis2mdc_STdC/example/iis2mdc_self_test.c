/*
 ******************************************************************************
 * @file    iis2mdc_self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file implements self test process.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI185V1
 * - NUCLEO_F411RE + STEVAL-MKI185V1
 * - DISCOVERY_SPC584B + STEVAL-MKI185V1
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
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
#define NUCLEO_F411RE    /* little endian */
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

#elif defined(SPC584B_DIS)
#include "components.h"
#endif

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME       20 //ms

#define SELF_TEST_SAMPLES  50

/* Self-test max value rang */
#define ST_MIN_POS    15.0f
#define ST_MAX_POS    500.0f

/* Private types -------------------------------------------------------------*/
typedef union{
  int16_t i16bit[3];
} axis3bit16_t;

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_magnetic[SELF_TEST_SAMPLES];
static float magnetic_mG[SELF_TEST_SAMPLES][3];
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

static inline float ABSF(float _x)
{
  return (_x < 0.0f) ? -(_x) : _x;
}

static int iis2mdc_flush_samples(stmdev_ctx_t *dev_ctx)
{
  uint8_t reg;
  axis3bit16_t dummy;
  int samples = 0;

  /* Discard old samples */
  iis2mdc_mag_data_ready_get(dev_ctx, &reg);
  if (reg)
  {
    iis2mdc_magnetic_raw_get(dev_ctx, dummy.i16bit);
    samples++;
  }

  return samples;
}

/*
 * @brief  execute self test procedure
 *
 * @param  dev_ctx   customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 *
 * @return 0: TEST PASSED
 *        -1: TEST FAILED
 */
static int test_self_test_iis2mdc(stmdev_ctx_t *dev_ctx)
{
  uint8_t reg;
  float media[3] = { 0.0f, 0.0f, 0.0f };
  float mediast[3] = { 0.0f, 0.0f, 0.0f };
  uint8_t match[3] = { 0, 0, 0 };
  uint8_t j = 0;
  uint16_t i = 0;
  uint8_t k = 0;
  uint8_t axis;
  int result = 0;

  /* Restore default configuration */
  iis2mdc_reset_set(dev_ctx, PROPERTY_ENABLE);
  do
  {
    iis2mdc_reset_get(dev_ctx, &rst);
  } while (rst);

  iis2mdc_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
  /* Set / Reset sensor mode */
  iis2mdc_set_rst_mode_set(dev_ctx, IIS2MDC_SENS_OFF_CANC_EVERY_ODR);

  /* Enable temperature compensation */
  iis2mdc_offset_temp_comp_set(dev_ctx, PROPERTY_ENABLE);

  /* Set device in continuous mode */
  iis2mdc_operating_mode_set(dev_ctx, IIS2MDC_CONTINUOUS_MODE);

  /* Set Output Data Rate to 100 Hz */
  iis2mdc_data_rate_set(dev_ctx, IIS2MDC_ODR_100Hz);

  /* Power up and wait for 20 ms for stable output */
  platform_delay(20);

  /* Flush old samples */
  iis2mdc_flush_samples(dev_ctx);

  do
  {
    iis2mdc_mag_data_ready_get(dev_ctx, &reg);
    if (reg)
    {
      /* Read magnetic field data */
      memset(data_raw_magnetic[i].i16bit, 0x00, 3 * sizeof(int16_t));
      iis2mdc_magnetic_raw_get(dev_ctx, data_raw_magnetic[i].i16bit);
      for (axis = 0; axis < 3; axis++)
        magnetic_mG[i][axis] =
          iis2mdc_from_lsb_to_mgauss(data_raw_magnetic[i].i16bit[axis]);

      i++;
    }
  } while (i < SELF_TEST_SAMPLES);

  for (k = 0; k < 3; k++)
  {
    for (j = 0; j < SELF_TEST_SAMPLES; j++)
    {
      media[k] += magnetic_mG[j][k];
    }

    media[k] = (media[k] / j);
  }

  /* Enable self test mode */
  iis2mdc_self_test_set(dev_ctx, PROPERTY_ENABLE);
  platform_delay(60);
  i = 0;

  /* Flush old samples */
  iis2mdc_flush_samples(dev_ctx);

  do
  {
    iis2mdc_mag_data_ready_get(dev_ctx, &reg);
    if (reg)
    {
      /* Read accelerometer data */
      memset(data_raw_magnetic[i].i16bit, 0x00, 3 * sizeof(int16_t));
      iis2mdc_magnetic_raw_get(dev_ctx, data_raw_magnetic[i].i16bit);
      for (axis = 0; axis < 3; axis++)
      magnetic_mG[i][axis] =
        iis2mdc_from_lsb_to_mgauss(data_raw_magnetic[i].i16bit[axis]);

      i++;
    }
  } while (i < SELF_TEST_SAMPLES);

  for (k = 0; k < 3; k++)
  {
    for (j = 0; j < SELF_TEST_SAMPLES; j++)
    {
      mediast[k] += magnetic_mG[j][k];
    }

    mediast[k] = (mediast[k] / j);
  }

  /* Check for all axis self test value range */
  for (k = 0; k < 3; k++)
  {
    if ((ABSF(mediast[k] - media[k]) >= ST_MIN_POS) &&
        (ABSF(mediast[k] - media[k]) <= ST_MAX_POS))
    {
      match[k] = 1;
      result += 1;
    }

    sprintf((char*)tx_buffer, "%d: |%f| <= |%f| <= |%f| %s\r\n", k,
            ST_MIN_POS, ABSF(mediast[k] - media[k]), ST_MAX_POS,
            match[k] == 1 ? "PASSED" : "FAILED");
    tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
  }

  /* Disable self test mode */
  iis2mdc_operating_mode_set(dev_ctx, IIS2MDC_POWER_DOWN);
  iis2mdc_self_test_set(dev_ctx, PROPERTY_DISABLE);

  return result == 3 ? 0 : -1;
}

/* Main Example --------------------------------------------------------------*/
void iis2mdc_self_test(void)
{
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Initialize platform specific hardware */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

#if defined(STEVAL_MKI109V3)
  /* Default SPI mode is 3 wire, so enable 4 wire mode */
  iis2mdc_spi_mode_set(&dev_ctx, LIS2MDL_SPI_4_WIRE);
#endif

  /* Check device ID */
  iis2mdc_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != IIS2MDC_ID) {
    while(1) {
      /* manage here device not found */
    }
  }

  while(1) {
    test_self_test_iis2mdc(&dev_ctx);
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
#if defined(NUCLEO_F411RE)
    HAL_I2C_Mem_Write(handle, IIS2MDC_I2C_ADD, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  IIS2MDC_I2C_ADD & 0xFE, reg, bufp, len);
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
  HAL_I2C_Mem_Read(handle, IIS2MDC_I2C_ADD, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
    reg |= 0x80;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, IIS2MDC_I2C_ADD & 0xFE, reg, bufp, len);
#endif
  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to trasmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  #ifdef NUCLEO_F411RE
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
  #endif
  #ifdef STEVAL_MKI109V3
  CDC_Transmit_FS(tx_buffer, len);
  #endif
}

/* @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
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
#ifdef STEVAL_MKI109V3
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}
