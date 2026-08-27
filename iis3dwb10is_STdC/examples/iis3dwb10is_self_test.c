/*
 ******************************************************************************
 * @file    iis3dwb10is_self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file implements the self test procedure.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
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
 * - STEVAL_MKI109D  +
 * - STEVAL_MKI109V3 + STEVAL-MKI208V1K
 * - DISCOVERY_SPC584B + STEVAL-MKI208V1K
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109D     - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
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
//#define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#if defined(STEVAL_MKI109D)
/* MKI109D: Define communication interface */
#define SENSOR_BUS hspi1

/* MKI109D: Vdd and Vddio power supply values */
#define LSM6DSV16X_VDD 1.8f
#define LSM6DSV16X_VDDIO 1.8f

#elif defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2
/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS NULL
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "iis3dwb10is_reg.h"

#if defined(STEVAL_MKI109D)
#include "board.h"
#include "usbd_cdc_if.h"

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
#define    BOOT_TIME        10 //ms
#define    WAIT_TIME       10 //ms

/* Self test limits. */
#define    MIN_ST_LIMIT_mg        4.0f
#define    MAX_ST_LIMIT_mg       9.0f

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

/* Private variables ---------------------------------------------------------*/
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

#define SENS 0.000095

static struct self_test {
  uint16_t sample_num;
  uint32_t turn_on_delay_ms;
  uint32_t self_test_delay_ms;
  uint8_t debug_en;
  int8_t sign;
  const float limits[3][2];
} self_test = {
  .sample_num = 8,
  .debug_en = 0,
  .limits = { { 4.0f, 9.0f }, { 4.0f, 9.0f }, { 1.0f, 7.0f } },
};

static volatile uint8_t drdy;

void iis3dwb10is_self_test_handler(void)
{
  drdy = 1;
}

/* Main Example --------------------------------------------------------------*/
static uint8_t iis3dwb10is_self_test_run(stmdev_ctx_t *ctx)
{
  int32_t acc[3];
  float no_st[3] = { 0.0f, };
  float st[3] = { 0.0f, };
  uint8_t ret = 0;
  uint8_t val;

  /* Wait stable output */
  platform_delay(WAIT_TIME);

  drdy = 0;
  iis3dwb10is_read_reg(ctx, 0x24, (uint8_t *)acc, sizeof(acc));

  for (uint16_t n = 0; n < self_test.sample_num; n++) {
    while (!drdy);
    drdy = 0;
    iis3dwb10is_read_reg(ctx, 0x24, (uint8_t *)acc, sizeof(acc));
    if (self_test.debug_en)
    {
      sprintf((char *)tx_buffer,
              "Acceleration [g]:%4.2f\t%4.2f\t%4.2f\r\n",
              acc[0] * SENS, acc[1] * SENS, acc[2] * SENS);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    for (uint8_t i = 0; i < 3; i++) {
      no_st[i] += acc[i] * SENS;
    }
  }

  for (uint8_t i = 0; i < 3; i++) {
    no_st[i] /= self_test.sample_num;
  }

  if (self_test.sign) {
    // enable positive self-test
    val = 0x06;
    iis3dwb10is_write_reg(ctx, 0x17, &val, 1);

  } else {
    // enable negative self-test
    val = 0x07;
    iis3dwb10is_write_reg(ctx, 0x17, &val, 1);
  }

  platform_delay(10);

  iis3dwb10is_read_reg(ctx, 0x24, (uint8_t *)acc, sizeof(acc));
  drdy = 0;

  for (uint16_t n = 0; n < self_test.sample_num; n++) {
    while (!drdy);
    drdy = 0;
    iis3dwb10is_read_reg(ctx, 0x24, (uint8_t *)acc, sizeof(acc));

    if (self_test.debug_en)
    {
      sprintf((char *)tx_buffer,
              "Acceleration slef-test [g]:%4.2f\t%4.2f\t%4.2f\r\n",
              acc[0] * SENS, acc[1] * SENS, acc[2] * SENS);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    for (uint8_t i = 0; i < 3; i++) {
      st[i] += acc[i] * SENS;
    }
  }

  for (uint8_t i = 0; i < 3; i++) {
    st[i] /= self_test.sample_num;
  }

  // disable self-test
  val = 0x00;
  iis3dwb10is_write_reg(ctx, 0x17, &val, 1);

  // int1_drdy_xl = 0
  val = 0x00;
  iis3dwb10is_write_reg(ctx, 0x0D, &val, 1);

  // odr_xl = 0 Hz
  val = 0x00;
  iis3dwb10is_write_reg(ctx, 0x10, &val, 1);

  // bdu = 0 + if_inc = 1
  val = 0x04;
  iis3dwb10is_write_reg(ctx, 0x12, &val, 1);

  for (uint8_t i = 0; i < 3; i++) {
    if (fabsf(st[i] - no_st[i]) < self_test.limits[i][0] || fabsf(st[i] - no_st[i]) > self_test.limits[i][1])
    {
      ret = 1;
    }
  }

  if (self_test.debug_en)
  {
    sprintf((char *)tx_buffer, "no_st = [%f, %f, %f]\r\nst = [%f, %f, %f]\r\ndiff = [%f, %f, %f]\r\n",
            (double)no_st[0], (double)no_st[1], (double)no_st[2],
            (double)st[0], (double)st[1], (double)st[2],
            (double)(fabsf(st[0] - no_st[0])), (double)(fabsf(st[1] - no_st[1])), (double)(fabsf(st[2] - no_st[2])));
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }

  return ret;
}

void iis3dwb10is_self_test(void)
{
  iis3dwb10is_int_pin_t pin_ctrl;
  iis3dwb10is_data_rate_t rate;
  iis3dwb10is_pin_int_route_t route = {0};
  stmdev_ctx_t dev_ctx;
  uint8_t whoamI;
  uint8_t ret = 0;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Init test platform */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  iis3dwb10is_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != IIS3DWB10IS_ID)
    while (1);

  /* Restore default configuration */
  iis3dwb10is_sw_por(&dev_ctx);

  /* Enable Block Data Update */
  iis3dwb10is_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* interrupt pins electrical configurations */
  pin_ctrl.pp_od = IIS3DWB10IS_PUSH_PULL;
  pin_ctrl.pd_dis = IIS3DWB10IS_PD_INT1_OFF_INT2_OFF;
  pin_ctrl.strength = IIS3DWB10IS_PAD_STRENGTH_LOWER;
  iis3dwb10is_interrupt_pin_mode_set(&dev_ctx,  pin_ctrl);

  while (1)
  {
    /*
    * Accelerometer Self Test
    */
    /* Set Output Data Rate */
    rate.burst = IIS3DWB10IS_CONTINUOS_MODE;
    rate.odr = IIS3DWB10IS_ODR_2KHz5;
    iis3dwb10is_xl_data_rate_set(&dev_ctx, rate);

    /* Set full scale */
    iis3dwb10is_xl_full_scale_set(&dev_ctx, IIS3DWB10IS_50g);

    iis3dwb10is_pin_int1_route_get(&dev_ctx, &route);
    route.drdy_xl = 1;
    iis3dwb10is_pin_int1_route_set(&dev_ctx, route);

    self_test.sign = (self_test.sign) ? 0 : 1;
    ret = iis3dwb10is_self_test_run(&dev_ctx);

    if (ret)
    {
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "Self Test (%d) - FAIL\r\n", self_test.sign);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    } else {
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "Self Test (%d) - PASS\r\n", self_test.sign);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    platform_delay(500);
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
#if defined(STEVAL_MKI109D)
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
#elif defined(STEVAL_MKI109V3)
  if (handle == &hspi2) {
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }

#elif defined(SPC584B_DIS)
  /* Add here the SPC5 write SPI interface */
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
#if defined(STEVAL_MKI109D)
  reg |= 0x80;
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(handle, bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
#elif defined(STEVAL_MKI109V3)
  if (handle == &hspi2) {
    /* Read command */
    reg |= 0x80;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }

#elif defined(SPC584B_DIS)
  /* Add here the SPC5 read SPI interface */
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
#if defined(STEVAL_MKI109D)
  CDC_Transmit_FS(tx_buffer, len);
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
#if defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(STEVAL_MKI109D)
  delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#if defined(STEVAL_MKI109D)
  struct spi_conf spi_conf;

  /* init SPI bus communication */
  spi_conf.wire = WIRE_4;
  spi_init(&spi_conf);

  /* set VDD/VDDIO on DIL24 */
  set_vdd(LSM6DSV16X_VDD);
  set_vddio(LSM6DSV16X_VDDIO);
  delay(100);

#elif defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}
