/*
 ******************************************************************************
 * @file    ispu_norm.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to get data from sensor.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI208V1K
 * - STEVAL_MKI109D  + STEVAL-MKI208V1K
 *
 * Used interfaces:
 *
 * STEVAL_MKI109D     - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default)
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default)
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

#if defined(STEVAL_MKI109D)
/* MKI109D: Define communication interface */
#define SENSOR_BUS hspi1

/* MKI109D: Vdd and Vddio power supply values */
#define ISS3DWB10IS_VDD 3.3f
#define ISS3DWB10IS_VDDIO 3.3f
#elif defined(STEVAL_MKI109V3)
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
#include "iis3dwb10is/norm/output/norm.h"
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
#define    BOOT_TIME        20 //ms

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
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

static   stmdev_ctx_t dev_ctx;
static volatile uint8_t task_event = 0;

void iis3dwb10is_ispu_norm_handler()
{
  task_event = 1;
}

/* Main Example --------------------------------------------------------------*/
void iis3dwb10is_ispu_norm(void)
{
  iis3dwb10is_int_pin_t pin_ctrl;
  uint16_t i;

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

  /* interrupt pins electrical configurations */
  pin_ctrl.pp_od = IIS3DWB10IS_PUSH_PULL;
  pin_ctrl.pd_dis = IIS3DWB10IS_PD_INT1_OFF_INT2_OFF;
  pin_ctrl.strength = IIS3DWB10IS_PAD_STRENGTH_LOWER;
  iis3dwb10is_interrupt_pin_mode_set(&dev_ctx,  pin_ctrl);

  /* Load ISPU configuration */
  for ( i = 0; i < (sizeof(ispu_conf_conf_0) / sizeof(struct mems_conf_op) ); i++ ) {
    switch(ispu_conf_conf_0[i].type) {
    case MEMS_CONF_OP_TYPE_DELAY:
      platform_delay(ispu_conf_conf_0[i].data);
      break;
    case MEMS_CONF_OP_TYPE_WRITE:
      iis3dwb10is_write_reg(&dev_ctx, ispu_conf_conf_0[i].address, (uint8_t *)&ispu_conf_conf_0[i].data, 1);
      break;
    }
  }

  /* Read norm result in interrupt handler */
  while (1)
  {
    if (task_event)
    {
      uint16_t ispu_int;
      uint8_t dout[16];
      int32_t x, y, z;
      int32_t temp;
      float_t norm;

      task_event = 0;

      iis3dwb10is_ispu_int_status_get(&dev_ctx, &ispu_int);

      /* handle only ISPU INT1 interrupts */
      if ((ispu_int & 0x1) == 0)
        continue;

      iis3dwb10is_ispu_read_data_raw_get(&dev_ctx, dout, 16);

      x = (int32_t)dout[3];
      x = (x * 256) + (int32_t)dout[2];
      x = (x * 256) + (int32_t)dout[1];
      x = (x * 256) + (int32_t)dout[0];
      y = (int32_t)dout[7];
      y = (y * 256) + (int32_t)dout[6];
      y = (y * 256) + (int32_t)dout[5];
      y = (y * 256) + (int32_t)dout[4];
      z = (int32_t)dout[11];
      z = (z * 256) + (int32_t)dout[10];
      z = (z * 256) + (int32_t)dout[9];
      z = (z * 256) + (int32_t)dout[8];

      temp = (dout[15] << 24) | (dout[14] << 16) | (dout[13] << 8) | dout[12];
      norm = *(float_t *)&temp;

      snprintf((char *)tx_buffer, sizeof(tx_buffer), "x: %4.2f mg\ty: %4.2f mg\tz: %4.2f mg\tnorm: %4.2f mg\r\n",
               iis3dwb10is_from_fs100g_to_mg(x), iis3dwb10is_from_fs100g_to_mg(y),
               iis3dwb10is_from_fs100g_to_mg(z), iis3dwb10is_from_fs100g_to_mg(norm));
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
#if defined(STEVAL_MKI109D)
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  IIS3DWB10IS_I2C_ADD_H & 0xFE, reg, (uint8_t*) bufp, len);
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
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, IIS3DWB10IS_I2C_ADD_H & 0xFE, reg, bufp, len);
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

  set_range_100x(1);

  /* set VDD/VDDIO on DIL24 */
  set_vdd(ISS3DWB10IS_VDD);
  set_vddio(ISS3DWB10IS_VDDIO);
  delay(100);
#elif defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}
