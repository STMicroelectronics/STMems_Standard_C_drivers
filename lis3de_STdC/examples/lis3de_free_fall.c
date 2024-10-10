/*
 ******************************************************************************
 * @file    free_fall.c
 * @author  Sensors Software Solution Team
 * @brief   LIS3DE driver file
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
 * - STEVAL_MKI109V3
 * - NUCLEO_F401RE
 * - DISCOVERY_SPC584B
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

/* Pin configured as platform interrupt from LIS3DE INT1. */
#define LIS3DE_INT1_PIN GPIO_PIN_4
#define LIS3DE_INT1_GPIO_PORT GPIOA

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS I2CD1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lis3de_reg.h"

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
#define TX_BUF_DIM          1000

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[TX_BUF_DIM];

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
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);
static int32_t platform_reap_int_pin(void);

/* Main Example --------------------------------------------------------------*/

/*
 * Set interrupt threshold to 16h -> 350 mg
 * Set interrupt Duration to 03h -> minimum event duration 30 ms
 * Configure free-fall recognition
 * Poll on platform INT pin 1 waiting for free fall event detection
 */
void lis3de_freefall(void)
{
  stmdev_ctx_t dev_ctx;
  lis3de_ctrl_reg3_t ctrl_reg3;
  lis3de_ig1_cfg_t ig1_cfg;
  uint8_t whoamI;
  /* Initialize mems driver interface. */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  /* Initialize platform specific hardware */
  platform_init();
  /* Check device ID. */
  whoamI = 0;
  lis3de_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LIS3DE_ID)
    while (1); /* manage here device not found */

  /* Set Output Data Rate to 100 Hz. */
  lis3de_data_rate_set(&dev_ctx, LIS3DE_ODR_100Hz);
  /* Set full scale to 2 g. */
  lis3de_full_scale_set(&dev_ctx, LIS3DE_2g);
  /* Enable AOI1 interrupt on INT pin 1. */
  memset((uint8_t *)&ctrl_reg3, 0, sizeof(ctrl_reg3));
  ctrl_reg3.int1_ig1 = PROPERTY_ENABLE;
  lis3de_pin_int1_config_set(&dev_ctx, &ctrl_reg3);
  /* Enable Interrupt 1 pin latched. */
  lis3de_int1_pin_notification_mode_set(&dev_ctx, LIS3DE_INT1_LATCHED);
  /*
   * Set threshold to 16h -> 350 mg
   * Set Duration to 03h -> minimum event duration
   * If acceleration an all axis is below the threshold for more
   * than 30 ms than device is falling down
   */
  lis3de_int1_gen_threshold_set(&dev_ctx, 0x16);
  lis3de_int1_gen_duration_set(&dev_ctx, 0x03);
  /*
   * Configure free-fall recognition
   * Enable condiction (AND) for x, y, z acc. data below threshold.
   */
  memset((uint8_t *)&ig1_cfg, 0, sizeof(ig1_cfg));
  ig1_cfg.aoi = PROPERTY_ENABLE;
  ig1_cfg.zlie = PROPERTY_ENABLE;
  ig1_cfg.ylie = PROPERTY_ENABLE;
  ig1_cfg.xlie = PROPERTY_ENABLE;
  lis3de_int1_gen_conf_set(&dev_ctx, &ig1_cfg);
  /* Set device in HR mode. */
  lis3de_operating_mode_set(&dev_ctx, LIS3DE_LP);

  while (1) {
    /* Read INT pin 1 in polling mode. */
    lis3de_ig1_source_t src;

    if (platform_reap_int_pin()) {
      lis3de_int1_gen_source_get(&dev_ctx, &src);
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "freefall detected\r\n");
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
  /* Write multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Write(handle, LIS3DE_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Write multiple command */
  reg |= 0x40;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  /* Write multiple command */
  reg |= 0x80;
  i2c_lld_write(handle,  LIS3DE_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  /* Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read(handle, LIS3DE_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Read multiple command */
  reg |= 0xC0;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  /* Read multiple command */
  reg |= 0x80;
  i2c_lld_read(handle, LIS3DE_I2C_ADD_L & 0xFE, reg, bufp, len);
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
  platform_delay(1000);
#endif
}

/*
 * @brief  Function to read external interrupt pin.
 */
static int32_t platform_reap_int_pin(void)
{
#ifdef NUCLEO_F401RE
  return HAL_GPIO_ReadPin(LIS3DE_INT1_GPIO_PORT, LIS3DE_INT1_PIN);
#else /* NUCLEO_STM32F401RE */
  return 0;
#endif /* NUCLEO_STM32F401RE */
}
