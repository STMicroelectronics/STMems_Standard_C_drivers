/*
 ******************************************************************************
 * @file    st1vafe3bx_mlc_six_d_position.c
 * @author  Sensors Software Solution Team
 * @brief   The purpose of this example is to show how use the device
 *          Machine Learning Code (MLC) starting from an ".h" file
 *          generated through with the tool "Machine Learning Code"
 *          of Unico GUI.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
 * Some MLC examples are available at:
 * https://github.com/STMicroelectronics/STMems_Machine_Learning_Core
 * the same repository is linked to this repository in folder "_resources"
 *
 * For more information about Machine Learning Code tool please refer
 * to AN6208 "ST1VAFE3BX: machine learning core".
 *
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-
 * - NUCLEO_F401RE + STEVAL-
 * - DISCOVERY_SPC584B + STEVAL-
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
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

#include "st1vafe3bx_six_d_position.h"
#include "st1vafe3bx_reg.h"

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
#define    BOOT_TIME         10 //ms

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
static void platform_delay(uint32_t ms);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void st1vafe3bx_mlc_six_d_position(void)
{
  /* Variable declaration */
  st1vafe3bx_mlc_status_mainpage_t status;
  stmdev_ctx_t dev_ctx;
  uint32_t i;
  st1vafe3bx_status_t rst;
  uint8_t id;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;
  st1vafe3bx_priv_t vafe_info;
  dev_ctx.priv_data = &vafe_info;

  /* Initialize platform specific hardware */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  st1vafe3bx_exit_deep_power_down(&dev_ctx);

  /* Check device ID */
  st1vafe3bx_device_id_get(&dev_ctx, &id);
  if (id != ST1VAFE3BX_ID)
    while(1);

  /* Restore default configuration */
  st1vafe3bx_init_set(&dev_ctx, ST1VAFE3BX_RESET);
  do {
    st1vafe3bx_status_get(&dev_ctx, &rst);
  } while (rst.sw_reset);

  /* Start Machine Learning Core configuration */
  for ( i = 0; i < (sizeof(st1vafe3bx_six_d_position) / sizeof(ucf_line_ext_t) ); i++ ) {
    switch(st1vafe3bx_six_d_position[i].op) {
    case MEMS_UCF_OP_DELAY:
      platform_delay(st1vafe3bx_six_d_position[i].data);
      break;
    case MEMS_UCF_OP_WRITE:
      st1vafe3bx_write_reg(&dev_ctx, st1vafe3bx_six_d_position[i].address,
                           (uint8_t *)&st1vafe3bx_six_d_position[i].data, 1);
      break;
    }
  }

  /* Main loop */
  while (1) {
    /* Read status registers */
    st1vafe3bx_mlc_status_get(&dev_ctx, &status);

    if (status.is_mlc1) {
      uint8_t six_d_pos[8];

      st1vafe3bx_mlc_out_get(&dev_ctx, six_d_pos);

      switch (six_d_pos[0])
      {
      default:
      case 0:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "NONE\r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      case 1:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "X-axis up\r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      case 2:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "X-axis down\r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      case 3:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "Y-axis up\r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      case 4:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "Y-axis down\r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      case 5:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "Z-axis up\r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      case 6:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "Z-axis down\r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      }
    }

    platform_delay(1);
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
  HAL_I2C_Mem_Write(handle, ST1VAFE3BX_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  ST1VAFE3BX_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, ST1VAFE3BX_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, ST1VAFE3BX_I2C_ADD_L & 0xFE, reg, bufp, len);
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
