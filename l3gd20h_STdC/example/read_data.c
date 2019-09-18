/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A2
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A2 - Host side: UART(COM) to USB bridge
 *                                       - I2C(Default) / SPI(N/A)
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
//#define STEVAL_MKI109V3
#define NUCLEO_F411RE_X_NUCLEO_IKS01A2

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "l3gd20h_reg.h"
#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
#include "usart.h"
#endif

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_angular_rate;
static float angular_rate_mdps[3];
static uint8_t whoamI;
static l3gd20h_status_reg_t status;
static uint8_t rst;
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
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void example_main_l3gd20h(void)
{
  /* Initialize platform specific hardware */
  platform_init();

  /* Initialize magnetic sensors driver interface */
  uint8_t i2c_add = L3GD20H_I2C_ADD_H;
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = (void*)&i2c_add;

  /* Check device ID */
  l3gd20h_dev_id_get(&dev_ctx, &whoamI);
  if (whoamI != L3GD20H_ID ){
    while(1){
      /* manage here device not found */
    }
  }

  /* Restore default configuration */
  l3gd20h_dev_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    l3gd20h_dev_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  l3gd20h_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set full scale */
  l3gd20h_gy_full_scale_set(&dev_ctx, L3GD20H_2000dps);

  /* Set Output Data Rate / Power mode */
  l3gd20h_gy_data_rate_set(&dev_ctx, L3GD20H_50Hz);

  /* Read samples in polling mode (no int) */
  while(1)
  {
    /* Read device status register */
    l3gd20h_dev_status_get(&dev_ctx, &status);

    if ( status.zyxda )
    {
      /* Read imu data */
      memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));

      l3gd20h_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);

      angular_rate_mdps[0] = l3gd20h_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
      angular_rate_mdps[1] = l3gd20h_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
      angular_rate_mdps[2] = l3gd20h_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);

      sprintf((char*)tx_buffer, "[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
    }
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor I2C address.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  uint8_t *i2c_address = handle;
  HAL_I2C_Mem_Write(&hi2c1, *i2c_address, reg,
                    I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor I2C address.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  uint8_t *i2c_address = handle;
  HAL_I2C_Mem_Read(&hi2c1, *i2c_address, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to trasmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  #ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
  #endif
  #ifdef STEVAL_MKI109V3
  CDC_Transmit_FS(tx_buffer, len);
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
  HAL_Delay(1000);
#endif
}
