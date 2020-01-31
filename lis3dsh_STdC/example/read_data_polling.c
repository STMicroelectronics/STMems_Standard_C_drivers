/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to extract data from the sensor in 
 *          polling mode.
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
 *    - NUCLEO_F411RE + X_NUCLEO_IKS01A1 + STEVAL-MKI134V1
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *    - Host side: UART(COM) to USB bridge
 *    - Sensor side: I2C
 *
 *  If you need to run this example on a different hardware platform a
 *  modification of the functions: `platform_sens_write`, `platform_sens_read`,
 * `platform_host_write` and 'platform_init' is required.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <lis3dsh_reg.h>
#include <string.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[1000];
static lis3dsh_data_t data;

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
void lis3dsh_read_data_polling(void)
{
  lis3dsh_pin_int1_route_t int1_route;
  lis3dsh_all_sources_t all_sources;
  lis3dsh_bus_mode_t bus_mode;
  lis3dsh_status_var_t status;
  stmdev_ctx_t dev_ctx;
  lis3dsh_id_t id;
  lis3dsh_md_t md;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /* Init test platform */
  platform_init();

  /* Check device ID */
  lis3dsh_id_get(&dev_ctx, &id);
  if (id.whoami != LIS3DSH_ID)
    while(1);

  /* Restore default configuration */
  lis3dsh_init_set(&dev_ctx, LIS3DSH_RESET);
  do {
    lis3dsh_status_get(&dev_ctx, &status);
  } while (status.sw_reset);

  /* Set bdu and if_inc racomended for driver usage */
  lis3dsh_init_set(&dev_ctx, LIS3DSH_DRV_RDY);
  
  /* Select bus interface */
  bus_mode = LIS3DSH_SEL_BY_HW;
  lis3dsh_bus_mode_set(&dev_ctx, &bus_mode);
  
  /* Set Output Data Rate */
  lis3dsh_mode_get(&dev_ctx, &md);
  md.fs =  LIS3DSH_4g;
  md.odr = LIS3DSH_25Hz;
  lis3dsh_mode_set(&dev_ctx, &md);

  /* Configure inerrupt pins */
  lis3dsh_pin_int1_route_get(&dev_ctx, &int1_route);
  int1_route.drdy_xl   = PROPERTY_DISABLE;
  lis3dsh_pin_int1_route_get(&dev_ctx, &int1_route);

  /* Read samples in polling mode (no int). */
  while(1)
  {
    /* Read output only if new values are available */
    lis3dsh_all_sources_get(&dev_ctx, &all_sources);
    if ( all_sources.drdy_xl ) {

      lis3dsh_data_get(&dev_ctx, &md, &data);

      /* print sensor data  */
      sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              data.xl.mg[0], data.xl.mg[1], data.xl.mg[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
    
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1) {
    HAL_I2C_Mem_Write(handle, LIS3DSH_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
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
  if (handle == &hi2c1) {
    HAL_I2C_Mem_Read(handle, LIS3DSH_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
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
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{

}
