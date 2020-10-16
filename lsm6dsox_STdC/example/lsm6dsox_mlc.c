/*
 ******************************************************************************
 * @file    machine_learning_core.c
 * @author  Sensors Software Solution Team
 * @brief   The purpose of this example is to show how use the device
 *          Machine Learning Core (MLC) starting from an ".h" file 
 *          generated through with the tool "Machine Learning Core" 
 *          of Unico GUI.
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
 *
 * Some MLC examples are available at:
 * https://github.com/STMicroelectronics/STMems_Machine_Learning_Core
 * the same repository is linked to this repository in folder "_resources"
 *
 * For more information about Machine Learning Core tool please refer
 * to AN5259 "LSM6DSOX: Machine Learning Core".
 *
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI197V1
 * - NUCLEO_F411RE + STEVAL-MKI197V1
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <lsm6dsox_reg.h>
#include <lsm6dsox_vibration_monitoring.h>
#include "gpio.h"
#include "i2c.h"
#include "usart.h"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
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
static void platform_delay(uint32_t ms);
static void tx_com( uint8_t *tx_buffer, uint16_t len );

/* Main Example --------------------------------------------------------------*/
void lsm6dsox_mlc(void)
{
  /* Variable declaration */
  lsm6dsox_pin_int1_route_t pin_int1_route;
  lsm6dsox_all_sources_t status;
  lsm6dsox_emb_sens_t emb_sens;
  stmdev_ctx_t dev_ctx;

  uint8_t                     mlc_out[8];
  uint32_t                    i;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg  = platform_read;
  dev_ctx.handle    = &hi2c1;

  /* Wait sensor boot time */
  platform_delay(10);

  /* Check device ID */
  lsm6dsox_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LSM6DSOX_ID)
    while(1);

  /* Restore default configuration */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Start Machine Learning Core configuration */
  for ( i = 0; i < (sizeof(lsm6dsox_vibration_monitoring) /
                    sizeof(ucf_line_t) ); i++ ){
  
    lsm6dsox_write_reg(&dev_ctx, lsm6dsox_vibration_monitoring[i].address,
                       (uint8_t*)&lsm6dsox_vibration_monitoring[i].data, 1);
     
  } 
  /* End Machine Learning Core configuration */

  /* At this point the device is ready to run but if you need you can also
   * interact with the device but taking in account the MLC configuration.
   *
   * For more information about Machine Learning Core tool please refer
   * to AN5259 "LSM6DSOX: Machine Learning Core".
   */

  /* Turn off embedded features */
  lsm6dsox_embedded_sens_get(&dev_ctx, &emb_sens);
  lsm6dsox_embedded_sens_off(&dev_ctx);
  platform_delay(10);

  /* Turn off Sensors */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_OFF);
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_OFF);
 
  /* Disable I3C interface */
  lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);

  /* Enable Block Data Update */
  lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set full scale */
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_4g);
  lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_2000dps);

  /* Route signals on interrupt pin 1 */
  lsm6dsox_pin_int1_route_get(&dev_ctx, &pin_int1_route);
  pin_int1_route.mlc1 = PROPERTY_ENABLE;
  lsm6dsox_pin_int1_route_set(&dev_ctx, pin_int1_route);

  /* Configure interrupt pin mode notification */
  lsm6dsox_int_notification_set(&dev_ctx, LSM6DSOX_BASE_PULSED_EMB_LATCHED);

  /* Enable embedded features */
  lsm6dsox_embedded_sens_set(&dev_ctx, &emb_sens);

  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_26Hz);
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_OFF);
 
  /* Main loop */
  while(1)
  {
    /* Read interrupt source registers in polling mode (no int) */
    lsm6dsox_all_sources_get(&dev_ctx, &status);
    if (status.mlc1){
     
      lsm6dsox_mlc_out_get(&dev_ctx, mlc_out);
      sprintf((char*)tx_buffer, "Detect MLC interrupt code: %02X\r\n",
              mlc_out[0]);
       
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
    HAL_I2C_Mem_Write(handle, LSM6DSOX_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
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
    HAL_I2C_Mem_Read(handle, LSM6DSOX_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
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

