/*
 ******************************************************************************
 * @file    pedometer.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to configure the step counter.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A3
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A3 - Host side: UART(COM) to USB bridge
 *                                       - I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: 'platform_write', 'platform_read' and
 * 'tx_com'  is required.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <lsm6dso_reg.h>
#include "gpio.h"
#include "i2c.h"
#include "usart.h"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t steps;
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

static void tx_com( uint8_t *tx_buffer, uint16_t len );

/* Main Example --------------------------------------------------------------*/
void lsm6dso_pedometer(void)
{
  stmdev_ctx_t ag_ctx;
  /* Uncomment to configure INT 1 */
  //lsm6dso_pin_int1_route_t int1_route;
  /* Uncomment to configure INT 2 */
  //lsm6dso_pin_int2_route_t int2_route;

  /* Initialize driver interface */
  ag_ctx.write_reg = platform_write;
  ag_ctx.read_reg = platform_read;
  ag_ctx.handle = &hi2c1;
 
  /* Check device ID */
  lsm6dso_device_id_get(&ag_ctx, &whoamI);
  if (whoamI != LSM6DSO_ID)
    while(1);
 
  /* Restore default configuration */
  lsm6dso_reset_set(&ag_ctx, PROPERTY_ENABLE);
  do {
    lsm6dso_reset_get(&ag_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dso_i3c_disable_set(&ag_ctx, LSM6DSO_I3C_DISABLE);

  /* Set XL full scale */
  lsm6dso_xl_full_scale_set(&ag_ctx, LSM6DSO_2g);

  /* Enable Block Data Update */
  lsm6dso_block_data_update_set(&ag_ctx, PROPERTY_ENABLE);

  /* Enable latched interrupt notification. */
  //lsm6dso_int_notification_set(&ag_ctx, LSM6DSO_ALL_INT_LATCHED);

  /* Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed. */
  //lsm6dso_data_ready_mode_set(&ag_ctx, LSM6DSO_DRDY_PULSED);

  /*
   * Uncomment to configure INT 1
   * Remember that INT1 pin is used by sensor to switch in I3C mode
   */
  //lsm6dso_pin_int1_route_get(&ag_ctx, &int1_route);
  //int1_route.reg.emb_func_int1.int1_step_detector = PROPERTY_ENABLE;
  //lsm6dso_pin_int1_route_set(&ag_ctx, &int1_route);

  /* Uncomment to configure INT 2 */
  //lsm6dso_pin_int2_route_get(&ag_ctx, &int2_route);
  //int2_route.reg.emb_func_int2.int2_step_detector = PROPERTY_ENABLE;
  //lsm6dso_pin_int2_route_set(&ag_ctx, &int2_route);

  /* Enable xl sensor */
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_26Hz); 
 
  /* Reset steps of pedometer */
  lsm6dso_steps_reset(&ag_ctx);

  /* Enable pedometer */
  lsm6dso_pedo_sens_set(&ag_ctx, LSM6DSO_FALSE_STEP_REJ_ADV_MODE);
 
  while(1) {
      /* Read steps */
      lsm6dso_number_of_steps_get(&ag_ctx, (uint8_t*)&steps);
      sprintf((char*)tx_buffer, "steps :%d\r\n", steps);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
      HAL_Delay(1000);
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
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, LSM6DSO_I2C_ADD_H, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }
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
  return HAL_I2C_Mem_Read(handle, LSM6DSO_I2C_ADD_H, reg,
                          I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
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

