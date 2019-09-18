/*
 ******************************************************************************
 * @file    multi_read_fifo_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor FIFO.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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
 * - STEVAL_MKI109V3
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A2
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
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
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2

/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
/* NUCLEO_F411RE_X_NUCLEO_IKS01A2: Define communication interface */
#define SENSOR_BUS hi2c1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <lsm6ds3_reg.h>
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

/* Private types ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
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
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void example_multi_read_fifo_simple_lsm6ds3(void)
{
  /*
   * Initialize mems driver interface
   */
  stmdev_ctx_t dev_ctx;
  uint16_t pattern_len;
  lsm6ds3_int1_route_t int_1_reg;

  /*
   * Uncomment if interrupt generation on Free Fall INT2 pin.
   */
  //lsm6ds3_int2_route_t int_2_reg;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /*
   * Initialize platform specific hardware
   */
  platform_init();

  /*
   * Check device ID
   */
  lsm6ds3_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LSM6DS3_ID)
    while(1)
    {
      /* manage here device not found */
    }

  /*
   * Restore default configuration
   */
  lsm6ds3_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
	  lsm6ds3_reset_get(&dev_ctx, &rst);
  } while (rst);

  /*
   * Set XL full scale and Gyro full scale
   */
  lsm6ds3_xl_full_scale_set(&dev_ctx, LSM6DS3_2g);
  lsm6ds3_gy_full_scale_set(&dev_ctx, LSM6DS3_2000dps);

  /*
   * Enable Block Data Update (BDU) when FIFO support selected
   */
  lsm6ds3_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set FIFO watermark to a multiple of a pattern
   * in this example we set watermark to 10 pattern
   * which means ten sequence of:
   * (GYRO + XL) = 12 bytes
   */
  pattern_len = 12;
  lsm6ds3_fifo_watermark_set(&dev_ctx, 10 * pattern_len);

  /*
   * Set FIFO mode to Stream mode
   */
  lsm6ds3_fifo_mode_set(&dev_ctx, LSM6DS3_STREAM_MODE);

  /*
   * Enable FIFO watermark interrupt generation on INT1 pin
   */
  lsm6ds3_pin_int1_route_get(&dev_ctx, &int_1_reg);
  int_1_reg.int1_fth = PROPERTY_ENABLE;
  lsm6ds3_pin_int1_route_set(&dev_ctx, &int_1_reg);

  /*
   * FIFO watermark interrupt on INT2 pin
   */
  //lsm6ds3_pin_int2_route_get(&dev_ctx, &int_2_reg);
  //int_2_reg.int2_fth = PROPERTY_ENABLE;
  //lsm6ds3_pin_int2_route_set(&dev_ctx, &int_2_reg);

  /*
   * Set FIFO sensor decimator
   */
  lsm6ds3_fifo_xl_batch_set(&dev_ctx, LSM6DS3_FIFO_XL_NO_DEC);
  lsm6ds3_fifo_gy_batch_set(&dev_ctx, LSM6DS3_FIFO_GY_NO_DEC);

  /*
   * Set ODR FIFO
   */
  lsm6ds3_fifo_data_rate_set(&dev_ctx, LSM6DS3_FIFO_52Hz);

  /*
   * Set XL and Gyro Output Data Rate:
   * in this example we set 52 Hz for Accelerometer and
   * 52 Hz for Gyroscope
   */
  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_52Hz);
  lsm6ds3_gy_data_rate_set(&dev_ctx, LSM6DS3_GY_ODR_52Hz);

  while(1)
  {
    uint16_t num = 0;
    uint16_t num_pattern = 0;
    uint8_t waterm = 0;

    /*
     * Read LSM6DS3 watermark flag
     */
    lsm6ds3_fifo_wtm_flag_get(&dev_ctx, &waterm);
    if (waterm)
    {
      /*
       * Read number of word in FIFO
       */
      lsm6ds3_fifo_data_level_get(&dev_ctx, &num);
      num_pattern = num / pattern_len;

      while (num_pattern-- > 0)
      {
        /*
         * Following the sensors ODR configuration, FIFO pattern is composed
         * by this sequence of samples: GYRO, XL
         */
    	lsm6ds3_fifo_raw_data_get(&dev_ctx,
                                  data_raw_angular_rate.u8bit,
                                  3 * sizeof(int16_t));
        angular_rate_mdps[0] =
          lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
        angular_rate_mdps[1] =
          lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
        angular_rate_mdps[2] =
          lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

        sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
        tx_com(tx_buffer, strlen((char const*)tx_buffer));

        lsm6ds3_fifo_raw_data_get(&dev_ctx,
                                  data_raw_acceleration.u8bit,
                                  3 * sizeof(int16_t));
        acceleration_mg[0] =
        		lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
        acceleration_mg[1] =
        		lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
        acceleration_mg[2] =
        		lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);

        sprintf((char*)tx_buffer, "Acc [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
        tx_com(tx_buffer, strlen((char const*)tx_buffer));
      }
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
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, LSM6DS3_I2C_ADD_L, reg,
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
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Read(handle, LSM6DS3_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
    /* Read command */
    reg |= 0x80;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }
#endif
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
