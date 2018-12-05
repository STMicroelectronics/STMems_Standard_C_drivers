/*
 ******************************************************************************
 * @file    wake_up.c
 * @author  MEMS Software Solution Team
 * @date    02-January-2018
 * @brief   This file show the simplest way to detect wake-up from sensor.
 *          This sample show how to use HP filter.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "lis3de_reg.h"
#include <string.h>

//#define MKI109V2
#define NUCLEO_STM32F411RE

#ifdef MKI109V2
#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"
#include "spi.h"
#include "i2c.h"
#endif

#ifdef NUCLEO_STM32F411RE
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#endif

/* Private macro -------------------------------------------------------------*/
#ifdef MKI109V2
#define CS_SPI2_GPIO_Port   CS_DEV_GPIO_Port
#define CS_SPI2_Pin         CS_DEV_Pin
#define CS_SPI1_GPIO_Port   CS_RF_GPIO_Port
#define CS_SPI1_Pin         CS_RF_Pin
#endif

#ifdef NUCLEO_STM32F411RE
/* N/A on NUCLEO_STM32F411RE + IKS01A1 */
/* N/A on NUCLEO_STM32F411RE + IKS01A2 */
#define CS_SPI2_GPIO_Port   0
#define CS_SPI2_Pin         0
#define CS_SPI1_GPIO_Port   0
#define CS_SPI1_Pin         0

/* Pin configured as platform interrupt from LIS3DE INT1. */
#define LIS3DE_INT1_PIN GPIO_PIN_4
#define LIS3DE_INT1_GPIO_PORT GPIOA

#endif

#define TX_BUF_DIM          1000

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[TX_BUF_DIM];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   Replace the functions "platform_write" and "platform_read" with your
 *   platform specific read and write function.
 *   This example use an STM32 evaluation board and CubeMX tool.
 *   In this case the "*handle" variable is useful in order to select the
 *   correct interface but the usage of "*handle" is not mandatory.
 */

static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80;
    HAL_I2C_Mem_Write(handle, LIS3DE_I2C_ADD_H, Reg,
                      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
#ifdef MKI109V2
  else if (handle == &hspi2)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x40;
    HAL_GPIO_WritePin(CS_SPI2_GPIO_Port, CS_SPI2_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Transmit(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_SPI2_GPIO_Port, CS_SPI2_Pin, GPIO_PIN_SET);
  }
  else if (handle == &hspi1)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x40;
    HAL_GPIO_WritePin(CS_SPI1_GPIO_Port, CS_SPI1_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Transmit(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_SPI1_GPIO_Port, CS_SPI1_Pin, GPIO_PIN_SET);
  }
#endif
  return 0;
}

static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  if (handle == &hi2c1)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80;
    HAL_I2C_Mem_Read(handle, LIS3DE_I2C_ADD_H, Reg,
                     I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
#ifdef MKI109V2
  else if (handle == &hspi2)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0xC0;
    HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Receive(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_SET);
  }
  else
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0xC0;
    HAL_GPIO_WritePin(CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Receive(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_SET);
  }
#endif
  return 0;
}

/*
 *  Function to print messages
 */
static void tx_com( uint8_t *tx_buffer, uint16_t len )
{
  #ifdef NUCLEO_STM32F411RE
  HAL_UART_Transmit( &huart2, tx_buffer, len, 1000 );
  #endif
  #ifdef MKI109V2
  CDC_Transmit_FS( tx_buffer, len );
  #endif
}

/*
 * Function to read external interrupt pin.
 */
static int32_t platform_reap_int_pin(void)
{
#ifdef NUCLEO_STM32F411RE
    return HAL_GPIO_ReadPin(LIS3DE_INT1_GPIO_PORT, LIS3DE_INT1_PIN);
#else /* NUCLEO_STM32F411RE */
    return 0;
#endif /* NUCLEO_STM32F411RE */
}

/* Main Example --------------------------------------------------------------*/

/*
 * High-pass filter enabled on interrupt activity 1
 * Threshold = 250 mg
 * Configure wake-up event
 * Poll INT1 pin waiting for the wake-up event
 */
void example_wake_up_lis3de(void)
{
  /*
   *  Initialize mems driver interface.
   */
  lis3de_ctx_t dev_ctx;
  lis3de_ig1_cfg_t ig1_cfg;
  lis3de_ctrl_reg3_t ctrl_reg3;
  uint8_t whoamI;
  //uint8_t dummy;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /*
   *  Check device ID.
   */
  whoamI = 0;
  lis3de_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LIS3DE_ID)
    while(1); /* manage here device not found */

  /*
   * Set Output Data Rate to 100 Hz.
   */
  lis3de_data_rate_set(&dev_ctx, LIS3DE_ODR_100Hz);

  /*
   * High-pass filter enabled on interrupt activity 1.
   * Uncomment if HP filter enabled.
   */
  //lis3de_high_pass_int_conf_set(&dev_ctx, LIS3DE_ON_INT1_GEN);

  /*
   * Enable HP filter for wake-up event detection.
   * Uncomment if HP filter enabled.
   */
  //lis3de_high_pass_on_outputs_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Enable AOI1 on int1 pin.
   */
  memset((uint8_t *)&ctrl_reg3, 0, sizeof(ctrl_reg3));
  ctrl_reg3.int1_ig1 = PROPERTY_ENABLE;
  lis3de_pin_int1_config_set(&dev_ctx, &ctrl_reg3);

  /*
   * Interrupt 1 pin latched.
   */
  lis3de_int1_pin_notification_mode_set(&dev_ctx,
		  	  	  	  LIS3DE_INT1_LATCHED);

  /*
   * Set full scale to 2 g.
   */
  lis3de_full_scale_set(&dev_ctx, LIS3DE_2g);

  /*
   * Set interrupt threshold to 0x10 -> 250 mg.
   */
  lis3de_int1_gen_threshold_set(&dev_ctx, 0x10);

  /*
   * Set no time duration.
   */
  lis3de_int1_gen_duration_set(&dev_ctx, 0);

  /*
   * Dummy read to force the HP filter to current acceleration value.
   * Uncomment if HP filter enabled.
   */
  //lis3de_filter_reference_get(&dev_ctx, &dummy);

  /*
   * Configure wake-up event.
   */
  ig1_cfg.zhie = PROPERTY_ENABLE;
  ig1_cfg.yhie = PROPERTY_ENABLE;
  ig1_cfg.xhie = PROPERTY_ENABLE;
  ig1_cfg.aoi = PROPERTY_DISABLE;
  lis3de_int1_gen_conf_set(&dev_ctx, &ig1_cfg);

  /*
   * Set device in HR mode.
   */
  lis3de_operating_mode_set(&dev_ctx, LIS3DE_LP);

  while(1)
  {
    /*
     * Read INT pin 1 in polling mode.
     */
     lis3de_ig1_source_t src;

    if (platform_reap_int_pin())
    {
      lis3de_int1_gen_source_get(&dev_ctx, &src);
      sprintf((char*)tx_buffer, "wake-up detected : "
    		  "x %d, y %d, z %d\r\n",
    		  src.xh, src.yh, src.zh);
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }
  }
}
