/*
 ******************************************************************************
 * @file    fifo_stream.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to configure sensor FIFO in stream mode.
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
#include "stm32f4xx_hal.h"
#include "a3g4250d_reg.h"
#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
#include "usart.h"
#endif

/* Private macro -------------------------------------------------------------*/
#if defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
#define A3G4250D_INT2_PIN GPIO_PIN_1
#define A3G4250D_INT2_GPIO_PORT GPIOC
#define A3G4250D_INT1_PIN GPIO_PIN_0
#define A3G4250D_INT1_GPIO_PORT GPIOC
#else /* NUCLEO_F411RE_X_NUCLEO_IKS01A2 */
#define A3G4250D_INT2_PIN GPIO_PIN_5
#define A3G4250D_INT2_GPIO_PORT GPIOB
#define A3G4250D_INT1_PIN GPIO_PIN_8
#define A3G4250D_INT1_GPIO_PORT GPIOB
#endif /* NUCLEO_F411RE_X_NUCLEO_IKS01A2 */

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_angular_rate;
static float angular_rate_mdps[3];
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void example_multi_read_fifo_a3g4250d(void)
{
  a3g4250d_ctx_t dev_ctx;

  /*
   * Uncomment to configure watermark interrupt on INT2
   */
  //a3g4250d_int2_route_t int2_route;

  /*
   *  Initialize mems driver interface
   */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /*
   * Initialize platform specific hardware
   */
  platform_init();

  /*
   *  Check device ID
   */
  a3g4250d_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != A3G4250D_ID)
    while(1); /*manage here device not found */

  /*
   * Set FIFO watermark to 10 samples
   */
  a3g4250d_fifo_watermark_set(&dev_ctx, 10);

  /*
   * Set FIFO mode to FIFO MODE
   */
  a3g4250d_fifo_mode_set(&dev_ctx, A3G4250D_FIFO_STREAM_MODE);

  /*
   * Enable FIFO
   */
  a3g4250d_fifo_enable_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Uncomment to configure watermark interrupt on INT2
   */
  //a3g4250d_pin_int2_route_get(&dev_ctx, &int2_route);
  //int2_route.i2_wtm = PROPERTY_ENABLE;
  //a3g4250d_pin_int2_route_set(&dev_ctx, int2_route);

  /*
   * Set Output Data Rate
   */
  a3g4250d_data_rate_set(&dev_ctx, A3G4250D_ODR_100Hz);

  /*
   * Wait Events Loop
   */
  while(1)
  {
    uint8_t flags;
    uint8_t num = 0;

    /*
     * Read watermark interrupt flag
     */
    a3g4250d_fifo_wtm_flag_get(&dev_ctx, &flags);
    if (flags)
    {
      /*
       * Read how many samples in fifo
       */
      a3g4250d_fifo_data_level_get(&dev_ctx, &num);
      while (num-- > 0)
      {
        /* Read angular rate data */
        memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
        a3g4250d_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
        angular_rate_mdps[0] = a3g4250d_from_fs245dps_to_mdps(data_raw_angular_rate.i16bit[0]);
        angular_rate_mdps[1] = a3g4250d_from_fs245dps_to_mdps(data_raw_angular_rate.i16bit[1]);
        angular_rate_mdps[2] = a3g4250d_from_fs245dps_to_mdps(data_raw_angular_rate.i16bit[2]);

        sprintf((char*)tx_buffer, "Angular Rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
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
    /* Write multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Write(handle, A3G4250D_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
    /* Write multiple command */
    reg |= 0x40;
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
    /* Read multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Read(handle, A3G4250D_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
    /* Read multiple command */
    reg |= 0xC0;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }
#endif
  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
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
