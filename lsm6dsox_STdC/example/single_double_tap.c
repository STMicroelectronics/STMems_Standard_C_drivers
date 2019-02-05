/*
 ******************************************************************************
 * @file    single_double_tap.c
 * @author  Sensor Solutions Software Team
 * @brief   This file show the simplest way to detect single and double tap
 *          from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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
#include <lsm6dsox_reg.h>
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
#endif

#define TX_BUF_DIM          1000

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[TX_BUF_DIM];
static uint8_t slave_address = LSM6DSOX_I2C_ADD_L;

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
    HAL_I2C_Mem_Write(handle, slave_address, Reg,
                      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
#ifdef MKI109V2
  else if (handle == &hspi2)
  {
    HAL_GPIO_WritePin(CS_SPI2_GPIO_Port, CS_SPI2_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Transmit(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_SPI2_GPIO_Port, CS_SPI2_Pin, GPIO_PIN_SET);
  }
  else if (handle == &hspi1)
  {
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
      HAL_I2C_Mem_Read(handle, slave_address, Reg,
                       I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
#ifdef MKI109V2
  else if (handle == &hspi2)
  {
    Reg |= 0x80;
    HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Receive(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_SET);
  }
  else
  {
    Reg |= 0x80;
    HAL_GPIO_WritePin(CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Receive(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_SET);
  }
#endif
  return 0;
}

/*
 *  Function to print messages.
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  #ifdef NUCLEO_STM32F411RE
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
  #endif
  #ifdef MKI109V2
  CDC_Transmit_FS(tx_buffer, len);
  #endif
}

/*
 * Function to wait for a timeout.
 */
static void platform_delay(uint32_t timeout)
{
	/*
	 * Force compiler to not optimize this code.
	 */
	volatile uint32_t i;

	for(i = 0; i < timeout; i++);
}

/*
 * Platform specific initialization.
 */
static void platform_init(void)
{
#ifdef NUCLEO_STM32F411RE
	uint8_t i;
	GPIO_InitTypeDef GPIO_InitStruct;

	/*
	 * Configure GPIO pin : PB8
	 * Set OUTPUT mode
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*
	 * Force sensor to exit from Hot Join I3C mode.
	 */
	for (i = 0; i < 9; i++) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		platform_delay(100);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		platform_delay(100);
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

	/*
	 * Init I2C interface.
	 */
	MX_I2C1_Init();
#endif /* NUCLEO_STM32F411RE */
}

/*
 * Main Example
 *
 * Set Acc/Gyro ODR to 417 Hz
 * Enable Tap detection on X, Y, Z
 * Set Tap threshold
 * Configure Single and Double Tap parameter
 * Enable Single and Double Tap detection
 * Enable interrupt generation on Single and Double Tap
 * Poll for Tap events
 */
void example_main_double_tap_lsm6dsox(void)
{
  lsm6dsox_ctx_t dev_ctx;
  uint8_t whoamI, rst;

  /*
   * If need to route tap detection interrupt on INT2 use int2_route
   */
#ifdef LSM6DSOX_INT1
  lsm6dsox_pin_int1_route_t int1_route;
#else /* LSM6DSOX_INT1 */
  lsm6dsox_pin_int2_route_t int2_route;
#endif /* LSM6DSOX_INT1 */

  /*
   *  Initialize mems driver interface.
   */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /*
   * Init test platform.
   */
  platform_init();

  /*
   *  Check device ID.
   */
  lsm6dsox_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LSM6DSOX_ID)
    while(1);

  /*
   *  Restore default configuration.
   */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst);

  /*
   * Disable I3C interface.
   */
  lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);

  /*
   * Set XL Output Data Rate to 416 Hz.
   */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_417Hz);

  /* Set 2g full XL scale */
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_2g);

  /*
   * Enable Tap detection on X, Y, Z.
   */
  lsm6dsox_tap_detection_on_z_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsox_tap_detection_on_y_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsox_tap_detection_on_x_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set Tap threshold to 01000b, therefore the tap threshold
   * is 500 mg (= 12 * FS_XL / 32 ).
   */
  lsm6dsox_tap_threshold_x_set(&dev_ctx, 0x08);
  lsm6dsox_tap_threshold_y_set(&dev_ctx, 0x08);
  lsm6dsox_tap_threshold_z_set(&dev_ctx, 0x08);

  /*
   * Configure Single and Double Tap parameter
   *
   * For the maximum time between two consecutive detected taps, the DUR
   * field of the INT_DUR2 register is set to 0111b, therefore the Duration
   * time is 538.5 ms (= 7 * 32 * ODR_XL).
   *
   * The SHOCK field of the INT_DUR2 register is set to 11b, therefore
   * the Shock time is 57.36 ms (= 3 * 8 * ODR_XL).
   *
   * The QUIET field of the INT_DUR2 register is set to 11b, therefore
   * the Quiet time is 28.68 ms (= 3 * 4 * ODR_XL).
   */
  lsm6dsox_tap_dur_set(&dev_ctx, 0x07);
  lsm6dsox_tap_quiet_set(&dev_ctx, 0x03);
  lsm6dsox_tap_shock_set(&dev_ctx, 0x03);

  /*
   * Enable Single and Double Tap detection.
   */
  lsm6dsox_tap_mode_set(&dev_ctx, LSM6DSOX_BOTH_SINGLE_DOUBLE);

  /*
   * For single tap only uncomments next function.
   */
  //lsm6dsox_tap_mode_set(&dev_ctx, LSM6DSOX_ONLY_SINGLE);

#ifdef LSM6DSOX_INT1
  /*
   * Enable interrupt generation on Single and Double Tap INT1 pin.
   */
  lsm6dsox_pin_int1_route_get(&dev_ctx, &int1_route);

  /*
   * For single tap only comment next function.
   */
  int1_route.md1_cfg.int1_double_tap = PROPERTY_ENABLE;
  int1_route.md1_cfg.int1_single_tap = PROPERTY_ENABLE;
  lsm6dsox_pin_int1_route_set(&dev_ctx, &int1_route);
#else /* LSM6DSOX_INT1 */
  /*
   * Uncomment if interrupt generation on Single and Double Tap INT2 pin.
   */
  lsm6dsox_pin_int2_route_get(&dev_ctx, &int2_route);

  /*
   * For single tap only comment next function.
   */
  int2_route.md2_cfg.int2_double_tap = PROPERTY_ENABLE;
  int2_route.md2_cfg.int2_single_tap = PROPERTY_ENABLE;
  lsm6dsox_pin_int2_route_set(&dev_ctx, &int2_route);
#endif /* LSM6DSOX_INT1 */

  /*
   * Wait Events.
   */
  while(1)
  {
	lsm6dsox_all_sources_t all_source;

	lsm6dsox_all_sources_get(&dev_ctx, &all_source);

    /*
     * Check if Tap events.
     */
	if (all_source.tap_src.double_tap)
	{
		sprintf((char*)tx_buffer, "D-Tap: ");
		if (all_source.tap_src.x_tap)
			strcat((char*)tx_buffer, "x-axis");
		else if (all_source.tap_src.y_tap)
			strcat((char*)tx_buffer, "y-axis");
		else
			strcat((char*)tx_buffer, "z-axis");
		if (all_source.tap_src.tap_sign)
			strcat((char*)tx_buffer, " negative");
		else
			strcat((char*)tx_buffer, " positive");
		strcat((char*)tx_buffer, " sign\r\n");
		tx_com(tx_buffer, strlen((char const*)tx_buffer));
	}

	if (all_source.tap_src.single_tap)
	{
		sprintf((char*)tx_buffer, "S-Tap: ");
		if (all_source.tap_src.x_tap)
			strcat((char*)tx_buffer, "x-axis");
		else if (all_source.tap_src.y_tap)
			strcat((char*)tx_buffer, "y-axis");
		else
			strcat((char*)tx_buffer, "z-axis");
		if (all_source.tap_src.tap_sign)
			strcat((char*)tx_buffer, " negative");
		else
			strcat((char*)tx_buffer, " positive");
		strcat((char*)tx_buffer, " sign\r\n");
		tx_com(tx_buffer, strlen((char const*)tx_buffer));
	}
  }
}
