/*
 ******************************************************************************
 * @file    activity.c
 * @author  Sensor Solutions Software Team
 * @brief   This file show the simplest way to detect activity/inactivity
 * 			from sensor.
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
 * Set Acc/Gyro ODR to 12.5 Hz
 * Set full scale to 2g (XL) 2000dps (Gyro)
 * Select LPF2 bandwidth to ODR/100
 * Enable digital data path to LPF2
 */
void example_main_activity_lsm6dsox(void)
{
  lsm6dsox_ctx_t dev_ctx;
  lsm6dsox_pin_int1_route_t int1_route;
  uint8_t whoamI, rst;

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
   * Set XL and Gyro Output Data Rate.
   */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_208Hz);
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_104Hz);

  /*
   * Set 2g full XL scale and 250 dps full Gyro.
   */
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_2g);
  lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_250dps);

  /*
   * Set duration for Activity detection to 9.62 ms (= 2 * 1 / ODR_XL).
   */
  lsm6dsox_wkup_dur_set(&dev_ctx, 0x02);

  /*
   * Set duration for Inactivity detection to 4.92 s (= 2 * 512 / ODR_XL).
   */
  lsm6dsox_act_sleep_dur_set(&dev_ctx, 0x02);

  /*
   * Set Activity/Inactivity threshold to 62.5 mg.
   */
  lsm6dsox_wkup_threshold_set(&dev_ctx, 0x02);

  /*
   * Inactivity configuration: XL to 12.5 in LP, gyro to Power-Down.
   */
  lsm6dsox_act_mode_set(&dev_ctx, LSM6DSOX_XL_12Hz5_GY_PD);

  /*
   * Enable interrupt generation on Inactivity INT1 pin.
   *
   * Uncomment if INT pin 1 is used to detect inactive
   * state events.
   */
  lsm6dsox_pin_int1_route_get(&dev_ctx, &int1_route);
  int1_route.md1_cfg.int1_sleep_change = PROPERTY_ENABLE;
  lsm6dsox_pin_int1_route_set(&dev_ctx, &int1_route);

  /*
   * Wait Events.
   */
  while(1)
  {
	lsm6dsox_all_sources_t all_source;

    /*
     * Check if Activity/Inactivity events.
     */
	lsm6dsox_all_sources_get(&dev_ctx, &all_source);
	if (all_source.wake_up_src.sleep_state)
	{
		sprintf((char*)tx_buffer, "Inactivity Detected\r\n");
		tx_com(tx_buffer, strlen((char const*)tx_buffer));
	}

	if (all_source.wake_up_src.wu_ia)
	{
		sprintf((char*)tx_buffer, "Activity Detected\r\n");
		tx_com(tx_buffer, strlen((char const*)tx_buffer));
	}
  }
}

