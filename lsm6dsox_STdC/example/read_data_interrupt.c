/*
 ******************************************************************************
 * @file    read_data_interrupt.c
 * @author  Sensor Solutions Software Team
 * @brief   This file show the simplest way to get data from sensor (interrupt
 * 			    mode).
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

/* Platform STM32F411RE + IKS01A2 Interrupt PIN */
#define LSM6DSOX_INT2_PIN GPIO_PIN_1
#define LSM6DSOX_INT2_GPIO_PORT GPIOC
#define LSM6DSOX_INT1_PIN GPIO_PIN_0
#define LSM6DSOX_INT1_GPIO_PORT GPIOC
#endif

#define TX_BUF_DIM          1000

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static uint8_t whoamI, rst;
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
 * Function to read external interrupt pin connected to INT sensor.
 */
static int32_t platform_read_int_pin(void)
{
#ifdef NUCLEO_STM32F411RE
#ifndef LSM6DSOX_INT1
    return HAL_GPIO_ReadPin(LSM6DSOX_INT2_GPIO_PORT, LSM6DSOX_INT2_PIN);
#else /* LSM6DSOX_INT1 */
    return HAL_GPIO_ReadPin(LSM6DSOX_INT1_GPIO_PORT, LSM6DSOX_INT1_PIN);
#endif /* LSM6DSOX_INT1 */

#else /* NUCLEO_STM32F411RE */
    return 0;
#endif /* NUCLEO_STM32F411RE */
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
 * Enable interrupt generation on DRDY INT2 pin
 *
 * NB: On target NUCLEO_STM32F411RE + IKS01A2, sensor INT1 pin
 * is routed to M_INT1 through a LDO (U6). On boot this pin is
 * forced by LDO to logical level 1 and this force sensor to
 * enter in Hot-Join I3C mode, disabling I2C interface.
 */
void example_main_interrupt_lsm6dsox(void)
{
  lsm6dsox_ctx_t dev_ctx;

#ifdef LSM6DSOX_INT1
  /*
   * If need to route drdy interrupt on INT1 use int1_route
   */
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
   *  Enable Block Data Update.
   */
  lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set Output Data Rate.
   */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_12Hz5);
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_12Hz5);

  /*
   * Set full scale.
   */
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_2g);
  lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_2000dps);

  /*
   * Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed.
   */
  //lsm6dsox_data_ready_mode_set(&dev_ctx, LSM6DSOX_DRDY_PULSED);

#ifdef LSM6DSOX_INT1
  /*
   * Enable interrupt generation on DRDY INT1 pin.
   *
   * Remember that INT1 pin is used by sensor to switch in I3C mode.
   */
  lsm6dsox_pin_int1_route_get(&dev_ctx, &int1_route);
  int1_route.int1_ctrl.int1_drdy_g = PROPERTY_ENABLE;
  int1_route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
  lsm6dsox_pin_int1_route_set(&dev_ctx, &int1_route);
#else /* LSM6DSOX_INT1 */
  /*
   * Interrupt generation routed on DRDY INT2 pin.
   */
  lsm6dsox_pin_int2_route_get(&dev_ctx, &int2_route);
  int2_route.int2_ctrl.int2_drdy_g = PROPERTY_ENABLE;
  int2_route.int2_ctrl.int2_drdy_xl = PROPERTY_ENABLE;
  lsm6dsox_pin_int2_route_set(&dev_ctx, &int2_route);
#endif /* LSM6DSOX_INT1 */

  /*
   * Wait samples.
   */
  while(1)
  {
    lsm6dsox_reg_t reg;

    /*
     * Read INT pin.
     */
    if (platform_read_int_pin())
    {
        /*
         * Read output only if new value is available.
         */
		lsm6dsox_status_reg_get(&dev_ctx, &reg.status_reg);

		if (reg.status_reg.xlda)
		{
		  /*
		   * Read acceleration field data.
		   */
		  memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
		  lsm6dsox_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
		  acceleration_mg[0] =
				  lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
		  acceleration_mg[1] =
				  lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
		  acceleration_mg[2] =
				  lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);

		  sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
				  acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
		  tx_com(tx_buffer, strlen((char const*)tx_buffer));
		}
		if (reg.status_reg.gda)
		{
		  /*
		   * Read angular rate field data.
		   */
		  memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
		  lsm6dsox_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
		  angular_rate_mdps[0] =
				  lsm6dsox_from_fs2_to_mg(data_raw_angular_rate.i16bit[0]);
		  angular_rate_mdps[1] =
				  lsm6dsox_from_fs2_to_mg(data_raw_angular_rate.i16bit[1]);
		  angular_rate_mdps[2] =
				  lsm6dsox_from_fs2_to_mg(data_raw_angular_rate.i16bit[2]);

		  sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
				  angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
		  tx_com(tx_buffer, strlen((char const*)tx_buffer));
		}
    }
  }
}
