/*
 ******************************************************************************
 * @file    fifo_pedo.c
 * @author  Sensor Solutions Software Team
 * @brief   This file show how to read step count from FIFO.
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
#include "i2c.h"

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

#if defined(NUCLEO_STM32F411RE)
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

#endif /* NUCLEO_XXX */

#define TX_BUF_DIM          1000
#define TIME_OPS			10000

/*
 * Define number of byte for each sensor sample.
 */
#define OUT_XYZ_SIZE		6

typedef union {
    struct {
		uint16_t step_count;
		uint32_t timestamp;
#ifdef __GNUC__
    } __attribute__((__packed__));
#else /* __GNUC__ */
	};
#endif /* __GNUC__ */
	uint8_t byte[6];
} pedo_count_sample_t;

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[TX_BUF_DIM];

/*
 * Local variables used by sample test only.
 */
static lsm6dsox_ctx_t ag_ctx;
static uint8_t slave_address = LSM6DSOX_I2C_ADD_L;

/* Extern variables ----------------------------------------------------------*/

/* Platform Functions --------------------------------------------------------*/

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
  HAL_UART_Transmit( &huart2, tx_buffer, len, 1000 );
  #endif /* NUCLEO_STM32F411RE */

  #ifdef MKI109V2
  CDC_Transmit_FS( tx_buffer, len );
  #endif /* MKI109V2 */
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
 * example_main - Main Example
 *
 * Set FIFO mode to Stream mode
 * Enable FIFO batching of Slave0 + ACC + Gyro samples
 * Enable batch FIFO of hw timestamp (LSB 25 us)
 * Poll for FIFO watermark interrupt and read samples
 */
void lsm6dsox_fifo_pedo_simple(void)
{
  uint8_t whoamI, rst;

#ifdef LSM6DSOX_INT1
  lsm6dsox_pin_int1_route_t int1_route;
#else /* LSM6DSOX_INT1 */
  lsm6dsox_pin_int2_route_t int2_route;
#endif /* LSM6DSOX_INT1 */

  ag_ctx.write_reg = platform_write;
  ag_ctx.read_reg = platform_read;
  ag_ctx.handle = &hi2c1;

  /*
   * Init test platform.
   */
  platform_init();

  /*
   *  Check device ID.
   */
  do {
	  lsm6dsox_device_id_get(&ag_ctx, &whoamI);
  } while(whoamI != LSM6DSOX_ID);

  /*
   *  Restore default configuration.
   */
  lsm6dsox_reset_set(&ag_ctx, PROPERTY_ENABLE);
  do {
    lsm6dsox_reset_get(&ag_ctx, &rst);
  } while (rst);

  /*
   * Disable I3C interface.
   */
  lsm6dsox_i3c_disable_set(&ag_ctx, LSM6DSOX_I3C_DISABLE);

  /*
   * Set XL full scale.
   */
  lsm6dsox_xl_full_scale_set(&ag_ctx, LSM6DSOX_2g);

  /*
   *  Enable Block Data Update.
   */
  lsm6dsox_block_data_update_set(&ag_ctx, PROPERTY_ENABLE);

  /*
   * Set FIFO mode to Stream mode (aka Continuous Mode).
   */
  lsm6dsox_fifo_mode_set(&ag_ctx, LSM6DSOX_STREAM_MODE);

  /*
   * Enable latched interrupt notification.
   */
  lsm6dsox_int_notification_set(&ag_ctx, LSM6DSOX_ALL_INT_LATCHED);

 /*
   * Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed.
   */
  //lsm6dsox_data_ready_mode_set(&ag_ctx, LSM6DSOX_DRDY_PULSED);

#ifdef LSM6DSOX_INT1
  /*
   * FIFO watermark interrupt routed on INT1 pin.
   *
   * Remember that INT1 pin is used by sensor to switch in I3C mode.
   */
  lsm6dsox_pin_int1_route_get(&ag_ctx, &int1_route);
  int1_route.emb_func_int1.int1_step_detector = PROPERTY_ENABLE;
  lsm6dsox_pin_int1_route_set(&ag_ctx, &int1_route);
#else /* LSM6DSOX_INT1 */
  /*
   * FIFO watermark interrupt routed on INT2 pin.
   */
  lsm6dsox_pin_int2_route_get(&ag_ctx, &int2_route);
  int2_route.emb_func_int2.int2_step_detector = PROPERTY_ENABLE;
  lsm6dsox_pin_int2_route_set(&ag_ctx, &int2_route);
#endif /* LSM6DSOX_INT1 */

  /*
   * Enable HW Timestamp.
   */
  lsm6dsox_timestamp_set(&ag_ctx, PROPERTY_ENABLE);

  /*
   * Enable pedometer.
   */
  lsm6dsox_pedo_sens_set(&ag_ctx, LSM6DSOX_PEDO_BASE_MODE);
  lsm6dsox_fifo_pedo_batch_set(&ag_ctx, PROPERTY_ENABLE);
  lsm6dsox_steps_reset(&ag_ctx);

  /*
   * Enable I2C Master.
   */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_ENABLE);

  /*
   * Set Output Data Rate.
   */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_26Hz);

  while(1)
  {
	uint16_t num = 0;
    lsm6dsox_fifo_tag_t reg_tag;
    pedo_count_sample_t pedo_sample;

    /*
     * Read INT pin value.
     */
    if (platform_read_int_pin())
    {
    	/*
    	 * Read number of samples in FIFO.
    	 */
    	lsm6dsox_fifo_data_level_get(&ag_ctx, &num);

		while(num--)
    	{
			/*
			 * Read FIFO tag.
			 */
			lsm6dsox_fifo_sensor_tag_get(&ag_ctx, &reg_tag);

			switch(reg_tag)
			{
			case LSM6DSOX_STEP_CPUNTER_TAG:
				lsm6dsox_fifo_out_raw_get(&ag_ctx, pedo_sample.byte);

				sprintf((char*)tx_buffer, "Step Count :%u T %u\r\n",
						(unsigned int)pedo_sample.step_count,
						(unsigned int)pedo_sample.timestamp);
				tx_com(tx_buffer, strlen((char const*)tx_buffer));
				break;
			default:
				break;
			}
      }
    }
  }
}

