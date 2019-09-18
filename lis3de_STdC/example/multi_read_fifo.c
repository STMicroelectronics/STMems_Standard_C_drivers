/*
 ******************************************************************************
 * @file    multi_read_fifo.c
 * @author  Sensors Software Solution Team
 * @brief   LIS3DE driver file
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

/* Includes ------------------------------------------------------------------*/
#include "lis3de_reg.h"
#include <string.h>
#include <stdio.h>

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

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

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
#endif /* NUCLEO_XXX */

#define TX_BUF_DIM          1000

/* Define number of byte for each sensor sample */
#define OUT_XYZ_SIZE		6

/* Define FIFO watermark to 10 samples */
#define FIFO_WATERMARK		10

/* Private variables ---------------------------------------------------------*/

static uint8_t tx_buffer[TX_BUF_DIM];

static float acceleration_mg[3];
static axis3bit16_t data_raw_acceleration;
static stmdev_ctx_t dev_ctx;

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
  #endif /* NUCLEO_STM32F411RE */

  #ifdef MKI109V2
  CDC_Transmit_FS( tx_buffer, len );
  #endif /* MKI109V2 */
}

/*
 * example_main - Main Example FIFO support
 *
 * FIFO read acc sample
 */
void example_lis3de_fifo(void)
{
  uint8_t whoamI;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  whoamI = 0;
  lis3de_device_id_get(&dev_ctx, &whoamI);
  if ( whoamI != LIS3DE_ID )
    while(1); /*manage here device not found */

  /* Enable Block Data Update. */
  lis3de_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set Output Data Rate to 100 hz. */
  lis3de_data_rate_set(&dev_ctx, LIS3DE_ODR_100Hz);

  /* Set full scale to 2 g. */
  lis3de_full_scale_set(&dev_ctx, LIS3DE_2g);

  /*
   * Set operating mode to high resolution.
   */
  lis3de_operating_mode_set(&dev_ctx, LIS3DE_LP);

  /* Set FIFO watermark to FIFO_WATERMARK samples. */
  lis3de_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);

  /*
   * Set FIFO mode to Stream mode: Accumulate samples and
   * override old data.
   */
  lis3de_fifo_mode_set(&dev_ctx, LIS3DE_DYNAMIC_STREAM_MODE);

  /* Enable FIFO. */
  lis3de_fifo_set(&dev_ctx, PROPERTY_ENABLE);

  while(1)
  {
	uint8_t flags;
	uint8_t num = 0;

	/* Check if FIFO level over threshold. */
	lis3de_fifo_fth_flag_get(&dev_ctx, &flags);
	if (flags)
	{
		/* Read number of sample in FIFO. */
		lis3de_fifo_data_level_get(&dev_ctx, &num);

		while (num-- > 0)
		{
			/* Read XL samples. */
			lis3de_acceleration_raw_get(&dev_ctx, data_raw_acceleration.i16bit);
			acceleration_mg[0] = lis3de_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
			acceleration_mg[1] = lis3de_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
			acceleration_mg[2] = lis3de_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);

			sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
			   		acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
			tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
		}
	}
	else
	{
		/* Force compiler to generate code, avoiding I2C polling stress. */
    	for (volatile uint32_t i = 0; i < 10000; i++);
    }
  }
}

