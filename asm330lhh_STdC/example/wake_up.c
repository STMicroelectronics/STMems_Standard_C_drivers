/*
 ******************************************************************************
 * @file    wake_up.c
 * @author  Sensor Solutions Software Team
 * @brief   This file show the simplest way to detect wake-up events from
 *          sensor.
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
#include <asm330lhh_reg.h>
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
static uint8_t slave_address = ASM330LHH_I2C_ADD_L;

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
 * Set XL Output Data Rate to 416 Hz
 * Set 2g full XL scale
 * Apply high-pass digital filter on Wake-Up function
 * Set Wake-Up threshold
 * Enable interrupt generation on Wake-Up INT1(2) pin
 * Poll to check Wake-Up events
 */
void example_main_wake_up_asm330lhh(void)
{
  stmdev_ctx_t dev_ctx;
  uint8_t whoamI, rst;

#ifdef ASM330LHH_INT1
  asm330lhh_pin_int1_route_t int1_route;
#else /* ASM330LHH_INT1 */
  /*
   * If need to route wake-up interrupt on INT2 use int2_route
   */
  asm330lhh_pin_int2_route_t int2_route;
#endif /* ASM330LHH_INT1 */

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
  asm330lhh_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != ASM330LHH_ID)
    while(1);

  /*
   *  Restore default configuration.
   */
  asm330lhh_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    asm330lhh_reset_get(&dev_ctx, &rst);
  } while (rst);

  /*
   * Set XL Output Data Rate to 416 Hz.
   */
  asm330lhh_xl_data_rate_set(&dev_ctx, ASM330LHH_XL_ODR_417Hz);

  /*
   * Set 2g full XL scale.
   */
  asm330lhh_xl_full_scale_set(&dev_ctx, ASM330LHH_2g);

  /*
   * Apply high-pass digital filter on Wake-Up function.
   */
  asm330lhh_xl_hp_path_internal_set(&dev_ctx, ASM330LHH_USE_SLOPE);

  /*
   * Set Wake-Up threshold: 1 LSb corresponds to FS_XL/2^6.
   */
  asm330lhh_wkup_threshold_set(&dev_ctx, 2);

#ifdef ASM330LHH_INT1
  /*
   * Enable interrupt generation on Wake-Up INT1 pin.
   */
  asm330lhh_pin_int1_route_get(&dev_ctx, &int1_route);
  int1_route.md1_cfg.int1_wu = PROPERTY_ENABLE;
  asm330lhh_pin_int1_route_set(&dev_ctx, &int1_route);
#else /* ASM330LHH_INT1 */
  /*
   * Uncomment if interrupt generation on Wake-Up INT2 pin.
   */
  asm330lhh_pin_int2_route_get(&dev_ctx, &int2_route);
  int2_route.md2_cfg.int2_wu = PROPERTY_ENABLE;
  asm330lhh_pin_int2_route_set(&dev_ctx, &int2_route);
#endif /* ASM330LHH_INT1 */


  /*
   * Wait Events.
   */
  while(1)
  {
	asm330lhh_all_sources_t all_source;

    /*
     * Check if Wake-Up events.
     */
	asm330lhh_all_sources_get(&dev_ctx, &all_source);
	if (all_source.wake_up_src.wu_ia)
	{
		sprintf((char*)tx_buffer, "Wake-Up event on ");
		if (all_source.wake_up_src.x_wu)
			strcat((char*)tx_buffer, "X");
		if (all_source.wake_up_src.y_wu)
			strcat((char*)tx_buffer, "Y");
		if (all_source.wake_up_src.z_wu)
			strcat((char*)tx_buffer, "Z");

		strcat((char*)tx_buffer, " direction\r\n");
		tx_com(tx_buffer, strlen((char const*)tx_buffer));
	}
  }
}
