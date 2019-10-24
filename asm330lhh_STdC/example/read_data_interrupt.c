/*
 ******************************************************************************
 * @file    read_data_interrupt.c
 * @author  Sensor Solutions Software Team
 * @brief   This file show the simplest way to get data from sensor (interrupt
 *           mode).
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

/* Platform STM32F411RE + IKS01A2 Interrupt PIN */
#define ASM330LHH_INT2_PIN GPIO_PIN_1
#define ASM330LHH_INT2_GPIO_PORT GPIOC
#define ASM330LHH_INT1_PIN GPIO_PIN_0
#define ASM330LHH_INT1_GPIO_PORT GPIOC
#endif

#define TX_BUF_DIM          1000

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static uint8_t whoamI, rst;
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
  if (handle == &hi2c1) {
    HAL_I2C_Mem_Write(handle, slave_address, Reg,
                      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
#ifdef MKI109V2
  else if (handle == &hspi2) {
    HAL_GPIO_WritePin(CS_SPI2_GPIO_Port, CS_SPI2_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Transmit(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_SPI2_GPIO_Port, CS_SPI2_Pin, GPIO_PIN_SET);
  } else if (handle == &hspi1) {
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
  if (handle == &hi2c1) {
      HAL_I2C_Mem_Read(handle, slave_address, Reg,
                       I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
#ifdef MKI109V2
  else if (handle == &hspi2) {
    Reg |= 0x80;
    HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Receive(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_SET);
  } else {
    Reg |= 0x80;
    HAL_GPIO_WritePin(CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Receive(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_RF_GPIO_Port, CS_RF_Pin, GPIO_PIN_SET);
  }
#endif
  return 0;
}

/* Function to print messages. */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  #ifdef NUCLEO_STM32F411RE
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
  #endif
  #ifdef MKI109V2
  CDC_Transmit_FS(tx_buffer, len);
  #endif
}

/* Function to read external interrupt pin connected to INT sensor. */
static int32_t platform_read_int_pin(void)
{
#ifdef NUCLEO_STM32F411RE
#ifndef ASM330LHH_INT1
    return HAL_GPIO_ReadPin(ASM330LHH_INT2_GPIO_PORT, ASM330LHH_INT2_PIN);
#else /* ASM330LHH_INT1 */
    return HAL_GPIO_ReadPin(ASM330LHH_INT1_GPIO_PORT, ASM330LHH_INT1_PIN);
#endif /* ASM330LHH_INT1 */

#else /* NUCLEO_STM32F411RE */
    return 0;
#endif /* NUCLEO_STM32F411RE */
}

/* Function to wait for a timeout. */
static void platform_delay(uint32_t timeout)
{
  /* Force compiler to not optimize this code. */
  volatile uint32_t i;
  for(i = 0; i < timeout; i++);
}

/* Platform specific initialization. */
static void platform_init(void)
{
#ifdef NUCLEO_STM32F411RE
  uint8_t i;
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Configure GPIO pin : PB8
   * Set OUTPUT mode
   */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Force sensor to exit from Hot Join I3C mode. */
  for (i = 0; i < 9; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    platform_delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    platform_delay(100);
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

  /* Init I2C interface. */
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
void example_main_interrupt_asm330lhh(void)
{
  stmdev_ctx_t dev_ctx;

#ifdef ASM330LHH_INT1
  /* If need to route drdy interrupt on INT1 use int1_route */
  asm330lhh_pin_int1_route_t int1_route;
#else /* ASM330LHH_INT1 */
  asm330lhh_pin_int2_route_t int2_route;
#endif /* ASM330LHH_INT1 */

  /* Initialize mems driver interface. */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /* Init test platform. */
  platform_init();

  /* Check device ID. */
  asm330lhh_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != ASM330LHH_ID)
    while(1);

  /* Restore default configuration. */
  asm330lhh_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    asm330lhh_reset_get(&dev_ctx, &rst);
  } while (rst);
  
  /* Start device configuration. */
  asm330lhh_device_conf_set(&dev_ctx, PROPERTY_ENABLE);

  /* Enable Block Data Update. */
  asm330lhh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set Output Data Rate. */
  asm330lhh_xl_data_rate_set(&dev_ctx, ASM330LHH_XL_ODR_12Hz5);
  asm330lhh_gy_data_rate_set(&dev_ctx, ASM330LHH_GY_ODR_12Hz5);

  /* Set full scale. */
  asm330lhh_xl_full_scale_set(&dev_ctx, ASM330LHH_2g);
  asm330lhh_gy_full_scale_set(&dev_ctx, ASM330LHH_2000dps);

  /* Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed. */
  //asm330lhh_data_ready_mode_set(&dev_ctx, ASM330LHH_DRDY_PULSED);

#ifdef ASM330LHH_INT1
  /* Enable interrupt generation on DRDY INT1 pin.
   * Remember that INT1 pin is used by sensor to switch in I3C mode.
   */
  asm330lhh_pin_int1_route_get(&dev_ctx, &int1_route);
  int1_route.int1_ctrl.int1_drdy_g = PROPERTY_ENABLE;
  int1_route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
  asm330lhh_pin_int1_route_set(&dev_ctx, &int1_route);
#else /* ASM330LHH_INT1 */
  /* Interrupt generation routed on DRDY INT2 pin. */
  asm330lhh_pin_int2_route_get(&dev_ctx, &int2_route);
  int2_route.int2_ctrl.int2_drdy_g = PROPERTY_ENABLE;
  int2_route.int2_ctrl.int2_drdy_xl = PROPERTY_ENABLE;
  asm330lhh_pin_int2_route_set(&dev_ctx, &int2_route);
#endif /* ASM330LHH_INT1 */

  /* Wait samples. */
  while(1) {
    asm330lhh_reg_t reg;

    /* Read INT pin. */
    if (platform_read_int_pin()) {
    /* Read output only if new value is available. */
    asm330lhh_status_reg_get(&dev_ctx, &reg.status_reg);

    if (reg.status_reg.xlda) {
        /* Read acceleration field data. */
        memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
        asm330lhh_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
        acceleration_mg[0] =
        asm330lhh_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
        acceleration_mg[1] =
        asm330lhh_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
        acceleration_mg[2] =
        asm330lhh_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);

        sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
        acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
        tx_com(tx_buffer, strlen((char const*)tx_buffer));
      }
      if (reg.status_reg.gda){
        /* Read angular rate field data. */
        memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
        asm330lhh_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
        angular_rate_mdps[0] =
        asm330lhh_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
        angular_rate_mdps[1] =
        asm330lhh_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
        angular_rate_mdps[2] =
        asm330lhh_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

        sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
        angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
        tx_com(tx_buffer, strlen((char const*)tx_buffer));
      }
    }
  }
}
