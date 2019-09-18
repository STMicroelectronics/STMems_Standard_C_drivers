/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  MEMS Software Solution Team
 * @date    20-December-2017
 * @brief   This file show the simplest way to get data from sensor.
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
#include "iis328dq_reg.h"
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
static axis3bit16_t data_raw_acceleration;
static float acceleration_mg[3];
static uint8_t whoamI;
static uint8_t tx_buffer[TX_BUF_DIM];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   Replace the functions "platform_write" and "platform_read" with your
 *   platform specific read and write function.
 *   This example use an STM32 evaluation board and CubeMX tool.
 *   In this case the "*handle" variable is usefull in order to select the
 *   correct interface but the usage of "*handle" is not mandatory.
 */

static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80;
    HAL_I2C_Mem_Write(handle, IIS328DQ_I2C_ADD_H, Reg,
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
    HAL_I2C_Mem_Read(handle, IIS328DQ_I2C_ADD_H, Reg,
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
void tx_com( uint8_t *tx_buffer, uint16_t len )
{
  #ifdef NUCLEO_STM32F411RE 
  HAL_UART_Transmit( &huart2, tx_buffer, len, 1000 );
  #endif
  #ifdef MKI109V2 
  CDC_Transmit_FS( tx_buffer, len );
  #endif
}

/* Main Example --------------------------------------------------------------*/
void example_main(void)
{
  /*
   *  Initialize mems driver interface
   */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  #ifdef NUCLEO_STM32F411RE
  dev_ctx.handle = &hi2c1; 
  #endif
  #ifdef MKI109V2 
  dev_ctx.handle = &hspi2;
  #endif
  /*
   *  Check device ID
   */
  whoamI = 0;
  iis328dq_device_id_get(&dev_ctx, &whoamI);
  if ( whoamI != IIS328DQ_ID )
    while(1); /*manage here device not found */
  /*
   *  Enable Block Data Update
   */
  iis328dq_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Set full scale
   */ 
  iis328dq_full_scale_set(&dev_ctx, IIS328DQ_2g);
  /*
   * Configure filtering chain
   */ 
  /* Accelerometer - High Pass / Slope path */
  iis328dq_hp_path_set(&dev_ctx, IIS328DQ_HP_DISABLE);
  //iis328dq_hp_path_set(&dev_ctx, IIS328DQ_HP_ON_OUT);
  //iis328dq_hp_reset_get(&dev_ctx);
  /*
   * Set Output Data Rate
   */
  iis328dq_data_rate_set(&dev_ctx, IIS328DQ_ODR_5Hz);
  /*
   * Read samples in polling mode (no int)
   */
  while(1)
  {
    /*
     * Read output only if new value is available
     */
    iis328dq_reg_t reg;
    iis328dq_status_reg_get(&dev_ctx, &reg.status_reg);

    if (reg.status_reg.zyxda)
    {
      /* Read acceleration data */
      memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
      iis328dq_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
      acceleration_mg[0] = IIS328DQ_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] = IIS328DQ_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] = IIS328DQ_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[2]);
     
      sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }
  }
}
