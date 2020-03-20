/*
 ******************************************************************************
 * @file    read_8bit_module.c
 * @author  Sensors Software Solution Team
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

/*
 *   GETTING STARTED:
 *
 *   If you have one of the following STMicroelectronics development tool
 *
 *   This example use an STM32F4xx development board and CubeMX tool.
 *
 *
 *   In this case the "*handle" variable is useful in order to select the
 *   correct interface but the usage of "*handle" is not mandatory.
 *
 * -> STEVAL_MKI109V3
 * -> NUCLEO_F411RE + X_NUCLEO_IKS01A1
 * -> NUCLEO_F411RE + X_NUCLEO_IKS01A2
 *
 *   If you need to run this example on a different hardware platform a
 *   modification of the functions: "platform_write", "platform_read" and
 *   "tx_com" is required.
 *
 */

//#define STEVAL_MKI109V3
//#define NUCLEO_F411RE_X_NUCLEO_IKS01A1
#define NUCLEO_F411RE_X_NUCLEO_IKS01A2

/* Includes ------------------------------------------------------------------*/
/*
 * Specific platform includes:
 *
 * STEVAL_MKI109V3    -> USB  (Virtual COM) to PC
 * NUCLEO_STM32F411RE -> UART (COM) to USB bridge
 *
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A1 -> SPI N/A
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A2 -> SPI N/A
 *
 */

#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "lis2ds12_reg.h"
#include "gpio.h"
#include "i2c.h"
#ifdef STEVAL_MKI109V3
#include "usbd_cdc_if.h"
#include "spi.h"
#endif
#ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A1
#include "usart.h"
#endif
#ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
#include "usart.h"
#endif

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static uint8_t magnitude_8bit;
static float acceleration_mg[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 *
 */
static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );


/* Main Example --------------------------------------------------------------*/
/*
 * @brief  This file shows the simplest way to get data from sensor.
 */
void lis2ds12_read_8bit_module(void)
{

  /* PWM Define */
  #define PWM_3V3 915
  #define PWM_1V8 498


  TIM3->CCR1 = PWM_1V8;
  TIM3->CCR2 = PWM_1V8;
  HAL_Delay(1000);
  /*
   *  Initialize mems driver interface.
   */
  stmdev_ctx_t dev_ctx;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /* Check device ID. */
  whoamI = 0;
  lis2ds12_device_id_get(&dev_ctx, &whoamI);
  if ( whoamI != LIS2DS12_ID )
    while(1); /*manage here device not found */

  /* Restore default configuration. */
  lis2ds12_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lis2ds12_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update. */
  lis2ds12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set full scale. */
  lis2ds12_xl_full_scale_set(&dev_ctx, LIS2DS12_4g);

  /* Enable pedometer algorithm. */
  //lis2ds12_pedo_sens_set(&dev_ctx, PROPERTY_ENABLE);
  lis2ds12_motion_sens_set(&dev_ctx, PROPERTY_ENABLE);
  //lis2ds12_tilt_sens_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set Output Data Rate. */
  lis2ds12_xl_data_rate_set(&dev_ctx, LIS2DS12_XL_ODR_1Hz_LP);

  while(1)
  {
    /*
     * Read output only if new value is available.
     */
    lis2ds12_reg_t reg;
    lis2ds12_status_reg_get(&dev_ctx, &reg.status);

    //lis2ds12_read_reg(&dev_ctx, LIS2DS12_FUNC_SRC, &reg.byte, 1);

    if (reg.status.drdy)
    //if (reg.func_src.module_ready)
    {
      /*
       * Read acceleration data.
       */
      lis2ds12_acceleration_module_raw_get(&dev_ctx, &magnitude_8bit);

      memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
      lis2ds12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
      acceleration_mg[0] = lis2ds12_from_fs2g_to_mg( data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] = lis2ds12_from_fs2g_to_mg( data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] = lis2ds12_from_fs2g_to_mg( data_raw_acceleration.i16bit[2]);

      sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\t%d\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],magnitude_8bit);
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }
  }
}


/**
  * @brief  Write generic device register
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t reg: register to write
  * @param  uint8_t* data: pointer to data to write in register reg
  * @param  uint16_t len: number of consecutive register to write
  *
*/
static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, LIS2DS12_I2C_ADD_H, Reg,
                      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Transmit(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }
#endif
  return 0;
}

/**
  * @brief  Write generic device register
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t reg: register to write
  * @param  uint8_t* data: pointer to data to write in register reg
  * @param  uint16_t len: number of consecutive register to write
  *
  *
  */
static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  if (handle == &hi2c1)
  {
      HAL_I2C_Mem_Read(handle, LIS2DS12_I2C_ADD_H, Reg,
                       I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
    Reg |= 0x80;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &Reg, 1, 1000);
    HAL_SPI_Receive(handle, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }
#endif
  return 0;
}

/**
  * @brief  Write generic device register
  *
  * @param  lis2dw12_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t reg: register to write
  * @param  uint8_t* data: pointer to data to write in register reg
  * @param  uint16_t len: number of consecutive register to write
  *
*/
static void tx_com( uint8_t *tx_buffer, uint16_t len )
{
  #ifdef NUCLEO_STM32F411RE
  HAL_UART_Transmit( &huart2, tx_buffer, len, 1000 );
  #endif
  #ifdef STEVAL_MKI109V3
  CDC_Transmit_FS( tx_buffer, len );
  #endif
}
