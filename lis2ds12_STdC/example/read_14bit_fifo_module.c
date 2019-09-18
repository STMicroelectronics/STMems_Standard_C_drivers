/*
 ******************************************************************************
 * @file    read_14bit_fifo_module.c
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
static uint16_t magnitude[30];

static uint8_t whoamI, rst;

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

uint16_t samples, fail;

/* Main Example --------------------------------------------------------------*/
/*
 * @brief  This file shows the simplest way to get data from sensor.
 */
void lis2ds12_14bit_module(void)
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

  /*
   *  Check device ID.
   */
  whoamI = 0;
  lis2ds12_device_id_get(&dev_ctx, &whoamI);
  if ( whoamI != LIS2DS12_ID )
    while(1); /*manage here device not found */

  /*
   *  Restore default configuration.
   */
  lis2ds12_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lis2ds12_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update. */
  lis2ds12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set full scale. */ 
  lis2ds12_xl_full_scale_set(&dev_ctx, LIS2DS12_4g);

  /* Configure filtering chain. */ 
  /* Accelerometer - High Pass / Slope path */
  //lis2ds12_xl_hp_path_set(&dev_ctx, LIS2DS12_HP_ON_OUTPUTS);
  /* Module processing enable. */
  lis2ds12_module_sens_set(&dev_ctx, PROPERTY_ENABLE);
  /* Module routine result is send to FIFO */
  lis2ds12_fifo_xl_module_batch_set(&dev_ctx, PROPERTY_ENABLE);
  /* FIFO mode selection: FIFO_MODE */
  lis2ds12_fifo_mode_set(&dev_ctx, LIS2DS12_FIFO_MODE);
  /* FIFO watermark level selection. */
  lis2ds12_fifo_watermark_set(&dev_ctx, 10);

  /* Select the signal that need to route on int1 pad. */
  lis2ds12_pin_int1_route_t pin1;
  pin1.int1_6d = 0;
  pin1.int1_drdy = 0;
  pin1.int1_ff = 0;
  pin1.int1_fss7 = 0;
  pin1.int1_fth = 0;
  pin1.int1_master_drdy = 0;
  pin1.int1_s_tap = 0;
  pin1.int1_tap = 0;
  pin1.int1_wu = 0;   
  lis2ds12_pin_int1_route_set(&dev_ctx, pin1);
 
  /* Select the signal that need to route on int2 pad. */
  lis2ds12_pin_int2_route_t pin2;
  pin2.int2_boot = 0;
  pin2.int2_drdy = 0;
  pin2.int2_fth = 0;
  pin2.int2_sig_mot = 0;
  pin2.int2_step_det = 0;
  pin2.int2_tilt = 0;
  lis2ds12_pin_int2_route_set(&dev_ctx, pin2);
 
  /* Set Output Data Rate. Maximum ODR supported is 800Hz */
  lis2ds12_xl_data_rate_set(&dev_ctx, LIS2DS12_XL_ODR_100Hz_HR);
  fail = 0;
  /* Read samples in polling mode (no int). */
  while(1)
  {
    uint8_t i, j;
    uint8_t fifo_ths_flag;
    uint8_t fifo_full_flag;   
     
    /* Read fifo when is full. */
    lis2ds12_fifo_wtm_flag_get(&dev_ctx, &fifo_ths_flag);
    lis2ds12_fifo_full_flag_get(&dev_ctx, &fifo_full_flag);
   

    if (fifo_ths_flag)
    {
      HAL_Delay(500);
       
      /* Read acceleration data. */
      //memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
      lis2ds12_fifo_data_level_get(&dev_ctx, &samples);
      //lis2ds12_read_reg(&dev_ctx,  LIS2DS12_OUT_X_L, (uint8_t*) acceleration_mg, 256 * 3);
      j = 0;
      for (i=0; i<10; i++){
        lis2ds12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
  /*     
        magnitude[j++] = data_raw_acceleration.i16bit[0] >> 2;
        magnitude[j++] = data_raw_acceleration.i16bit[1] >> 2;
        magnitude[j++] = data_raw_acceleration.i16bit[2] >> 2;
  */
    magnitude[j++] = (uint16_t)lis2ds12_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]);    
    magnitude[j++] = (uint16_t)lis2ds12_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]);    
    magnitude[j++] = (uint16_t)lis2ds12_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]);    
     
  /*
    LIS2DS12_FROM_FS_2g_TO_mg(lsb)     
    LIS2DS12_FROM_FS_4g_TO_mg(lsb)     
    LIS2DS12_FROM_FS_8g_TO_mg(lsb)     
    LIS2DS12_FROM_FS_16g_TO_mg(lsb)    
   */    
       
       
        /*
        sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
        tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
        */
      }
      for (i=0; i<30; i++){
        if ( ( magnitude[i] > 4200 ) || ( magnitude[i] < 3900 ) ){
         fail++;
        }
      } 
      /* FIFO mode selection: FIFO_MODE */
      lis2ds12_fifo_mode_set(&dev_ctx, LIS2DS12_BYPASS_MODE); 
      /* FIFO mode selection: FIFO_MODE */
      lis2ds12_fifo_mode_set(&dev_ctx, LIS2DS12_FIFO_MODE);     
     
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
