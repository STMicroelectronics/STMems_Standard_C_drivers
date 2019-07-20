/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  MEMS Software Solution Team
 * @date    30-November-2017
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "..\..\stdc\lsm6ds3tr_c_STdC\driver\lsm6ds3tr_c_reg.h"
#include <string.h>

#define MKI109V2
//#define NUCLEO_STM32F411RE

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
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[TX_BUF_DIM];

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
    HAL_I2C_Mem_Write(handle, LSM6DS3TR_C_I2C_ADD_H, Reg,
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
      HAL_I2C_Mem_Read(handle, LSM6DS3TR_C_I2C_ADD_H, Reg,
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
  lsm6ds3tr_c_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hspi2;  
  /*
   *  Check device ID
   */
  whoamI = 0;
  lsm6ds3tr_c_device_id_get(&dev_ctx, &whoamI);
  if ( whoamI != LSM6DS3TR_C_ID )
    while(1); /*manage here device not found */
  /*
   *  Restore default configuration
   */
  lsm6ds3tr_c_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6ds3tr_c_reset_get(&dev_ctx, &rst);
  } while (rst);
  /*
   *  Enable Block Data Update
   */
  lsm6ds3tr_c_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Set Output Data Rate
   */
  lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_12Hz5);
  lsm6ds3tr_c_gy_data_rate_set(&dev_ctx, LSM6DS3TR_C_GY_ODR_12Hz5);
  /*
   * Set full scale
   */  
  lsm6ds3tr_c_xl_full_scale_set(&dev_ctx, LSM6DS3TR_C_2g);
  lsm6ds3tr_c_gy_full_scale_set(&dev_ctx, LSM6DS3TR_C_2000dps);
  
  /*
   * Configure filtering chain(No aux interface)
   */  
  /* Accelerometer - analog filter */
  lsm6ds3tr_c_xl_filter_analog_set(&dev_ctx, LSM6DS3TR_C_XL_ANA_BW_400Hz);
  
  /* Accelerometer - LPF1 path ( LPF2 not used )*/
  //lsm6ds3tr_c_xl_lp1_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_LP1_ODR_DIV_4);
  
  /* Accelerometer - LPF1 + LPF2 path */   
  lsm6ds3tr_c_xl_lp2_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_LOW_NOISE_LP_ODR_DIV_100);
  
  /* Accelerometer - High Pass / Slope path */
  //lsm6ds3tr_c_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
  //lsm6ds3tr_c_xl_hp_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_HP_ODR_DIV_100);
  
  /* Gyroscope - filtering chain */
  lsm6ds3tr_c_gy_band_pass_set(&dev_ctx, LSM6DS3TR_C_HP_260mHz_LP1_STRONG);
  
  /*
   * Read samples in polling mode (no int)
   */
  while(1)
  {
    /*
     * Read output only if new value is available
     */
    lsm6ds3tr_c_reg_t reg;
    lsm6ds3tr_c_status_reg_get(&dev_ctx, &reg.status_reg);

    if (reg.status_reg.xlda)
    {
      /* Read magnetic field data */
      memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
      lsm6ds3tr_c_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
      acceleration_mg[0] = LSM6DS3TR_C_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] = LSM6DS3TR_C_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] = LSM6DS3TR_C_FROM_FS_2g_TO_mg( data_raw_acceleration.i16bit[2]);
      
      sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    } 
    if (reg.status_reg.gda)
    {
      /* Read magnetic field data */
      memset(data_raw_angular_rate.u8bit, 0x00, 3*sizeof(int16_t));
      lsm6ds3tr_c_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
      angular_rate_mdps[0] = LSM6DS3TR_C_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[0]);
      angular_rate_mdps[1] = LSM6DS3TR_C_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[1]);
      angular_rate_mdps[2] = LSM6DS3TR_C_FROM_FS_2000dps_TO_mdps(data_raw_angular_rate.i16bit[2]);
      
      sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }    
    if (reg.status_reg.tda)
    {   
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      lsm6ds3tr_c_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
      temperature_degC = LSM6DS3TR_C_FROM_LSB_TO_degC( data_raw_temperature.i16bit );
       
      sprintf((char*)tx_buffer, "Temperature [degC]:%6.2f\r\n", temperature_degC );
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }
  }
}

