/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  MEMS Software Solution Team
 * @date    12-October-2017
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
#include "lsm303agr_reg.h"
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
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_magnetic;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float magnetic_mG[3];
static float temperature_degC;
static uint8_t whoamI, rst;
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
  uint32_t i2c_add = (uint32_t)handle;
  if (i2c_add == LSM303AGR_I2C_ADD_XL)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80; 
  }
  HAL_I2C_Mem_Write(&hi2c1, i2c_add, Reg,
                    I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  uint32_t i2c_add = (uint32_t)handle;
  if (i2c_add == LSM303AGR_I2C_ADD_XL)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80;
  }
  HAL_I2C_Mem_Read(&hi2c1, (uint8_t) i2c_add, Reg,
                   I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
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
  
  lsm303agr_ctx_t dev_ctx_xl;
  dev_ctx_xl.write_reg = platform_write;
  dev_ctx_xl.read_reg = platform_read;
  dev_ctx_xl.handle = (void*)LSM303AGR_I2C_ADD_XL;  
  
  lsm303agr_ctx_t dev_ctx_mg;
  dev_ctx_mg.write_reg = platform_write;
  dev_ctx_mg.read_reg = platform_read;
  dev_ctx_mg.handle = (void*)LSM303AGR_I2C_ADD_MG;  
  
  /*
   *  Check device ID
   */
  whoamI = 0;
  lsm303agr_xl_device_id_get(&dev_ctx_xl, &whoamI);
  if ( whoamI != LSM303AGR_ID_XL )
    while(1); /*manage here device not found */
  
  whoamI = 0;
  lsm303agr_mag_device_id_get(&dev_ctx_mg, &whoamI);
  if ( whoamI != LSM303AGR_ID_MG )
    while(1); /*manage here device not found */  
    
  /*
   *  Restore default configuration for magnetometer
   */
  lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);
  do {
     lsm303agr_mag_reset_get(&dev_ctx_mg, &rst);
  } while (rst);  
    
  /*
   *  Enable Block Data Update
   */
  lsm303agr_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
  lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /*
   * Set Output Data Rate
   */
  lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_1Hz);
  lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_10Hz);
  /*
   * Set accelerometer full scale
   */  
  lsm303agr_xl_full_scale_set(&dev_ctx_xl, LSM303AGR_2g);
  /*
   * Set / Reset magnetic sensor mode
   */  
  lsm303agr_mag_set_rst_mode_set(&dev_ctx_mg, LSM303AGR_SENS_OFF_CANC_EVERY_ODR);  
  /*
   * Enable temperature compensation on mag sensor
   */  
  lsm303agr_mag_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);  
  /*
   * Enable temperature sensor
   */   
  lsm303agr_temperature_meas_set(&dev_ctx_xl, LSM303AGR_TEMP_ENABLE);
  /*
   * Set device in continuos mode
   */   
  lsm303agr_xl_operating_mode_set(&dev_ctx_xl, LSM303AGR_HR_12bit);
  /*
   * Set magnetometer in continuos mode
   */   
  lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_CONTINUOUS_MODE);
  
  /*
   * Read samples in polling mode (no int)
   */
  while(1)
  {
    /*
     * Read output only if new value is available
     */
    lsm303agr_reg_t reg;
    lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
 
    if (reg.status_reg_a.zyxda)
    {
      /* Read accelerometer data */
      memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
      lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw_acceleration.u8bit);
      acceleration_mg[0] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[0] );
      acceleration_mg[1] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[1] );
      acceleration_mg[2] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[2] );
      
      sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }
    
    lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
    if (reg.status_reg_m.zyxda)
    {
      /* Read magnetic field data */
      memset(data_raw_magnetic.u8bit, 0x00, 3*sizeof(int16_t));
      lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw_magnetic.u8bit);
      magnetic_mG[0] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[0]);
      magnetic_mG[1] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[1]);
      magnetic_mG[2] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[2]);
      
      sprintf((char*)tx_buffer, "Magnetic field [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
              magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }
    
    lsm303agr_temp_data_ready_get(&dev_ctx_xl, &reg.byte);      
    if (reg.byte)      
    {
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      lsm303agr_temperature_raw_get(&dev_ctx_xl, data_raw_temperature.u8bit);
      temperature_degC = lsm303agr_from_lsb_hr_to_celsius( data_raw_temperature.i16bit );
       
      sprintf((char*)tx_buffer, "Temperature [degC]:%6.2f\r\n", temperature_degC );
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }
  }
}

