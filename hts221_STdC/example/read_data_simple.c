/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  MEMS Software Solution Team
 * @date    27-September-2017
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
#include "..\..\stdc\hts221_STdC\driver\hts221_reg.h"
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

typedef struct {
  float x0;
  float y0;
  float x1;
  float y1;  
} lin_t;

/* Private variables ---------------------------------------------------------*/
static axis1bit16_t data_raw_humidity;
static axis1bit16_t data_raw_temperature;
static float humidity_perc;
static float temperature_degC;
static uint8_t whoamI;
static uint8_t tx_buffer[TX_BUF_DIM];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   Replace the functions "platform_write" and "platform_read" with your
 *   platform specific read and write function.
 *   This example use an STM32 evaluation board and CubeMX tool.
 *   In this case the "*handle" variable is usefull in order to select the
 *   correct interface but the usage uf "*handle" is not mandatory.
 */

static int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80; 
    HAL_I2C_Mem_Write(handle, HTS221_I2C_ADDRESS, Reg,
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
    HAL_I2C_Mem_Read(handle, HTS221_I2C_ADDRESS, Reg,
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

/*
 *  Function used to apply coefficient
 */
float linear_interpolation(lin_t *lin, int16_t x) 
{
 return ((lin->y1 - lin->y0) * x +  ((lin->x1 * lin->y0) - (lin->x0 * lin->y1)))
        / (lin->x1 - lin->x0);
}

/* Main Example --------------------------------------------------------------*/
void example_main(void)
{
  /*
   *  Initialize mems driver interface
   */
  hts221_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;  
  
  /*
   *  Check device ID
   */
  whoamI = 0;
  hts221_device_id_get(&dev_ctx, &whoamI);
  if ( whoamI != HTS221_ID )
    while(1); /*manage here device not found */

  /*
   *  Read humidity calibration coefficient
   */  
  //
  axis1bit16_t coeff;
  lin_t lin_hum;
  hts221_hum_adc_point_0_get(&dev_ctx, coeff.u8bit);
  lin_hum.x0 = (float)coeff.i16bit; 
  hts221_hum_rh_point_0_get(&dev_ctx, coeff.u8bit);
  lin_hum.y0 = (float)coeff.u8bit[0];
  hts221_hum_adc_point_1_get(&dev_ctx, coeff.u8bit);
  lin_hum.x1 = (float)coeff.i16bit; 
  hts221_hum_rh_point_1_get(&dev_ctx, coeff.u8bit);  
  lin_hum.y1 = (float)coeff.u8bit[0];
    
  /*
   *  Read temperature calibration coefficient
   */  
  lin_t lin_temp;
  hts221_temp_adc_point_0_get(&dev_ctx, coeff.u8bit);
  lin_temp.x0 = (float)coeff.i16bit; 
  hts221_temp_deg_point_0_get(&dev_ctx, coeff.u8bit);
  lin_temp.y0 = (float)coeff.u8bit[0];
  hts221_temp_adc_point_1_get(&dev_ctx, coeff.u8bit);
  lin_temp.x1 = (float)coeff.i16bit; 
  hts221_temp_deg_point_1_get(&dev_ctx, coeff.u8bit);  
  lin_temp.y1 = (float)coeff.u8bit[0];  
  
  /*
   *  Enable Block Data Update
   */
  hts221_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Set Output Data Rate
   */
  hts221_data_rate_set(&dev_ctx, HTS221_ODR_1Hz);
  /*
   * Device power on
   */
  hts221_power_on_set(&dev_ctx, PROPERTY_ENABLE);
  
  /*
   * Read samples in polling mode (no int)
   */
  while(1)
  {
    /*
     * Read output only if new value is available
     */
    hts221_reg_t reg;
    hts221_status_get(&dev_ctx, &reg.status_reg);
    
    if (reg.status_reg.h_da)
    {
      /* Read humidity data */
      memset(data_raw_humidity.u8bit, 0x00, sizeof(int16_t));
      hts221_humidity_raw_get(&dev_ctx, data_raw_humidity.u8bit);
      humidity_perc = linear_interpolation(&lin_hum, data_raw_humidity.i16bit);
      if (humidity_perc < 0) humidity_perc = 0;
      if (humidity_perc > 100) humidity_perc = 100;
      sprintf((char*)tx_buffer, "Humidity [%%]:%3.2f\r\n", humidity_perc);
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );      
    }
    if (reg.status_reg.t_da)
    {
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      hts221_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
      temperature_degC = linear_interpolation(&lin_temp, data_raw_temperature.i16bit);
      sprintf((char*)tx_buffer, "Temperature [degC]:%6.2f\r\n", temperature_degC );
      tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
    }
  }
}


  
