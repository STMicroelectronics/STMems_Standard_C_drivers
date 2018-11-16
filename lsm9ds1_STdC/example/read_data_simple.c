/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A2
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A2 - Host side: UART(COM) to USB bridge
 *                                       - I2C(Default) / SPI(N/A)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */
#define STEVAL_MKI109V3
//#define NUCLEO_F411RE_X_NUCLEO_IKS01A2

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "stm32f4xx_hal.h"
#include "lsm9ds1_reg.h"
#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
#include "usart.h"
#endif

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t data_raw_magnetic_field;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void example_main_lsm9ds1(void)
{
  /* Initialize platform specific hardware */
  platform_init();

  /* Initialize inertial sensors (IMU) driver interface */
  uint8_t i2c_add_mag = LSM9DS1_MAG_I2C_ADD_L;
  lsm9ds1_ctx_t dev_ctx_mag;
  dev_ctx_mag.write_reg = platform_write;
  dev_ctx_mag.read_reg = platform_read;
  dev_ctx_mag.handle = (void*)&i2c_add_mag;

  /* Initialize magnetic sensors driver interface */
  uint8_t i2c_add_imu = LSM9DS1_IMU_I2C_ADD_H;
  lsm9ds1_ctx_t dev_ctx_imu;
  dev_ctx_imu.write_reg = platform_write;
  dev_ctx_imu.read_reg = platform_read;
  dev_ctx_imu.handle = (void*)&i2c_add_imu;

  /* Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);
  if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID){
    while(1){
      /* manage here device not found */
    }
  }

  /* Restore default configuration */
  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
  do {
    lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

  /* Set full scale */
  lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
  lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
  lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);

  /* Configure filtering chain - See datasheet for filtering chain details */
  /* Accelerometer filtering chain */
  lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
  lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
  lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
  /* Gyroscope filtering chain */
  lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
  lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
  lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);

  /* Set Output Data Rate / Power mode */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
  lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

  /* Read samples in polling mode (no int) */
  while(1)
  {
    /* Read device status register */
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

    if ( reg.status_imu.xlda && reg.status_imu.gda )
    {
      /* Read imu data */
      memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
      memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));

      lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration.u8bit);
      lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate.u8bit);

      acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]);

      angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
      angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
      angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

      sprintf((char*)tx_buffer, "IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
    }

    if ( reg.status_mag.zyxda )
    {
      /* Read magnetometer data */
      memset(data_raw_magnetic_field.u8bit, 0x00, 3 * sizeof(int16_t));

      lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field.u8bit);

      magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[0]);
      magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[1]);
      magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[2]);

      sprintf((char*)tx_buffer, "MAG - [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
              magnetic_field_mgauss[0], magnetic_field_mgauss[1], magnetic_field_mgauss[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
    }
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor I2C address.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  uint8_t *i2c_address = handle;
  HAL_I2C_Mem_Write(&hi2c1, *i2c_address, reg,
                    I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor I2C address.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  uint8_t *i2c_address = handle;
  HAL_I2C_Mem_Read(&hi2c1, *i2c_address, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to trasmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  #ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
  #endif
  #ifdef STEVAL_MKI109V3
  CDC_Transmit_FS(tx_buffer, len);
  #endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#ifdef STEVAL_MKI109V3
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_Delay(1000);
#endif
}
