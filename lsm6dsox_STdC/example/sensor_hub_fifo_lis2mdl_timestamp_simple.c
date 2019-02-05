/*
 ******************************************************************************
 * @file    sensor_hub_fifo_lis2mdl_timestamp_simple.c
 * @author  Sensor Solutions Software Team
 * @brief   This file show the simplest way to read LIS2MDL mag
 *          connected to LSM6DSOX I2C master interface (with FIFO support).
 *          Enable also Timestamp in FIFO.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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
#include <lsm6dsox_reg.h>
#include <lis2mdl_reg.h>
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

#if defined(NUCLEO_STM32F411RE)
/* N/A on NUCLEO_STM32F411RE + IKS01A1 */
/* N/A on NUCLEO_STM32F411RE + IKS01A2 */
#define CS_SPI2_GPIO_Port   0
#define CS_SPI2_Pin         0
#define CS_SPI1_GPIO_Port   0
#define CS_SPI1_Pin         0

/* Platform STM32F411RE + IKS01A2 Interrupt PIN */
#define LSM6DSOX_INT2_PIN GPIO_PIN_1
#define LSM6DSOX_INT2_GPIO_PORT GPIOC
#define LSM6DSOX_INT1_PIN GPIO_PIN_0
#define LSM6DSOX_INT1_GPIO_PORT GPIOC

#endif /* NUCLEO_XXX */

#define TX_BUF_DIM          1000
#define TIME_OPS			10000

/*
 * Define number of byte for each sensor sample.
 */
#define OUT_XYZ_SIZE		6

typedef union {
	struct {
		uint32_t tick;
		uint16_t unused;
	} reg;
	uint8_t byte[6];
} timestamp_sample_t;

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[TX_BUF_DIM];

/*
 * Local variables used by sample test only.
 */
static lsm6dsox_ctx_t ag_ctx;
static lis2mdl_ctx_t mag_ctx;
static uint8_t slave_address = LSM6DSOX_I2C_ADD_L;

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
  HAL_UART_Transmit( &huart2, tx_buffer, len, 1000 );
  #endif /* NUCLEO_STM32F411RE */

  #ifdef MKI109V2
  CDC_Transmit_FS( tx_buffer, len );
  #endif /* MKI109V2 */
}

/*
 * Function to read external interrupt pin connected to INT sensor.
 */
static int32_t platform_read_int_pin(void)
{
#ifdef NUCLEO_STM32F411RE
#ifndef LSM6DSOX_INT1
    return HAL_GPIO_ReadPin(LSM6DSOX_INT2_GPIO_PORT, LSM6DSOX_INT2_PIN);
#else /* LSM6DSOX_INT1 */
    return HAL_GPIO_ReadPin(LSM6DSOX_INT1_GPIO_PORT, LSM6DSOX_INT1_PIN);
#endif /* LSM6DSOX_INT1 */

#else /* NUCLEO_STM32F411RE */
    return 0;
#endif /* NUCLEO_STM32F411RE */
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
 * Function to report platform error.
 */
static void error_platform(int32_t error)
{
	(void)error;

	while(1);
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
 * Read data byte from internal register of a slave device connected
 * to master I2C interface.
 */
static int32_t lsm6dsox_read_lis2mdl_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t mm_error;
  uint8_t drdy;
  lsm6dsox_status_master_t master_status;
  lsm6dsox_sh_cfg_read_t val = {
		  .slv_add = LIS2MDL_I2C_ADD,
		  .slv_subadd = reg,
		  .slv_len = len,
  };

  (void)ctx;

  /*
   * Disable accelerometer.
   */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_OFF);

  /*
   * Configure Sensor Hub to read LIS2MDL.
   */
  mm_error = lsm6dsox_sh_slv0_cfg_read(&ag_ctx, &val);
  lsm6dsox_sh_slave_connected_set(&ag_ctx, LSM6DSOX_SLV_0);

  /*
   * Enable I2C Master and I2C master.
   */
   lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_ENABLE);

  /*
   * Enable accelerometer to trigger Sensor Hub operation.
   */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_104Hz);

  /*
   * Wait Sensor Hub operation flag set.
   */
  lsm6dsox_acceleration_raw_get(&ag_ctx, data_raw_acceleration.u8bit);
  do
  {
    platform_delay(TIME_OPS);
	lsm6dsox_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while (!drdy);

  do
  {
    platform_delay(TIME_OPS);
    lsm6dsox_sh_status_get(&ag_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /*
   * Disable I2C master and XL(trigger).
   */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_OFF);

  /*
   * Read SensorHub registers.
   */
  lsm6dsox_sh_read_data_raw_get(&ag_ctx, (lsm6dsox_emb_sh_read_t*)&data);

  return mm_error;
}

/*
 * Write data byte to internal register of a slave device connected
 * to master I2C interface.
 */
static int32_t lsm6dsox_write_lis2mdl_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len)
{
  axis3bit16_t data_raw_acceleration;
  int32_t mm_error;
  uint8_t drdy;
  lsm6dsox_status_master_t master_status;
  lsm6dsox_sh_cfg_write_t val = {
		  .slv0_add = LIS2MDL_I2C_ADD,
		  .slv0_subadd = reg,
		  .slv0_data = *data,
  };

  (void)ctx;
  (void)len;

  /*
   * Disable accelerometer.
   */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_OFF);

  /*
   * Configure Sensor Hub to write.
   */
  mm_error = lsm6dsox_sh_cfg_write(&ag_ctx, &val);

  /*
   * Enable I2C Master.
   */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_ENABLE);

  /*
   * Enable accelerometer to trigger Sensor Hub operation.
   */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_104Hz);

  /*
   * Wait Sensor Hub operation flag set.
   */
  lsm6dsox_acceleration_raw_get(&ag_ctx, data_raw_acceleration.u8bit);
  do
  {
    platform_delay(TIME_OPS);
	lsm6dsox_xl_flag_data_ready_get(&ag_ctx, &drdy);
  } while (!drdy);

  do
  {
    platform_delay(TIME_OPS);
    lsm6dsox_sh_status_get(&ag_ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /*
   * Disable I2C master and XL (trigger).
   */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_DISABLE);
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_OFF);

  return mm_error;
}

/*
 * Configure LIS2MDL magnetometer sensor over I2C master line (Slave0)
 *
 * Enable LIS2MDL Magnetometer:
 * set continuous mode
 * set temperature compensation
 * enable BDU
 * set ODR to 20 Hz
 */
static void configure_lis2mdl(lis2mdl_ctx_t* ctx)
{
  lsm6dsox_sh_cfg_read_t val =
  {
		  .slv_add = LIS2MDL_I2C_ADD,
		  .slv_subadd = LIS2MDL_OUTX_L_REG,
		  .slv_len = OUT_XYZ_SIZE,
  };

  lis2mdl_operating_mode_set(ctx, LIS2MDL_CONTINUOUS_MODE);
  lis2mdl_offset_temp_comp_set(ctx, PROPERTY_ENABLE);
  lis2mdl_block_data_update_set(ctx, PROPERTY_ENABLE);
  lis2mdl_data_rate_set(ctx, LIS2MDL_ODR_20Hz);

  /*
   * Prepare sensor hub to read data from external Slave0.
   */
  lsm6dsox_sh_slv0_cfg_read(&ag_ctx, &val);
}

/*
 * example_main - Main Example
 *
 * Configure low level function to access to external device
 * Check if LIS2MDL connected to Sensor Hub
 * Configure lis2mdl in CONTINUOUS_MODE with 20 ODR
 * Configure Sensor Hub to read one slave with XL trigger
 * Set FIFO watermark
 * Set FIFO mode to Stream mode
 * Enable FIFO batching of Slave0 + ACC + Gyro samples
 * Enable batch FIFO of hw timestamp (LSB 25 us)
 * Poll for FIFO watermark interrupt and read samples
 */
void lsm6dsox_hub_fifo_lis2mdl_timestamp_simple(void)
{
  uint8_t whoamI, rst;
  float acceleration_mg[3];
  float angular_rate_mdps[3];
  float magnetic_mG[3];
  axis3bit16_t data_raw_magnetic;
  axis3bit16_t data_raw_acceleration;
  axis3bit16_t data_raw_angular_rate;
  axis3bit16_t dummy;

#ifdef LSM6DSOX_INT1
  lsm6dsox_pin_int1_route_t int1_route;
#else /* LSM6DSOX_INT1 */
  lsm6dsox_pin_int2_route_t int2_route;
#endif /* LSM6DSOX_INT1 */

  ag_ctx.write_reg = platform_write;
  ag_ctx.read_reg = platform_read;
  ag_ctx.handle = &hi2c1;

  /*
   * Init test platform.
   */
  platform_init();

  /*
   * Configure low level function to access to external device.
   */
  mag_ctx.read_reg = lsm6dsox_read_lis2mdl_cx;
  mag_ctx.write_reg = lsm6dsox_write_lis2mdl_cx;
  mag_ctx.handle = &hi2c1;

  /*
   *  Check device ID.
   */
  lsm6dsox_device_id_get(&ag_ctx, &whoamI);
  if (whoamI != LSM6DSOX_ID)
  {
	  /*
	   * Sensor not detected.
	   */
	  error_platform(-1);
  }

  /*
   *  Restore default configuration.
   */
  lsm6dsox_reset_set(&ag_ctx, PROPERTY_ENABLE);
  do {
    lsm6dsox_reset_get(&ag_ctx, &rst);
  } while (rst);

  /*
   * Disable I3C interface.
   */
  lsm6dsox_i3c_disable_set(&ag_ctx, LSM6DSOX_I3C_DISABLE);

  /*
   * Some hardware require to enable pull up on master I2C interface.
   */
  //lsm6dsox_sh_pin_mode_set(&ag_ctx, LSM6DSOX_INTERNAL_PULL_UP);

  /*
   * Check if LIS2MDL connected to Sensor Hub.
   */
  lis2mdl_device_id_get(&mag_ctx, &whoamI);
  if (whoamI != LIS2MDL_ID)
  {
	  /*
	   * Magnetometer not detected.
	   */
	  error_platform(-1);
  }

  /*
   * Configure LIS2MDL on the I2C master line.
   */
  configure_lis2mdl(&mag_ctx);

  /*
   * Configure Sensor Hub to read one slave.
   */
  lsm6dsox_sh_slave_connected_set(&ag_ctx, LSM6DSOX_SLV_0);

  /*
   * Set XL full scale and Gyro full scale.
   */
  lsm6dsox_xl_full_scale_set(&ag_ctx, LSM6DSOX_2g);
  lsm6dsox_gy_full_scale_set(&ag_ctx, LSM6DSOX_2000dps);

  /*
   *  Enable Block Data Update.
   */
  lsm6dsox_block_data_update_set(&ag_ctx, PROPERTY_ENABLE);

  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to 15 samples. 5 * (Acc + Gyro + Mag)
   */
  lsm6dsox_fifo_watermark_set(&ag_ctx, 15);

  /*
   * Set FIFO mode to Stream mode (aka Continuous Mode).
   */
  lsm6dsox_fifo_mode_set(&ag_ctx, LSM6DSOX_STREAM_MODE);

  /*
   * Enable latched interrupt notification.
   */
  lsm6dsox_int_notification_set(&ag_ctx, LSM6DSOX_ALL_INT_LATCHED);

 /*
   * Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed.
   */
  //lsm6dsox_data_ready_mode_set(&ag_ctx, LSM6DSOX_DRDY_PULSED);

#ifdef LSM6DSOX_INT1
  /*
   * FIFO watermark interrupt routed on INT1 pin.
   *
   * Remember that INT1 pin is used by sensor to switch in I3C mode.
   */
  lsm6dsox_pin_int1_route_get(&ag_ctx, &int1_route);
  int1_route.int1_ctrl.int1_fifo_th = PROPERTY_ENABLE;
  lsm6dsox_pin_int1_route_set(&ag_ctx, &int1_route);
#else /* LSM6DSOX_INT1 */
  /*
   * FIFO watermark interrupt routed on INT2 pin.
   */
  lsm6dsox_pin_int2_route_get(&ag_ctx, &int2_route);
  int2_route.int2_ctrl.int2_fifo_th = PROPERTY_ENABLE;
  lsm6dsox_pin_int2_route_set(&ag_ctx, &int2_route);
#endif /* LSM6DSOX_INT1 */

  /*
   * Enable FIFO batching of Slave0.
   * ODR batching is 13 Hz.
   */
  lsm6dsox_sh_batch_slave_0_set(&ag_ctx, PROPERTY_ENABLE);
  lsm6dsox_sh_data_rate_set(&ag_ctx, LSM6DSOX_SH_ODR_13Hz);

  /*
   * Set FIFO batch XL/Gyro ODR to 12.5Hz.
   * Enable Timestamp batching with no decimation.
   */
  lsm6dsox_fifo_xl_batch_set(&ag_ctx, LSM6DSOX_XL_BATCHED_AT_12Hz5);
  lsm6dsox_fifo_gy_batch_set(&ag_ctx, LSM6DSOX_GY_BATCHED_AT_12Hz5);
  lsm6dsox_fifo_timestamp_decimation_set(&ag_ctx, LSM6DSOX_DEC_1);
  lsm6dsox_timestamp_set(&ag_ctx, PROPERTY_ENABLE);

  /*
   * Enable I2C Master.
   */
  lsm6dsox_sh_master_set(&ag_ctx, PROPERTY_ENABLE);

  /*
   * Set Output Data Rate.
   */
  lsm6dsox_xl_data_rate_set(&ag_ctx, LSM6DSOX_XL_ODR_12Hz5);
  lsm6dsox_gy_data_rate_set(&ag_ctx, LSM6DSOX_GY_ODR_12Hz5);

  while(1)
  {
	uint16_t num = 0;
    lsm6dsox_fifo_tag_t reg_tag;
    timestamp_sample_t ts_tick;
    uint32_t timestamp = 0.0f;

    /*
     * Read INT pin value.
     */
    if (platform_read_int_pin())
    {
    	/*
    	 * Read number of samples in FIFO.
    	 */
    	lsm6dsox_fifo_data_level_get(&ag_ctx, &num);

		while(num--)
    	{
			/*
			 * Read FIFO tag.
			 */
			lsm6dsox_fifo_sensor_tag_get(&ag_ctx, &reg_tag);

			switch(reg_tag)
			{
			case LSM6DSOX_XL_NC_TAG:
			  memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
			  lsm6dsox_fifo_out_raw_get(&ag_ctx, data_raw_acceleration.u8bit);

			  acceleration_mg[0] =
				lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
			  acceleration_mg[1] =
				lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
			  acceleration_mg[2] =
				lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);

			  sprintf((char*)tx_buffer,
					  "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f T %u.%u ms\r\n",
				      acceleration_mg[0], acceleration_mg[1],
					  acceleration_mg[2], (unsigned int)(timestamp / 1000ULL),
					  (unsigned int)(timestamp % 1000ULL));
			  tx_com(tx_buffer, strlen((char const*)tx_buffer));
			  break;
			case LSM6DSOX_GYRO_NC_TAG:
			  memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
			  lsm6dsox_fifo_out_raw_get(&ag_ctx, data_raw_angular_rate.u8bit);
			  angular_rate_mdps[0] =
			    lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
			  angular_rate_mdps[1] =
			    lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
			  angular_rate_mdps[2] =
			    lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);

			  sprintf((char*)tx_buffer,
					  "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f T %u.%u ms\r\n",
					  angular_rate_mdps[0], angular_rate_mdps[1],
					  angular_rate_mdps[2], (unsigned int)(timestamp / 1000ULL),
					  (unsigned int)(timestamp % 1000ULL));
			  tx_com(tx_buffer, strlen((char const*)tx_buffer));
			  break;
			case LSM6DSOX_SENSORHUB_SLAVE0_TAG:
			  memset(data_raw_magnetic.u8bit, 0x00, 3 * sizeof(int16_t));
			  lsm6dsox_fifo_out_raw_get(&ag_ctx, data_raw_magnetic.u8bit);
		      magnetic_mG[0] =
		    		  lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[0]);
		      magnetic_mG[1] =
		    		  lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[1]);
		      magnetic_mG[2] =
		    		  lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[2]);

		      sprintf((char*)tx_buffer,
		    		  "Mag [mG]:%4.2f\t%4.2f\t%4.2f T %u.%u ms\r\n",
		    	      magnetic_mG[0], magnetic_mG[1], magnetic_mG[2],
					  (unsigned int)(timestamp / 1000ULL),
					  (unsigned int)(timestamp % 1000ULL));
		      tx_com(tx_buffer, strlen((char const*)tx_buffer));
		      break;
			case LSM6DSOX_TIMESTAMP_TAG:
			  lsm6dsox_fifo_out_raw_get(&ag_ctx, ts_tick.byte);
			  timestamp = (unsigned int)lsm6dsox_from_lsb_to_nsec(ts_tick.reg.tick);
			  break;
			default:
			  /*
			   * Flush unused samples.
			   */
			  memset(dummy.u8bit, 0x00, 3 * sizeof(int16_t));
			  lsm6dsox_fifo_out_raw_get(&ag_ctx, dummy.u8bit);
			  break;
        }
      }
    }
  }
}

