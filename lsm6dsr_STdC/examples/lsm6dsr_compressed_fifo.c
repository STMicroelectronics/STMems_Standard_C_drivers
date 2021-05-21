/*
 ******************************************************************************
 * @file    compressed_fifo.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to configure compressed FIFO and
 *          to retrieve acc and gyro data. This sample use a fifo
 *          utility library tool for FIFO decompression.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI194V1
 * - NUCLEO_F411RE + X_STEVAL-MKI194V1
 * - DISCOVERY_SPC584B + STEVAL-MKI194V1
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
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

//#define STEVAL_MKI109V3  /* little endian */
//#define NUCLEO_F411RE    /* little endian */
//#define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2
/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F411RE)
/* NUCLEO_F411RE: Define communication interface */
#define SENSOR_BUS hi2c1

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS I2CD1

#endif


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lsm6dsr_reg.h"
#include "fifo_utility.h"

#if defined(NUCLEO_F411RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#elif defined(STEVAL_MKI109V3)
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"

#elif defined(SPC584B_DIS)
#include "components.h"
#endif

#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <lsm6dsr_reg.h>

#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
#include "usart.h"
#endif

/* Private macro -------------------------------------------------------------*/
/*
 * Select FIFO samples watermark, max value is 512
 * in FIFO are stored acc, gyro and timestamp samples
 */
#define FIFO_WATERMARK    10
#define FIFO_COMPRESSION  3
#define SLOT_NUMBER      (FIFO_WATERMARK * FIFO_COMPRESSION)

#define    BOOT_TIME            10 //ms

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
static st_fifo_raw_slot raw_slot[SLOT_NUMBER];
static st_fifo_out_slot out_slot[SLOT_NUMBER];
static st_fifo_out_slot acc_slot[SLOT_NUMBER];
static st_fifo_out_slot gyr_slot[SLOT_NUMBER];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

sensor_data_t sensor_data;

/* Main Example --------------------------------------------------------------*/
void lsm6dsr_compressed_fifo(void)
{
  stmdev_ctx_t dev_ctx;
  uint16_t out_slot_size;
  /* Uncomment to configure INT 1 */
  //lsm6dsr_pin_int1_route_t int1_route;
  /* Uncomment to configure INT 2 */
  //lsm6dsr_pin_int2_route_t int2_route;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Init utility for FIFO decompression */
  st_fifo_init(0, 0, 0);
  /* Check device ID */
  lsm6dsr_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSR_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsr_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsr_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dsr_i3c_disable_set(&dev_ctx, LSM6DSR_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dsr_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lsm6dsr_xl_full_scale_set(&dev_ctx, LSM6DSR_2g);
  lsm6dsr_gy_full_scale_set(&dev_ctx, LSM6DSR_2000dps);
  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to FIFO_WATERMARK samples
   */
  lsm6dsr_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);
  /* Set FIFO batch XL/Gyro ODR to 12.5Hz */
  lsm6dsr_fifo_xl_batch_set(&dev_ctx, LSM6DSR_XL_BATCHED_AT_12Hz5);
  lsm6dsr_fifo_gy_batch_set(&dev_ctx, LSM6DSR_GY_BATCHED_AT_12Hz5);
  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dsr_fifo_mode_set(&dev_ctx, LSM6DSR_STREAM_MODE);
  /* Enable FIFO compression on all samples */
  lsm6dsr_compression_algo_set(&dev_ctx, LSM6DSR_CMP_DISABLE);
  /* Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed */
  //lsm6dsr_data_ready_mode_set(&dev_ctx, LSM6DSR_DRDY_PULSED);
  /*
   * FIFO watermark interrupt routed on INT1 pin
   * WARNING: INT1 pin is used by sensor to switch in I3C mode.
   */
  //lsm6dsr_pin_int1_route_get(&dev_ctx, &int1_route);
  //int1_route.reg.int1_ctrl.int1_fifo_th = PROPERTY_ENABLE;
  //lsm6dsr_pin_int1_route_set(&dev_ctx, &int1_route);
  /* FIFO watermark interrupt routed on INT2 pin */
  //lsm6dsr_pin_int2_route_get(&dev_ctx, &int2_route);
  //int2_route.reg.int2_ctrl.int2_fifo_th = PROPERTY_ENABLE;
  //lsm6dsr_pin_int2_route_set(&dev_ctx, &int2_route);
  /* Set Output Data Rate */
  lsm6dsr_xl_data_rate_set(&dev_ctx, LSM6DSR_XL_ODR_12Hz5);
  lsm6dsr_gy_data_rate_set(&dev_ctx, LSM6DSR_GY_ODR_12Hz5);
  lsm6dsr_fifo_timestamp_decimation_set(&dev_ctx, LSM6DSR_DEC_1);
  lsm6dsr_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

  /* Wait samples */
  while (1) {
    uint16_t num = 0;
    uint8_t wmflag = 0;
    uint16_t slots = 0;
    uint16_t acc_samples;
    uint16_t gyr_samples;
    /* Read watermark flag */
    lsm6dsr_fifo_wtm_flag_get(&dev_ctx, &wmflag);

    if (wmflag > 0) {
      /* Read number of samples in FIFO */
      lsm6dsr_fifo_data_level_get(&dev_ctx, &num);

      while (num--) {
        /*
         * Read FIFO sensor tag
         *
         * To reorder data samples in FIFO is needed the register
         * LSM6DSR_FIFO_DATA_OUT_TAG, including tag counter and parity.
         */
        lsm6dsr_read_reg(&dev_ctx, LSM6DSR_FIFO_DATA_OUT_TAG,
                         (uint8_t *)&raw_slot[slots].fifo_data_out[0], 1);
        /* Read FIFO sensor value */
        lsm6dsr_fifo_out_raw_get(&dev_ctx, &raw_slot[slots].fifo_data_out[1]);
        slots++;
      }

      /* Uncompress FIFO samples and filter based on sensor type */
      st_fifo_decompress(out_slot, raw_slot, &out_slot_size, slots);
      st_fifo_sort(out_slot, out_slot_size);
      acc_samples = st_fifo_get_sensor_occurrence(out_slot,
                                                  out_slot_size,
                                                  ST_FIFO_ACCELEROMETER);
      gyr_samples = st_fifo_get_sensor_occurrence(out_slot,
                                                  out_slot_size,
                                                  ST_FIFO_GYROSCOPE);
      /* Count how many acc and gyro samples */
      st_fifo_extract_sensor(acc_slot, out_slot, out_slot_size,
                             ST_FIFO_ACCELEROMETER);
      st_fifo_extract_sensor(gyr_slot, out_slot, out_slot_size,
                             ST_FIFO_GYROSCOPE);

      for (int i = 0; i < acc_samples; i++) {
        memcpy( sensor_data.raw_data, acc_slot[i].raw_data,
                sizeof(sensor_data) );
        sprintf((char *)tx_buffer, "ACC:\t%u\t%d\t%4.2f\t%4.2f\t%4.2f\r\n",
                (unsigned int)acc_slot[i].timestamp,
                acc_slot[i].sensor_tag,
                lsm6dsr_from_fs2g_to_mg(sensor_data.data[0]),
                lsm6dsr_from_fs2g_to_mg(sensor_data.data[1]),
                lsm6dsr_from_fs2g_to_mg(sensor_data.data[2]));
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
      }

      for (int i = 0; i < gyr_samples; i++) {
        memcpy( sensor_data.raw_data, gyr_slot[i].raw_data,
                sizeof(sensor_data) );
        sprintf((char *)tx_buffer, "GYR:\t%u\t%d\t%4.2f\t%4.2f\t%4.2f\r\n",
                (unsigned int)gyr_slot[i].timestamp,
                gyr_slot[i].sensor_tag,
                lsm6dsr_from_fs2000dps_to_mdps(sensor_data.data[0]),
                lsm6dsr_from_fs2000dps_to_mdps(sensor_data.data[1]),
                lsm6dsr_from_fs2000dps_to_mdps(sensor_data.data[2]));
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
      }

      slots = 0;
    }
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
#if defined(NUCLEO_F411RE)
  HAL_I2C_Mem_Write(handle, LSM6DSR_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSR_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
#endif
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
#if defined(NUCLEO_F411RE)
  HAL_I2C_Mem_Read(handle, LSM6DSR_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LSM6DSR_I2C_ADD_L & 0xFE, reg, bufp, len);
#endif
  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
#if defined(NUCLEO_F411RE)
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
#elif defined(STEVAL_MKI109V3)
  CDC_Transmit_FS(tx_buffer, len);
#elif defined(SPC584B_DIS)
  sd_lld_write(&SD2, tx_buffer, len);
#endif
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
#if defined(NUCLEO_F411RE) | defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#if defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}
