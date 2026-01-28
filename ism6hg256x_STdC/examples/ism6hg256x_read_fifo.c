/*
 ******************************************************************************
 * @file    read_fifo.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to get data from sensor.
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
 * - STEVAL_MKI109V3 +
 * - NUCLEO_F401RE +
 * - DISCOVERY_SPC584B +
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
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
//#define NUCLEO_F401RE    /* little endian */
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

#elif defined(NUCLEO_F401RE)
/* NUCLEO_F401RE: Define communication interface */
#define SENSOR_BUS hi2c1

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS I2CD1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "ism6hg256x_reg.h"

#if defined(NUCLEO_F401RE)
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

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms
#define    FIFO_WATERMARK       128
#define    CNT_FOR_OUTPUT       100

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static int16_t *datax;
static int16_t *datay;
static int16_t *dataz;
static int32_t *ts;
static ism6hg256x_filt_settling_mask_t filt_settling_mask;

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

static   stmdev_ctx_t dev_ctx;
static double_t lowg_xl_sum[3], hg_xl_sum[3], gyro_sum[3];
static uint16_t lowg_xl_cnt = 0, hg_xl_cnt = 0, gyro_cnt = 0;

static uint16_t gravity_cnt = 0, gbias_cnt = 0, rot_cnt = 0;
static float_t gravity_sum[3], gbias_sum[3];
static double_t rot_sum[4];

static   uint8_t fifo_thread_run = 0;

static float_t npy_half_to_float(uint16_t h)
{
    union { float_t ret; uint32_t retbits; } conv;
    conv.retbits = ism6hg256x_from_f16_to_f32(h);
    return conv.ret;
}


static void ism6hg256x_fifo_thread(void)
{
  float_t acceleration_mg[3];
  float_t angular_rate_mdps[3];
  float_t gravity_mg[3], gbias_mdps[3], quat[4];
  uint8_t *axis;

  if (fifo_thread_run) {
    ism6hg256x_fifo_status_t fifo_status;
    uint16_t num;

    fifo_thread_run = 0;

    /* Read watermark flag */
    ism6hg256x_fifo_status_get(&dev_ctx, &fifo_status);

    num = fifo_status.fifo_level;

    while (num--) {
      ism6hg256x_fifo_out_raw_t f_data;

      /* Read FIFO sensor value */
      ism6hg256x_fifo_out_raw_get(&dev_ctx, &f_data);
      datax = (int16_t *)&f_data.data[0];
      datay = (int16_t *)&f_data.data[2];
      dataz = (int16_t *)&f_data.data[4];
      ts = (int32_t *)&f_data.data[0];

      switch (f_data.tag) {
      case ISM6HG256X_XL_NC_TAG:
        /* Read acceleration field data */
        acceleration_mg[0] = ism6hg256x_from_fs2_to_mg(*datax);
        acceleration_mg[1] = ism6hg256x_from_fs2_to_mg(*datay);
        acceleration_mg[2] = ism6hg256x_from_fs2_to_mg(*dataz);

        lowg_xl_sum[0] += acceleration_mg[0];
        lowg_xl_sum[1] += acceleration_mg[1];
        lowg_xl_sum[2] += acceleration_mg[2];
        lowg_xl_cnt++;
        break;

      case ISM6HG256X_XL_HG_TAG:
        acceleration_mg[0] = ism6hg256x_from_fs256_to_mg(*datax);
        acceleration_mg[1] = ism6hg256x_from_fs256_to_mg(*datay);
        acceleration_mg[2] = ism6hg256x_from_fs256_to_mg(*dataz);

        hg_xl_sum[0] += acceleration_mg[0];
        hg_xl_sum[1] += acceleration_mg[1];
        hg_xl_sum[2] += acceleration_mg[2];
        hg_xl_cnt++;
        break;

      case ISM6HG256X_GY_NC_TAG:
        angular_rate_mdps[0] = ism6hg256x_from_fs2000_to_mdps(*datax);
        angular_rate_mdps[1] = ism6hg256x_from_fs2000_to_mdps(*datay);
        angular_rate_mdps[2] = ism6hg256x_from_fs2000_to_mdps(*dataz);

        gyro_sum[0] += angular_rate_mdps[0];
        gyro_sum[1] += angular_rate_mdps[1];
        gyro_sum[2] += angular_rate_mdps[2];
        gyro_cnt++;
        break;

     case ISM6HG256X_SFLP_GYROSCOPE_BIAS_TAG:
        axis = &f_data.data[0];
        gbias_mdps[0] = ism6hg256x_from_fs125_to_mdps(axis[0] | (axis[1] << 8));
        gbias_mdps[1] = ism6hg256x_from_fs125_to_mdps(axis[2] | (axis[3] << 8));
        gbias_mdps[2] = ism6hg256x_from_fs125_to_mdps(axis[4] | (axis[5] << 8));

        gbias_sum[0] += gbias_mdps[0];
        gbias_sum[1] += gbias_mdps[1];
        gbias_sum[2] += gbias_mdps[2];
        gbias_cnt++;
        break;

      case ISM6HG256X_SFLP_GRAVITY_VECTOR_TAG:
        axis = &f_data.data[0];
        gravity_mg[0] = ism6hg256x_from_sflp_to_mg(axis[0] | (axis[1] << 8));
        gravity_mg[1] = ism6hg256x_from_sflp_to_mg(axis[2] | (axis[3] << 8));
        gravity_mg[2] = ism6hg256x_from_sflp_to_mg(axis[4] | (axis[5] << 8));

        gravity_sum[0] += gravity_mg[0];
        gravity_sum[1] += gravity_mg[1];
        gravity_sum[2] += gravity_mg[2];
        gravity_cnt++;
        break;

      case ISM6HG256X_SFLP_GAME_ROTATION_VECTOR_TAG:
      {
        uint16_t *sflp = (uint16_t *)&f_data.data[2];

        if (f_data.data[0] == 0x00) {
          /* Game Rotation first word */
          quat[0] = npy_half_to_float(sflp[0]);
          quat[1] = npy_half_to_float(sflp[1]);
        } else if (f_data.data[0] == 0x01) {
          /* Game Rotation second word */
          quat[2] = npy_half_to_float(sflp[0]);
          quat[3] = npy_half_to_float(sflp[1]);

          rot_sum[0] += quat[0];
          rot_sum[1] += quat[1];
          rot_sum[2] += quat[2];
          rot_sum[3] += quat[3];

          rot_cnt++;
        } else {
          /* error */
          snprintf((char *)tx_buffer, sizeof(tx_buffer), "[%02x - %02x] wrong word \r\n", f_data.data[0], f_data.data[1]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
       }

        break;
      }


      case ISM6HG256X_TIMESTAMP_TAG:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "TIMESTAMP [ms] %d\r\n", *ts);
        tx_com(tx_buffer, strlen((char const *)tx_buffer));

        /* print avg low-g xl data */
        if (lowg_xl_cnt > 0) {
          acceleration_mg[0] = lowg_xl_sum[0] / lowg_xl_cnt;
          acceleration_mg[1] = lowg_xl_sum[1] / lowg_xl_cnt;
          acceleration_mg[2] = lowg_xl_sum[2] / lowg_xl_cnt;

          snprintf((char *)tx_buffer, sizeof(tx_buffer), "lg xl (avg of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                  lowg_xl_cnt, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          lowg_xl_sum[0] = lowg_xl_sum[1] = lowg_xl_sum[2] = 0.0;
          lowg_xl_cnt = 0;
        }

        /* print avg high-g xl data */
        if (hg_xl_cnt > 0) {
          acceleration_mg[0] = hg_xl_sum[0] / hg_xl_cnt;
          acceleration_mg[1] = hg_xl_sum[1] / hg_xl_cnt;
          acceleration_mg[2] = hg_xl_sum[2] / hg_xl_cnt;

          snprintf((char *)tx_buffer, sizeof(tx_buffer), "hg xl (avg of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                  hg_xl_cnt, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          hg_xl_sum[0] = hg_xl_sum[1] = hg_xl_sum[2] = 0.0;
          hg_xl_cnt = 0;
        }

        /* print avg gyro data */
        if (gyro_cnt > 0) {
          angular_rate_mdps[0] = gyro_sum[0] / gyro_cnt;
          angular_rate_mdps[1] = gyro_sum[1] / gyro_cnt;
          angular_rate_mdps[2] = gyro_sum[2] / gyro_cnt;

          snprintf((char *)tx_buffer, sizeof(tx_buffer), "gyro (avg of %d samples) [mdps]:%4.2f\t%4.2f\t%4.2f\r\n\r\n",
                  gyro_cnt, angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0;
          gyro_cnt = 0;
        }

         /* print SFLP gbias data */
        if (gbias_cnt > 0) {
          gbias_mdps[0] = gbias_sum[0] / gbias_cnt;
          gbias_mdps[1] = gbias_sum[1] / gbias_cnt;
          gbias_mdps[2] = gbias_sum[2] / gbias_cnt;

          snprintf((char *)tx_buffer, sizeof(tx_buffer), "SFLP gbias (avg of %d samples) [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                  gbias_cnt, gbias_mdps[0], gbias_mdps[1], gbias_mdps[2]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          gbias_sum[0] = gbias_sum[1] = gbias_sum[2] = 0.0;
          gbias_cnt = 0;
        }

         /* print SFLP gravity data */
        if (gravity_cnt > 0) {
          gravity_mg[0] = gravity_sum[0] / gravity_cnt;
          gravity_mg[1] = gravity_sum[1] / gravity_cnt;
          gravity_mg[2] = gravity_sum[2] / gravity_cnt;

          snprintf((char *)tx_buffer, sizeof(tx_buffer), "SFLP gravity (avg of %d samples) [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                  gravity_cnt, gravity_mg[0], gravity_mg[1], gravity_mg[2]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          gravity_sum[0] = gravity_sum[1] = gravity_sum[2] = 0.0;
          gravity_cnt = 0;
        }

         /* print SFLP game rotation data */
        if (rot_cnt > 0) {
          quat[0] = rot_sum[0] / rot_cnt;
          quat[1] = rot_sum[1] / rot_cnt;
          quat[2] = rot_sum[2] / rot_cnt;
          quat[3] = rot_sum[3] / rot_cnt;

          snprintf((char *)tx_buffer, sizeof(tx_buffer), "SFLP rotation (avg of %d samples) [quaternions]:X: %2.3f\tY: %2.3f\tZ: %2.3f\tW: %2.3f\r\n\r\n",
                  rot_cnt, quat[0], quat[1], quat[2], quat[3]);
          tx_com(tx_buffer, strlen((char const *)tx_buffer));
          rot_sum[0] = rot_sum[1] = rot_sum[2] = rot_sum[3] = 0.0;
          rot_cnt = 0;
        }

        break;

      default:
        snprintf((char *)tx_buffer, sizeof(tx_buffer), "[%02x] UNHANDLED TAG \r\n", f_data.tag);
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        break;
      }
    }
  }
}

void ism6hg256x_read_fifo_handler(void)
{
  fifo_thread_run = 1;
}

/* Main Example --------------------------------------------------------------*/
void ism6hg256x_read_fifo(void)
{
  ism6hg256x_pin_int1_route_t pin_int = { 0 };

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Init test platform */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  ism6hg256x_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != ISM6HG256X_ID)
    while (1);

  /* Perform device power-on-reset */
  ism6hg256x_sw_por(&dev_ctx);

  /* Enable Block Data Update */
  ism6hg256x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to FIFO_WATERMARK samples
   */
  ism6hg256x_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);

  /* Set FIFO batch XL/Gyro ODR */
  ism6hg256x_fifo_xl_batch_set(&dev_ctx, ISM6HG256X_XL_BATCHED_AT_60Hz);
  ism6hg256x_fifo_hg_xl_batch_set(&dev_ctx, 1);
  ism6hg256x_fifo_gy_batch_set(&dev_ctx, ISM6HG256X_GY_BATCHED_AT_120Hz);

  /* Set FIFO batch SFLP */
  ism6hg256x_sflp_data_rate_set(&dev_ctx, ISM6HG256X_SFLP_120Hz);

  ism6hg256x_fifo_sflp_raw_t sflp =
        {
          .game_rotation = 1,
          .gravity = 1,
          .gbias = 1,
        };
  ism6hg256x_fifo_sflp_batch_set(&dev_ctx, sflp);

  ism6hg256x_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  ism6hg256x_fifo_mode_set(&dev_ctx, ISM6HG256X_STREAM_MODE);
  ism6hg256x_fifo_timestamp_batch_set(&dev_ctx, ISM6HG256X_TMSTMP_DEC_32);
  ism6hg256x_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  ism6hg256x_xl_setup(&dev_ctx, ISM6HG256X_ODR_AT_60Hz, ISM6HG256X_XL_HIGH_PERFORMANCE_MD);
  ism6hg256x_hg_xl_data_rate_set(&dev_ctx, ISM6HG256X_HG_XL_ODR_AT_480Hz, 0);
  ism6hg256x_gy_setup(&dev_ctx, ISM6HG256X_ODR_AT_120Hz, ISM6HG256X_GY_HIGH_PERFORMANCE_MD);

  /* Set full scale */
  ism6hg256x_xl_full_scale_set(&dev_ctx, ISM6HG256X_2g);
  ism6hg256x_hg_xl_full_scale_set(&dev_ctx, ISM6HG256X_256g);
  ism6hg256x_gy_full_scale_set(&dev_ctx, ISM6HG256X_2000dps);

  /* Configure filtering chain */
  filt_settling_mask.drdy = PROPERTY_ENABLE;
  filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  filt_settling_mask.irq_g = PROPERTY_ENABLE;
  ism6hg256x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
  ism6hg256x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
  ism6hg256x_filt_gy_lp1_bandwidth_set(&dev_ctx, ISM6HG256X_GY_ULTRA_LIGHT);
  ism6hg256x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
  ism6hg256x_filt_xl_lp2_bandwidth_set(&dev_ctx, ISM6HG256X_XL_STRONG);

  /* enable fifo_th on High-G XL (sensor at highest frequency) */
  pin_int.fifo_th = PROPERTY_ENABLE;
  ism6hg256x_pin_int1_route_set(&dev_ctx, &pin_int);
  //ism6hg256x_pin_int2_route_set(&dev_ctx, &pin_int);

  /* Read samples in polling mode (no int) */
  while (1) {
    ism6hg256x_fifo_thread();
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
#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Write(handle, ISM6HG256X_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  ISM6HG256X_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Read(handle, ISM6HG256X_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, ISM6HG256X_I2C_ADD_L & 0xFE, reg, bufp, len);
#endif
  return 0;
}

/*
 * @brief  platform specific outputs on terminal (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
#if defined(NUCLEO_F401RE)
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
#if defined(NUCLEO_F401RE) | defined(STEVAL_MKI109V3)
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
