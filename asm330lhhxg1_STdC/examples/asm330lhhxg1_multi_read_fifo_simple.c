/*
 ******************************************************************************
 * @file    asm330lhhxg1_multi_read_fifo_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor FIFO.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI195V1
 * - NUCLEO_F401RE + STEVAL-MKI195V1
 * - DISCOVERY_SPC584B + STEVAL-MKI195V1
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
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
#include <asm330lhhxg1_reg.h>

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

#elif defined(SPC584B_DIS)
#include "components.h"
#endif

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

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

/* Main Example --------------------------------------------------------------*/
void asm330lhhxg1_read_fifo_simple(void)
{
  stmdev_ctx_t dev_ctx;

  /* Uncomment to configure INT 1 */
  //asm330lhhxg1_pin_int1_route_t int1_route;

  /* Uncomment to configure INT 2 */
  //asm330lhhxg1_pin_int2_route_t int2_route;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &hi2c1;

  /* Init test platform */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  asm330lhhxg1_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != ASM330LHHXG1_ID)
    while(1);

  /* Restore default configuration */
  asm330lhhxg1_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    asm330lhhxg1_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  asm330lhhxg1_i3c_disable_set(&dev_ctx, ASM330LHHXG1_I3C_DISABLE);

  /* Enable Block Data Update */
  asm330lhhxg1_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set full scale */
  asm330lhhxg1_xl_full_scale_set(&dev_ctx, ASM330LHHXG1_2g);
  asm330lhhxg1_gy_full_scale_set(&dev_ctx, ASM330LHHXG1_2000dps);

  /* Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to 10 samples
   */
  asm330lhhxg1_fifo_watermark_set(&dev_ctx, 10);

  /* Set FIFO batch XL/Gyro ODR to 12.5Hz */
  asm330lhhxg1_fifo_xl_batch_set(&dev_ctx, ASM330LHHXG1_XL_BATCHED_AT_12Hz5);
  asm330lhhxg1_fifo_gy_batch_set(&dev_ctx, ASM330LHHXG1_GY_BATCHED_AT_12Hz5);

  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  asm330lhhxg1_fifo_mode_set(&dev_ctx, ASM330LHHXG1_STREAM_MODE);

  /* Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed */
  //asm330lhhxg1_data_ready_mode_set(&dev_ctx, ASM330LHHXG1_DRDY_PULSED);

  /* Uncomment if interrupt generation on Free Fall INT1 pin */
  //asm330lhhxg1_pin_int1_route_get(&dev_ctx, &int1_route);
  //int1_route.reg.int1_ctrl.int1_fifo_th = PROPERTY_ENABLE;
  //asm330lhhxg1_pin_int1_route_set(&dev_ctx, &int1_route);

  /* Uncomment if interrupt generation on Free Fall INT2 pin */
  //asm330lhhxg1_pin_int2_route_get(&dev_ctx, &int2_route);
  //int2_route.reg.int2_ctrl.int2_fifo_th = PROPERTY_ENABLE;
  //asm330lhhxg1_pin_int2_route_set(&dev_ctx, &int2_route);

  /* Set Output Data Rate */
  asm330lhhxg1_xl_data_rate_set(&dev_ctx, ASM330LHHXG1_XL_ODR_12Hz5);
  asm330lhhxg1_gy_data_rate_set(&dev_ctx, ASM330LHHXG1_GY_ODR_12Hz5);

  /* Wait samples. */
  while(1)
  {

    asm330lhhxg1_fifo_tag_t reg_tag;
    uint8_t wmflag = 0;
    uint16_t num = 0;
    int16_t dummy[3];

    /* Read watermark flag */
    asm330lhhxg1_fifo_wtm_flag_get(&dev_ctx, &wmflag);
    if (wmflag > 0)
    {
      /* Read number of samples in FIFO */
      asm330lhhxg1_fifo_data_level_get(&dev_ctx, &num);
      while(num--)
      {
        /* Read FIFO tag */
        asm330lhhxg1_fifo_sensor_tag_get(&dev_ctx, &reg_tag);
        switch(reg_tag)
        {
          case ASM330LHHXG1_XL_NC_TAG:
            memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
            asm330lhhxg1_fifo_out_raw_get(&dev_ctx, (uint8_t *)data_raw_acceleration);
            acceleration_mg[0] =
              asm330lhhxg1_from_fs2g_to_mg(data_raw_acceleration[0]);
            acceleration_mg[1] =
              asm330lhhxg1_from_fs2g_to_mg(data_raw_acceleration[1]);
            acceleration_mg[2] =
              asm330lhhxg1_from_fs2g_to_mg(data_raw_acceleration[2]);

            sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                    acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
            tx_com(tx_buffer, strlen((char const*)tx_buffer));
            break;
          case ASM330LHHXG1_GYRO_NC_TAG:
            memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
            asm330lhhxg1_fifo_out_raw_get(&dev_ctx, (uint8_t *)data_raw_angular_rate);
            angular_rate_mdps[0] =
              asm330lhhxg1_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
            angular_rate_mdps[1] =
              asm330lhhxg1_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
            angular_rate_mdps[2] =
              asm330lhhxg1_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);

            sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                    angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
            tx_com(tx_buffer, strlen((char const*)tx_buffer));
            break;
          default:
            /* Flush unused samples */
            memset(dummy, 0x00, 3 * sizeof(int16_t));
            asm330lhhxg1_fifo_out_raw_get(&dev_ctx, (uint8_t *)dummy);
            break;
        }
      }
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
#if defined(NUCLEO_F401RE)
    HAL_I2C_Mem_Write(handle, ASM330LHHXG1_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  ASM330LHHXG1_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, ASM330LHHXG1_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
    reg |= 0x80;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, ASM330LHHXG1_I2C_ADD_L & 0xFE, reg, bufp, len);
#endif
  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
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
