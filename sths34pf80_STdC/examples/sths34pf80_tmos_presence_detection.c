/*
 ******************************************************************************
 * @file    tmos_presence_detection.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
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
 * - STEVAL_MKI109V3
 * - NUCLEO_F411RE
 * - DISCOVERY_SPC584B
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(N/A)
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
/* MKI109V3: Vdd and Vddio power supply values at 1V8 */
#define PWM_1V8 500  /* ((1.8 / 3.6) * htim3.Init.Period) */

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
#include "sths34pf80_reg.h"

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

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME         10 //ms

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[1000];
//static sths34pf80_data_t data;

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

static stmdev_ctx_t dev_ctx;
static int wakeup_thread = 0;

void sths34pf80_tmos_presence_detection_handler(void)
{
  wakeup_thread = 1;
}

/* Main Example --------------------------------------------------------------*/
void sths34pf80_tmos_presence_detection(void)
{
  uint8_t whoami;
  sths34pf80_lpf_bandwidth_t lpf_m, lpf_p, lpf_p_m, lpf_a_t;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Initialize platform specific hardware */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  sths34pf80_device_id_get(&dev_ctx, &whoami);
  if (whoami != STHS34PF80_ID)
    while(1);

  /* Set averages (AVG_TAMB = 8, AVG_TMOS = 32) */
  sths34pf80_avg_tobject_num_set(&dev_ctx, STHS34PF80_AVG_TMOS_32);
  sths34pf80_avg_tambient_num_set(&dev_ctx, STHS34PF80_AVG_T_8);

  /* read filters */
  sths34pf80_lpf_m_bandwidth_get(&dev_ctx, &lpf_m);
  sths34pf80_lpf_p_bandwidth_get(&dev_ctx, &lpf_p);
  sths34pf80_lpf_p_m_bandwidth_get(&dev_ctx, &lpf_p_m);
  sths34pf80_lpf_a_t_bandwidth_get(&dev_ctx, &lpf_a_t);

  sprintf((char *)tx_buffer,
          "lpf_m: %02d, lpf_p: %02d, lpf_p_m: %02d, lpf_a_t: %02d\r\n", lpf_m, lpf_p, lpf_p_m, lpf_a_t);
  tx_com(tx_buffer, strlen((char const *)tx_buffer));

  /* Set BDU */
  sths34pf80_block_data_update_set(&dev_ctx, 1);

  sths34pf80_presence_threshold_set(&dev_ctx, 200);
  sths34pf80_presence_hysteresis_set(&dev_ctx, 20);
  sths34pf80_motion_threshold_set(&dev_ctx, 300);
  sths34pf80_motion_hysteresis_set(&dev_ctx, 30);

  sths34pf80_algo_reset(&dev_ctx);

  /* Set interrupt */
  sths34pf80_tmos_int_or_set(&dev_ctx, STHS34PF80_TMOS_INT_PRESENCE);
  sths34pf80_tmos_route_int_set(&dev_ctx, STHS34PF80_TMOS_INT_OR);

  /* Set ODR */
  sths34pf80_tmos_odr_set(&dev_ctx, STHS34PF80_TMOS_ODR_AT_30Hz);

  /* Presence event detected in irq handler */
  while(1) {
    sths34pf80_tmos_func_status_t func_status;
    uint8_t motion;
    uint8_t presence;

    /* handle event in a "thread" alike code */
    if (wakeup_thread) {
      wakeup_thread = 0;
      motion = 0;
      presence = 0;

      do {
        sths34pf80_tmos_func_status_get(&dev_ctx, &func_status);

        if (func_status.pres_flag != presence)
        {
          presence = func_status.pres_flag;

          if (presence) {
            sprintf((char *)tx_buffer, "Start of Presence\r\n");
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
          } else {
            sprintf((char *)tx_buffer, "End of Presence\r\n");
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
          }
        }

        if (func_status.mot_flag != motion)
        {
          motion = func_status.mot_flag;

          if (motion) {
            sprintf((char *)tx_buffer, "Motion Detected!\r\n");
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
          }
        }
      } while (func_status.pres_flag);
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
  HAL_I2C_Mem_Write(handle, STHS34PF80_I2C_ADD, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t *)bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  STHS34PF80_I2C_ADD & 0xFE, reg, (uint8_t *)bufp, len);
#endif
  return 0;
}

#if defined(STEVAL_MKI109V3)
static void SPI_3W_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val)
{
  __disable_irq();

  __HAL_SPI_ENABLE(xSpiHandle);
  __asm("dsb\n");
  __asm("dsb\n");
  __HAL_SPI_DISABLE(xSpiHandle);

  __enable_irq();

  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

static void SPI_3W_Receive(uint8_t *pBuffer, uint16_t nBytesToRead)
{
  __HAL_SPI_DISABLE(&hspi2);
  SPI_1LINE_RX(&hspi2);

  for(uint16_t i = 0; i < nBytesToRead; i++) {
    SPI_3W_Read(&hspi2, pBuffer++);
  }

  SPI_1LINE_TX(&hspi2);
  __HAL_SPI_ENABLE(&hspi2);
}
#endif

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
  HAL_I2C_Mem_Read(handle, STHS34PF80_I2C_ADD, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  //HAL_SPI_Receive(handle, bufp, len, 1000);
  SPI_3W_Receive(bufp, len);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, STHS34PF80_I2C_ADD & 0xFE, reg, bufp, len);
#endif
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
  TIM3->CCR1 = PWM_1V8;
  TIM3->CCR2 = PWM_1V8;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}
