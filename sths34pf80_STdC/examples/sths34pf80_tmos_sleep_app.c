/*
 ******************************************************************************
 * @file    sths34pf80_tmos_sleep_app.c
 * @author  AME MEMS Applications Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
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
 * - NUCLEO_F401RE
 * - DISCOVERY_SPC584B
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
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
//#define NUCLEO_F401RE    /* little endian */
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
#include "sths34pf80_reg.h"

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
#define    BOOT_TIME         10 //ms

// SW timer value to assess end of movement [s]
#define    SW_PRES_TIMER    15

#define MOT_THS_ALGO_MOT   100    // Embedded motion threshold of the motion-based algorithm [LSB]
#define MOT_THS_ALGO_PRES  200    // Embedded motion threshold of the presence-based algorithm [LSB]
#define HYST_COEFF         0.2    // Embedded common hysteresis coefficient [-]

/* Private types ---------------------------------------------------------*/
// Define the State type
typedef enum
{
  STATE_0,
  STATE_1
} State;


/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[1000];
static stmdev_ctx_t dev_ctx;
static int wakeup_thread = 0;

static State current_state = STATE_0;
static volatile uint8_t counter = 0;

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
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

static void sths34pf80_tmos_sleep_app_state_0_run(void);
static void sths34pf80_tmos_sleep_app_state_1_set(void);

/* Timer handler  --------------------------------------------------------*/
void sths34pf80_tmos_sleep_app_handler_timer(void)
{
  switch (current_state)
  {
    case STATE_0:
      counter++;
      break;
    default:
      break;
  }
}

/* Interrupt handler  --------------------------------------------------------*/
void sths34pf80_tmos_sleep_app_handler_interrupt(void)
{
  switch (current_state)
  {
    case STATE_1:
      wakeup_thread = 1;
      break;

    default:
      break;
  }
}

/* Main Example --------------------------------------------------------------*/
void sths34pf80_tmos_sleep_app(void)
{
  uint8_t whoami;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Initialize platform specific hardware */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  sths34pf80_device_id_get(&dev_ctx, &whoami);
  if (whoami != STHS34PF80_ID)
    while (1);

  /* Set averages (AVG_TAMB = 8, AVG_TMOS = 32) */
  sths34pf80_avg_tobject_num_set(&dev_ctx, STHS34PF80_AVG_TMOS_32);
  sths34pf80_avg_tambient_num_set(&dev_ctx, STHS34PF80_AVG_T_8);

  /* Set BDU */
  sths34pf80_block_data_update_set(&dev_ctx, 1);

  snprintf((char *)tx_buffer, sizeof(tx_buffer),
           "TObj, TAmb, TPres, TMot, Pres_Flag, Mot_Flag, SW_PRES_FLAG, MOT_COUNTER\r\n");
  tx_com(tx_buffer, strlen((char const *)tx_buffer));
  platform_delay(100);

  /* Set ODR */
  sths34pf80_odr_set(&dev_ctx, STHS34PF80_ODR_AT_15Hz);

  while (1)
  {
    switch (current_state)
    {
      case STATE_0:
        sths34pf80_tmos_sleep_app_state_0_run();
        break;

      case STATE_1:
        sths34pf80_tmos_sleep_app_state_1_set();
        break;

      default:
        break;
    }
  }
}

static void sths34pf80_tmos_sleep_app_state_0_run(void)
{
  // Check for motion detection
  sths34pf80_drdy_status_t status;
  sths34pf80_func_status_t func_status;

  uint8_t motion_flag = 0;

  /* Read samples in drdy mode */
  sths34pf80_drdy_status_get(&dev_ctx, &status);
  if (status.drdy)
  {
    int16_t tobject;
    int16_t tambient;
    int16_t tpres;
    int16_t tmot;

    sths34pf80_func_status_get(&dev_ctx, &func_status);
    sths34pf80_tobject_raw_get(&dev_ctx, &tobject);
    sths34pf80_tambient_raw_get(&dev_ctx, &tambient);
    sths34pf80_tpresence_raw_get(&dev_ctx, &tpres);
    sths34pf80_tmotion_raw_get(&dev_ctx, &tmot);

    if (func_status.mot_flag)
    {
      motion_flag = 1;
    }

    snprintf((char *)tx_buffer, sizeof(tx_buffer),
             "%d, %d, %d, %d, %d, %d, %d, %d\r\n",
             tobject, tambient, tpres, tmot, func_status.pres_flag, func_status.mot_flag, func_status.mot_flag,
             counter);
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }

  if (motion_flag == 1)
  {
    if (current_state == STATE_0)
    {
      counter = 0;
    }
  }
  else if (counter == SW_PRES_TIMER)
  {
    sths34pf80_algo_reset(&dev_ctx);

    snprintf((char *)tx_buffer, sizeof(tx_buffer),
             "Algo reset performed.\r\n");
    tx_com(tx_buffer, strlen((char const *)tx_buffer));

    current_state = STATE_1;

    platform_delay(1000);
  }
}

static void sths34pf80_tmos_sleep_app_state_1_set(void)
{
  sths34pf80_presence_threshold_set(&dev_ctx, 100);
  sths34pf80_presence_hysteresis_set(&dev_ctx, 40);
  sths34pf80_motion_threshold_set(&dev_ctx, 120);
  sths34pf80_motion_hysteresis_set(&dev_ctx, 40);

  sths34pf80_tobject_algo_compensation_set(&dev_ctx, 1);

  /* Set interrupt */
  sths34pf80_route_int_set(&dev_ctx, STHS34PF80_INT_DRDY);

  /* Read samples in drdy handler */
  while (1)
  {
    sths34pf80_func_status_t func_status;
    sths34pf80_drdy_status_t status;

    int16_t tobject;
    int16_t tambient;
    int16_t tpres;
    int16_t tmot;

    if (wakeup_thread)
    {
      wakeup_thread = 0;

      sths34pf80_drdy_status_get(&dev_ctx, &status);

      if (status.drdy)
      {
        sths34pf80_func_status_get(&dev_ctx, &func_status);
        sths34pf80_tobject_raw_get(&dev_ctx, &tobject);
        sths34pf80_tambient_raw_get(&dev_ctx, &tambient);
        sths34pf80_tpresence_raw_get(&dev_ctx, &tpres);
        sths34pf80_tmotion_raw_get(&dev_ctx, &tmot);

        snprintf((char *)tx_buffer, sizeof(tx_buffer),
                 "%d, %d, %d, %d, %d, %d, %d, %d\r\n",
                 tobject, tambient, tpres, tmot, func_status.pres_flag, func_status.mot_flag, func_status.pres_flag,
                 counter);
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
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
static void SPI_3W_Read(SPI_HandleTypeDef *xSpiHandle, uint8_t *val)
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

  for (uint16_t i = 0; i < nBytesToRead; i++)
  {
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
#if defined(NUCLEO_F401RE)
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
  TIM3->CCR1 = PWM_1V8;
  TIM3->CCR2 = PWM_1V8;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}
