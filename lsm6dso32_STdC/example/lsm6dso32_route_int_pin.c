/*
 ******************************************************************************
 * @file    route_int_pin.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to route dataready signals on int pins.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI215V1
 * - NUCLEO_F411RE + STEVAL-MKI215V1
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
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
//#define STEVAL_MKI109V3
#define NUCLEO_F411RE

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2

/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F411RE)
/* NUCLEO_F411RE: Define communication interface */
#define SENSOR_BUS hi2c1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <lsm6dso32_reg.h>
#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#elif defined(NUCLEO_F411RE)
#include "usart.h"
#endif

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME              10

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI, rst;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* WARNING:
 * Functions declare in this section are defined at the end of this file
 * and are strictly related to the hardware platform used.
 */
static int32_t platform_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lsm6dso32_route_int_pin(void)
{
  stmdev_ctx_t dev_ctx;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Variables used for routing signals on the pins */
  lsm6dso32_pin_int1_route_t int1_route;
  lsm6dso32_pin_int2_route_t int2_route;
  /* Check device ID */
  lsm6dso32_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSO32_ID)
    while (1);

  /* Restore default configuration */
  lsm6dso32_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dso32_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dso32_i3c_disable_set(&dev_ctx, LSM6DSO32_I3C_DISABLE);
  /* Set full scale */
  lsm6dso32_xl_full_scale_set(&dev_ctx, LSM6DSO32_4g);
  lsm6dso32_gy_full_scale_set(&dev_ctx, LSM6DSO32_2000dps);
  /* Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed */
  lsm6dso32_data_ready_mode_set(&dev_ctx, LSM6DSO32_DRDY_PULSED);
  /* Route accelerometer data ready signal on INT1 pin (config all signals) */
  int1_route.int1_ctrl.den_drdy_flag           = PROPERTY_DISABLE;
  int1_route.int1_ctrl.int1_boot               = PROPERTY_DISABLE;
  int1_route.int1_ctrl.int1_cnt_bdr            = PROPERTY_DISABLE;
  int1_route.int1_ctrl.int1_drdy_g             = PROPERTY_DISABLE;
  int1_route.int1_ctrl.int1_drdy_xl            = PROPERTY_ENABLE;
  int1_route.int1_ctrl.int1_fifo_full          = PROPERTY_DISABLE;
  int1_route.int1_ctrl.int1_fifo_ovr           = PROPERTY_DISABLE;
  int1_route.int1_ctrl.int1_fifo_th            = PROPERTY_DISABLE;
  int1_route.md1_cfg.int1_6d                   = PROPERTY_DISABLE;
  int1_route.md1_cfg.int1_double_tap           = PROPERTY_DISABLE;
  int1_route.md1_cfg.int1_ff                   = PROPERTY_DISABLE;
  int1_route.md1_cfg.int1_wu                   = PROPERTY_DISABLE;
  int1_route.md1_cfg.int1_single_tap           = PROPERTY_DISABLE;
  int1_route.md1_cfg.int1_sleep_change         = PROPERTY_DISABLE;
  int1_route.emb_func_int1.int1_fsm_lc         = PROPERTY_DISABLE;
  int1_route.emb_func_int1.int1_sig_mot        = PROPERTY_DISABLE;
  int1_route.emb_func_int1.int1_step_detector  = PROPERTY_DISABLE;
  int1_route.emb_func_int1.int1_tilt           = PROPERTY_DISABLE;
  int1_route.fsm_int1_a.int1_fsm1              = PROPERTY_DISABLE;
  int1_route.fsm_int1_a.int1_fsm2              = PROPERTY_DISABLE;
  int1_route.fsm_int1_a.int1_fsm3              = PROPERTY_DISABLE;
  int1_route.fsm_int1_a.int1_fsm4              = PROPERTY_DISABLE;
  int1_route.fsm_int1_a.int1_fsm5              = PROPERTY_DISABLE;
  int1_route.fsm_int1_a.int1_fsm6              = PROPERTY_DISABLE;
  int1_route.fsm_int1_a.int1_fsm7              = PROPERTY_DISABLE;
  int1_route.fsm_int1_a.int1_fsm8              = PROPERTY_DISABLE;
  int1_route.fsm_int1_b.int1_fsm9              = PROPERTY_DISABLE;
  int1_route.fsm_int1_b.int1_fsm10             = PROPERTY_DISABLE;
  int1_route.fsm_int1_b.int1_fsm11             = PROPERTY_DISABLE;
  int1_route.fsm_int1_b.int1_fsm12             = PROPERTY_DISABLE;
  int1_route.fsm_int1_b.int1_fsm13             = PROPERTY_DISABLE;
  int1_route.fsm_int1_b.int1_fsm14             = PROPERTY_DISABLE;
  int1_route.fsm_int1_b.int1_fsm15             = PROPERTY_DISABLE;
  int1_route.fsm_int1_b.int1_fsm16             = PROPERTY_DISABLE;
  lsm6dso32_pin_int1_route_set(&dev_ctx, &int1_route);
  /* Route gyroscope data ready signal on INT2 pin (mask previous values signals)*/
  lsm6dso32_pin_int2_route_get(&dev_ctx, &int2_route);
  int2_route.int2_ctrl.int2_drdy_g = PROPERTY_ENABLE;
  lsm6dso32_pin_int2_route_set(&dev_ctx, &int2_route);
  /* Set ODR (Output Data Rate) and power mode*/
  lsm6dso32_xl_data_rate_set(&dev_ctx, LSM6DSO32_XL_ODR_12Hz5_LOW_PW);
  lsm6dso32_gy_data_rate_set(&dev_ctx, LSM6DSO32_GY_ODR_26Hz_HIGH_PERF);

  while (1) {
    /* Accelerometer and gyto data ready signals in pulse mode are routed
         * on INT1 and INT2 pin.
         */
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
static int32_t platform_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1) {
    HAL_I2C_Mem_Write(handle, LSM6DSO32_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }

#ifdef STEVAL_MKI109V3

  else if (handle == &hspi2) {
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }

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
  if (handle == &hi2c1) {
    HAL_I2C_Mem_Read(handle, LSM6DSO32_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }

#ifdef STEVAL_MKI109V3

  else if (handle == &hspi2) {
    /* Read command */
    reg |= 0x80;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }

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
#ifdef NUCLEO_F411RE
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
#endif
#ifdef STEVAL_MKI109V3
  CDC_Transmit_FS(tx_buffer, len);
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
  HAL_Delay(ms);
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
