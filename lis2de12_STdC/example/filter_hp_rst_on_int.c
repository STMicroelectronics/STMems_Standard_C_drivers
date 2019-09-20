/*
 ******************************************************************************
 * @file    filter_hp_rst_on_int.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to configure the high-pass filter in 
 *          "autoreset on inteerupt" mode.
 *          The filter is reset when the interrupt threshold is reached
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A2
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A2 - Host side: UART(COM) to USB bridge
 *                                       - I2C(Default) / SPI
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
#define NUCLEO_F411RE_X_NUCLEO_IKS01A2

#ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
/* NUCLEO_F411RE_X_NUCLEO_IKS01A2: Define communication interface */
#define SENSOR_BUS hi2c1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "lis2de12_reg.h"
#include "gpio.h"
#include "i2c.h"
#ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
#include "usart.h"
#endif

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static float acceleration_mg[3];
static uint8_t whoamI;
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
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lis2de12_filter_hp_rst_on_int(void)
{
  
  lis2de12_reg_t reg;
  stmdev_ctx_t dev_ctx;
  
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Initialize platform specific hardware */
  platform_init();

  /* Check device ID */
  lis2de12_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LIS2DE12_ID) {
    while(1) {
      /* manage here device not found */
    }
  }
  
  /* Set full scale to 2g */
  lis2de12_full_scale_set(&dev_ctx, LIS2DE12_2g);  
  
  /* route HP filter output on outputs registers */
  lis2de12_high_pass_on_outputs_set(&dev_ctx, PROPERTY_ENABLE);  
  /* route HP filter output on interrupt generator 1 */
  lis2de12_high_pass_int_conf_set(&dev_ctx, LIS2DE12_ON_INT1_GEN);
  /* HP filter mode is "autoreset on inteerupt event" */
  lis2de12_high_pass_mode_set(&dev_ctx, LIS2DE12_AUTORST_ON_INT);
  /* Set HP filter on mode */
  lis2de12_high_pass_bandwidth_set(&dev_ctx, LIS2DE12_AGGRESSIVE);
  
  /* Configure interrupt on threshold on z axis low / high events */
  reg.byte = PROPERTY_DISABLE;
  reg.int1_cfg.xhie = PROPERTY_ENABLE;
  reg.int1_cfg.yhie = PROPERTY_ENABLE;
  lis2de12_int1_gen_conf_set(&dev_ctx, &reg.int1_cfg);
  /* Set interrupt treshold at ~800 mg -> 1bit = 16mg@2g */
  lis2de12_int1_gen_threshold_set(&dev_ctx, 0x30);
  /* Set duration to zero - 1 bit = 1/ODR */
  lis2de12_int1_gen_duration_set(&dev_ctx, 0);
  
  /* Set Output Data Rate to 25Hz */
  lis2de12_data_rate_set(&dev_ctx, LIS2DE12_ODR_25Hz);
  
  /* Read samples in polling mode (no int) */
  while(1)
  {

    /* Read output only if new value available */
    lis2de12_xl_data_ready_get(&dev_ctx, &reg.byte);
    if (reg.byte)
    {
      lis2de12_int1_gen_source_get(&dev_ctx, &reg.int1_src);
      /* Read accelerometer data */
      memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
      lis2de12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
      acceleration_mg[0] =
        lis2de12_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] =
        lis2de12_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] =
        lis2de12_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);
    
      sprintf((char*)tx_buffer, "Acceleration [mg]:\t%4.2f\t%4.2f\t%4.2f\t%02X\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2], reg.byte);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    /* Write multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Write(handle, LIS2DE12_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
    /* Write multiple command */
    reg |= 0x40;
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
  if (handle == &hi2c1)
  {
    /* Read multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Read(handle, LIS2DE12_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
    /* Read multiple command */
    reg |= 0xC0;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }
#endif
  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
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
