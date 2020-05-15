/*
 ******************************************************************************
 * @file    fifo_stream.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to extract data from the sensor in 
 *          polling mode.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI134V1
 * - NUCLEO_F411RE +STEVAL-MKI134V1
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
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
#include "lis3dsh_reg.h"
#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#elif defined(NUCLEO_F411RE)
#include "usart.h"
#endif


/* Private typedef -----------------------------------------------------------*/
typedef union {
  uint8_t u8[6];
  int16_t i16[3];
} fifo_data_t;

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME         10 //ms

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[1000];
static fifo_data_t fifo_data[32];

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
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lis3dsh_fifo_stream(void)
{
  lis3dsh_pin_int1_route_t int1_route;
  lis3dsh_all_sources_t all_sources;
  lis3dsh_bus_mode_t bus_mode;
  lis3dsh_status_var_t status;
  lis3dsh_int_mode_t int_mode;
  stmdev_ctx_t dev_ctx;
  lis3dsh_reg_t reg;
  lis3dsh_id_t id;
  lis3dsh_md_t md;
  uint8_t i;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Initialize platform specific hardware */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  lis3dsh_id_get(&dev_ctx, &id);
  if (id.whoami != LIS3DSH_ID)
    while(1);

  /* Restore default configuration */
  lis3dsh_init_set(&dev_ctx, LIS3DSH_RESET);
  do {
    lis3dsh_status_get(&dev_ctx, &status);
  } while (status.sw_reset);

  /* Set bdu and if_inc recommended for driver usage */
  lis3dsh_init_set(&dev_ctx, LIS3DSH_DRV_RDY);
  
  /* Select bus interface */
  bus_mode = LIS3DSH_SEL_BY_HW;
  lis3dsh_bus_mode_set(&dev_ctx, &bus_mode);

  /* FIFO configuration */
  lis3dsh_read_reg(&dev_ctx, LIS3DSH_FIFO_CTRL, &reg.byte,1);
  reg.fifo_ctrl.fmode = 0x02; /* FIFO stream mode */
  lis3dsh_write_reg(&dev_ctx, LIS3DSH_FIFO_CTRL, &reg.byte,1);
  lis3dsh_read_reg(&dev_ctx, LIS3DSH_CTRL_REG6, &reg.byte,1);
  reg.ctrl_reg6.fifo_en = PROPERTY_ENABLE;
  lis3dsh_write_reg(&dev_ctx, LIS3DSH_CTRL_REG6, &reg.byte,1);

  /* Configure interrupt pins */
  lis3dsh_interrupt_mode_get(&dev_ctx, &int_mode);
  int_mode.latched = PROPERTY_DISABLE;
  lis3dsh_interrupt_mode_set(&dev_ctx, &int_mode);
  lis3dsh_pin_int1_route_get(&dev_ctx, &int1_route);
  int1_route.fifo_full   = PROPERTY_ENABLE; /* Enable hardware notification */
  lis3dsh_pin_int1_route_set(&dev_ctx, &int1_route);

  /* Set Output Data Rate */
  lis3dsh_mode_get(&dev_ctx, &md);
  md.fs =  LIS3DSH_4g;
  md.odr = LIS3DSH_800Hz;
  lis3dsh_mode_set(&dev_ctx, &md);

  /* Read samples in polling mode (no int). */
  while(1)
  {
    /* Read output only if new values are available */
    lis3dsh_all_sources_get(&dev_ctx, &all_sources);
    if ( all_sources.fifo_full ) {

      lis3dsh_read_reg(&dev_ctx, LIS3DSH_FIFO_SRC, &reg.byte,1);
      lis3dsh_read_reg(&dev_ctx, LIS3DSH_OUT_X_L, &fifo_data[0].u8[0],
                       reg.fifo_src.fss*6);
      
      /* print sensor data  */
      for (i = 0; i < reg.fifo_src.fss; i++) {
        sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                lis3dsh_from_fs4_to_mg(fifo_data[i].i16[0]),
                lis3dsh_from_fs4_to_mg(fifo_data[i].i16[1]),
                lis3dsh_from_fs4_to_mg(fifo_data[i].i16[2]));
       tx_com(tx_buffer, strlen((char const*)tx_buffer));
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1) {
    /* Write multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Write(handle, LIS3DSH_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#if defined(STEVAL_MKI109V3)
  else if (handle == &hspi2) {
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
  if (handle == &hi2c1) {
    /* Read multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Read(handle, LIS3DSH_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#if defined(STEVAL_MKI109V3)
  else if (handle == &hspi2) {
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
#if defined(STEVAL_MKI109V3)
  CDC_Transmit_FS(tx_buffer, len);
#elif defined(NUCLEO_F411RE)
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
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