/*
 ******************************************************************************
 * @file    wake_up.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to detect wake_up from sensor.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI110V1
 * - NUCLEO_F411RE + STEVAL-MKI110V1
 * - DISCOVERY_SPC584B + STEVAL-MKI110V1
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
#define NUCLEO_F411RE    /* little endian */
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
#include "ais328dq_reg.h"

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

#elif defined(SPC584B_DIS)
#include "components.h"
#endif

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME          5 //ms
/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;

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
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void ais328dq_wake_up(void)
{
  /* Variables */
  stmdev_ctx_t dev_ctx;
  ais328dq_int1_on_th_conf_t int1_on_th_conf;
  ais328dq_int1_src_t  int1_src;

  /* Initialize SPI mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Start Main loop */
  while(1)
  {
    /* turn device on  */
    platform_init();
    /* Wait sensor boot time */
    platform_delay(BOOT_TIME);
   
    /* Check device ID */
    ais328dq_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != AIS328DQ_ID)
      while(1); /* manage here device not found */   
   
   
    /* Set ODR (normal mode)*/
    ais328dq_data_rate_set(&dev_ctx, AIS328DQ_ODR_100Hz);
    /* Set Full scale */
    ais328dq_full_scale_set(&dev_ctx, AIS328DQ_4g); 
   
    /* Enable axis: Default all axis are enabled so you can avoid this step */
    //ais328dq_axis_x_data_set(&dev_ctx, PROPERTY_ENABLE);
    //ais328dq_axis_y_data_set(&dev_ctx, PROPERTY_ENABLE);
    //ais328dq_axis_z_data_set(&dev_ctx, PROPERTY_ENABLE);
   
    /* interrupt request latched */
    ais328dq_int1_notification_set(&dev_ctx, AIS328DQ_INT1_LATCHED);
   
    /* interrupt polarity: Default is active high so you can avoid this step */
    ais328dq_pin_polarity_set(&dev_ctx, AIS328DQ_ACTIVE_HIGH);
   
    /* interrupt pin routing */
    ais328dq_pin_int1_route_set(&dev_ctx, AIS328DQ_PAD1_INT1_SRC);
   
    /* interrupt pin mode */
    ais328dq_pin_mode_set(&dev_ctx, AIS328DQ_PUSH_PULL);
   
    /* set HP filter path on output and interrupt generator 1 */
    ais328dq_hp_path_set(&dev_ctx, AIS328DQ_HP_ON_INT1_OUT);
   
    HAL_Delay(150);
   
    /* set interrupt threshold */
    ais328dq_int1_treshold_set(&dev_ctx, 0x04);
    /* set interrupt duration */
    ais328dq_int1_dur_set(&dev_ctx, 0x01);
   
    /* set interrupt on pin mode */
    ais328dq_int1_on_threshold_mode_set(&dev_ctx, AIS328DQ_INT1_ON_THRESHOLD_OR);
    /* set interrupt on pin mode */
    int1_on_th_conf.int1_xlie = 0;
    int1_on_th_conf.int1_xhie = 1;
    int1_on_th_conf.int1_ylie = 0;
    int1_on_th_conf.int1_yhie = 1;
    int1_on_th_conf.int1_zlie = 0;
    int1_on_th_conf.int1_zhie = 1;
    ais328dq_int1_on_threshold_conf_set(&dev_ctx, int1_on_th_conf);
   
    ais328dq_hp_reset_get(&dev_ctx);
    ais328dq_int1_src_get(&dev_ctx, &int1_src);
   
    if (int1_src.xh | int1_src.yh | int1_src.zh){
      while(1); //wake up detected
    }
   
    platform_delay(1000);
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
#if defined(NUCLEO_F411RE)
  /* Write multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Write(handle, AIS328DQ_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Write multiple command */
  reg |= 0x40;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  /* Write multiple command */
  reg |= 0x80;
  i2c_lld_write(handle,  AIS328DQ_I2C_ADD_L & 0xFE, reg, bufp, len);
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
  /* Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read(handle, AIS328DQ_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Read multiple command */
  reg |= 0xC0;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  /* Read multiple command */
  reg |= 0x80;
  i2c_lld_read(handle, AIS328DQ_I2C_ADD_L & 0xFE, reg, bufp, len);
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
