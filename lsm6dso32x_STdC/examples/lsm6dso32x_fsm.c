/*
 ******************************************************************************
 * @file    fsm.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to use some fsm programs.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI197V1
 * - NUCLEO_F401RE + STEVAL-MKI197V1
 * - DISCOVERY_SPC584B + STEVAL-MKI197V1
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
#include "lsm6dso32x_reg.h"

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

/*
 * Programs can be extracted from ".ucf" configuration file generated
 * by Unico / Unicleo tool.
 *
 * This example contains 7 programs of the Finite State Machine:
 *    - glance
 *    - motion
 *    - no_motion
 *    - wakeup
 *    - pickup
 *    - orientation
 *    - wrist_tilt
 *
 */

/* Program: glance */
static const uint8_t lsm6so_prg_glance[] = {
  0xb2, 0x10, 0x24, 0x20, 0x17, 0x17, 0x66, 0x32,
  0x66, 0x3c, 0x20, 0x20, 0x02, 0x02, 0x08, 0x08,
  0x00, 0x04, 0x0c, 0x00, 0xc7, 0x66, 0x33, 0x73,
  0x77, 0x64, 0x88, 0x75, 0x99, 0x66, 0x33, 0x53,
  0x44, 0xf5, 0x22, 0x00,
};

/* Program: motion */
static const uint8_t lsm6so_prg_motion[] = {
  0x51, 0x10, 0x16, 0x00, 0x00, 0x00, 0x66, 0x3c,
  0x02, 0x00, 0x00, 0x7d, 0x00, 0xc7, 0x05, 0x99,
  0x33, 0x53, 0x44, 0xf5, 0x22, 0x00,
};

/* Program: no_motion */
static const uint8_t lsm6so_prg_no_motion[] = {
  0x51, 0x00, 0x10, 0x00, 0x00, 0x00, 0x66, 0x3c,
  0x02, 0x00, 0x00, 0x7d, 0xff, 0x53, 0x99, 0x50,
};
/* Program: wakeup */
static const uint8_t lsm6so_prg_wakeup[] = {
  0xe2, 0x00, 0x1e, 0x20, 0x13, 0x15, 0x66, 0x3e,
  0x66, 0xbe, 0xcd, 0x3c, 0xc0, 0xc0, 0x02, 0x02,
  0x0b, 0x10, 0x05, 0x66, 0xcc, 0x35, 0x38, 0x35,
  0x77, 0xdd, 0x03, 0x54, 0x22, 0x00,
};

/* Program: pickup */
static const uint8_t lsm6so_prg_pickup[] = {
  0x51, 0x00, 0x10, 0x00, 0x00, 0x00, 0x33, 0x3c,
  0x02, 0x00, 0x00, 0x05, 0x05, 0x99, 0x30, 0x00,
};

/* Program: orientation */
static const uint8_t lsm6so_prg_orientation[] = {
  0x91, 0x10, 0x16, 0x00, 0x00, 0x00, 0x66, 0x3a,
  0x66, 0x32, 0xf0, 0x00, 0x00, 0x0d, 0x00, 0xc7,
  0x05, 0x73, 0x99, 0x08, 0xf5, 0x22,
};

/* Program: wrist_tilt */
static const uint8_t lsm6so_prg_wrist_tilt[] = {
  0x52, 0x00, 0x14, 0x00, 0x00, 0x00, 0xae, 0xb7,
  0x80, 0x00, 0x00, 0x06, 0x0f, 0x05, 0x73, 0x33,
  0x07, 0x54, 0x44, 0x22,
};

/*
 * End of lsm6dso32x_prg_defs.h
 */

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms

/* Private variables ---------------------------------------------------------*/
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
void lsm6dso32x_fsm(void)
{
  /* Variable declaration */
  stmdev_ctx_t                dev_ctx;
  lsm6dso32x_pin_int1_route_t   pin_int1_route;
  lsm6dso32x_emb_fsm_enable_t   fsm_enable;
  lsm6dso32x_emb_sens_t         emb_sens;
  lsm6dso32x_fsm_out_t          fsm_out;
  lsm6dso32x_all_sources_t      status;
  uint16_t                   fsm_addr;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg  = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle    = &SENSOR_BUS;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm6dso32x_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSO32X_ID)
    while (1);

  /* Restore default configuration (not FSM) */
  lsm6dso32x_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dso32x_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dso32x_i3c_disable_set(&dev_ctx, LSM6DSO32X_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dso32x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lsm6dso32x_xl_full_scale_set(&dev_ctx, LSM6DSO32X_4g);
  lsm6dso32x_gy_full_scale_set(&dev_ctx, LSM6DSO32X_2000dps);
  /* Route signals on interrupt pin 1 */
  lsm6dso32x_pin_int1_route_get(&dev_ctx, &pin_int1_route);
  pin_int1_route.fsm1               = PROPERTY_ENABLE;
  pin_int1_route.fsm2               = PROPERTY_ENABLE;
  pin_int1_route.fsm3               = PROPERTY_ENABLE;
  pin_int1_route.fsm4               = PROPERTY_ENABLE;
  pin_int1_route.fsm5               = PROPERTY_ENABLE;
  pin_int1_route.fsm6               = PROPERTY_ENABLE;
  pin_int1_route.fsm7               = PROPERTY_ENABLE;
  lsm6dso32x_pin_int1_route_set(&dev_ctx, pin_int1_route);
  /* Configure interrupt pin mode notification */
  lsm6dso32x_int_notification_set(&dev_ctx,
                                  LSM6DSO32X_BASE_PULSED_EMB_LATCHED);
  /*
   * Start Finite State Machine configuration
   */
  /* Reset Long Counter */
  lsm6dso32x_long_cnt_int_value_set(&dev_ctx, 0x0000U);
  /* Set the first address where the programs are written */
  lsm6dso32x_fsm_start_address_set(&dev_ctx, LSM6DSO32X_START_FSM_ADD);
  /* Set the number of the programs */
  lsm6dso32x_fsm_number_of_programs_set(&dev_ctx, 7 );
  /* Enable final state machine */
  fsm_enable.fsm_enable_a.fsm1_en    = PROPERTY_ENABLE ;
  fsm_enable.fsm_enable_a.fsm2_en    = PROPERTY_ENABLE ;
  fsm_enable.fsm_enable_a.fsm3_en    = PROPERTY_ENABLE ;
  fsm_enable.fsm_enable_a.fsm4_en    = PROPERTY_ENABLE ;
  fsm_enable.fsm_enable_a.fsm5_en    = PROPERTY_ENABLE ;
  fsm_enable.fsm_enable_a.fsm6_en    = PROPERTY_ENABLE ;
  fsm_enable.fsm_enable_a.fsm7_en    = PROPERTY_ENABLE ;
  fsm_enable.fsm_enable_a.fsm8_en    = PROPERTY_DISABLE;
  fsm_enable.fsm_enable_b.fsm9_en    = PROPERTY_DISABLE;
  fsm_enable.fsm_enable_b.fsm10_en   = PROPERTY_DISABLE;
  fsm_enable.fsm_enable_b.fsm11_en   = PROPERTY_DISABLE;
  fsm_enable.fsm_enable_b.fsm12_en   = PROPERTY_DISABLE;
  fsm_enable.fsm_enable_b.fsm13_en   = PROPERTY_DISABLE;
  fsm_enable.fsm_enable_b.fsm14_en   = PROPERTY_DISABLE;
  fsm_enable.fsm_enable_b.fsm15_en   = PROPERTY_DISABLE;
  fsm_enable.fsm_enable_b.fsm16_en   = PROPERTY_DISABLE;
  lsm6dso32x_fsm_enable_set(&dev_ctx, &fsm_enable);
  /* Set Finite State Machine data rate */
  lsm6dso32x_fsm_data_rate_set(&dev_ctx, LSM6DSO32X_ODR_FSM_26Hz);
  /* Write Programs */
  fsm_addr = LSM6DSO32X_START_FSM_ADD;
  /* Glance */
  lsm6dso32x_ln_pg_write(&dev_ctx, fsm_addr,
                         (uint8_t *)lsm6so_prg_glance,
                         sizeof(lsm6so_prg_glance));
  fsm_addr += sizeof(lsm6so_prg_glance);
  /* motion */
  lsm6dso32x_ln_pg_write(&dev_ctx, fsm_addr,
                         (uint8_t *)lsm6so_prg_motion,
                         sizeof(lsm6so_prg_motion));
  fsm_addr += sizeof(lsm6so_prg_motion);
  /* no_motion */
  lsm6dso32x_ln_pg_write(&dev_ctx, fsm_addr,
                         (uint8_t *)lsm6so_prg_no_motion,
                         sizeof(lsm6so_prg_no_motion));
  fsm_addr += sizeof(lsm6so_prg_no_motion);
  /* wakeup */
  lsm6dso32x_ln_pg_write(&dev_ctx, fsm_addr,
                         (uint8_t *)lsm6so_prg_wakeup,
                         sizeof(lsm6so_prg_wakeup));
  fsm_addr += sizeof(lsm6so_prg_wakeup);
  /* pickup */
  lsm6dso32x_ln_pg_write(&dev_ctx, fsm_addr,
                         (uint8_t *)lsm6so_prg_pickup,
                         sizeof(lsm6so_prg_pickup));
  fsm_addr += sizeof(lsm6so_prg_pickup);
  /* orientation */
  lsm6dso32x_ln_pg_write(&dev_ctx, fsm_addr,
                         (uint8_t *)lsm6so_prg_orientation,
                         sizeof(lsm6so_prg_orientation));
  fsm_addr += sizeof(lsm6so_prg_orientation);
  /* wrist_tilt */
  lsm6dso32x_ln_pg_write(&dev_ctx, fsm_addr,
                         (uint8_t *)lsm6so_prg_wrist_tilt,
                         sizeof(lsm6so_prg_wrist_tilt));
  emb_sens.fsm = PROPERTY_ENABLE;
  lsm6dso32x_embedded_sens_set(&dev_ctx, &emb_sens);
  /*
   * End Finite State Machine configuration
   */
  /* Set Output Data Rate */
  lsm6dso32x_xl_data_rate_set(&dev_ctx, LSM6DSO32X_XL_ODR_104Hz);
  lsm6dso32x_gy_data_rate_set(&dev_ctx, LSM6DSO32X_GY_ODR_104Hz);

  /* Main loop */
  while (1) {
    /* Read interrupt source registers in polling mode (no int) */
    lsm6dso32x_all_sources_get(&dev_ctx, &status);

    if (status.fsm1) {
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "glance detected\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (status.fsm2) {
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "motion detected\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (status.fsm3) {
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "no motion detected\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (status.fsm4) {
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "wakeup detected\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (status.fsm5) {
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "pickup detected\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (status.fsm6) {
      lsm6dso32x_fsm_out_get(&dev_ctx, &fsm_out);
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "orientation detected (%d, %d, %d, %d, %d, %d, %d, %d)\r\n",
              fsm_out.fsm_outs6.n_v, fsm_out.fsm_outs6.p_v,
              fsm_out.fsm_outs6.n_z, fsm_out.fsm_outs6.p_z,
              fsm_out.fsm_outs6.n_y, fsm_out.fsm_outs6.p_y,
              fsm_out.fsm_outs6.n_x, fsm_out.fsm_outs6.p_x);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (status.fsm7) {
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "wrist tilt detected\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    platform_delay(20);
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
  HAL_I2C_Mem_Write(handle, LSM6DSO32X_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSO32X_I2C_ADD_L & 0xFE, reg,
               (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LSM6DSO32X_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LSM6DSO32X_I2C_ADD_L & 0xFE, reg, bufp, len);
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
