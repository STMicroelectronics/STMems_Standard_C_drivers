/*
 ******************************************************************************
 * @file    lsm6dsrx_fifo_pedo.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to read step count from FIFO.
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
#include <lsm6dsrx_reg.h>

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

/* Private variables ---------------------------------------------------------*/
typedef union {
  struct {
    uint16_t step_count;
    uint32_t timestamp;
#ifdef __GNUC__
  } __attribute__((__packed__));
#else /* __GNUC__ */
  };
#endif /* __GNUC__ */
  uint8_t byte[6];
} pedo_count_sample_t;

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
void lsm6dsrx_fifo_pedo_simple(void)
{
  stmdev_ctx_t ag_ctx;
  /* Uncomment to configure INT 1 */
  //lsm6dsrx_pin_int1_route_t int1_route;
  /* Uncomment to configure INT 2 */
  //lsm6dsrx_pin_int2_route_t int2_route;
  ag_ctx.write_reg = platform_write;
  ag_ctx.read_reg = platform_read;
  ag_ctx.handle = &SENSOR_BUS;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm6dsrx_device_id_get(&ag_ctx, &whoamI);

  if (whoamI != LSM6DSRX_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsrx_reset_set(&ag_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsrx_reset_get(&ag_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dsrx_i3c_disable_set(&ag_ctx, LSM6DSRX_I3C_DISABLE);
  /* Set XL full scale */
  lsm6dsrx_xl_full_scale_set(&ag_ctx, LSM6DSRX_2g);
  /* Enable Block Data Update */
  lsm6dsrx_block_data_update_set(&ag_ctx, PROPERTY_ENABLE);
  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dsrx_fifo_mode_set(&ag_ctx, LSM6DSRX_STREAM_MODE);
  /* Enable latched interrupt notification. */
  lsm6dsrx_int_notification_set(&ag_ctx, LSM6DSRX_ALL_INT_LATCHED);
  /* Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed. */
  //lsm6dsrx_data_ready_mode_set(&ag_ctx, LSM6DSRX_DRDY_PULSED);
  /* FIFO watermark interrupt routed on INT1 pin
   * Remember that INT1 pin is used by sensor to switch in I3C mode
   * Uncomment to configure INT 1
   */
  //lsm6dsrx_pin_int1_route_get(&ag_ctx, &int1_route);
  //int1_route.reg.emb_func_int1.int1_step_detector = PROPERTY_ENABLE;
  //lsm6dsrx_pin_int1_route_set(&ag_ctx, &int1_route);
  /* FIFO watermark interrupt routed on INT2 pin
   * Uncomment to configure INT 2
   */
  //lsm6dsrx_pin_int2_route_get(&ag_ctx, &int2_route);
  //int2_route.reg.emb_func_int2.int2_step_detector = PROPERTY_ENABLE;
  //lsm6dsrx_pin_int2_route_set(&ag_ctx, &int2_route);
  /* Enable HW Timestamp */
  lsm6dsrx_timestamp_set(&ag_ctx, PROPERTY_ENABLE);
  /* Enable pedometer */
  lsm6dsrx_pedo_sens_set(&ag_ctx, PROPERTY_ENABLE);
  lsm6dsrx_fifo_pedo_batch_set(&ag_ctx, PROPERTY_ENABLE);
  lsm6dsrx_steps_reset(&ag_ctx);
  /* Enable I2C Master */
  lsm6dsrx_sh_master_set(&ag_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm6dsrx_xl_data_rate_set(&ag_ctx, LSM6DSRX_XL_ODR_26Hz);

  while (1) {
    uint16_t num = 0;
    lsm6dsrx_fifo_tag_t reg_tag;
    pedo_count_sample_t pedo_sample;
    /* Read FIFO samples number */
    lsm6dsrx_fifo_data_level_get(&ag_ctx, &num);

    if (num > 0) {
      while (num--) {
        /* Read FIFO tag */
        lsm6dsrx_fifo_sensor_tag_get(&ag_ctx, &reg_tag);

        switch (reg_tag) {
          case LSM6DSRX_STEP_CPUNTER_TAG:
            lsm6dsrx_fifo_out_raw_get(&ag_ctx, pedo_sample.byte);
            sprintf((char *)tx_buffer, "Step Count :%u T %u\r\n",
                    (unsigned int)pedo_sample.step_count,
                    (unsigned int)pedo_sample.timestamp);
            tx_com(tx_buffer, strlen((char const *)tx_buffer));
            break;

          default:
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
  HAL_I2C_Mem_Write(handle, LSM6DSRX_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSRX_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LSM6DSRX_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LSM6DSRX_I2C_ADD_L & 0xFE, reg, bufp, len);
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
