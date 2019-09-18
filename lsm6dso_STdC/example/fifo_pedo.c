/*
 ******************************************************************************
 * @file    fifo_pedo.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to read step count from FIFO.
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
 * - STEVAL_MKI109V3
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A2
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A2 - Host side: UART(COM) to USB bridge
 *                                       - I2C(Default) / SPI(N/A)
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
#define NUCLEO_F411RE_X_NUCLEO_IKS01A2

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2

/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
/* NUCLEO_F411RE_X_NUCLEO_IKS01A2: Define communication interface */
#define SENSOR_BUS hi2c1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include <lsm6dso_reg.h>
#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
#include "usart.h"
#endif

/* Private macro -------------------------------------------------------------*/

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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lsm6dso_fifo_pedo_simple(void)
{
  stmdev_ctx_t ag_ctx;

  /*
   * Uncomment to configure INT 1
   */
  //lsm6dso_pin_int1_route_t int1_route;
  /*
   * Uncomment to configure INT 2
   */
  //lsm6dso_pin_int2_route_t int2_route;

  ag_ctx.write_reg = platform_write;
  ag_ctx.read_reg = platform_read;
  ag_ctx.handle = &hi2c1;

  /*
   * Init test platform
   */
  platform_init();

  /*
   *  Check device ID
   */
  lsm6dso_device_id_get(&ag_ctx, &whoamI);
  if (whoamI != LSM6DSO_ID)
    while(1);

  /*
   *  Restore default configuration
   */
  lsm6dso_reset_set(&ag_ctx, PROPERTY_ENABLE);
  do {
    lsm6dso_reset_get(&ag_ctx, &rst);
  } while (rst);

  /*
   * Disable I3C interface
   */
  lsm6dso_i3c_disable_set(&ag_ctx, LSM6DSO_I3C_DISABLE);

  /*
   * Set XL full scale
   */
  lsm6dso_xl_full_scale_set(&ag_ctx, LSM6DSO_2g);

  /*
   *  Enable Block Data Update
   */
  lsm6dso_block_data_update_set(&ag_ctx, PROPERTY_ENABLE);

  /*
   * Set FIFO mode to Stream mode (aka Continuous Mode)
   */
  lsm6dso_fifo_mode_set(&ag_ctx, LSM6DSO_STREAM_MODE);

  /*
   * Enable latched interrupt notification.
   */
  lsm6dso_int_notification_set(&ag_ctx, LSM6DSO_ALL_INT_LATCHED);

 /*
   * Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed.
   */
  //lsm6dso_data_ready_mode_set(&ag_ctx, LSM6DSO_DRDY_PULSED);

  /*
   * FIFO watermark interrupt routed on INT1 pin
   *
   * Remember that INT1 pin is used by sensor to switch in I3C mode
   * Uncomment to configure INT 1
   */
  //lsm6dso_pin_int1_route_get(&ag_ctx, &int1_route);
  //int1_route.reg.emb_func_int1.int1_step_detector = PROPERTY_ENABLE;
  //lsm6dso_pin_int1_route_set(&ag_ctx, &int1_route);

  /*
   * FIFO watermark interrupt routed on INT2 pin
   * Uncomment to configure INT 2
   */
  //lsm6dso_pin_int2_route_get(&ag_ctx, &int2_route);
  //int2_route.reg.emb_func_int2.int2_step_detector = PROPERTY_ENABLE;
  //lsm6dso_pin_int2_route_set(&ag_ctx, &int2_route);

  /*
   * Enable HW Timestamp
   */
  lsm6dso_timestamp_set(&ag_ctx, PROPERTY_ENABLE);

  /*
   * Enable pedometer
   */
  lsm6dso_pedo_sens_set(&ag_ctx, LSM6DSO_PEDO_BASE_MODE);
  lsm6dso_fifo_pedo_batch_set(&ag_ctx, PROPERTY_ENABLE);
  lsm6dso_steps_reset(&ag_ctx);

  /*
   * Enable I2C Master
   */
  lsm6dso_sh_master_set(&ag_ctx, PROPERTY_ENABLE);

  /*
   * Set Output Data Rate
   */
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_26Hz);

  while(1)
  {
	uint16_t num = 0;
    lsm6dso_fifo_tag_t reg_tag;
    pedo_count_sample_t pedo_sample;

    /*
     * Read FIFO samples number
     */
    lsm6dso_fifo_data_level_get(&ag_ctx, &num);
    if (num > 0)
    {
      while(num--)
      {
        /*
         * Read FIFO tag
         */
        lsm6dso_fifo_sensor_tag_get(&ag_ctx, &reg_tag);
        switch(reg_tag)
        {
          case LSM6DSO_STEP_CPUNTER_TAG:
            lsm6dso_fifo_out_raw_get(&ag_ctx, pedo_sample.byte);

            sprintf((char*)tx_buffer, "Step Count :%u T %u\r\n",
                    (unsigned int)pedo_sample.step_count,
                    (unsigned int)pedo_sample.timestamp);
            tx_com(tx_buffer, strlen((char const*)tx_buffer));
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
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, LSM6DSO_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
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
    HAL_I2C_Mem_Read(handle, LSM6DSO_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
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
