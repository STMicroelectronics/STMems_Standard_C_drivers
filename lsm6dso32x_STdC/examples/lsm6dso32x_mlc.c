/*
 ******************************************************************************
 * @file    mlc.c
 * @author  Sensors Software Solution Team
 * @brief   The purpose of this example is to show how use the device
 *          Machine Learning Core (MLC) starting from an ".h" file
 *          generated through with the tool "Machine Learning Core"
 *          of Unico GUI.
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
 * Some MLC examples are available at:
 * https://github.com/STMicroelectronics/STMems_Machine_Learning_Core
 * the same repository is linked to this repository in folder "_resources"
 *
 * For more information about Machine Learning Core tool please refer
 * to AN5656 "LSM6DSO32X: Machine Learning Core".
 *
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI197V1
 * - NUCLEO_F401RE + STEVAL-MKI197V1
 * - DISCOVERY_SPC584B + STEVAL-MKI197V1
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

#include "lsm6dso32x_vibration_monitoring.h"
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
static void platform_delay(uint32_t ms);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lsm6dso32x_mlc(void)
{
  /* Variable declaration */
  lsm6dso32x_pin_int1_route_t pin_int1_route;
  lsm6dso32x_all_sources_t status;
  lsm6dso32x_emb_sens_t emb_sens;
  stmdev_ctx_t dev_ctx;
  uint8_t mlc_out[8];
  uint32_t i;
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

  /* Restore default configuration */
  lsm6dso32x_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dso32x_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Start Machine Learning Core configuration */
  for ( i = 0; i < (sizeof(lsm6dso32x_vibration_monitoring) /
                    sizeof(ucf_line_t) ); i++ ) {
    lsm6dso32x_write_reg(&dev_ctx, lsm6dso32x_vibration_monitoring[i].address,
                       (uint8_t *)&lsm6dso32x_vibration_monitoring[i].data, 1);
  }

  /* End Machine Learning Core configuration */
  /* At this point the device is ready to run but if you need you can also
   * interact with the device but taking in account the MLC configuration.
   *
   * For more information about Machine Learning Core tool please refer
   * to AN5656 "LSM6DSO32X: Machine Learning Core".
   */
  /* Turn off embedded features */
  lsm6dso32x_embedded_sens_get(&dev_ctx, &emb_sens);
  lsm6dso32x_embedded_sens_off(&dev_ctx);
  platform_delay(10);
  /* Turn off Sensors */
  lsm6dso32x_xl_data_rate_set(&dev_ctx, LSM6DSO32X_XL_ODR_OFF);
  lsm6dso32x_gy_data_rate_set(&dev_ctx, LSM6DSO32X_GY_ODR_OFF);
  /* Disable I3C interface */
  lsm6dso32x_i3c_disable_set(&dev_ctx, LSM6DSO32X_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dso32x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lsm6dso32x_xl_full_scale_set(&dev_ctx, LSM6DSO32X_4g);
  lsm6dso32x_gy_full_scale_set(&dev_ctx, LSM6DSO32X_2000dps);
  /* Route signals on interrupt pin 1 */
  lsm6dso32x_pin_int1_route_get(&dev_ctx, &pin_int1_route);
  pin_int1_route.mlc1 = PROPERTY_ENABLE;
  lsm6dso32x_pin_int1_route_set(&dev_ctx, pin_int1_route);
  /* Configure interrupt pin mode notification */
  lsm6dso32x_int_notification_set(&dev_ctx,
                                LSM6DSO32X_BASE_PULSED_EMB_LATCHED);
  /* Enable embedded features */
  lsm6dso32x_embedded_sens_set(&dev_ctx, &emb_sens);
  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  lsm6dso32x_xl_data_rate_set(&dev_ctx, LSM6DSO32X_XL_ODR_26Hz);
  lsm6dso32x_gy_data_rate_set(&dev_ctx, LSM6DSO32X_GY_ODR_OFF);

  /* Main loop */
  while (1) {
    /* Read interrupt source registers in polling mode (no int) */
    lsm6dso32x_all_sources_get(&dev_ctx, &status);

    if (status.mlc1) {
      lsm6dso32x_mlc_out_get(&dev_ctx, mlc_out);
      sprintf((char *)tx_buffer, "Detect MLC interrupt code: %02X\r\n",
              mlc_out[0]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
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
  HAL_I2C_Mem_Write(handle, LSM6DSO32X_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSO32X_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
