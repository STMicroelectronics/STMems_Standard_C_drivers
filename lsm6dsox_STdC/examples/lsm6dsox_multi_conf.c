/*
 ******************************************************************************
 * @file    multi_conf.c
 * @author  Sensors Software Solution Team
 * @brief   The purpose of this example is to show how use the device
 *          Machine Learning Core (MLC) starting from an ".h" file
 *          generated through with the tool "Machine Learning Core"
 *          of Unico GUI mixed with other functionalities.
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
 *
 * Some MLC examples are available at:
 * https://github.com/STMicroelectronics/STMems_Machine_Learning_Core
 * the same repository is linked to this repository in folder "_resources"
 *
 * For more information about Machine Learning Core tool please refer
 * to AN5259 "LSM6DSOX: Machine Learning Core".
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

#include "lsm6dsox_yoga_pose_recognition.h"
#include "lsm6dsox_reg.h"

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
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lsm6dsox_multi_conf(void)
{
  /* Variable declaration */
  stmdev_ctx_t dev_ctx;
  lsm6dsox_pin_int1_route_t pin_int1_route;
  lsm6dsox_all_sources_t status;
  lsm6dsox_emb_sens_t emb_sens;
  uint8_t mlc_out[8];
  uint32_t i;
  uint16_t steps;
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
  lsm6dsox_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSOX_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Start Machine Learning Core configuration */
  for ( i = 0; i < (sizeof(lsm6dsox_yoga_pose_recognition) /
                    sizeof(ucf_line_t) ); i++ ) {
    lsm6dsox_write_reg(&dev_ctx,
                       lsm6dsox_yoga_pose_recognition[i].address,
                       (uint8_t *)&lsm6dsox_yoga_pose_recognition[i].data, 1);
  }

  /* End Machine Learning Core configuration */
  /* At this point the device is ready to run but if you need you can also
   * interact with the device but taking in account the MLC configuration.
   *
   * For more information about Machine Learning Core tool please refer
   * to AN5259 "LSM6DSOX: Machine Learning Core".
   */
  /* Turn off embedded features */
  lsm6dsox_embedded_sens_get(&dev_ctx, &emb_sens);
  lsm6dsox_embedded_sens_off(&dev_ctx);
  platform_delay(10);
  /* Turn off Sensors */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_OFF);
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_OFF);
  /* Disable I3C interface */
  lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_4g);
  lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_2000dps);
  /* Enable Tap detection on X, Y, Z */
  lsm6dsox_tap_detection_on_z_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsox_tap_detection_on_y_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsox_tap_detection_on_x_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Tap threshold to 01000b, therefore the tap threshold
   * is 500 mg (= 12 * FS_XL / 32 )
   */
  lsm6dsox_tap_threshold_x_set(&dev_ctx, 0x04);
  lsm6dsox_tap_threshold_y_set(&dev_ctx, 0x04);
  lsm6dsox_tap_threshold_z_set(&dev_ctx, 0x04);
  /* Configure Single and Double Tap parameter
   *
   * For the maximum time between two consecutive detected taps, the DUR
   * field of the INT_DUR2 register is set to 0111b, therefore the Duration
   * time is 538.5 ms (= 7 * 32 * ODR_XL)
   *
   * The SHOCK field of the INT_DUR2 register is set to 11b, therefore
   * the Shock time is 57.36 ms (= 3 * 8 * ODR_XL)
   *
   * The QUIET field of the INT_DUR2 register is set to 11b, therefore
   * the Quiet time is 28.68 ms (= 3 * 4 * ODR_XL)
   */
  lsm6dsox_tap_dur_set(&dev_ctx, 0x07);
  lsm6dsox_tap_quiet_set(&dev_ctx, 0x03);
  lsm6dsox_tap_shock_set(&dev_ctx, 0x03);
  /* Enable Single and Double Tap detection. */
  lsm6dsox_tap_mode_set(&dev_ctx, LSM6DSOX_BOTH_SINGLE_DOUBLE);
  /* Apply high-pass digital filter on Wake-Up function */
  lsm6dsox_xl_hp_path_internal_set(&dev_ctx, LSM6DSOX_USE_SLOPE);
  /* Set Wake-Up threshold: 1 LSb corresponds to FS_XL/2^6 */
  lsm6dsox_wkup_threshold_set(&dev_ctx, 4);
  /* Set threshold to 60 degrees. */
  lsm6dsox_6d_threshold_set(&dev_ctx, LSM6DSOX_DEG_60);
  /* LPF2 on 6D/4D function selection. */
  lsm6dsox_xl_lp2_on_6d_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsox_4d_mode_set(&dev_ctx, PROPERTY_DISABLE);
  /* Set Free Fall duration to 3 and 6 samples event duration */
  lsm6dsox_ff_dur_set(&dev_ctx, 0x06);
  lsm6dsox_ff_threshold_set(&dev_ctx, LSM6DSOX_FF_TSH_312mg);
  /* Enable Tilt in embedded function. */
  emb_sens.tilt = PROPERTY_ENABLE;
  /* Enable pedometer */
  emb_sens.step = PROPERTY_ENABLE;
  lsm6dsox_pedo_sens_set(&dev_ctx, LSM6DSOX_FALSE_STEP_REJ_ADV_MODE);
  /* Route signals on interrupt pin 1 */
  lsm6dsox_pin_int1_route_get(&dev_ctx, &pin_int1_route);
  pin_int1_route.mlc1 = PROPERTY_ENABLE;
  pin_int1_route.step_detector = PROPERTY_ENABLE;
  pin_int1_route.wake_up =  PROPERTY_ENABLE;
  pin_int1_route.tilt = PROPERTY_ENABLE;
  pin_int1_route.double_tap = PROPERTY_ENABLE;
  pin_int1_route.single_tap = PROPERTY_ENABLE;
  pin_int1_route.six_d = PROPERTY_ENABLE;
  pin_int1_route.free_fall = PROPERTY_ENABLE;
  lsm6dsox_pin_int1_route_set(&dev_ctx, pin_int1_route);
  /* Configure interrupt pin mode notification */
  lsm6dsox_int_notification_set(&dev_ctx,
                                LSM6DSOX_BASE_PULSED_EMB_LATCHED);
  /* Enable embedded features */
  lsm6dsox_embedded_sens_set(&dev_ctx, &emb_sens);
  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_417Hz);
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_208Hz);
  /* Reset steps of pedometer */
  lsm6dsox_steps_reset(&dev_ctx);

  /* Main loop */
  while (1) {
    /* Read interrupt source registers in polling mode (no int) */
    lsm6dsox_all_sources_get(&dev_ctx, &status);

    if (status.wake_up) {
      sprintf((char *)tx_buffer, "Wake-Up event on ");

      if (status.wake_up_x) {
        strcat((char *)tx_buffer, "X");
      }

      if (status.wake_up_y) {
        strcat((char *)tx_buffer, "Y");
      }

      if (status.wake_up_z) {
        strcat((char *)tx_buffer, "Z");
      }

      strcat((char *)tx_buffer, " direction\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (status.step_detector) {
      /* Read steps */
      lsm6dsox_number_of_steps_get(&dev_ctx, &steps);
      sprintf((char *)tx_buffer, "steps :%d\r\n", steps);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (status.mlc1) {
      lsm6dsox_mlc_out_get(&dev_ctx, mlc_out);
      sprintf((char *)tx_buffer, "Detect MLC interrupt code: %02X\r\n",
              mlc_out[0]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (status.double_tap) {
      sprintf((char *)tx_buffer, "D-Tap: ");

      if (status.tap_x) {
        strcat((char *)tx_buffer, "x-axis");
      }

      else if (status.tap_y) {
        strcat((char *)tx_buffer, "y-axis");
      }

      else {
        strcat((char *)tx_buffer, "z-axis");
      }

      if (status.tap_sign) {
        strcat((char *)tx_buffer, " negative");
      }

      else {
        strcat((char *)tx_buffer, " positive");
      }

      strcat((char *)tx_buffer, " sign\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (status.single_tap) {
      sprintf((char *)tx_buffer, "S-Tap: ");

      if (status.tap_x) {
        strcat((char *)tx_buffer, "x-axis");
      }

      else if (status.tap_y) {
        strcat((char *)tx_buffer, "y-axis");
      }

      else {
        strcat((char *)tx_buffer, "z-axis");
      }

      if (status.tap_sign) {
        strcat((char *)tx_buffer, " negative");
      }

      else {
        strcat((char *)tx_buffer, " positive");
      }

      strcat((char *)tx_buffer, " sign\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (status.tilt) {
      sprintf((char *)tx_buffer, "TILT Detected\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (status.six_d) {
      sprintf((char *)tx_buffer, "6D Or. switched to ");

      if (status.six_d_xh) {
        strcat((char *)tx_buffer, "XH");
      }

      if (status.six_d_xl) {
        strcat((char *)tx_buffer, "XL");
      }

      if (status.six_d_yh) {
        strcat((char *)tx_buffer, "YH");
      }

      if (status.six_d_yl) {
        strcat((char *)tx_buffer, "YL");
      }

      if (status.six_d_zh) {
        strcat((char *)tx_buffer, "ZH");
      }

      if (status.six_d_zl) {
        strcat((char *)tx_buffer, "ZL");
      }

      strcat((char *)tx_buffer, "\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (status.free_fall) {
      sprintf((char *)tx_buffer, "Free Fall Detected\r\n");
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
  HAL_I2C_Mem_Write(handle, LSM6DSOX_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LSM6DSOX_I2C_ADD_L & 0xFE, reg, (uint8_t*) bufp, len);
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
  HAL_I2C_Mem_Read(handle, LSM6DSOX_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LSM6DSOX_I2C_ADD_L & 0xFE, reg, bufp, len);
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
