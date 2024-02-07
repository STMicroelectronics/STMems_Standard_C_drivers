/*
 ******************************************************************************
 * @file    fsm_sh_mag_anomalies_detection.c
 * @author  Sensors Software Solution Team
 * @brief   The purpose of this example is to show how use the device
 *          Finite State Machine (FSM) starting from an ".h" file
 *          generated through with the tool "Finite State Machine"
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
 * https://github.com/STMicroelectronics/STMems_Finite_State_Machine
 * the same repository is linked to this repository in folder "_resources"
 *
 * For more information about Finite State Machine tool please refer
 * to AN5273 "LSM6DSOX: Finite State Machine".
 *
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI197V1
 * - NUCLEO_F411RE + STEVAL-MKI197V1
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
//#define NUCLEO_F411RE    /* little endian */
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

#include "mag_anomalies_detection_fsm.h"
#include "lsm6dsox_reg.h"
#include "lis2mdl_reg.h"

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
#include "tim.h"

#elif defined(SPC584B_DIS)
#include "components.h"
#endif

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
static stmdev_ctx_t dev_ctx;
static stmdev_ctx_t mag_ctx;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

static int32_t lsm6dsox_write_lis2mdl_cx(void *ctx, uint8_t reg,
                                         const uint8_t *data,
					 uint16_t len);

static int32_t lsm6dsox_read_lis2mdl_cx(void *ctx, uint8_t reg,
                                        uint8_t *data,
                                        uint16_t len);

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

void lsm6dsox_fsm_sh_mag_anomalies_detection_handler(void)
{
  lsm6dsox_all_sources_t status;

  /* Read interrupt source registers in polling mode (no int) */
  lsm6dsox_all_sources_get(&dev_ctx, &status);

  if (status.fsm1) {
    sprintf((char *)tx_buffer, "MAG anomaly detected\r\n");
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }
}

/* Main Example --------------------------------------------------------------*/
void lsm6dsox_fsm_sh_mag_anomalies_detection(void)
{
  /* Variable declaration */
  lsm6dsox_pin_int2_route_t pin_int_route;
  uint32_t i;
  lsm6dsox_sh_cfg_read_t sh_cfg_read;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg  = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle    = &SENSOR_BUS;
  /* Initialize lis2mdl driver interface */
  mag_ctx.read_reg = lsm6dsox_read_lis2mdl_cx;
  mag_ctx.write_reg = lsm6dsox_write_lis2mdl_cx;
  mag_ctx.handle = &dev_ctx;

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

  /* Disable I3C interface */
  lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);

  /* Start FSM configuration */
  for ( i = 0; i < (sizeof(mag_anomalies_detection_fsm) /
                    sizeof(ucf_line_t) ); i++ ) {
    lsm6dsox_write_reg(&dev_ctx, mag_anomalies_detection_fsm[i].address,
                       (uint8_t *)&mag_anomalies_detection_fsm[i].data, 1);
  }


  /*
   * The loaded configuration already routes FSM1 events on INT1, but
   * we add routing on interrupt pin 2 as well.
   */
  lsm6dsox_pin_int2_route_get(&dev_ctx, NULL, &pin_int_route);
  pin_int_route.fsm1               = PROPERTY_ENABLE;
  lsm6dsox_pin_int2_route_set(&dev_ctx, NULL, pin_int_route);

  /* Configure interrupt pin mode notification */
  lsm6dsox_int_notification_set(&dev_ctx, LSM6DSOX_BASE_PULSED_EMB_LATCHED);

  /* Configure LIS2MDL. */
  lis2mdl_block_data_update_set(&mag_ctx, PROPERTY_ENABLE);
  lis2mdl_offset_temp_comp_set(&mag_ctx, PROPERTY_ENABLE);
  lis2mdl_operating_mode_set(&mag_ctx, LIS2MDL_CONTINUOUS_MODE);
  lis2mdl_data_rate_set(&mag_ctx, LIS2MDL_ODR_20Hz);

  /* Enable latched interrupt notification. */
  lsm6dsox_int_notification_set(&dev_ctx, LSM6DSOX_ALL_INT_LATCHED);

  /*
   * Prepare sensor hub to read data from external Slave0 continuously
   * in order to store data in FIFO.
   */
  sh_cfg_read.slv_add = (LIS2MDL_I2C_ADD & 0xFEU) >>
                        1; /* 7bit I2C address */
  sh_cfg_read.slv_subadd = LIS2MDL_OUTX_L_REG;
  sh_cfg_read.slv_len = 6;
  lsm6dsox_sh_slv0_cfg_read(&dev_ctx, &sh_cfg_read);
  /* Configure Sensor Hub to read one slave. */
  lsm6dsox_sh_slave_connected_set(&dev_ctx, LSM6DSOX_SLV_0);
  /* Enable I2C Master. */
  lsm6dsox_sh_master_set(&dev_ctx, PROPERTY_ENABLE);
  /* Configure LSM6DSOX. */
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_2g);
  lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_2000dps);
  lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_12Hz5);
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_12Hz5);

  /* FSM1 events are processed in interrupt handler */
  while (1);
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
#if defined(NUCLEO_F411RE)
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
#if defined(NUCLEO_F411RE)
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

/*
 * @brief  Write lsm2mdl device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t lsm6dsox_write_lis2mdl_cx(void *ctx, uint8_t reg,
                                         const uint8_t *data, uint16_t len)
{
  int16_t data_raw_acceleration[3];
  int32_t ret;
  uint8_t drdy;
  lsm6dsox_status_master_t master_status;
  lsm6dsox_sh_cfg_write_t sh_cfg_write;
  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_write.slv0_add = (LIS2MDL_I2C_ADD & 0xFEU) >> 1; /* 7bit I2C address */
  sh_cfg_write.slv0_subadd = reg,
  sh_cfg_write.slv0_data = *data,
  ret = lsm6dsox_sh_cfg_write((stmdev_ctx_t *)ctx, &sh_cfg_write);
  /* Disable accelerometer. */
  lsm6dsox_xl_data_rate_set((stmdev_ctx_t *)ctx, LSM6DSOX_XL_ODR_OFF);
  /* Enable I2C Master. */
  lsm6dsox_sh_master_set((stmdev_ctx_t *)ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dsox_xl_data_rate_set((stmdev_ctx_t *)ctx, LSM6DSOX_XL_ODR_104Hz);
  /* Wait Sensor Hub operation flag set. */
  lsm6dsox_acceleration_raw_get((stmdev_ctx_t *)ctx, data_raw_acceleration);

  do {
    platform_delay(20);
    lsm6dsox_xl_flag_data_ready_get((stmdev_ctx_t *)ctx, &drdy);
  } while (!drdy);

  do {
    platform_delay(20);
    lsm6dsox_sh_status_get((stmdev_ctx_t *)ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL (trigger). */
  lsm6dsox_sh_master_set((stmdev_ctx_t *)ctx, PROPERTY_DISABLE);
  lsm6dsox_xl_data_rate_set((stmdev_ctx_t *)ctx, LSM6DSOX_XL_ODR_OFF);
  return ret;
}

/*
 * @brief  Read lsm2mdl device register (used by configuration functions)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t lsm6dsox_read_lis2mdl_cx(void *ctx, uint8_t reg,
                                        uint8_t *data,
                                        uint16_t len)
{
  lsm6dsox_sh_cfg_read_t sh_cfg_read;
  int16_t data_raw_acceleration[3];
  int32_t ret;
  uint8_t drdy;
  lsm6dsox_status_master_t master_status;

  /* Disable accelerometer. */
  lsm6dsox_xl_data_rate_set((stmdev_ctx_t *)ctx, LSM6DSOX_XL_ODR_OFF);
  /* Configure Sensor Hub to read LIS2MDL. */
  sh_cfg_read.slv_add = (LIS2MDL_I2C_ADD & 0xFEU) >>
                        1; /* 7bit I2C address */
  sh_cfg_read.slv_subadd = reg;
  sh_cfg_read.slv_len = len;
  ret = lsm6dsox_sh_slv0_cfg_read((stmdev_ctx_t *)ctx, &sh_cfg_read);
  lsm6dsox_sh_slave_connected_set((stmdev_ctx_t *)ctx, LSM6DSOX_SLV_0);
  /* Enable I2C Master and I2C master. */
  lsm6dsox_sh_master_set((stmdev_ctx_t *)ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation. */
  lsm6dsox_xl_data_rate_set((stmdev_ctx_t *)ctx, LSM6DSOX_XL_ODR_104Hz);
  /* Wait Sensor Hub operation flag set. */
  lsm6dsox_acceleration_raw_get((stmdev_ctx_t *)ctx, data_raw_acceleration);

  do {
    platform_delay(20);
    lsm6dsox_xl_flag_data_ready_get((stmdev_ctx_t *)ctx, &drdy);
  } while (!drdy);

  do {
    //platform_delay(20);
    lsm6dsox_sh_status_get((stmdev_ctx_t *)ctx, &master_status);
  } while (!master_status.sens_hub_endop);

  /* Disable I2C master and XL(trigger). */
  lsm6dsox_sh_master_set((stmdev_ctx_t *)ctx, PROPERTY_DISABLE);
  lsm6dsox_xl_data_rate_set((stmdev_ctx_t *)ctx, LSM6DSOX_XL_ODR_OFF);
  /* Read SensorHub registers. */
  lsm6dsox_sh_read_data_raw_get((stmdev_ctx_t *)ctx, (lsm6dsox_emb_sh_read_t *)data,
                                len);
  return ret;
}