/*
 ******************************************************************************
 * @file    self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file implement the self test procedure.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI172V1
 * - NUCLEO_F401RE + X-NUCLEO-IKS01A2
 * - DISCOVERY_SPC584B + STEVAL-MKI173V1
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_F401RE      - Host side: UART(COM) to USB bridge
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
#include "lsm303agr_reg.h"
#include <string.h>
#include <stdio.h>

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

typedef struct {
  void   *hbus;
  uint8_t i2c_address;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
} sensbus_t;

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME               5 //ms

/* Self test limits. */
#define    MIN_ST_XL_LIMIT_mg     68.0f
#define    MAX_ST_XL_LIMIT_mg   1440.0f
#define    MIN_ST_MG_LIMIT_mG     15.0f
#define    MAX_ST_MG_LIMIT_mG    500.0f
/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

/* Private variables ---------------------------------------------------------*/
#if defined(STEVAL_MKI109V3)
static sensbus_t xl_bus  = {&SENSOR_BUS,
                            0,
                            CS_up_GPIO_Port,
                            CS_up_Pin
                           };
static sensbus_t mag_bus = {&SENSOR_BUS,
                            0,
                            CS_A_up_GPIO_Port,
                            CS_A_up_Pin
                           };
#elif defined(NUCLEO_F401RE) || defined(SPC584B_DIS)
static sensbus_t xl_bus  = {&SENSOR_BUS,
                            LSM303AGR_I2C_ADD_XL,
                            0,
                            0
                           };
static sensbus_t mag_bus = {&SENSOR_BUS,
                            LSM303AGR_I2C_ADD_MG,
                            0,
                            0
                           };
#endif

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
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lsm303agr_self_test(void)
{
  uint8_t tx_buffer[1000];
  int16_t data_raw[3];
  float maes_st_off[3];
  float maes_st_on[3];
  lsm303agr_reg_t reg;
  float test_val[3];
  uint8_t st_result;
  uint8_t i, j;
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx_xl;
  dev_ctx_xl.write_reg = platform_write;
  dev_ctx_xl.read_reg = platform_read;
  dev_ctx_xl.handle = (void *)&xl_bus;
  stmdev_ctx_t dev_ctx_mg;
  dev_ctx_mg.write_reg = platform_write;
  dev_ctx_mg.read_reg = platform_read;
  dev_ctx_mg.handle = (void *)&mag_bus;
  /* Initialize self test results */
  st_result = ST_PASS;
  /* Wait boot time and initialize platform specific hardware */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  reg.byte = 0;
  lsm303agr_xl_device_id_get(&dev_ctx_xl, &reg.byte);

  if ( reg.byte != LSM303AGR_ID_XL )
    while (1); /*manage here device not found */

  reg.byte = 0;
  lsm303agr_mag_device_id_get(&dev_ctx_mg, &reg.byte);

  if ( reg.byte != LSM303AGR_ID_MG )
    while (1); /*manage here device not found */

  /*
   * Accelerometer Self Test
   */
  /* Enable Block Data Update. */
  lsm303agr_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
  /* Set full scale to 2g. */
  lsm303agr_xl_full_scale_set(&dev_ctx_xl, LSM303AGR_2g);
  /* Set device in normal mode. */
  lsm303agr_xl_operating_mode_set(&dev_ctx_xl, LSM303AGR_NM_10bit);
  /* Set Output Data Rate. */
  lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_100Hz);
  /* Wait stable output. */
  platform_delay(90);

  /* Check if new value available */
  do {
    lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
  } while (!reg.status_reg_a.zyxda);

  /* Read dummy data and discard it */
  lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw);

  /* Read 5 sample and get the average value for each axis */
  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
    } while (!reg.status_reg_a.zyxda);

    /* Read data and accumulate the mg value */
    lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw);

    for (j = 0; j < 3; j++) {
      maes_st_off[j] += lsm303agr_from_fs_2g_nm_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    maes_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm303agr_xl_self_test_set(&dev_ctx_xl, LSM303AGR_ST_POSITIVE);
  //lsm303agr_xl_self_test_set(&dev_ctx, LSM303AGR_XL_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(90);

  /* Check if new value available */
  do {
    lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
  } while (!reg.status_reg_a.zyxda);

  /* Read dummy data and discard it */
  lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw);

  /* Read 5 sample and get the average value for each axis */
  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
    } while (!reg.status_reg_a.zyxda);

    /* Read data and accumulate the mg value */
    lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw);

    for (j = 0; j < 3; j++) {
      maes_st_on[j] += lsm303agr_from_fs_2g_nm_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    maes_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((maes_st_on[i] - maes_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( MIN_ST_XL_LIMIT_mg > test_val[i] ) ||
        ( test_val[i] > MAX_ST_XL_LIMIT_mg)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm303agr_xl_self_test_set(&dev_ctx_xl, LSM303AGR_ST_DISABLE);
  /* Disable sensor. */
  lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_POWER_DOWN);
  /*
   * Magnetometer  Self Test
   */
  /* Restore default configuration for magnetometer */
  lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);

  do {
    lsm303agr_mag_reset_get(&dev_ctx_mg, &reg.byte);
  } while (reg.byte);

  /* Enable Block Data Update. */
  lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Set / Reset sensor mode. */
  lsm303agr_mag_set_rst_mode_set(&dev_ctx_mg,
                                 LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
  /* Enable temperature compensation. */
  lsm303agr_mag_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Set device in continuous mode. */
  lsm303agr_mag_operating_mode_set(&dev_ctx_mg,
                                   LSM303AGR_CONTINUOUS_MODE);
  /* Set Output Data Rate. */
  lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_100Hz);
  /* Wait stable output. */
  platform_delay(20);

  /* Check if new value available .*/
  do {
    lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
  } while (!reg.status_reg_m.zyxda);

  /* Read dummy data and discard it. */
  lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw);

  /* Read 50 sample and get the average value for each axis */
  for (i = 0; i < 50; i++) {
    /* Check if new value available */
    do {
      lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
    } while (!reg.status_reg_m.zyxda);

    /* Read data and accumulate the mg value */
    lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw);

    for (j = 0; j < 3; j++) {
      maes_st_off[j] += lsm303agr_from_lsb_to_mgauss(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    maes_st_off[i] /= 50.0f;
  }

  /* Enable Self Test. */
  lsm303agr_mag_self_test_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Wait stable output */
  platform_delay(60);

  /* Check if new value available .*/
  do {
    lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
  } while (!reg.status_reg_m.zyxda);

  /* Read dummy data and discard it. */
  lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw);

  /* Read 50 sample and get the average value for each axis */
  for (i = 0; i < 50; i++) {
    /* Check if new value available */
    do {
      lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
    } while (!reg.status_reg_m.zyxda);

    /* Read data and accumulate the mg value */
    lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw);

    for (j = 0; j < 3; j++) {
      maes_st_on[j] += lsm303agr_from_lsb_to_mgauss(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    maes_st_on[i] /= 50.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((maes_st_on[i] - maes_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( MIN_ST_MG_LIMIT_mG > test_val[i] ) ||
        ( test_val[i] > MAX_ST_MG_LIMIT_mG)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm303agr_mag_self_test_set(&dev_ctx_mg, PROPERTY_DISABLE);
  /* Disable sensor. */
  lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_POWER_DOWN);

  /* Print self test result */
  if (st_result == ST_PASS) {
    snprintf((char *)tx_buffer, sizeof(tx_buffer), "Self Test - PASS\r\n" );
  }

  else {
    snprintf((char *)tx_buffer, sizeof(tx_buffer), "Self Test - FAIL\r\n" );
  }

  tx_com(tx_buffer, strlen((char const *)tx_buffer));
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
  sensbus_t *sensbus = (sensbus_t *)handle;
#if defined(NUCLEO_F401RE)

  if (sensbus->i2c_address == LSM303AGR_I2C_ADD_XL) {
    /* enable auto incremented in multiple read/write commands */
    reg |= 0x80;
  }

  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)

  if (sensbus->cs_pin == CS_up_Pin) {
    /* enable auto incremented in multiple read/write commands */
    reg |= 0x40;
  }

  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  HAL_SPI_Transmit(sensbus->hbus, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)

  if (sensbus->i2c_address == LSM303AGR_I2C_ADD_XL) {
    /* enable auto incremented in multiple read/write commands */
    reg |= 0x80;
  }

  i2c_lld_write(sensbus->hbus, sensbus->i2c_address & 0xFE, reg,
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
  sensbus_t *sensbus = (sensbus_t *)handle;
#if defined(NUCLEO_F401RE)

  if (sensbus->i2c_address == LSM303AGR_I2C_ADD_XL) {
    /* enable auto incremented in multiple read/write commands */
    reg |= 0x80;
  }

  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;

  if (sensbus->cs_pin == CS_up_Pin) {
    /* enable auto incremented in multiple read/write commands */
    reg |= 0x40;
  }

  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  HAL_SPI_Receive(sensbus->hbus, bufp, len, 1000);
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)

  if (sensbus->i2c_address == LSM303AGR_I2C_ADD_XL) {
    /* enable auto incremented in multiple read/write commands */
    reg |= 0x80;
  }

  i2c_lld_read(sensbus->hbus, sensbus->i2c_address & 0xFE, reg, bufp,
               len);
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

