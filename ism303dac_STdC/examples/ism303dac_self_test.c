/*
 ******************************************************************************
 * @file    self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file implement the self test procedure.
 *
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

/* This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI184V1
 * - NUCLEO_F401RE + STEVAL-MKI184V1
 * - DISCOVERY_SPC584B + STEVAL-MKI184V1
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
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
#include "ism303dac_reg.h"

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

#define    BOOT_TIME         20 //ms
#define    WAIT_TIME_XL     200 //ms
#define    WAIT_TIME_01     20 //ms
#define    WAIT_TIME_02     60 //ms

#define    SAMPLES_XL        5 //number of samples
#define    SAMPLES_MG       50 //number of samples

/* Self test limits. */
#define    MIN_ST_LIMIT_mg         70.0f
#define    MAX_ST_LIMIT_mg       1500.0f
#define    MIN_ST_LIMIT_mG         15.0f
#define    MAX_ST_LIMIT_mG        500.0f

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

/* Private variables ---------------------------------------------------------*/
#if defined(STEVAL_MKI109V3)
static sensbus_t xl_bus =  {&SENSOR_BUS,
                            0,
                            CS_up_GPIO_Port,
                            CS_up_Pin
                           };
static sensbus_t mag_bus = {&SENSOR_BUS,
                            0,
                            CS_A_up_GPIO_Port,
                            CS_A_up_Pin
                           };
#elif defined(NUCLEO_F401RE)
static sensbus_t xl_bus =  {&SENSOR_BUS,
                            ISM303DAC_I2C_ADD_XL,
                            0,
                            0
                           };
static sensbus_t mag_bus = {&SENSOR_BUS,
                            ISM303DAC_I2C_ADD_MG,
                            0,
                            0
                           };
#elif defined(SPC584B_DIS)
static sensbus_t xl_bus =  {&SENSOR_BUS,
                            ISM303DAC_I2C_ADD_XL,
                            0,
                            0
                           };
static sensbus_t mag_bus = {&SENSOR_BUS,
                            ISM303DAC_I2C_ADD_MG,
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
void ism303dac_self_test(void)
{
  stmdev_ctx_t dev_ctx_xl;
  stmdev_ctx_t dev_ctx_mg;
  uint8_t tx_buffer[1000];
  float_t meas_st_off[3];
  int16_t data_raw[3];
  float_t meas_st_on[3];
  ism303dac_reg_t reg;
  float_t test_val[3];
  uint8_t st_result;
  uint8_t i, j;
  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_xl.write_reg = platform_write;
  dev_ctx_xl.read_reg = platform_read;
  dev_ctx_xl.handle = (void *)&xl_bus;
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mg.write_reg = platform_write;
  dev_ctx_mg.read_reg = platform_read;
  dev_ctx_mg.handle = (void *)&mag_bus;
  /* Initialize self test results */
  st_result = ST_PASS;
  /* Initialize platform specific hardware */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  reg.byte = 0;
  ism303dac_xl_device_id_get(&dev_ctx_xl, &reg.byte);

  if ( reg.byte != ISM303DAC_ID_XL )
    while (1); /*manage here device not found */

  reg.byte = 0;
  ism303dac_mg_device_id_get(&dev_ctx_mg, &reg.byte);

  if ( reg.byte != ISM303DAC_ID_MG )
    while (1); /*manage here device not found */

  /* Restore default configuration */
  ism303dac_xl_reset_set(&dev_ctx_xl, PROPERTY_ENABLE);

  do {
    ism303dac_xl_reset_get(&dev_ctx_xl, &reg.byte);
  } while (reg.byte);

  ism303dac_mg_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);

  do {
    ism303dac_mg_reset_get(&dev_ctx_mg, &reg.byte);
  } while (reg.byte);

  /* Enable Block Data Update */
  ism303dac_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
  ism303dac_mg_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /*
   * START ACCELEROMETER SELF TEST PROCEDURE
   */
  /* Set full scale */
  ism303dac_xl_full_scale_set(&dev_ctx_xl, ISM303DAC_XL_2g);
  /* Set Output Data Rate. */
  ism303dac_xl_data_rate_set(&dev_ctx_xl, ISM303DAC_XL_ODR_50Hz_HR);
  /* Wait stable output */
  platform_delay(WAIT_TIME_XL);

  /* Check if new value available */
  do {
    ism303dac_xl_status_reg_get(&dev_ctx_xl, &reg.status_a);
  } while (!reg.status_a.drdy);

  /* Read dummy data and discard it */
  ism303dac_acceleration_raw_get(&dev_ctx_xl, data_raw);

  /* Read samples and get the average vale for each axis */
  for (i = 0; i < SAMPLES_XL; i++) {
    /* Check if new value available */
    do {
      ism303dac_xl_status_reg_get(&dev_ctx_xl, &reg.status_a);
    } while (!reg.status_a.drdy);

    /* Read data and accumulate the mg value */
    ism303dac_acceleration_raw_get(&dev_ctx_xl, data_raw);

    for (j = 0; j < 3; j++) {
      meas_st_off[j] += ism303dac_from_fs2g_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    meas_st_off[i] /= SAMPLES_XL;
  }

  /* Enable Self Test positive (or negative) */
  ism303dac_xl_self_test_set(&dev_ctx_xl, ISM303DAC_XL_ST_POSITIVE);
  //ism303dac_xl_self_test_set(&dev_ctx_xl, ISM303DAC_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(WAIT_TIME_XL);

  /* Check if new value available */
  do {
    ism303dac_xl_status_reg_get(&dev_ctx_xl, &reg.status_a);
  } while (!reg.status_a.drdy);

  /* Read dummy data and discard it */
  ism303dac_acceleration_raw_get(&dev_ctx_xl, data_raw);

  /* Read samples and get the average vale for each axis */
  for (i = 0; i < SAMPLES_XL; i++) {
    /* Check if new value available */
    do {
      ism303dac_xl_status_reg_get(&dev_ctx_xl, &reg.status_a);
    } while (!reg.status_a.drdy);

    /* Read data and accumulate the mg value */
    ism303dac_acceleration_raw_get(&dev_ctx_xl, data_raw);

    for (j = 0; j < 3; j++) {
      meas_st_on[j] += ism303dac_from_fs2g_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    meas_st_on[i] /= SAMPLES_XL;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabsf((meas_st_on[i] - meas_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mg > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mg)) {
      st_result = ST_FAIL;
    }

    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }

  /* Disable Self Test */
  ism303dac_xl_self_test_set(&dev_ctx_xl, ISM303DAC_XL_ST_DISABLE);
  /* Disable sensor. */
  ism303dac_xl_data_rate_set(&dev_ctx_xl, ISM303DAC_XL_ODR_OFF);
  /*
   * END ACCELEROMETER SELF TEST PROCEDURE
   */
  /*
   * START MAGNETOMETER SELF TEST PROCEDURE
   */
  /* Temperature compensation enable */
  ism303dac_mg_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Set restore magnetic condition policy */
  ism303dac_mg_set_rst_mode_set(&dev_ctx_mg,
                                ISM303DAC_MG_SET_SENS_ODR_DIV_63);
  /* Set power mode */
  ism303dac_mg_power_mode_set(&dev_ctx_mg,
                              ISM303DAC_MG_HIGH_RESOLUTION);
  /* Set Output Data Rate */
  ism303dac_mg_data_rate_set(&dev_ctx_mg, ISM303DAC_MG_ODR_100Hz);
  /* Set Operating mode */
  ism303dac_mg_operating_mode_set(&dev_ctx_mg,
                                  ISM303DAC_MG_CONTINUOUS_MODE);
  /* Wait stable output */
  platform_delay(WAIT_TIME_01);

  /* Check if new value available */
  do {
    ism303dac_mg_status_get(&dev_ctx_mg, &reg.status_reg_m);
  } while (!reg.status_reg_m.zyxda);

  /* Read dummy data and discard it */
  ism303dac_magnetic_raw_get(&dev_ctx_mg, data_raw);
  /* Read samples and get the average vale for each axis */
  memset(meas_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < SAMPLES_MG; i++) {
    /* Check if new value available */
    do {
      ism303dac_mg_status_get(&dev_ctx_mg, &reg.status_reg_m);
    } while (!reg.status_reg_m.zyxda);

    /* Read data and accumulate the mg value */
    ism303dac_magnetic_raw_get(&dev_ctx_mg, data_raw);

    for (j = 0; j < 3; j++) {
      meas_st_off[j] += ism303dac_from_lsb_to_mG(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    meas_st_off[i] /= SAMPLES_MG;
  }

  /* Enable Self Test */
  ism303dac_mg_self_test_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Wait stable output */
  platform_delay(WAIT_TIME_02);

  /* Check if new value available */
  do {
    ism303dac_mg_status_get(&dev_ctx_mg, &reg.status_reg_m);
  } while (!reg.status_reg_m.zyxda);

  /* Read dummy data and discard it */
  ism303dac_magnetic_raw_get(&dev_ctx_mg, data_raw);
  /* Read samples and get the average vale for each axis */
  memset(meas_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < SAMPLES_MG; i++) {
    /* Check if new value available */
    do {
      ism303dac_mg_status_get(&dev_ctx_mg, &reg.status_reg_m);
    } while (!reg.status_reg_m.zyxda);

    /* Read data and accumulate the mg value */
    ism303dac_magnetic_raw_get(&dev_ctx_mg, data_raw);

    for (j = 0; j < 3; j++) {
      meas_st_on[j] += ism303dac_from_lsb_to_mG(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    meas_st_on[i] /= SAMPLES_MG;
  }

  st_result = ST_PASS;

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabsf((meas_st_on[i] - meas_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mG > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mG)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  ism303dac_mg_self_test_set(&dev_ctx_mg, PROPERTY_DISABLE);
  /* Disable sensor. */
  ism303dac_mg_operating_mode_set(&dev_ctx_mg, ISM303DAC_MG_POWER_DOWN);

  /*
   * END MAGNETOMETER SELF TEST PROCEDURE
   */

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
  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  HAL_SPI_Transmit(sensbus->hbus, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
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
  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  HAL_SPI_Receive(sensbus->hbus, bufp, len, 1000);
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
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
