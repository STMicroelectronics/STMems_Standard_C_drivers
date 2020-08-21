/*
 ******************************************************************************
 * @file    self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file implements self test process described by AN5069.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI159V1
 * - NUCLEO_F411RE + STEVAL-MKI159V11
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
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
#include "lsm9ds1_reg.h"
#include "gpio.h"
#include "i2c.h"
#if defined(STEVAL_MKI109V3)
#include "usbd_cdc_if.h"
#include "spi.h"
#elif defined(NUCLEO_F411RE)
#include "usart.h"
#endif

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef struct {
  void*   hbus;
  uint8_t i2c_address;
  uint8_t cs_port;
  uint8_t cs_pin;
} sensbus_t;

/* Private macro -------------------------------------------------------------*/

#define    BOOT_TIME            20 //ms

#define    WAIT_TIME_MAG        60 //ms
#define    WAIT_TIME_XL        200 //ms
#define    WAIT_TIME_GY        800 //ms

#define    SAMPLES               5 //number of samples

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

/* Self test limits in mgauss @ 12G*/
static const float min_st_mag_limit[] = {1000.0f, 1000.0f,  100.0f};
static const float max_st_mag_limit[] = {3000.0f, 3000.0f, 1000.0f};

/* Self test limits in mg @ 2g*/
static const float min_st_xl_limit[] = {70.0f, 70.0f,  70.0f};
static const float max_st_xl_limit[] = {1500.0f, 1500.0f, 1500.0f};

/* Self test limits in mdps @ 2000 dps*/
static const float min_st_gy_limit[] = {200.0f, 200.0f,  200.0f};
static const float max_st_gy_limit[] = {800.0f, 800.0f, 800.0f};

/* Private variables ---------------------------------------------------------*/
#if defined(STEVAL_MKI109V3)
static sensbus_t imu_bus = {&SENSOR_BUS,
                            0,
                            CS_DEV_GPIO_Port,
                            CS_DEV_Pin};
static sensbus_t mag_bus = {&SENSOR_BUS,
                            0,
                            CS_RF_GPIO_Port,
                            CS_RF_Pin};
#elif defined(NUCLEO_F411RE)
static sensbus_t mag_bus = {&SENSOR_BUS,
                            LSM9DS1_MAG_I2C_ADD_H,
                            0,
                            0};
static sensbus_t imu_bus = {&SENSOR_BUS,
                            LSM9DS1_IMU_I2C_ADD_H,
                            0,
                            0};
#endif
/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write_imu(void *handle, uint8_t reg, uint8_t *bufp,
                                  uint16_t len);
static int32_t platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp,
                                 uint16_t len);
static int32_t platform_write_mag(void *handle, uint8_t reg, uint8_t *bufp,
                                  uint16_t len);
static int32_t platform_read_mag(void *handle, uint8_t reg, uint8_t *bufp,
                                 uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lis9ds1_self_test(void)
{
  stmdev_ctx_t dev_ctx_imu;
  stmdev_ctx_t dev_ctx_mag;
  uint8_t tx_buffer[1000];
  axis3bit16_t data_raw;
  lsm9ds1_status_t reg;
  lsm9ds1_id_t whoamI;
  float val_st_off[3];
  float val_st_on[3];
  float test_val[3];
  uint8_t st_result;
  uint8_t rst;
  uint8_t i;
  uint8_t j;


  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_imu.write_reg = platform_write_imu;
  dev_ctx_imu.read_reg = platform_read_imu;
  dev_ctx_imu.handle = (void*)&imu_bus;

  /* Initialize magnetic sensors driver interface */
  dev_ctx_mag.write_reg = platform_write_mag;
  dev_ctx_mag.read_reg = platform_read_mag;
  dev_ctx_mag.handle = (void*)&mag_bus;

  /* Init test platform */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);
  if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID){
    while(1){
      /* manage here device not found */
    }
  }

  /* Restore default configuration */
  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
  do {
    lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

  /* Initialize test variable */
  st_result = ST_PASS;

  /*
   * START MAGNETOMETER SELF TEST PROCEDURE
   */

  /* Set Full Scale */
  lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_12Ga);
  /* Set Output Data Rate */
  lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_LP_80Hz);

  /* Wait stable output */
  platform_delay(WAIT_TIME_MAG);

  /* Check if new value available */
  do {
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);
  } while(!reg.status_mag.zyxda);
  /* Read dummy data and discard it */
  lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw.u8bit);

  /* Read samples and get the average vale for each axis */
  memset(val_st_off, 0x00, 3*sizeof(float));
  for (i = 0; i < SAMPLES; i++){
    /* Check if new value available */
    do {
      lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);
    } while(reg.status_mag.zyxda);
    /* Read data and accumulate */
    lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw.u8bit);
    for (j = 0; j < 3; j++){
      val_st_off[j] += lsm9ds1_from_fs12gauss_to_mG(data_raw.i16bit[j]);
    }
  }
  /* Calculate the average values */
  for (i = 0; i < 3; i++){
    val_st_off[i] /= SAMPLES;
  }

  /* Enable Self Test */
  lsm9ds1_mag_self_test_set(&dev_ctx_mag, PROPERTY_ENABLE);

  /* Wait stable output */
  platform_delay(WAIT_TIME_MAG);

  /* Check if new value available */
  do {
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);
  } while(!reg.status_mag.zyxda);
  /* Read dummy data and discard it */
  lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw.u8bit);

  /* Read samples and get the average vale for each axis */
  memset(val_st_on, 0x00, 3*sizeof(float));
  for (i = 0; i < SAMPLES; i++){
    /* Check if new value available */
    do {
      lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);
    } while(!reg.status_mag.zyxda);
    /* Read data and accumulate */
    lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw.u8bit);
    for (j = 0; j < 3; j++){
      val_st_on[j] += lsm9ds1_from_fs12gauss_to_mG(data_raw.i16bit[j]);
    }
  }

  /* Calculate the average values */
  for (i = 0; i < 3; i++){
    val_st_on[i] /= SAMPLES;
  }

  /* Calculate the values for self test */
  for (i = 0; i < 3; i++){
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }
  /* Check self test limit */
  for (i = 0; i < 3; i++){
    if (( min_st_mag_limit[i] > test_val[i] ) ||
        ( test_val[i] > max_st_mag_limit[i])){
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm9ds1_mag_self_test_set(&dev_ctx_mag, PROPERTY_DISABLE);
  /* Disable sensor. */
  lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_POWER_DOWN);

  /*
   * END MAGNETOMETER SELF TEST PROCEDURE
   */

  /*
   * START ACCELEROMETER SELF TEST PROCEDURE
   */

  /* Set Full Scale */
  lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_2g);
  /* Set Output Data Rate */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_GY_OFF_XL_50Hz);

  /* Wait stable output */
  platform_delay(WAIT_TIME_XL);

  /* Check if new value available */
  do {
    lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
  } while(!reg.status_imu.xlda);
  /* Read dummy data and discard it */
  lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw.u8bit);

  /* Read samples and get the average vale for each axis */
  memset(val_st_off, 0x00, 3*sizeof(float));
  for (i = 0; i < SAMPLES; i++){
    /* Check if new value available */
    do {
      lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
    } while(reg.status_imu.xlda);
    /* Read data and accumulate */
    lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw.u8bit);
    for (j = 0; j < 3; j++){
      val_st_off[j] += lsm9ds1_from_fs2g_to_mg(data_raw.i16bit[j]);
    }
  }
  /* Calculate the average values */
  for (i = 0; i < 3; i++){
    val_st_off[i] /= SAMPLES;
  }

  /* Enable Self Test */
  lsm9ds1_xl_self_test_set(&dev_ctx_imu, PROPERTY_ENABLE);

  /* Wait stable output */
  platform_delay(WAIT_TIME_XL);

  /* Check if new value available */
  do {
    lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
  } while(!reg.status_imu.xlda);
  /* Read dummy data and discard it */
  lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw.u8bit);

  /* Read samples and get the average vale for each axis */
  memset(val_st_on, 0x00, 3*sizeof(float));
  for (i = 0; i < SAMPLES; i++){
    /* Check if new value available */
    do {
      lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
    } while(!reg.status_imu.xlda);
    /* Read data and accumulate */
    lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw.u8bit);
    for (j = 0; j < 3; j++){
      val_st_on[j] += lsm9ds1_from_fs2g_to_mg(data_raw.i16bit[j]);
    }
  }

  /* Calculate the average values */
  for (i = 0; i < 3; i++){
    val_st_on[i] /= SAMPLES;
  }

  /* Calculate the values for self test */
  for (i = 0; i < 3; i++){
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }
  /* Check self test limit */
  for (i = 0; i < 3; i++){
    if (( min_st_xl_limit[i] > test_val[i] ) ||
        ( test_val[i] > max_st_xl_limit[i])){
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm9ds1_xl_self_test_set(&dev_ctx_imu, PROPERTY_DISABLE);
  /* Disable sensor. */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_OFF);

  /*
   * END ACCELEROMETER SELF TEST PROCEDURE
   */

  /*
   * START GYROSCOPE SELF TEST PROCEDURE
   */

  /* Set Full Scale */
  lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
  /* Set Output Data Rate */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_XL_OFF_GY_238Hz);

  /* Wait stable output */
  platform_delay(WAIT_TIME_GY);

  /* Check if new value available */
  do {
    lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
  } while(!reg.status_imu.gda);
  /* Read dummy data and discard it */
  lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw.u8bit);

  /* Read samples and get the average vale for each axis */
  memset(val_st_off, 0x00, 3*sizeof(float));
  for (i = 0; i < SAMPLES; i++){
    /* Check if new value available */
    do {
      lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
    } while(reg.status_imu.gda);
    /* Read data and accumulate */
    lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw.u8bit);
    for (j = 0; j < 3; j++){
      val_st_off[j] += lsm9ds1_from_fs2000dps_to_mdps(data_raw.i16bit[j]);
    }
  }
  /* Calculate the average values */
  for (i = 0; i < 3; i++){
    val_st_off[i] /= SAMPLES;
  }

  /* Enable Self Test */
  lsm9ds1_gy_self_test_set(&dev_ctx_imu, PROPERTY_ENABLE);

  /* Wait stable output */
  platform_delay(WAIT_TIME_GY);

  /* Check if new value available */
  do {
    lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
  } while(!reg.status_imu.gda);
  /* Read dummy data and discard it */
  lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw.u8bit);

  /* Read samples and get the average vale for each axis */
  memset(val_st_on, 0x00, 3*sizeof(float));
  for (i = 0; i < SAMPLES; i++){
    /* Check if new value available */
    do {
      lsm9ds1_dev_status_get(&dev_ctx_imu, &dev_ctx_imu, &reg);
    } while(!reg.status_imu.gda);
    /* Read data and accumulate */
    lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw.u8bit);
    for (j = 0; j < 3; j++){
      val_st_on[j] += lsm9ds1_from_fs2000dps_to_mdps(data_raw.i16bit[j]);
    }
  }

  /* Calculate the average values */
  for (i = 0; i < 3; i++){
    val_st_on[i] /= SAMPLES;
  }

  /* Calculate the values for self test */
  for (i = 0; i < 3; i++){
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }
  /* Check self test limit */
  for (i = 0; i < 3; i++){
    if (( min_st_gy_limit[i] > test_val[i] ) ||
        ( test_val[i] > max_st_gy_limit[i])){
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm9ds1_gy_self_test_set(&dev_ctx_imu, PROPERTY_DISABLE);
  /* Disable sensor. */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_OFF);

  /*
   * END GYROSCOPE SELF TEST PROCEDURE
   */

  if (st_result == ST_PASS) {
    sprintf((char*)tx_buffer, "Self Test - PASS\r\n" );
  }
  else {
    sprintf((char*)tx_buffer, "Self Test - FAIL\r\n" );
  }
  tx_com(tx_buffer, strlen((char const*)tx_buffer));
}

/*
 * @brief  Write generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_imu(void *handle, uint8_t reg, uint8_t *bufp,
                                  uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t*)handle;

#if defined(NUCLEO_F411RE)
  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
                    I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  HAL_SPI_Transmit(sensbus->hbus, bufp, len, 1000);
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
#endif
  return 0;
}

/*
 * @brief  Write generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_mag(void *handle, uint8_t reg, uint8_t *bufp,
                                  uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t*)handle;

#if defined(NUCLEO_F411RE)
  /* Write multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
                    I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Write multiple command */
  reg |= 0x40;
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  HAL_SPI_Transmit(sensbus->hbus, bufp, len, 1000);
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
#endif
  return 0;
}

/*
 * @brief  Read generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t*)handle;

#if defined(NUCLEO_F411RE)
  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Read command */
  reg |= 0x80;
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  HAL_SPI_Receive(sensbus->hbus, bufp, len, 1000);
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
#endif
  return 0;
}

/*
 * @brief  Read generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_mag(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t*)handle;

#if defined(NUCLEO_F411RE)
  /* Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Read multiple command */
  reg |= 0xC0;
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  HAL_SPI_Receive(sensbus->hbus, bufp, len, 1000);
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
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
  #ifdef NUCLEO_F411RE
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
  #endif
  #ifdef STEVAL_MKI109V3
  CDC_Transmit_FS(tx_buffer, len);
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
